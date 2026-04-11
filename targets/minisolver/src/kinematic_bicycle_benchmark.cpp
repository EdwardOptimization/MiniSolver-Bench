#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include "asset_benchmark_common.h"
#include "kinematicbicycletrackmodel.h"
#include "minisolver/solver/solver.h"

using namespace minisolver;

namespace {

constexpr int kMaxHorizon = 128;
constexpr double kWheelbase = 0.33;
constexpr double kVMin = 0.1;
constexpr double kVMax = 12.0;
constexpr double kDeltaMin = -0.50;
constexpr double kDeltaMax = 0.50;

template <typename Model, int MAX_N>
void shift_primal_guess(MiniSolver<Model, MAX_N>& solver) {
    const int N = solver.get_horizon();
    for (int k = 0; k < N; ++k) {
        for (int i = 0; i < Model::NX; ++i) {
            solver.set_state_guess(k, i, solver.get_state(k + 1, i));
        }
    }
    for (int i = 0; i < Model::NX; ++i) {
        solver.set_state_guess(N, i, solver.get_state(N, i));
    }

    for (int k = 0; k < N; ++k) {
        const int src_k = std::min(k + 1, N - 1);
        for (int i = 0; i < Model::NU; ++i) {
            solver.set_control_guess(k, i, solver.get_control(src_k, i));
        }
    }
}

struct DrivingState {
    double x = 0.0;
    double y = 0.0;
    double psi = 0.0;
    double v = 0.0;
    double delta = 0.0;
};

DrivingState simulate(const DrivingState& state, double a, double delta_rate, double dt) {
    auto deriv = [&](const DrivingState& s) {
        return DrivingState{
            s.v * std::cos(s.psi),
            s.v * std::sin(s.psi),
            s.v * std::tan(s.delta) / kWheelbase,
            a,
            delta_rate,
        };
    };
    auto add_scaled = [&](const DrivingState& s, const DrivingState& ds, double scale) {
        return DrivingState{
            s.x + scale * ds.x,
            s.y + scale * ds.y,
            s.psi + scale * ds.psi,
            s.v + scale * ds.v,
            s.delta + scale * ds.delta,
        };
    };

    const DrivingState k1 = deriv(state);
    const DrivingState k2 = deriv(add_scaled(state, k1, 0.5 * dt));
    const DrivingState k3 = deriv(add_scaled(state, k2, 0.5 * dt));
    const DrivingState k4 = deriv(add_scaled(state, k3, dt));

    DrivingState next{
        state.x + (dt / 6.0) * (k1.x + 2.0 * k2.x + 2.0 * k3.x + k4.x),
        state.y + (dt / 6.0) * (k1.y + 2.0 * k2.y + 2.0 * k3.y + k4.y),
        state.psi + (dt / 6.0) * (k1.psi + 2.0 * k2.psi + 2.0 * k3.psi + k4.psi),
        state.v + (dt / 6.0) * (k1.v + 2.0 * k2.v + 2.0 * k3.v + k4.v),
        state.delta + (dt / 6.0) * (k1.delta + 2.0 * k2.delta + 2.0 * k3.delta + k4.delta),
    };
    next.psi = nmpc_bench::wrap_angle(next.psi);
    next.v = std::clamp(next.v, kVMin, kVMax);
    next.delta = std::clamp(next.delta, kDeltaMin, kDeltaMax);
    return next;
}

double current_step_constraint_violation(const DrivingState& current,
                                         const nmpc_bench::DrivingTrackAsset& asset,
                                         size_t ref_idx) {
    const auto sample = nmpc_bench::sample_driving_track(asset, asset.s[ref_idx]);
    const double lateral = sample.nx * (current.x - sample.x) + sample.ny * (current.y - sample.y);
    return std::max({
        0.0,
        lateral - sample.w_left,
        -lateral - sample.w_right,
        current.v - kVMax,
        kVMin - current.v,
        current.delta - kDeltaMax,
        kDeltaMin - current.delta,
    });
}

SolverConfig make_config() {
    SolverConfig config;
    config.print_level = PrintLevel::NONE;
    config.integrator = IntegratorType::RK4_EXPLICIT;
    config.barrier_strategy = BarrierStrategy::MEHROTRA;
    config.line_search_type = LineSearchType::FILTER;
    config.enable_soc = false;
    config.enable_feasibility_restoration = true;
    config.enable_slack_reset = true;
    config.max_iters = 60;
    config.tol_con = 1e-3;
    config.tol_dual = 1e-3;
    config.backend = Backend::CPU_SERIAL;
    return config;
}

struct Args {
    std::string asset_path;
    std::string output_path;
    int horizon = 40;
    int steps = 100;
    double dt = 0.05;
    double target_speed = 2.5;
};

Args parse_args(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        auto need = [&](const char* name) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error(std::string("missing value for ") + name);
            return argv[++i];
        };
        if (key == "--asset") args.asset_path = need("--asset");
        else if (key == "--output") args.output_path = need("--output");
        else if (key == "--horizon") args.horizon = std::stoi(need("--horizon"));
        else if (key == "--steps") args.steps = std::stoi(need("--steps"));
        else if (key == "--dt") args.dt = std::stod(need("--dt"));
        else if (key == "--target-speed") args.target_speed = std::stod(need("--target-speed"));
        else throw std::runtime_error("unknown argument: " + key);
    }
    if (args.asset_path.empty() || args.output_path.empty()) {
        throw std::runtime_error("required args: --asset --output");
    }
    return args;
}

void seed_solver(MiniSolver<KinematicBicycleTrackModel, kMaxHorizon>& solver, const nmpc_bench::DrivingTrackAsset& asset, double ref_s, double dt, double target_speed) {
    const int N = solver.get_horizon();
    for (int k = 0; k <= N; ++k) {
        const auto sample = nmpc_bench::sample_driving_track(asset, ref_s + static_cast<double>(k) * target_speed * dt);
        solver.set_parameter(k, 0, sample.x);
        solver.set_parameter(k, 1, sample.y);
        solver.set_parameter(k, 2, sample.psi);
        solver.set_parameter(k, 3, target_speed);
        solver.set_parameter(k, 4, sample.nx);
        solver.set_parameter(k, 5, sample.ny);
        solver.set_parameter(k, 6, sample.w_left);
        solver.set_parameter(k, 7, sample.w_right);
        solver.set_state_guess(k, 0, sample.x);
        solver.set_state_guess(k, 1, sample.y);
        solver.set_state_guess(k, 2, sample.psi);
        solver.set_state_guess(k, 3, target_speed);
        solver.set_state_guess(k, 4, 0.0);
        if (k < N) {
            solver.set_control_guess(k, 0, 0.0);
            solver.set_control_guess(k, 1, 0.0);
        }
    }
}

void update_parameters(MiniSolver<KinematicBicycleTrackModel, kMaxHorizon>& solver, const nmpc_bench::DrivingTrackAsset& asset, double ref_s, double dt, double target_speed) {
    const int N = solver.get_horizon();
    for (int k = 0; k <= N; ++k) {
        const auto sample = nmpc_bench::sample_driving_track(asset, ref_s + static_cast<double>(k) * target_speed * dt);
        solver.set_parameter(k, 0, sample.x);
        solver.set_parameter(k, 1, sample.y);
        solver.set_parameter(k, 2, sample.psi);
        solver.set_parameter(k, 3, target_speed);
        solver.set_parameter(k, 4, sample.nx);
        solver.set_parameter(k, 5, sample.ny);
        solver.set_parameter(k, 6, sample.w_left);
        solver.set_parameter(k, 7, sample.w_right);
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parse_args(argc, argv);
        const auto asset = nmpc_bench::load_driving_track(args.asset_path);
        const size_t stride_hint = std::max<size_t>(1, static_cast<size_t>(std::llround(args.target_speed * args.dt / std::max(asset.avg_ds, 1e-6))));

        MiniSolver<KinematicBicycleTrackModel, kMaxHorizon> solver(args.horizon, Backend::CPU_SERIAL, make_config());
        solver.set_dt(args.dt);
        DrivingState current{asset.x.front(), asset.y.front(), asset.psi.front(), args.target_speed, 0.0};
        size_t ref_idx = 0;

        std::ofstream out(args.output_path);
        out << "step,status,iterations,time_ms,x,y,psi,v,delta,tracking_error,max_constraint_violation,ref_index\n";

        seed_solver(solver, asset, asset.s[ref_idx], args.dt, args.target_speed);
        solver.set_initial_state({current.x, current.y, current.psi, current.v, current.delta});
        solver.rollout_dynamics();

        for (int step = 0; step < args.steps; ++step) {
            if (step > 0) {
                shift_primal_guess(solver);
                SolverConfig cfg = solver.get_config();
                cfg.initialization = InitializationMode::REUSE_PRIMAL;
                solver.set_config(cfg);
            } else {
                SolverConfig cfg = solver.get_config();
                cfg.initialization = InitializationMode::COLD_START;
                solver.set_config(cfg);
            }
            update_parameters(solver, asset, asset.s[ref_idx], args.dt, args.target_speed);
            solver.set_initial_state({current.x, current.y, current.psi, current.v, current.delta});
            solver.rollout_dynamics();
            solver.reset(ResetOption::ALG_STATE);
            const auto start = std::chrono::steady_clock::now();
            const SolverStatus status = solver.solve();
            const auto end = std::chrono::steady_clock::now();
            const double time_ms = std::chrono::duration<double, std::milli>(end - start).count();
            const double tracking_error = std::hypot(current.x - asset.x[ref_idx], current.y - asset.y[ref_idx]);
            const double max_viol = current_step_constraint_violation(current, asset, ref_idx);
            out << step << ',' << status_to_string(status) << ',' << solver.get_iteration_count() << ','
                << std::setprecision(17) << time_ms << ',' << current.x << ',' << current.y << ','
                << current.psi << ',' << current.v << ',' << current.delta << ',' << tracking_error << ','
                << max_viol << ',' << ref_idx << '\n';

            const auto u = solver.get_control(0);
            current = simulate(current, u[0], u[1], args.dt);
            ref_idx = nmpc_bench::nearest_cyclic_index(asset, current.x, current.y, ref_idx + stride_hint, 30 + 4 * stride_hint);
        }
        std::cout << "wrote " << args.output_path << "\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "kinematic_bicycle_benchmark error: " << exc.what() << "\n";
        return 1;
    }
}
