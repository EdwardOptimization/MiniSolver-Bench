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
double max_positive_constraint(const MiniSolver<Model, MAX_N>& solver) {
    double result = 0.0;
    for (int k = 0; k <= solver.N; ++k) {
        for (int i = 0; i < Model::NC; ++i) {
            result = std::max(result, std::max(0.0, solver.get_constraint_val(k, i)));
        }
    }
    return result;
}

struct DrivingState {
    double x = 0.0;
    double y = 0.0;
    double psi = 0.0;
    double v = 0.0;
    double delta = 0.0;
};

DrivingState simulate(const DrivingState& state, double a, double delta_rate, double dt) {
    DrivingState next = state;
    next.x += state.v * std::cos(state.psi) * dt;
    next.y += state.v * std::sin(state.psi) * dt;
    next.psi = nmpc_bench::wrap_angle(state.psi + state.v * std::tan(state.delta) / kWheelbase * dt);
    next.v = std::clamp(state.v + a * dt, kVMin, kVMax);
    next.delta = std::clamp(state.delta + delta_rate * dt, kDeltaMin, kDeltaMax);
    return next;
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

void seed_solver(MiniSolver<KinematicBicycleTrackModel, kMaxHorizon>& solver, const nmpc_bench::DrivingTrackAsset& asset, size_t ref_idx, size_t stride, double target_speed) {
    for (int k = 0; k <= solver.N; ++k) {
        const size_t idx = (ref_idx + static_cast<size_t>(k) * stride) % asset.x.size();
        solver.set_parameter(k, 0, asset.x[idx]);
        solver.set_parameter(k, 1, asset.y[idx]);
        solver.set_parameter(k, 2, asset.psi[idx]);
        solver.set_parameter(k, 3, target_speed);
        solver.set_parameter(k, 4, asset.nx[idx]);
        solver.set_parameter(k, 5, asset.ny[idx]);
        solver.set_parameter(k, 6, asset.w_left[idx]);
        solver.set_parameter(k, 7, asset.w_right[idx]);
        solver.set_state_guess(k, 0, asset.x[idx]);
        solver.set_state_guess(k, 1, asset.y[idx]);
        solver.set_state_guess(k, 2, asset.psi[idx]);
        solver.set_state_guess(k, 3, target_speed);
        solver.set_state_guess(k, 4, 0.0);
        if (k < solver.N) {
            solver.set_control_guess(k, 0, 0.0);
            solver.set_control_guess(k, 1, 0.0);
        }
    }
}

void update_parameters(MiniSolver<KinematicBicycleTrackModel, kMaxHorizon>& solver, const nmpc_bench::DrivingTrackAsset& asset, size_t ref_idx, size_t stride, double target_speed) {
    for (int k = 0; k <= solver.N; ++k) {
        const size_t idx = (ref_idx + static_cast<size_t>(k) * stride) % asset.x.size();
        solver.set_parameter(k, 0, asset.x[idx]);
        solver.set_parameter(k, 1, asset.y[idx]);
        solver.set_parameter(k, 2, asset.psi[idx]);
        solver.set_parameter(k, 3, target_speed);
        solver.set_parameter(k, 4, asset.nx[idx]);
        solver.set_parameter(k, 5, asset.ny[idx]);
        solver.set_parameter(k, 6, asset.w_left[idx]);
        solver.set_parameter(k, 7, asset.w_right[idx]);
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parse_args(argc, argv);
        const auto asset = nmpc_bench::load_driving_track(args.asset_path);
        const size_t stride = std::max<size_t>(1, static_cast<size_t>(std::llround(args.target_speed * args.dt / std::max(asset.avg_ds, 1e-6))));

        MiniSolver<KinematicBicycleTrackModel, kMaxHorizon> solver(args.horizon, Backend::CPU_SERIAL, make_config());
        solver.set_dt(args.dt);
        DrivingState current{asset.x.front(), asset.y.front(), asset.psi.front(), args.target_speed, 0.0};
        size_t ref_idx = 0;

        std::ofstream out(args.output_path);
        out << "step,status,iterations,time_ms,x,y,psi,v,delta,tracking_error,max_constraint_violation,ref_index\n";

        seed_solver(solver, asset, ref_idx, stride, args.target_speed);
        solver.set_initial_state({current.x, current.y, current.psi, current.v, current.delta});
        solver.rollout_dynamics();

        for (int step = 0; step < args.steps; ++step) {
            if (step > 0) {
                solver.shift_trajectory();
                solver.is_warm_started = true;
            }
            update_parameters(solver, asset, ref_idx, stride, args.target_speed);
            solver.set_initial_state({current.x, current.y, current.psi, current.v, current.delta});
            const auto start = std::chrono::steady_clock::now();
            const SolverStatus status = solver.solve();
            const auto end = std::chrono::steady_clock::now();
            const double time_ms = std::chrono::duration<double, std::milli>(end - start).count();
            const double tracking_error = std::hypot(current.x - asset.x[ref_idx], current.y - asset.y[ref_idx]);
            const double max_viol = max_positive_constraint(solver);
            out << step << ',' << status_to_string(status) << ',' << solver.current_iter << ','
                << std::setprecision(17) << time_ms << ',' << current.x << ',' << current.y << ','
                << current.psi << ',' << current.v << ',' << current.delta << ',' << tracking_error << ','
                << max_viol << ',' << ref_idx << '\n';

            const auto u = solver.get_control(0);
            current = simulate(current, u[0], u[1], args.dt);
            ref_idx = nmpc_bench::nearest_cyclic_index(asset, current.x, current.y, ref_idx + stride, 30 + 4 * stride);
        }
        std::cout << "wrote " << args.output_path << "\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "kinematic_bicycle_benchmark error: " << exc.what() << "\n";
        return 1;
    }
}
