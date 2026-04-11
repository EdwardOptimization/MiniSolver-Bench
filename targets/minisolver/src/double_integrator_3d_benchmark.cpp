#include <array>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "asset_benchmark_common.h"
#include "doubleintegrator3dtrackingmodel.h"
#include "minisolver/solver/solver.h"

using namespace minisolver;

namespace {

constexpr int kMaxHorizon = 128;

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
    int horizon = 25;
    int steps = 500;
    double dt = 0.04;
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
        else throw std::runtime_error("unknown argument: " + key);
    }
    if (args.asset_path.empty() || args.output_path.empty()) {
        throw std::runtime_error("required args: --asset --output");
    }
    return args;
}

void seed_solver(MiniSolver<DoubleIntegrator3DTrackingModel, kMaxHorizon>& solver, const nmpc_bench::RobotReferenceAsset& asset, size_t ref_idx) {
    const int N = solver.get_horizon();
    for (int k = 0; k <= N; ++k) {
        const size_t idx = std::min(ref_idx + static_cast<size_t>(k), asset.x.size() - 1);
        solver.set_parameter(k, 0, asset.x[idx]);
        solver.set_parameter(k, 1, asset.y[idx]);
        solver.set_parameter(k, 2, asset.z[idx]);
        solver.set_parameter(k, 3, asset.vx[idx]);
        solver.set_parameter(k, 4, asset.vy[idx]);
        solver.set_parameter(k, 5, asset.vz[idx]);
        solver.set_state_guess(k, 0, asset.x[idx]);
        solver.set_state_guess(k, 1, asset.y[idx]);
        solver.set_state_guess(k, 2, asset.z[idx]);
        solver.set_state_guess(k, 3, asset.vx[idx]);
        solver.set_state_guess(k, 4, asset.vy[idx]);
        solver.set_state_guess(k, 5, asset.vz[idx]);
        if (k < N) {
            solver.set_control_guess(k, 0, 0.0);
            solver.set_control_guess(k, 1, 0.0);
            solver.set_control_guess(k, 2, 0.0);
        }
    }
}

void update_parameters(MiniSolver<DoubleIntegrator3DTrackingModel, kMaxHorizon>& solver, const nmpc_bench::RobotReferenceAsset& asset, size_t ref_idx) {
    const int N = solver.get_horizon();
    for (int k = 0; k <= N; ++k) {
        const size_t idx = std::min(ref_idx + static_cast<size_t>(k), asset.x.size() - 1);
        solver.set_parameter(k, 0, asset.x[idx]);
        solver.set_parameter(k, 1, asset.y[idx]);
        solver.set_parameter(k, 2, asset.z[idx]);
        solver.set_parameter(k, 3, asset.vx[idx]);
        solver.set_parameter(k, 4, asset.vy[idx]);
        solver.set_parameter(k, 5, asset.vz[idx]);
    }
}

void set_constant_control_guess(MiniSolver<DoubleIntegrator3DTrackingModel, kMaxHorizon>& solver,
                                const std::array<double, 3>& u) {
    const int N = solver.get_horizon();
    for (int k = 0; k < N; ++k) {
        solver.set_control_guess(k, 0, u[0]);
        solver.set_control_guess(k, 1, u[1]);
        solver.set_control_guess(k, 2, u[2]);
    }
}

double current_step_constraint_violation(const std::vector<double>& u0) {
    return std::max({
        0.0,
        u0[0] - 15.0, -15.0 - u0[0],
        u0[1] - 15.0, -15.0 - u0[1],
        u0[2] - 15.0, -15.0 - u0[2],
    });
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parse_args(argc, argv);
        const auto asset = nmpc_bench::load_robot_reference(args.asset_path);
        MiniSolver<DoubleIntegrator3DTrackingModel, kMaxHorizon> solver(args.horizon, Backend::CPU_SERIAL, make_config());
        solver.set_dt(args.dt);

        std::vector<double> current = {asset.x.front(), asset.y.front(), asset.z.front(), asset.vx.front(), asset.vy.front(), asset.vz.front()};
        std::array<double, 3> last_applied_u{0.0, 0.0, 0.0};
        bool prev_success = false;
        std::ofstream out(args.output_path);
        out << "step,status,iterations,time_ms,x,y,z,vx,vy,vz,tracking_error,max_constraint_violation\n";

        seed_solver(solver, asset, 0);
        solver.set_initial_state(current);
        solver.rollout_dynamics();

        const int steps = std::min<int>(args.steps, static_cast<int>(asset.x.size()));
        for (int step = 0; step < steps; ++step) {
            if (step > 0 && prev_success) {
                shift_primal_guess(solver);
                SolverConfig cfg = solver.get_config();
                cfg.initialization = InitializationMode::REUSE_PRIMAL;
                solver.set_config(cfg);
            } else {
                SolverConfig cfg = solver.get_config();
                cfg.initialization = InitializationMode::COLD_START;
                solver.set_config(cfg);
            }
            update_parameters(solver, asset, static_cast<size_t>(step));
            if (!(step > 0 && prev_success)) {
                set_constant_control_guess(solver, last_applied_u);
            }
            solver.set_initial_state(current);
            solver.rollout_dynamics();
            solver.reset(ResetOption::ALG_STATE);
            const auto start = std::chrono::steady_clock::now();
            const SolverStatus status = solver.solve();
            const auto end = std::chrono::steady_clock::now();
            const double time_ms = std::chrono::duration<double, std::milli>(end - start).count();

            const double dx = current[0] - asset.x[static_cast<size_t>(step)];
            const double dy = current[1] - asset.y[static_cast<size_t>(step)];
            const double dz = current[2] - asset.z[static_cast<size_t>(step)];
            const double tracking_error = std::sqrt(dx * dx + dy * dy + dz * dz);
            const auto u = solver.get_control(0);
            const double max_viol = current_step_constraint_violation(u);
            out << step << ',' << status_to_string(status) << ',' << solver.get_iteration_count() << ','
                << std::setprecision(17) << time_ms << ',' << current[0] << ',' << current[1] << ','
                << current[2] << ',' << current[3] << ',' << current[4] << ',' << current[5] << ','
                << tracking_error << ',' << max_viol << '\n';
            const bool solve_ok = (status == SolverStatus::OPTIMAL || status == SolverStatus::FEASIBLE);
            if (solve_ok) {
                last_applied_u = {u[0], u[1], u[2]};
            }
            current[0] += current[3] * args.dt + 0.5 * last_applied_u[0] * args.dt * args.dt;
            current[1] += current[4] * args.dt + 0.5 * last_applied_u[1] * args.dt * args.dt;
            current[2] += current[5] * args.dt + 0.5 * last_applied_u[2] * args.dt * args.dt;
            current[3] += last_applied_u[0] * args.dt;
            current[4] += last_applied_u[1] * args.dt;
            current[5] += last_applied_u[2] * args.dt;
            prev_success = solve_ok;
        }
        std::cout << "wrote " << args.output_path << "\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "double_integrator_3d_benchmark error: " << exc.what() << "\n";
        return 1;
    }
}
