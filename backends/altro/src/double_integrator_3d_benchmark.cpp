#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "altro/altro.hpp"
#include "altro/solver/solver.hpp"
#include "asset_benchmark_common.h"

namespace {

constexpr int kNx = 6;
constexpr int kNu = 3;
constexpr double kAccelLimit = 15.0;
constexpr std::array<double, kNx> kQStage{30.0, 30.0, 30.0, 5.0, 5.0, 5.0};
constexpr std::array<double, kNu> kRStage{0.1, 0.1, 0.1};

using State = std::array<double, kNx>;
using Control = std::array<double, kNu>;

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
            if (i + 1 >= argc) {
                throw std::runtime_error(std::string("missing value for ") + name);
            }
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

State reference_state(const nmpc_bench::RobotReferenceAsset& asset, size_t idx) {
    idx = std::min(idx, asset.x.size() - 1);
    return {asset.x[idx], asset.y[idx], asset.z[idx], asset.vx[idx], asset.vy[idx], asset.vz[idx]};
}

void double_integrator_dynamics(double* xnext, const double* x, const double* u, float h) {
    const double dt = static_cast<double>(h);
    const double half_dt2 = 0.5 * dt * dt;
    xnext[0] = x[0] + dt * x[3] + half_dt2 * u[0];
    xnext[1] = x[1] + dt * x[4] + half_dt2 * u[1];
    xnext[2] = x[2] + dt * x[5] + half_dt2 * u[2];
    xnext[3] = x[3] + dt * u[0];
    xnext[4] = x[4] + dt * u[1];
    xnext[5] = x[5] + dt * u[2];
}

void double_integrator_jacobian(double* jac, const double* x, const double* u, float h) {
    (void)x;
    (void)u;
    const double dt = static_cast<double>(h);
    const double half_dt2 = 0.5 * dt * dt;
    Eigen::Map<Eigen::Matrix<double, kNx, kNx + kNu, Eigen::ColMajor>> J(jac);
    J.setZero();
    for (int i = 0; i < 3; ++i) {
        J(i, i) = 1.0;
        J(i + 3, i + 3) = 1.0;
        J(i, i + 3) = dt;
        J(i, kNx + i) = half_dt2;
        J(i + 3, kNx + i) = dt;
    }
}

void accel_bound_constraint(double* c, const double* x, const double* u) {
    (void)x;
    c[0] = u[0] - kAccelLimit;
    c[1] = -kAccelLimit - u[0];
    c[2] = u[1] - kAccelLimit;
    c[3] = -kAccelLimit - u[1];
    c[4] = u[2] - kAccelLimit;
    c[5] = -kAccelLimit - u[2];
}

void accel_bound_jacobian(double* jac, const double* x, const double* u) {
    (void)x;
    (void)u;
    Eigen::Map<Eigen::Matrix<double, 6, kNx + kNu, Eigen::ColMajor>> J(jac);
    J.setZero();
    J(0, kNx + 0) = 1.0;
    J(1, kNx + 0) = -1.0;
    J(2, kNx + 1) = 1.0;
    J(3, kNx + 1) = -1.0;
    J(4, kNx + 2) = 1.0;
    J(5, kNx + 2) = -1.0;
}

const char* status_to_string(altro::SolverStatus status) {
    switch (status) {
        case altro::SolverStatus::kSolved:
            return "Solved";
        case altro::SolverStatus::kUnsolved:
            return "Unsolved";
        case altro::SolverStatus::kStateLimit:
            return "StateLimit";
        case altro::SolverStatus::kControlLimit:
            return "ControlLimit";
        case altro::SolverStatus::kCostIncrease:
            return "CostIncrease";
        case altro::SolverStatus::kMaxIterations:
            return "MaxIterations";
        case altro::SolverStatus::kMaxOuterIterations:
            return "MaxOuterIterations";
        case altro::SolverStatus::kMaxInnerIterations:
            return "MaxInnerIterations";
        case altro::SolverStatus::kMaxPenalty:
            return "MaxPenalty";
        case altro::SolverStatus::kBackwardPassRegularizationFailed:
            return "BackwardPassRegularizationFailed";
        default:
            return "Unknown";
    }
}

bool is_success(altro::SolverStatus status) {
    return status == altro::SolverStatus::kSolved;
}

void configure_problem(altro::ALTROSolver& solver,
                       const nmpc_bench::RobotReferenceAsset& asset,
                       int horizon,
                       double dt,
                       size_t ref_idx,
                       const State& current) {
    solver.SetDimension(kNx, kNu, 0, altro::LastIndex);
    solver.SetTimeStep(static_cast<float>(dt), 0, altro::LastIndex);
    solver.SetExplicitDynamics(double_integrator_dynamics, double_integrator_jacobian, 0, horizon);
    solver.SetConstraint(
        accel_bound_constraint,
        accel_bound_jacobian,
        6,
        altro::ConstraintType::INEQUALITY,
        "acceleration_box",
        0,
        horizon,
        nullptr);

    const Control zero_u{0.0, 0.0, 0.0};
    for (int k = 0; k < horizon; ++k) {
        const State ref = reference_state(asset, ref_idx + static_cast<size_t>(k));
        solver.SetLQRCost(kQStage.data(), kRStage.data(), ref.data(), zero_u.data(), k, k + 1);
    }
    const State terminal_ref = reference_state(asset, ref_idx + static_cast<size_t>(horizon));
    solver.SetLQRCost(kQStage.data(), kRStage.data(), terminal_ref.data(), zero_u.data(), horizon, horizon + 1);
    solver.SetInitialState(current.data(), kNx);
}

void configure_options(altro::ALTROSolver& solver) {
    altro::AltroOptions opts;
    opts.tol_cost = 1e-4;
    opts.tol_cost_intermediate = 1e-4;
    opts.tol_stationarity = 1e-10;
    opts.tol_primal_feasibility = 1e-4;
    opts.penalty_initial = 1.0;
    opts.penalty_scaling = 10.0;
    opts.penalty_max = 1e8;
    opts.max_state_value = 1e8;
    opts.max_input_value = 1e8;
    opts.throw_errors = false;
    opts.verbose = altro::Verbosity::Silent;
    solver.SetOptions(opts);

    solver.solver_->alsolver_.GetOptions().constraint_tolerance = 1e-4;
    solver.solver_->alsolver_.GetOptions().max_iterations_total = 300;
    solver.solver_->alsolver_.GetOptions().max_iterations_outer = 30;
    solver.solver_->alsolver_.GetOptions().max_iterations_inner = 100;
    solver.solver_->alsolver_.GetOptions().state_max = 1e8;
    solver.solver_->alsolver_.GetOptions().control_max = 1e8;
}

State propagate(const State& x, const Control& u, double dt) {
    State next = x;
    next[0] += x[3] * dt + 0.5 * u[0] * dt * dt;
    next[1] += x[4] * dt + 0.5 * u[1] * dt * dt;
    next[2] += x[5] * dt + 0.5 * u[2] * dt * dt;
    next[3] += u[0] * dt;
    next[4] += u[1] * dt;
    next[5] += u[2] * dt;
    return next;
}

void rollout_guess(altro::ALTROSolver& solver,
                   int horizon,
                   double dt,
                   const State& x0,
                   const Control& u_seed) {
    State x = x0;
    for (int k = 0; k <= horizon; ++k) {
        solver.SetState(x.data(), kNx, k, k + 1);
        if (k < horizon) {
            solver.SetInput(u_seed.data(), kNu, k, k + 1);
            x = propagate(x, u_seed, dt);
        }
    }
}

void seed_shifted_previous_solution(altro::ALTROSolver& solver,
                                    const std::vector<State>& previous_states,
                                    const std::vector<Control>& previous_controls,
                                    int horizon) {
    for (int k = 0; k <= horizon; ++k) {
        const int src_k = std::min(k + 1, horizon);
        solver.SetState(previous_states[static_cast<size_t>(src_k)].data(), kNx, k, k + 1);
    }
    for (int k = 0; k < horizon; ++k) {
        const int src_k = std::min(k + 1, horizon - 1);
        solver.SetInput(previous_controls[static_cast<size_t>(src_k)].data(), kNu, k, k + 1);
    }
}

void copy_solution(const altro::ALTROSolver& solver,
                   std::vector<State>& states,
                   std::vector<Control>& controls,
                   int horizon) {
    states.resize(static_cast<size_t>(horizon + 1));
    controls.resize(static_cast<size_t>(horizon));
    for (int k = 0; k <= horizon; ++k) {
        const auto& x = solver.solver_->trajectory_->State(k);
        for (int i = 0; i < kNx; ++i) {
            states[static_cast<size_t>(k)][static_cast<size_t>(i)] = x(i);
        }
    }
    for (int k = 0; k < horizon; ++k) {
        const auto& u = solver.solver_->trajectory_->Control(k);
        for (int i = 0; i < kNu; ++i) {
            controls[static_cast<size_t>(k)][static_cast<size_t>(i)] = u(i);
        }
    }
}

Control first_input(const altro::ALTROSolver& solver) {
    const auto& u = solver.solver_->trajectory_->Control(0);
    return {u(0), u(1), u(2)};
}

double current_step_constraint_violation(const Control& u0) {
    return std::max({
        0.0,
        u0[0] - kAccelLimit, -kAccelLimit - u0[0],
        u0[1] - kAccelLimit, -kAccelLimit - u0[1],
        u0[2] - kAccelLimit, -kAccelLimit - u0[2],
    });
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parse_args(argc, argv);
        const auto asset = nmpc_bench::load_robot_reference(args.asset_path);
        const int steps = std::min<int>(args.steps, static_cast<int>(asset.x.size()));

        State current{
            asset.x.front(), asset.y.front(), asset.z.front(),
            asset.vx.front(), asset.vy.front(), asset.vz.front()};
        Control last_applied_u{0.0, 0.0, 0.0};
        std::vector<State> previous_states;
        std::vector<Control> previous_controls;
        bool prev_success = false;

        std::ofstream out(args.output_path);
        out << "step,status,iterations,time_ms,x,y,z,vx,vy,vz,u0,u1,u2,tracking_error,max_constraint_violation,primal_feasibility,objective\n";

        int success_count = 0;
        for (int step = 0; step < steps; ++step) {
            altro::ALTROSolver solver(args.horizon);
            configure_problem(solver, asset, args.horizon, args.dt, static_cast<size_t>(step), current);
            solver.Initialize();
            configure_options(solver);

            if (prev_success) {
                seed_shifted_previous_solution(solver, previous_states, previous_controls, args.horizon);
                solver.SetState(current.data(), kNx, 0, 1);
            } else {
                rollout_guess(solver, args.horizon, args.dt, current, last_applied_u);
            }

            const auto start = std::chrono::steady_clock::now();
            (void)solver.Solve();
            const auto end = std::chrono::steady_clock::now();
            const double time_ms = std::chrono::duration<double, std::milli>(end - start).count();
            const altro::SolverStatus status = solver.solver_->alsolver_.GetStatus();
            const auto& stats = solver.solver_->alsolver_.GetStats();

            const bool ok = is_success(status);
            if (ok) {
                ++success_count;
                last_applied_u = first_input(solver);
                copy_solution(solver, previous_states, previous_controls, args.horizon);
            }

            const double dx = current[0] - asset.x[static_cast<size_t>(step)];
            const double dy = current[1] - asset.y[static_cast<size_t>(step)];
            const double dz = current[2] - asset.z[static_cast<size_t>(step)];
            const double tracking_error = std::sqrt(dx * dx + dy * dy + dz * dz);
            const double max_viol = current_step_constraint_violation(last_applied_u);
            const double primal_feasibility = stats.violations.empty() ? 0.0 : stats.violations.back();

            out << step << ',' << status_to_string(status) << ',' << stats.iterations_total << ','
                << std::setprecision(17) << time_ms << ',' << current[0] << ',' << current[1] << ','
                << current[2] << ',' << current[3] << ',' << current[4] << ',' << current[5] << ','
                << last_applied_u[0] << ',' << last_applied_u[1] << ',' << last_applied_u[2] << ','
                << tracking_error << ',' << max_viol << ',' << primal_feasibility << ','
                << solver.CalcCost() << '\n';

            current = propagate(current, last_applied_u, args.dt);
            prev_success = ok;
        }

        std::cout << "steps=" << steps << " success=" << success_count << " output=" << args.output_path << "\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "altro double_integrator_3d_benchmark error: " << exc.what() << "\n";
        return 1;
    }
}
