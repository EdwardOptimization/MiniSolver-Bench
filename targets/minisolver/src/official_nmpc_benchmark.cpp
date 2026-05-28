#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "minisolver/solver/solver.h"
#include "official_case_data.h"

#include "pendulummodel.h"

#if defined(MINISOLVER_CASE_RACE_CARS) || defined(MINISOLVER_CASE_QUADROTOR_NAV)
#include "official_track_geometry.h"
#endif

#if defined(MINISOLVER_CASE_RACE_CARS)
#include "racecarsmodel.h"
#endif

#if defined(MINISOLVER_CASE_QUADROTOR_NAV)
#include "quadrotornavmodel.h"
#endif

using namespace minisolver;

namespace minisolver_bench {

using generated::chain_mass_n_mass;
using generated::chain_mass_steps;
using generated::chain_mass_tf;
using generated::chain_mass_ts;
using generated::chain_mass_u_init;
using generated::chain_mass_wall_count;
using generated::chain_mass_xrest;
using generated::chain_mass_y_pos_wall;
using generated::pendulum_horizon;
using generated::pendulum_steps;
using generated::pendulum_tf;
using generated::pendulum_x0;
using generated::quadrotor_nav_horizon;
using generated::quadrotor_nav_init_zeta;
using generated::quadrotor_nav_s_max;
using generated::quadrotor_nav_s_ref;
using generated::quadrotor_nav_steps;
using generated::quadrotor_nav_tf;
using generated::quadrotor_nav_u_hov;
using generated::race_cars_horizon;
using generated::race_cars_pathlength;
using generated::race_cars_sref_n;
using generated::race_cars_steps;
using generated::race_cars_tf;
using generated::race_cars_x0;

namespace {

template <size_t N>
constexpr std::array<const char*, N> filled_names(const char* value) {
    std::array<const char*, N> result{};
    for (size_t i = 0; i < N; ++i) {
        result[i] = value;
    }
    return result;
}

template <int N>
const double* raw_ptr(const MSVec<double, N>& vec) {
    return &vec(0);
}

template <int R, int C>
void set_zero_matrix(MSMat<double, R, C>& mat) {
    MatOps::setZero(mat);
}

template <int N>
void set_zero_vector(MSVec<double, N>& vec) {
    MatOps::setZero(vec);
}

template <int N>
MSVec<double, N> to_msvec(const std::vector<double>& values) {
    MSVec<double, N> vec;
    for (int i = 0; i < N; ++i) {
        vec(i) = values.at(static_cast<size_t>(i));
    }
    return vec;
}

template <size_t N>
MSVec<double, static_cast<int>(N)> to_msvec(const std::array<double, N>& values) {
    MSVec<double, static_cast<int>(N)> vec;
    for (size_t i = 0; i < N; ++i) {
        vec(i) = values[static_cast<size_t>(i)];
    }
    return vec;
}

template <int N>
std::array<double, N> to_array(const std::vector<double>& values) {
    std::array<double, N> result{};
    for (int i = 0; i < N; ++i) {
        result[static_cast<size_t>(i)] = values.at(static_cast<size_t>(i));
    }
    return result;
}

template <int N>
std::array<double, N> to_array(const MSVec<double, N>& values) {
    std::array<double, N> result{};
    for (int i = 0; i < N; ++i) {
        result[static_cast<size_t>(i)] = values(i);
    }
    return result;
}

#if defined(MINISOLVER_CASE_PENDULUM_ON_CART)
template <size_t N>
void set_stage_array_parameters(
    MiniSolver<PendulumModel, 80>& /*solver*/,
    int /*stage*/,
    int /*offset*/,
    const std::array<double, N>& /*values*/) {}
#endif

template <typename SolverType, size_t N>
void set_stage_array_parameters(SolverType& solver, int stage, int offset, const std::array<double, N>& values) {
    for (size_t i = 0; i < N; ++i) {
        solver.set_parameter(stage, offset + static_cast<int>(i), values[i]);
    }
}

struct Args {
    int steps = -1;
    int warmup_runs = 5;
    std::string output_csv;
};

constexpr const char* kBackendTag = "minisolver";

double percentile(std::vector<double> values, double fraction) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const double idx = fraction * static_cast<double>(values.size() - 1);
    const auto lo = static_cast<size_t>(std::floor(idx));
    const auto hi = static_cast<size_t>(std::ceil(idx));
    if (lo == hi) {
        return values[lo];
    }
    const double weight = idx - static_cast<double>(lo);
    return values[lo] * (1.0 - weight) + values[hi] * weight;
}

bool success_status(SolverStatus status) {
    return status == SolverStatus::OPTIMAL || status == SolverStatus::FEASIBLE;
}

template <typename Model, int MAX_N>
double total_cost(const MiniSolver<Model, MAX_N>& solver) {
    double result = 0.0;
    const int N = solver.get_horizon();
    for (int k = 0; k <= N; ++k) {
        result += solver.get_stage_cost(k);
    }
    return result;
}

template <typename Model, int MAX_N>
double max_positive_constraint(const MiniSolver<Model, MAX_N>& solver) {
    double result = 0.0;
    const int N = solver.get_horizon();
    for (int k = 0; k <= N; ++k) {
        for (int i = 0; i < Model::NC; ++i) {
            result = std::max(result, std::max(0.0, solver.get_constraint_val(k, i)));
        }
    }
    return result;
}

template <typename Model, int MAX_N>
void shift_primal_guess(MiniSolver<Model, MAX_N>& solver) {
    const int N = solver.get_horizon();
    if (N <= 0) {
        return;
    }

    for (int k = 0; k < N; ++k) {
        for (int i = 0; i < Model::NX; ++i) {
            solver.set_state_guess(k, i, solver.get_state(k + 1, i));
        }
    }
    for (int k = 0; k < N - 1; ++k) {
        for (int i = 0; i < Model::NU; ++i) {
            solver.set_control_guess(k, i, solver.get_control(k + 1, i));
        }
    }
    for (int i = 0; i < Model::NU; ++i) {
        solver.set_control_guess(N - 1, i, solver.get_control(N - 1, i));
    }
}

SolverConfig make_solver_config() {
    SolverConfig config;
    config.print_level = PrintLevel::NONE;
    config.integrator = IntegratorType::RK4_EXPLICIT;
    config.barrier_strategy = BarrierStrategy::MEHROTRA;
    config.line_search_type = LineSearchType::FILTER;
    config.max_iters = 40;
    config.tol_con = 1e-4;
    config.tol_dual = 1e-4;
    config.backend = Backend::CPU_SERIAL;
    config.enable_soc = false;
    config.enable_feasibility_restoration = false;
    config.enable_slack_reset = false;
    return config;
}

const char* env_value(const char* name) {
    const char* value = std::getenv(name);
    return value && value[0] != '\0' ? value : nullptr;
}

void apply_env_int(const char* name, int& target) {
    if (const char* value = env_value(name)) {
        char* end = nullptr;
        const long parsed = std::strtol(value, &end, 10);
        if (end != value) {
            target = static_cast<int>(parsed);
        }
    }
}

void apply_env_double(const char* name, double& target) {
    if (const char* value = env_value(name)) {
        char* end = nullptr;
        const double parsed = std::strtod(value, &end);
        if (end != value) {
            target = parsed;
        }
    }
}

void apply_env_bool(const char* name, bool& target) {
    if (const char* value = env_value(name)) {
        const std::string text(value);
        target = !(text == "0" || text == "false" || text == "FALSE" || text == "off" || text == "OFF");
    }
}

template <typename... Ts>
void print_csv_line(std::ofstream& out, const Ts&... values) {
    bool first = true;
    auto write_one = [&](const auto& value) {
        if (!first) {
            out << ',';
        }
        first = false;
        out << value;
    };
    (write_one(values), ...);
    out << '\n';
}

template <int NX, int NU, int NP, typename EvalFn>
MSVec<double, NX> rk4_step(
    const MSVec<double, NX>& x,
    const MSVec<double, NU>& u,
    const MSVec<double, NP>& p,
    double dt,
    EvalFn&& eval,
    MSMat<double, NX, NX>* A,
    MSMat<double, NX, NU>* B) {
    MSVec<double, NX> k1;
    MSVec<double, NX> k2;
    MSVec<double, NX> k3;
    MSVec<double, NX> k4;
    MSMat<double, NX, NX> fx1;
    MSMat<double, NX, NX> fx2;
    MSMat<double, NX, NX> fx3;
    MSMat<double, NX, NX> fx4;
    MSMat<double, NX, NU> fu1;
    MSMat<double, NX, NU> fu2;
    MSMat<double, NX, NU> fu3;
    MSMat<double, NX, NU> fu4;
    MSMat<double, NX, NX> I;
    MatOps::setIdentity(I);

    eval(x, u, p, k1, fx1, fu1);
    const MSVec<double, NX> x2 = x + k1 * (0.5 * dt);
    const MSMat<double, NX, NX> dx2_dx = I + fx1 * (0.5 * dt);
    const MSMat<double, NX, NU> dx2_du = fu1 * (0.5 * dt);

    eval(x2, u, p, k2, fx2, fu2);
    const MSMat<double, NX, NX> dk2_dx = fx2 * dx2_dx;
    const MSMat<double, NX, NU> dk2_du = fx2 * dx2_du + fu2;

    const MSVec<double, NX> x3 = x + k2 * (0.5 * dt);
    const MSMat<double, NX, NX> dx3_dx = I + dk2_dx * (0.5 * dt);
    const MSMat<double, NX, NU> dx3_du = dk2_du * (0.5 * dt);

    eval(x3, u, p, k3, fx3, fu3);
    const MSMat<double, NX, NX> dk3_dx = fx3 * dx3_dx;
    const MSMat<double, NX, NU> dk3_du = fx3 * dx3_du + fu3;

    const MSVec<double, NX> x4 = x + k3 * dt;
    const MSMat<double, NX, NX> dx4_dx = I + dk3_dx * dt;
    const MSMat<double, NX, NU> dx4_du = dk3_du * dt;

    eval(x4, u, p, k4, fx4, fu4);
    const MSMat<double, NX, NX> dk4_dx = fx4 * dx4_dx;
    const MSMat<double, NX, NU> dk4_du = fx4 * dx4_du + fu4;

    if (A != nullptr) {
        *A = I + (fx1 + dk2_dx * 2.0 + dk3_dx * 2.0 + dk4_dx) * (dt / 6.0);
    }
    if (B != nullptr) {
        *B = (fu1 + dk2_du * 2.0 + dk3_du * 2.0 + dk4_du) * (dt / 6.0);
    }

    return x + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);
}

// RaceCarsModel is generated via SymPy (see targets/minisolver/generated/racecarsmodel.h).

#if defined(MINISOLVER_CASE_QUADROTOR_NAV)
// QuadrotorNavModel is generated via SymPy (see targets/minisolver/generated/quadrotornavmodel.h).
#endif

#if defined(MINISOLVER_CASE_CHAIN_MASS)
struct ChainMassModel {
    static constexpr int M = chain_mass_n_mass - 2;
    static constexpr int POS_DIM = (M + 1) * 3;
    static constexpr int VEL_DIM = M * 3;
    static constexpr int DIST_DIM = M * 3;
    static constexpr int NX = (2 * M + 1) * 3;
    static constexpr int NU = 3;
    static constexpr int NC = 10;
    static constexpr int NP = DIST_DIM + NX + NX + NU;

    static constexpr std::array<const char*, NX> state_names = filled_names<NX>("x");
    static constexpr std::array<const char*, NU> control_names = {"ux", "uy", "uz"};
    static constexpr std::array<const char*, NP> param_names = filled_names<NP>("param");
    static constexpr std::array<double, NC> constraint_weights = {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        1e3, 1e3, 1e3, 1e3,
    };
    static constexpr std::array<int, NC> constraint_types = {
        0, 0, 0, 0, 0, 0,
        2, 2, 2, 2,
    };

    static constexpr double mass = 0.033;
    static constexpr double spring = 1.0;
    static constexpr double rest_length = 0.033;
    static constexpr int P_DIST = 0;
    static constexpr int P_XREF = P_DIST + DIST_DIM;
    static constexpr int P_WX = P_XREF + NX;
    static constexpr int P_WU = P_WX + NX;

    template <typename T>
    static void continuous_eval(
        const MSVec<T, NX>& x,
        const MSVec<T, NU>& u,
        const MSVec<T, NP>& p,
        MSVec<T, NX>& f,
        MSMat<T, NX, NX>& fx,
        MSMat<T, NX, NU>& fu) {
        static_assert(std::is_same_v<T, double>, "ChainMassModel only supports double evaluation");
        set_zero_vector(f);
        set_zero_matrix(fx);
        set_zero_matrix(fu);

        for (int i = 0; i < VEL_DIM; ++i) {
            f(i) = x(POS_DIM + i);
            fx(i, POS_DIM + i) = 1.0;
        }
        for (int i = 0; i < NU; ++i) {
            f(VEL_DIM + i) = u(i);
            fu(VEL_DIM + i, i) = 1.0;
        }
        for (int i = 0; i < M; ++i) {
            f(POS_DIM + 3 * i + 2) = -9.81 + p(P_DIST + 3 * i + 2);
            f(POS_DIM + 3 * i + 0) = p(P_DIST + 3 * i + 0);
            f(POS_DIM + 3 * i + 1) = p(P_DIST + 3 * i + 1);
        }

        const double k = spring / mass;
        for (int seg = 0; seg < M + 1; ++seg) {
            std::array<double, 3> dist{};
            const int right_pos = 3 * seg;
            if (seg == 0) {
                for (int a = 0; a < 3; ++a) {
                    dist[static_cast<size_t>(a)] = x(right_pos + a);
                }
            } else {
                const int left_pos = 3 * (seg - 1);
                for (int a = 0; a < 3; ++a) {
                    dist[static_cast<size_t>(a)] = x(right_pos + a) - x(left_pos + a);
                }
            }

            double norm_sq = 0.0;
            for (double value : dist) {
                norm_sq += value * value;
            }
            const double norm = std::sqrt(std::max(norm_sq, 1e-12));
            const double scale = k * (1.0 - rest_length / norm);
            std::array<double, 3> force{};
            for (int a = 0; a < 3; ++a) {
                force[static_cast<size_t>(a)] = scale * dist[static_cast<size_t>(a)];
            }

            MSMat<double, 3, 3> jac_force;
            MatOps::setIdentity(jac_force);
            jac_force *= scale;
            const double coeff = k * rest_length / (norm * norm * norm);
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    jac_force(r, c) += coeff * dist[static_cast<size_t>(r)] * dist[static_cast<size_t>(c)];
                }
            }

            if (seg < M) {
                const int acc_row = POS_DIM + 3 * seg;
                for (int a = 0; a < 3; ++a) {
                    f(acc_row + a) -= force[static_cast<size_t>(a)];
                }
                for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c) {
                        fx(acc_row + r, right_pos + c) -= jac_force(r, c);
                        if (seg > 0) {
                            fx(acc_row + r, 3 * (seg - 1) + c) += jac_force(r, c);
                        }
                    }
                }
            }
            if (seg > 0) {
                const int acc_row = POS_DIM + 3 * (seg - 1);
                for (int a = 0; a < 3; ++a) {
                    f(acc_row + a) += force[static_cast<size_t>(a)];
                }
                for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c) {
                        fx(acc_row + r, right_pos + c) += jac_force(r, c);
                        fx(acc_row + r, 3 * (seg - 1) + c) -= jac_force(r, c);
                    }
                }
            }
        }
    }

    template <typename T>
    static MSVec<T, NX> dynamics_continuous(
        const MSVec<T, NX>& x,
        const MSVec<T, NU>& u,
        const MSVec<T, NP>& p) {
        static_assert(std::is_same_v<T, double>, "ChainMassModel only supports double evaluation");
        MSVec<double, NX> f;
        MSMat<double, NX, NX> fx;
        MSMat<double, NX, NU> fu;
        continuous_eval(x, u, p, f, fx, fu);
        return f;
    }

    template <typename T>
    static MSVec<T, NX> integrate(
        const MSVec<T, NX>& x,
        const MSVec<T, NU>& u,
        const MSVec<T, NP>& p,
        double dt,
        IntegratorType /*type*/) {
        static_assert(std::is_same_v<T, double>, "ChainMassModel only supports double evaluation");
        auto eval = [](const auto& xv, const auto& uv, const auto& pv, auto& f, auto& fx, auto& fu) {
            continuous_eval(xv, uv, pv, f, fx, fu);
        };
        return rk4_step<NX, NU, NP>(x, u, p, dt, eval, nullptr, nullptr);
    }

    template <typename T>
    static void compute_dynamics(KnotPoint<T, NX, NU, NC, NP>& kp, IntegratorType /*type*/, double dt) {
        static_assert(std::is_same_v<T, double>, "ChainMassModel only supports double evaluation");
        auto eval = [](const auto& xv, const auto& uv, const auto& pv, auto& f, auto& fx, auto& fu) {
            continuous_eval(xv, uv, pv, f, fx, fu);
        };
        kp.f_resid = rk4_step<NX, NU, NP>(kp.x, kp.u, kp.p, dt, eval, &kp.A, &kp.B);
    }

    template <typename T>
    static void compute_constraints(KnotPoint<T, NX, NU, NC, NP>& kp) {
        kp.g_val(0) = kp.u(0) - 1.0;
        kp.g_val(1) = -1.0 - kp.u(0);
        kp.g_val(2) = kp.u(1) - 1.0;
        kp.g_val(3) = -1.0 - kp.u(1);
        kp.g_val(4) = kp.u(2) - 1.0;
        kp.g_val(5) = -1.0 - kp.u(2);
        kp.g_val(6) = chain_mass_y_pos_wall - kp.x(1);
        kp.g_val(7) = chain_mass_y_pos_wall - kp.x(4);
        kp.g_val(8) = chain_mass_y_pos_wall - kp.x(7);
        kp.g_val(9) = chain_mass_y_pos_wall - kp.x(10);

        set_zero_matrix(kp.C);
        set_zero_matrix(kp.D);
        kp.D(0, 0) = 1.0;
        kp.D(1, 0) = -1.0;
        kp.D(2, 1) = 1.0;
        kp.D(3, 1) = -1.0;
        kp.D(4, 2) = 1.0;
        kp.D(5, 2) = -1.0;
        kp.C(6, 1) = -1.0;
        kp.C(7, 4) = -1.0;
        kp.C(8, 7) = -1.0;
        kp.C(9, 10) = -1.0;
    }

    template <typename T>
    static void compute_cost_exact(KnotPoint<T, NX, NU, NC, NP>& kp) {
        kp.cost = 0.0;
        set_zero_vector(kp.q);
        set_zero_vector(kp.r);
        set_zero_matrix(kp.Q);
        set_zero_matrix(kp.R);
        set_zero_matrix(kp.H);

        for (int i = 0; i < NX; ++i) {
            const double ref = kp.p(P_XREF + i);
            const double weight = kp.p(P_WX + i);
            const double diff = kp.x(i) - ref;
            kp.cost += 0.5 * weight * diff * diff;
            kp.q(i) = weight * diff;
            kp.Q(i, i) = weight;
        }
        for (int i = 0; i < NU; ++i) {
            const double weight = kp.p(P_WU + i);
            kp.cost += 0.5 * weight * kp.u(i) * kp.u(i);
            kp.r(i) = weight * kp.u(i);
            kp.R(i, i) = weight;
        }
    }

    template <typename T>
    static void compute_cost_gn(KnotPoint<T, NX, NU, NC, NP>& kp) {
        compute_cost_exact(kp);
    }

    template <typename T>
    static void compute_cost(KnotPoint<T, NX, NU, NC, NP>& kp) {
        compute_cost_exact(kp);
    }

    template <typename T>
    static void compute(KnotPoint<T, NX, NU, NC, NP>& kp, IntegratorType type, double dt) {
        compute_cost(kp);
        compute_dynamics(kp, type, dt);
        compute_constraints(kp);
    }
};
#endif

struct PendulumStep {
    int step = 0;
    SolverStatus status = SolverStatus::UNSOLVED;
    int iterations = 0;
    double time_ms = 0.0;
    double deriv_ms = 0.0;
    double riccati_ms = 0.0;
    double line_search_ms = 0.0;
    double force = 0.0;
    double total_cost_value = 0.0;
    double max_constraint_violation_value = 0.0;
    std::array<double, 4> state{};
};

struct RaceStep {
    int step = 0;
    SolverStatus status = SolverStatus::UNSOLVED;
    int iterations = 0;
    double time_ms = 0.0;
    double deriv_ms = 0.0;
    double riccati_ms = 0.0;
    double line_search_ms = 0.0;
    double total_cost_value = 0.0;
    double max_constraint_violation_value = 0.0;
    double s = 0.0;
    double n = 0.0;
    double alpha = 0.0;
    double v = 0.0;
    double D = 0.0;
    double delta = 0.0;
    double derD = 0.0;
    double derDelta = 0.0;
};

struct QuadStep {
    int step = 0;
    SolverStatus status = SolverStatus::UNSOLVED;
    int iterations = 0;
    double time_ms = 0.0;
    double deriv_ms = 0.0;
    double riccati_ms = 0.0;
    double line_search_ms = 0.0;
    double total_cost_value = 0.0;
    double max_constraint_violation_value = 0.0;
    double abs_n = 0.0;
    double abs_b = 0.0;
};

struct ChainStep {
    int step = 0;
    SolverStatus status = SolverStatus::UNSOLVED;
    int iterations = 0;
    double time_ms = 0.0;
    double deriv_ms = 0.0;
    double riccati_ms = 0.0;
    double line_search_ms = 0.0;
    double total_cost_value = 0.0;
    double max_constraint_violation_value = 0.0;
    double wall_dist = 0.0;
};

std::string current_case_name() {
#if defined(MINISOLVER_CASE_PENDULUM_ON_CART)
    return "pendulum_on_cart";
#elif defined(MINISOLVER_CASE_RACE_CARS)
    return "race_cars";
#elif defined(MINISOLVER_CASE_QUADROTOR_NAV)
    return "quadrotor_nav";
#elif defined(MINISOLVER_CASE_CHAIN_MASS)
    return "chain_mass";
#else
    return "unknown";
#endif
}

int default_steps() {
#if defined(MINISOLVER_CASE_PENDULUM_ON_CART)
    return pendulum_steps;
#elif defined(MINISOLVER_CASE_RACE_CARS)
    return race_cars_steps;
#elif defined(MINISOLVER_CASE_QUADROTOR_NAV)
    return quadrotor_nav_steps;
#elif defined(MINISOLVER_CASE_CHAIN_MASS)
    return chain_mass_steps;
#else
    return 0;
#endif
}

void print_usage(const char* argv0) {
    std::cout << "Usage: " << argv0
              << " [--steps N] [--warmup N] [--output path]\n";
}

bool parse_args(int argc, char** argv, Args& args) {
    args.steps = default_steps();
    args.output_csv = "results/raw/minisolver/" + current_case_name() + ".csv";
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--steps" && i + 1 < argc) {
            args.steps = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--warmup" && i + 1 < argc) {
            args.warmup_runs = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--output" && i + 1 < argc) {
            args.output_csv = argv[++i];
            continue;
        }
        if (arg == "--help") {
            print_usage(argv[0]);
            return false;
        }
        std::cerr << "Unknown argument: " << arg << "\n";
        return false;
    }
    return true;
}

template <typename Record>
void print_common_summary(
    const std::string& case_name,
    const std::vector<Record>& results,
    const std::filesystem::path& csv_path,
    const std::filesystem::path& summary_path) {
    std::vector<double> times;
    times.reserve(results.size());
    int success_count = 0;
    for (const auto& result : results) {
        times.push_back(result.time_ms);
        if (success_status(result.status)) {
            success_count++;
        }
    }

    std::cout << "\n=== Official NMPC Benchmark ===\n";
    std::cout << "case      : " << case_name << "\n";
    std::cout << "backend   : " << kBackendTag << "\n";
    std::cout << "steps     : " << results.size() << "\n";
    std::cout << "success   : " << success_count << "/" << results.size() << "\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "median ms : " << percentile(times, 0.5) << "\n";
    std::cout << "p95 ms    : " << percentile(times, 0.95) << "\n";
    std::cout << "max ms    : " << (times.empty() ? 0.0 : *std::max_element(times.begin(), times.end()))
              << "\n";
    std::cout << "steps csv : " << csv_path << "\n";
    std::cout << "summary   : " << summary_path << "\n";
}

template <typename Record>
void print_profile_breakdown(const std::vector<Record>& results) {
    if (results.empty()) {
        return;
    }

    double total_ms = 0.0;
    double deriv_ms = 0.0;
    double riccati_ms = 0.0;
    double line_search_ms = 0.0;
    for (const auto& record : results) {
        total_ms += record.time_ms;
        deriv_ms += record.deriv_ms;
        riccati_ms += record.riccati_ms;
        line_search_ms += record.line_search_ms;
    }

    if (!(total_ms > 0.0)) {
        return;
    }

    const double other_ms =
        std::max(0.0, total_ms - (deriv_ms + riccati_ms + line_search_ms));

    auto pct = [&](double value) { return 100.0 * value / total_ms; };

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "profile avg ms (Deriv / Riccati / LS / Other): "
              << (deriv_ms / results.size()) << " / " << (riccati_ms / results.size()) << " / "
              << (line_search_ms / results.size()) << " / " << (other_ms / results.size())
              << "\n";
    std::cout << std::setprecision(1);
    std::cout << "profile share  (Deriv / Riccati / LS / Other): "
              << pct(deriv_ms) << "% / " << pct(riccati_ms) << "% / " << pct(line_search_ms)
              << "% / " << pct(other_ms) << "%\n";
}

void initialize_pendulum_solver(MiniSolver<PendulumModel, 80>& solver) {
    solver.set_initial_state("x", 0.0);
    solver.set_initial_state("theta", M_PI);
    solver.set_initial_state("v", 0.0);
    solver.set_initial_state("omega", 0.0);
    const int N = solver.get_horizon();
    for (int k = 0; k < N; ++k) {
        solver.set_control_guess(k, "force", 0.0);
    }
    solver.rollout_dynamics();
}

PendulumStep run_pendulum_step(MiniSolver<PendulumModel, 80>& solver, int step) {
    PendulumStep result;
    result.step = step;
    const auto start = std::chrono::steady_clock::now();
    result.status = solver.solve();
    const auto end = std::chrono::steady_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(end - start).count();
    result.iterations = solver.get_iteration_count();
    result.deriv_ms = solver.get_profile_time_ms("Derivatives");
    result.riccati_ms = solver.get_profile_time_ms("Linear Solve");
    result.line_search_ms = solver.get_profile_time_ms("Line Search");
    result.force = solver.get_control(0, 0);
    result.state = to_array<4>(solver.get_state(0));
    result.total_cost_value = total_cost(solver);
    result.max_constraint_violation_value = max_positive_constraint(solver);
    return result;
}

std::vector<PendulumStep> run_pendulum_case(const Args& args) {
    SolverConfig config = make_solver_config();
    // Match the official acados pendulum example ("as_rti"): one RTI-style iteration per control step.
    config.termination_profile = TerminationProfile::RTI_FIXED_ITERATION;
    // In RTI (single-iteration) mode, use a rollout trial point so the iterate remains dynamically
    // consistent after the step. Otherwise postsolve can fail on large multiple-shooting defects.
    config.enable_line_search_rollout = true;
    config.max_iters = 1;
    config.tol_con = 1e-2;
    config.tol_dual = 1e-2;
    MiniSolver<PendulumModel, 80> warmup_solver(pendulum_horizon, Backend::CPU_SERIAL, config);
    warmup_solver.set_dt(pendulum_tf / static_cast<double>(pendulum_horizon));
    initialize_pendulum_solver(warmup_solver);
    for (int i = 0; i < args.warmup_runs; ++i) {
        warmup_solver.solve();
        shift_primal_guess(warmup_solver);
        warmup_solver.set_initial_state(std::vector<double>{0.0, M_PI, 0.0, 0.0});
        warmup_solver.reset(ResetOption::ALG_STATE);
    }

    MiniSolver<PendulumModel, 80> solver(pendulum_horizon, Backend::CPU_SERIAL, config);
    solver.set_dt(pendulum_tf / static_cast<double>(pendulum_horizon));
    initialize_pendulum_solver(solver);

    std::vector<PendulumStep> results;
    results.reserve(static_cast<size_t>(args.steps));
    auto current_state = pendulum_x0;
    for (int step = 0; step < args.steps; ++step) {
        auto result = run_pendulum_step(solver, step);
        results.push_back(result);

        MSVec<double, PendulumModel::NX> x = to_msvec<PendulumModel::NX>(current_state);
        MSVec<double, PendulumModel::NU> u;
        u(0) = result.force;
        MSVec<double, 0> p;
        const auto next = PendulumModel::integrate(
            x,
            u,
            p,
            pendulum_tf / static_cast<double>(pendulum_horizon),
            IntegratorType::RK4_EXPLICIT);
        current_state = to_array(next);

        shift_primal_guess(solver);
        solver.set_initial_state(std::vector<double>(current_state.begin(), current_state.end()));
        // In RTI mode we typically only take one iteration, so we must keep the multiple-shooting
        // trajectory dynamically consistent; otherwise postsolve can mark the step INFEASIBLE due
        // to large dynamics defects even when bounds are satisfied.
        solver.rollout_dynamics();
        solver.reset(ResetOption::ALG_STATE);
    }
    return results;
}

void write_pendulum_outputs(const Args& args, const std::vector<PendulumStep>& results) {
    const std::filesystem::path csv_path(args.output_csv);
    const std::filesystem::path summary_path =
        csv_path.parent_path() / (csv_path.stem().string() + "_summary.csv");
    std::filesystem::create_directories(csv_path.parent_path());

    std::ofstream out(csv_path);
    out << "step,backend,status,time_ms,deriv_ms,riccati_ms,line_search_ms,iterations,force,total_cost,max_constraint_violation,"
           "cart_x,pole_theta,cart_v,pole_omega\n";
    for (const auto& result : results) {
        print_csv_line(
            out,
            result.step,
            kBackendTag,
            status_to_string(result.status),
            result.time_ms,
            result.deriv_ms,
            result.riccati_ms,
            result.line_search_ms,
            result.iterations,
            result.force,
            result.total_cost_value,
            result.max_constraint_violation_value,
            result.state[0],
            result.state[1],
            result.state[2],
            result.state[3]);
    }

    std::vector<double> times;
    int success_count = 0;
    int total_iters = 0;
    double max_viol = 0.0;
    double final_theta = 0.0;
    for (const auto& result : results) {
        times.push_back(result.time_ms);
        total_iters += result.iterations;
        max_viol = std::max(max_viol, result.max_constraint_violation_value);
        final_theta = result.state[1];
        if (success_status(result.status)) {
            success_count++;
        }
    }

    std::ofstream summary(summary_path);
    summary << "case_name,backend,steps,success_rate,avg_iterations,median_ms,p95_ms,max_ms,"
               "max_constraint_violation,final_theta_abs,avg_deriv_ms,avg_riccati_ms,avg_line_search_ms\n";
    print_csv_line(
        summary,
        "pendulum_on_cart",
        kBackendTag,
        results.size(),
        results.empty() ? 0.0 : static_cast<double>(success_count) / results.size(),
        results.empty() ? 0.0 : static_cast<double>(total_iters) / results.size(),
        percentile(times, 0.5),
        percentile(times, 0.95),
        times.empty() ? 0.0 : *std::max_element(times.begin(), times.end()),
        max_viol,
        std::abs(final_theta),
        results.empty() ? 0.0 : std::accumulate(results.begin(), results.end(), 0.0, [](double acc, const PendulumStep& record) { return acc + record.deriv_ms; }) / results.size(),
        results.empty() ? 0.0 : std::accumulate(results.begin(), results.end(), 0.0, [](double acc, const PendulumStep& record) { return acc + record.riccati_ms; }) / results.size(),
        results.empty() ? 0.0 : std::accumulate(results.begin(), results.end(), 0.0, [](double acc, const PendulumStep& record) { return acc + record.line_search_ms; }) / results.size());

    print_common_summary("pendulum_on_cart", results, csv_path, summary_path);
    print_profile_breakdown(results);
}

#if defined(MINISOLVER_CASE_RACE_CARS)
constexpr int RACE_P_XREF = 0;
constexpr int RACE_P_UREF = RACE_P_XREF + RaceCarsModel::NX;
constexpr int RACE_P_WX = RACE_P_UREF + RaceCarsModel::NU;
constexpr int RACE_P_WU = RACE_P_WX + RaceCarsModel::NX;
constexpr int RACE_P_TRACK_BREAKS = RaceCarsModel::P_CALLBACK_BREAKS;
constexpr int RACE_P_KAPPA_COEFFS = RaceCarsModel::P_CALLBACK_COEFFS;
constexpr int RACE_ERK_STEPS = 3;
static_assert(RACE_P_WU + RaceCarsModel::NU == RACE_P_TRACK_BREAKS, "race callback parameter offsets must match generated model");
static_assert(
    RACE_P_KAPPA_COEFFS + RaceCarsModel::P_CALLBACK_WINDOW_SEGMENTS * 4 == RaceCarsModel::NP,
    "race callback parameter offsets must match generated model");

void apply_race_env_config(SolverConfig& config) {
    if (const char* value = env_value("MINISOLVER_RACE_PROFILE")) {
        const std::string profile(value);
        if (profile == "strict") {
            config.termination_profile = TerminationProfile::STRICT_KKT;
        } else if (profile == "acceptable") {
            config.termination_profile = TerminationProfile::ACCEPTABLE_NMPC;
        }
    }
    if (const char* value = env_value("MINISOLVER_RACE_INIT")) {
        const std::string mode(value);
        if (mode == "cold") {
            config.initialization = InitializationMode::COLD_START;
        } else if (mode == "primal") {
            config.initialization = InitializationMode::REUSE_PRIMAL;
        } else if (mode == "primal_dual") {
            config.initialization = InitializationMode::REUSE_PRIMAL_DUAL;
        }
    }
    if (const char* value = env_value("MINISOLVER_RACE_WARM_BARRIER")) {
        const std::string mode(value);
        if (mode == "reset") {
            config.warm_start_barrier = WarmStartBarrierMode::RESET_TO_MU_INIT;
        } else if (mode == "reuse") {
            config.warm_start_barrier = WarmStartBarrierMode::REUSE_PREVIOUS_MU;
        } else if (mode == "gap") {
            config.warm_start_barrier = WarmStartBarrierMode::FROM_COMPLEMENTARITY_GAP;
        }
    }
    if (const char* value = env_value("MINISOLVER_RACE_WARM_REG")) {
        const std::string mode(value);
        if (mode == "reset") {
            config.warm_start_regularization = WarmStartRegularizationMode::RESET_TO_REG_INIT;
        } else if (mode == "reuse") {
            config.warm_start_regularization = WarmStartRegularizationMode::REUSE_PREVIOUS_REG;
        } else if (mode == "decay") {
            config.warm_start_regularization = WarmStartRegularizationMode::DECAY_PREVIOUS_REG;
        }
    }
    if (const char* value = env_value("MINISOLVER_RACE_LINE_SEARCH")) {
        const std::string mode(value);
        if (mode == "filter") {
            config.line_search_type = LineSearchType::FILTER;
        } else if (mode == "merit") {
            config.line_search_type = LineSearchType::MERIT;
        } else if (mode == "none") {
            config.line_search_type = LineSearchType::NONE;
        }
    }
    if (const char* value = env_value("MINISOLVER_RACE_BARRIER")) {
        const std::string mode(value);
        if (mode == "mehrotra") {
            config.barrier_strategy = BarrierStrategy::MEHROTRA;
        } else if (mode == "adaptive") {
            config.barrier_strategy = BarrierStrategy::ADAPTIVE;
        } else if (mode == "monotone") {
            config.barrier_strategy = BarrierStrategy::MONOTONE;
        }
    }
    if (const char* value = env_value("MINISOLVER_RACE_CONSTRAINT_SCALING")) {
        const std::string mode(value);
        config.constraint_scaling =
            mode == "row" ? ConstraintScalingMethod::ROW_INF_NORM : ConstraintScalingMethod::NONE;
    }
    if (const char* value = env_value("MINISOLVER_RACE_OBJECTIVE_SCALING")) {
        const std::string mode(value);
        config.objective_scaling =
            mode == "hessian" ? ObjectiveScalingMethod::HESSIAN_GERSHGORIN : ObjectiveScalingMethod::NONE;
    }
    if (const char* value = env_value("MINISOLVER_RACE_PROBLEM_SCALING")) {
        const std::string mode(value);
        config.problem_scaling =
            mode == "ruiz" ? ProblemScalingMethod::RUIZ_EQUILIBRATION : ProblemScalingMethod::NONE;
    }
    if (const char* value = env_value("MINISOLVER_RACE_HESSIAN")) {
        const std::string mode(value);
        config.hessian_approximation =
            mode == "exact" ? HessianApproximation::EXACT : HessianApproximation::OBJECTIVE_HESSIAN_ONLY;
    }
    apply_env_int("MINISOLVER_RACE_MAX_ITERS", config.max_iters);
    apply_env_int("MINISOLVER_RACE_LS_MAX_ITERS", config.line_search_max_iters);
    apply_env_int("MINISOLVER_RACE_RESTORATION_MAX_ITERS", config.max_restoration_iters);
    apply_env_double("MINISOLVER_RACE_TOL_CON", config.tol_con);
    apply_env_double("MINISOLVER_RACE_TOL_DUAL", config.tol_dual);
    apply_env_double("MINISOLVER_RACE_TOL_MU", config.tol_mu);
    apply_env_double("MINISOLVER_RACE_FEASIBLE_SCALE", config.feasible_tol_scale);
    apply_env_double("MINISOLVER_RACE_MU_INIT", config.mu_init);
    apply_env_double("MINISOLVER_RACE_REG_INIT", config.reg_init);
    apply_env_double("MINISOLVER_RACE_LS_BACKTRACK", config.line_search_backtrack_factor);
    apply_env_double("MINISOLVER_RACE_LS_TAU", config.line_search_tau);
    apply_env_bool("MINISOLVER_RACE_SOC", config.enable_soc);
    apply_env_bool("MINISOLVER_RACE_RESTORATION", config.enable_feasibility_restoration);
    apply_env_bool("MINISOLVER_RACE_SLACK_RESET", config.enable_slack_reset);
    apply_env_bool("MINISOLVER_RACE_ROLLOUT", config.enable_line_search_rollout);
    apply_env_bool("MINISOLVER_RACE_RESIDUAL_STAGNATION", config.enable_residual_stagnation_detection);
    apply_env_bool("MINISOLVER_RACE_PROFILING", config.enable_profiling);
}

int find_race_kappa_segment(double s) {
    using generated::race_kappa_breaks;
    constexpr int segment_count = static_cast<int>(race_kappa_breaks.size()) - 1;
    int segment = 0;
    if (s >= race_kappa_breaks.back()) {
        segment = segment_count - 1;
    } else {
        while (segment + 1 < segment_count && s >= race_kappa_breaks[segment + 1]) {
            ++segment;
        }
    }
    return segment;
}

template <typename Setter>
ApiStatus set_race_track_parameter_values(Setter&& setter, double s_lin) {
    using generated::race_kappa_breaks;
    using generated::race_kappa_coeffs;

    constexpr int window_segments = RaceCarsModel::P_CALLBACK_WINDOW_SEGMENTS;
    constexpr int segment_count = static_cast<int>(race_kappa_breaks.size()) - 1;
    const int center_segment = find_race_kappa_segment(s_lin);
    const int first_segment = std::clamp(center_segment - window_segments / 2, 0, segment_count - window_segments);

    for (int i = 0; i <= window_segments; ++i) {
        const ApiStatus status = setter(RACE_P_TRACK_BREAKS + i, race_kappa_breaks[first_segment + i]);
        if (status != ApiStatus::OK) {
            return status;
        }
    }

    for (int segment = 0; segment < window_segments; ++segment) {
        const int source_offset = 4 * (first_segment + segment);
        const int target_offset = RACE_P_KAPPA_COEFFS + 4 * segment;
        for (int coeff = 0; coeff < 4; ++coeff) {
            const ApiStatus status = setter(target_offset + coeff, race_kappa_coeffs[source_offset + coeff]);
            if (status != ApiStatus::OK) {
                return status;
            }
        }
    }
    return ApiStatus::OK;
}

ApiStatus set_race_track_parameters(MiniSolver<RaceCarsModel, 80>& solver, int stage, double s_lin) {
    auto setter = [&](int idx, double value) { return solver.set_parameter(stage, idx, value); };
    return set_race_track_parameter_values(setter, s_lin);
}

void fill_race_track_parameter_vector(MSVec<double, RaceCarsModel::NP>& p, double s_lin) {
    auto setter = [&](int idx, double value) {
        p(idx) = value;
        return ApiStatus::OK;
    };
    set_race_track_parameter_values(setter, s_lin);
}

std::array<double, RaceCarsModel::NX> integrate_race_closed_loop_state(
    const std::array<double, RaceCarsModel::NX>& state,
    const std::vector<double>& control_values) {
    MSVec<double, RaceCarsModel::NX> sub_state = to_msvec<RaceCarsModel::NX>(state);
    const MSVec<double, RaceCarsModel::NU> control = to_msvec<RaceCarsModel::NU>(control_values);
    const double sub_dt = race_cars_tf / static_cast<double>(race_cars_horizon * RACE_ERK_STEPS);
    for (int substep = 0; substep < RACE_ERK_STEPS; ++substep) {
        MSVec<double, RaceCarsModel::NP> p;
        p.setZero();
        fill_race_track_parameter_vector(p, sub_state(0));
        sub_state = RaceCarsModel::integrate(
            sub_state,
            control,
            p,
            sub_dt,
            IntegratorType::RK4_EXPLICIT);
    }
    return to_array(sub_state);
}

ApiStatus refresh_race_track_parameters(MiniSolver<RaceCarsModel, 80>& solver) {
    for (int k = 0; k <= solver.get_horizon(); ++k) {
        const ApiStatus status = set_race_track_parameters(solver, k, solver.get_state(k, 0));
        if (status != ApiStatus::OK) {
            return status;
        }
    }
    return ApiStatus::OK;
}

ApiStatus update_race_track_parameters(MiniSolver<RaceCarsModel, 80>& solver, void* /*user*/) {
    return refresh_race_track_parameters(solver);
}

void set_race_cost_parameters(MiniSolver<RaceCarsModel, 80>& solver, double s0) {
    const double unscale = static_cast<double>(race_cars_horizon) / race_cars_tf;
    const std::array<double, RaceCarsModel::NX> stage_wx = {
        unscale * 1e-1,
        unscale * 1e-8,
        unscale * 1e-8,
        unscale * 1e-8,
        unscale * 1e-3,
        unscale * 5e-3,
    };
    const std::array<double, RaceCarsModel::NU> stage_wu = {
        unscale * 1e-3,
        unscale * 5e-3,
    };
    const std::array<double, RaceCarsModel::NX> terminal_wx = {
        5.0 / unscale,
        10.0 / unscale,
        1e-8 / unscale,
        1e-8 / unscale,
        5e-3 / unscale,
        2e-3 / unscale,
    };
    const std::array<double, RaceCarsModel::NU> terminal_wu = {0.0, 0.0};
    const double sref = s0 + race_cars_sref_n;
    const int N = solver.get_horizon();

    for (int k = 0; k < N; ++k) {
        const double sref_k = s0 + (sref - s0) * static_cast<double>(k) / static_cast<double>(N);
        std::array<double, RaceCarsModel::NX> xref = {sref_k, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::array<double, RaceCarsModel::NU> uref = {0.0, 0.0};
        set_stage_array_parameters(solver, k, RACE_P_XREF, xref);
        set_stage_array_parameters(solver, k, RACE_P_UREF, uref);
        set_stage_array_parameters(solver, k, RACE_P_WX, stage_wx);
        set_stage_array_parameters(solver, k, RACE_P_WU, stage_wu);
    }

    std::array<double, RaceCarsModel::NX> xref_terminal = {sref, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, RaceCarsModel::NU> uref_terminal = {0.0, 0.0};
    set_stage_array_parameters(solver, N, RACE_P_XREF, xref_terminal);
    set_stage_array_parameters(solver, N, RACE_P_UREF, uref_terminal);
    set_stage_array_parameters(solver, N, RACE_P_WX, terminal_wx);
    set_stage_array_parameters(solver, N, RACE_P_WU, terminal_wu);
}

void initialize_race_solver(MiniSolver<RaceCarsModel, 80>& solver, const std::array<double, 6>& x0) {
    solver.set_initial_state(std::vector<double>(x0.begin(), x0.end()));
    const int N = solver.get_horizon();
    for (int k = 0; k < N; ++k) {
        solver.set_control_guess(k, 0, 0.0);
        solver.set_control_guess(k, 1, 0.0);
    }
    refresh_race_track_parameters(solver);
    solver.rollout_dynamics();
    refresh_race_track_parameters(solver);
}

std::vector<RaceStep> run_race_case(const Args& args) {
    SolverConfig config = make_solver_config();
    config.termination_profile = TerminationProfile::ACCEPTABLE_NMPC;
    config.initialization = InitializationMode::REUSE_PRIMAL;
    config.warm_start_barrier = WarmStartBarrierMode::RESET_TO_MU_INIT;
    config.warm_start_regularization = WarmStartRegularizationMode::RESET_TO_REG_INIT;
    config.objective_scaling = ObjectiveScalingMethod::HESSIAN_GERSHGORIN;
    config.max_iters = 52;
    config.tol_con = 1e-4;
    config.tol_dual = 1e-4;
    config.tol_mu = 1e-4;
    config.feasible_tol_scale = 20.0;
    config.enable_soc = true;
    config.enable_feasibility_restoration = true;
    config.enable_slack_reset = true;
    config.enable_line_search_rollout = true;
    apply_race_env_config(config);
    MiniSolver<RaceCarsModel, 80> warmup_solver(race_cars_horizon, Backend::CPU_SERIAL, config);
    warmup_solver.set_dt(race_cars_tf / static_cast<double>(race_cars_horizon));
    initialize_race_solver(warmup_solver, race_cars_x0);
    warmup_solver.set_model_update_callback(update_race_track_parameters);
    auto warm_state = race_cars_x0;
    for (int i = 0; i < args.warmup_runs; ++i) {
        set_race_cost_parameters(warmup_solver, warm_state[0]);
        warmup_solver.solve();
        warm_state = integrate_race_closed_loop_state(warm_state, warmup_solver.get_control(0));
        shift_primal_guess(warmup_solver);
        warmup_solver.set_initial_state(std::vector<double>(warm_state.begin(), warm_state.end()));
    }

    MiniSolver<RaceCarsModel, 80> solver(race_cars_horizon, Backend::CPU_SERIAL, config);
    solver.set_dt(race_cars_tf / static_cast<double>(race_cars_horizon));
    initialize_race_solver(solver, race_cars_x0);
    solver.set_model_update_callback(update_race_track_parameters);

    std::vector<RaceStep> results;
    results.reserve(static_cast<size_t>(args.steps));
    auto current_state = race_cars_x0;
    for (int step = 0; step < args.steps; ++step) {
        set_race_cost_parameters(solver, current_state[0]);

        RaceStep result;
        result.step = step;
        const auto start = std::chrono::steady_clock::now();
        result.status = solver.solve();
        const auto end = std::chrono::steady_clock::now();
        result.time_ms = std::chrono::duration<double, std::milli>(end - start).count();
        result.iterations = solver.get_iteration_count();
        result.deriv_ms = solver.get_profile_time_ms("Derivatives");
        result.riccati_ms = solver.get_profile_time_ms("Linear Solve");
        result.line_search_ms = solver.get_profile_time_ms("Line Search");
        result.total_cost_value = total_cost(solver);
        result.max_constraint_violation_value = max_positive_constraint(solver);
        const auto control = solver.get_control(0);
        current_state = integrate_race_closed_loop_state(current_state, control);
        result.s = current_state[0];
        result.n = current_state[1];
        result.alpha = current_state[2];
        result.v = current_state[3];
        result.D = current_state[4];
        result.delta = current_state[5];
        result.derD = control[0];
        result.derDelta = control[1];
        results.push_back(result);

        shift_primal_guess(solver);
        solver.set_initial_state(std::vector<double>(current_state.begin(), current_state.end()));

        if (current_state[0] > race_cars_pathlength + 0.1) {
            auto crossing = std::find_if(
                results.begin() + 1,
                results.end(),
                [](const RaceStep& record) { return record.s > 0.0; });
            if (crossing != results.end()) {
                results.erase(results.begin(), crossing);
            }
            break;
        }
    }
    return results;
}

void write_race_outputs(const Args& args, const std::vector<RaceStep>& results) {
    const std::filesystem::path csv_path(args.output_csv);
    const std::filesystem::path summary_path =
        csv_path.parent_path() / (csv_path.stem().string() + "_summary.csv");
    std::filesystem::create_directories(csv_path.parent_path());

    std::ofstream out(csv_path);
    out << "step,backend,status,time_ms,deriv_ms,riccati_ms,line_search_ms,iterations,total_cost,max_constraint_violation,"
           "s,n,alpha,v,D,delta,derD,derDelta\n";
    for (const auto& result : results) {
        print_csv_line(
            out,
            result.step,
            kBackendTag,
            status_to_string(result.status),
            result.time_ms,
            result.deriv_ms,
            result.riccati_ms,
            result.line_search_ms,
            result.iterations,
            result.total_cost_value,
            result.max_constraint_violation_value,
            result.s,
            result.n,
            result.alpha,
            result.v,
            result.D,
            result.delta,
            result.derD,
            result.derDelta);
    }

    std::vector<double> times;
    int success_count = 0;
    int total_iters = 0;
    double max_viol = 0.0;
    double avg_speed = 0.0;
    for (const auto& result : results) {
        times.push_back(result.time_ms);
        total_iters += result.iterations;
        max_viol = std::max(max_viol, result.max_constraint_violation_value);
        avg_speed += result.v;
        if (success_status(result.status)) {
            success_count++;
        }
    }
    if (!results.empty()) {
        avg_speed /= static_cast<double>(results.size());
    }

    std::ofstream summary(summary_path);
    summary << "case_name,backend,steps,success_rate,avg_iterations,median_ms,p95_ms,max_ms,"
               "max_constraint_violation,avg_speed,avg_deriv_ms,avg_riccati_ms,avg_line_search_ms\n";
    print_csv_line(
        summary,
        "race_cars",
        kBackendTag,
        results.size(),
        results.empty() ? 0.0 : static_cast<double>(success_count) / results.size(),
        results.empty() ? 0.0 : static_cast<double>(total_iters) / results.size(),
        percentile(times, 0.5),
        percentile(times, 0.95),
        times.empty() ? 0.0 : *std::max_element(times.begin(), times.end()),
        max_viol,
        avg_speed,
        results.empty() ? 0.0 : std::accumulate(results.begin(), results.end(), 0.0, [](double acc, const RaceStep& record) { return acc + record.deriv_ms; }) / results.size(),
        results.empty() ? 0.0 : std::accumulate(results.begin(), results.end(), 0.0, [](double acc, const RaceStep& record) { return acc + record.riccati_ms; }) / results.size(),
        results.empty() ? 0.0 : std::accumulate(results.begin(), results.end(), 0.0, [](double acc, const RaceStep& record) { return acc + record.line_search_ms; }) / results.size());

    print_common_summary("race_cars", results, csv_path, summary_path);
    print_profile_breakdown(results);
}
#endif

#if defined(MINISOLVER_CASE_QUADROTOR_NAV)
constexpr int QUAD_P_XREF = 0;
constexpr int QUAD_P_UREF = QUAD_P_XREF + QuadrotorNavModel::NX;
constexpr int QUAD_P_WX = QUAD_P_UREF + QuadrotorNavModel::NU;
constexpr int QUAD_P_WU = QUAD_P_WX + QuadrotorNavModel::NX;
constexpr int QUAD_P_TRACK_X_BREAKS = QuadrotorNavModel::P_CALLBACK_TRACK_X_BREAKS;
constexpr int QUAD_P_TRACK_X_COEFFS = QuadrotorNavModel::P_CALLBACK_TRACK_X_COEFFS;
constexpr int QUAD_P_TRACK_Y_BREAKS = QuadrotorNavModel::P_CALLBACK_TRACK_Y_BREAKS;
constexpr int QUAD_P_TRACK_Y_COEFFS = QuadrotorNavModel::P_CALLBACK_TRACK_Y_COEFFS;
constexpr int QUAD_P_TRACK_Z_BREAKS = QuadrotorNavModel::P_CALLBACK_TRACK_Z_BREAKS;
constexpr int QUAD_P_TRACK_Z_COEFFS = QuadrotorNavModel::P_CALLBACK_TRACK_Z_COEFFS;
static_assert(QUAD_P_WU + QuadrotorNavModel::NU == QUAD_P_TRACK_X_BREAKS, "quad callback parameter offsets must match generated model");
static_assert(
    QUAD_P_TRACK_Z_COEFFS + QuadrotorNavModel::P_CALLBACK_WINDOW_SEGMENTS * 6 == QuadrotorNavModel::NP,
    "quad callback parameter offsets must match generated model");

template <std::size_t NBreaks>
int find_quad_track_segment(double s, const std::array<double, NBreaks>& breaks) {
    constexpr int segment_count = static_cast<int>(NBreaks) - 1;
    int segment = 0;
    if (s >= breaks.back()) {
        segment = segment_count - 1;
    } else {
        while (segment + 1 < segment_count && s >= breaks[segment + 1]) {
            ++segment;
        }
    }
    return segment;
}

template <std::size_t NBreaks, std::size_t NCoeffs, typename Setter>
ApiStatus set_quad_track_axis_parameters(
    Setter&& setter,
    int break_base,
    int coeff_base,
    double s_lin,
    const std::array<double, NBreaks>& breaks,
    const std::array<double, NCoeffs>& coeffs) {
    constexpr int window_segments = QuadrotorNavModel::P_CALLBACK_WINDOW_SEGMENTS;
    constexpr int segment_count = static_cast<int>(NBreaks) - 1;
    const int center_segment = find_quad_track_segment(s_lin, breaks);
    const int first_segment = std::clamp(center_segment - window_segments / 2, 0, segment_count - window_segments);

    for (int i = 0; i <= window_segments; ++i) {
        const ApiStatus status = setter(break_base + i, breaks[first_segment + i]);
        if (status != ApiStatus::OK) {
            return status;
        }
    }

    for (int segment = 0; segment < window_segments; ++segment) {
        const int source_offset = 6 * (first_segment + segment);
        const int target_offset = coeff_base + 6 * segment;
        for (int coeff = 0; coeff < 6; ++coeff) {
            const ApiStatus status = setter(target_offset + coeff, coeffs[source_offset + coeff]);
            if (status != ApiStatus::OK) {
                return status;
            }
        }
    }
    return ApiStatus::OK;
}

ApiStatus set_quad_track_parameters(MiniSolver<QuadrotorNavModel, 80>& solver, int stage, double s_lin) {
    auto setter = [&](int idx, double value) { return solver.set_parameter(stage, idx, value); };
    ApiStatus status = set_quad_track_axis_parameters(
        setter, QUAD_P_TRACK_X_BREAKS, QUAD_P_TRACK_X_COEFFS, s_lin, generated::quad_track_x_breaks, generated::quad_track_x_coeffs);
    if (status != ApiStatus::OK) {
        return status;
    }
    status = set_quad_track_axis_parameters(
        setter, QUAD_P_TRACK_Y_BREAKS, QUAD_P_TRACK_Y_COEFFS, s_lin, generated::quad_track_y_breaks, generated::quad_track_y_coeffs);
    if (status != ApiStatus::OK) {
        return status;
    }
    return set_quad_track_axis_parameters(
        setter, QUAD_P_TRACK_Z_BREAKS, QUAD_P_TRACK_Z_COEFFS, s_lin, generated::quad_track_z_breaks, generated::quad_track_z_coeffs);
}

void fill_quad_track_parameter_vector(MSVec<double, QuadrotorNavModel::NP>& p, double s_lin) {
    auto setter = [&](int idx, double value) {
        p(idx) = value;
        return ApiStatus::OK;
    };
    set_quad_track_axis_parameters(
        setter, QUAD_P_TRACK_X_BREAKS, QUAD_P_TRACK_X_COEFFS, s_lin, generated::quad_track_x_breaks, generated::quad_track_x_coeffs);
    set_quad_track_axis_parameters(
        setter, QUAD_P_TRACK_Y_BREAKS, QUAD_P_TRACK_Y_COEFFS, s_lin, generated::quad_track_y_breaks, generated::quad_track_y_coeffs);
    set_quad_track_axis_parameters(
        setter, QUAD_P_TRACK_Z_BREAKS, QUAD_P_TRACK_Z_COEFFS, s_lin, generated::quad_track_z_breaks, generated::quad_track_z_coeffs);
}

ApiStatus refresh_quad_track_parameters(MiniSolver<QuadrotorNavModel, 80>& solver) {
    for (int k = 0; k <= solver.get_horizon(); ++k) {
        const ApiStatus status = set_quad_track_parameters(solver, k, solver.get_state(k, 0));
        if (status != ApiStatus::OK) {
            return status;
        }
    }
    return ApiStatus::OK;
}

ApiStatus update_quad_track_parameters(MiniSolver<QuadrotorNavModel, 80>& solver, void* /*user*/) {
    return refresh_quad_track_parameters(solver);
}

void set_quad_cost_parameters(MiniSolver<QuadrotorNavModel, 80>& solver, double s0) {
    std::array<double, QuadrotorNavModel::NX> stage_wx = {
        1.0, 1e-1, 1e-1,
        1e-5, 1e-5, 1e-5, 1e-5,
        1e-5, 1e-5, 1e-5,
        1e-5, 1e-5, 1e-5,
        1e-5, 1e-5, 1e-5,
        1e-8, 1e-8, 1e-8, 1e-8,
    };
    std::array<double, QuadrotorNavModel::NU> stage_wu = {1e-5, 1e-5, 1e-5, 1e-5};
    std::array<double, QuadrotorNavModel::NX> terminal_wx = {
        10.0, 1e-3, 1e-2,
        1e-5, 1e-5, 1e-5, 1e-5,
        1e-5, 1e-5, 1e-5,
        1e-5, 1e-5, 1e-5,
        1e-5, 1e-5, 1e-5,
        1e-8, 1e-8, 1e-8, 1e-8,
    };
    std::array<double, QuadrotorNavModel::NU> terminal_wu = {0.0, 0.0, 0.0, 0.0};
    const double sref = s0 + quadrotor_nav_s_ref;
    const double sref_dot = quadrotor_nav_s_ref / quadrotor_nav_tf;
    const int N = solver.get_horizon();
    for (int k = 0; k < N; ++k) {
        const double sref_k =
            s0 + (sref - s0) * static_cast<double>(k) / static_cast<double>(N);
        std::array<double, QuadrotorNavModel::NX> xref = {
            sref_k, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            sref_dot, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            quadrotor_nav_u_hov, quadrotor_nav_u_hov, quadrotor_nav_u_hov, quadrotor_nav_u_hov,
        };
        std::array<double, QuadrotorNavModel::NU> uref = {0.0, 0.0, 0.0, 0.0};
        set_stage_array_parameters(solver, k, QUAD_P_XREF, xref);
        set_stage_array_parameters(solver, k, QUAD_P_UREF, uref);
        set_stage_array_parameters(solver, k, QUAD_P_WX, stage_wx);
        set_stage_array_parameters(solver, k, QUAD_P_WU, stage_wu);
    }

    const double terminal_sref = sref;
    std::array<double, QuadrotorNavModel::NX> xref_terminal = {
        terminal_sref, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
        sref_dot, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        quadrotor_nav_u_hov, quadrotor_nav_u_hov, quadrotor_nav_u_hov, quadrotor_nav_u_hov,
    };
    std::array<double, QuadrotorNavModel::NU> uref_terminal = {0.0, 0.0, 0.0, 0.0};
    set_stage_array_parameters(solver, N, QUAD_P_XREF, xref_terminal);
    set_stage_array_parameters(solver, N, QUAD_P_UREF, uref_terminal);
    set_stage_array_parameters(solver, N, QUAD_P_WX, terminal_wx);
    set_stage_array_parameters(solver, N, QUAD_P_WU, terminal_wu);
}

void initialize_quad_solver(MiniSolver<QuadrotorNavModel, 80>& solver, const std::array<double, 20>& x0) {
    solver.set_initial_state(std::vector<double>(x0.begin(), x0.end()));
    const int N = solver.get_horizon();
    for (int k = 0; k < N; ++k) {
        for (int j = 0; j < QuadrotorNavModel::NU; ++j) {
            solver.set_control_guess(k, j, 0.0);
        }
    }
    refresh_quad_track_parameters(solver);
    solver.rollout_dynamics();
    refresh_quad_track_parameters(solver);
}

std::vector<QuadStep> run_quad_case(const Args& args) {
    SolverConfig config = make_solver_config();
    config.termination_profile = TerminationProfile::STRICT_KKT;
    config.initialization = InitializationMode::REUSE_PRIMAL_DUAL;
    config.warm_start_barrier = WarmStartBarrierMode::REUSE_PREVIOUS_MU;
    config.warm_start_regularization = WarmStartRegularizationMode::DECAY_PREVIOUS_REG;
    config.max_iters = 5;
    config.tol_con = 1e-4;
    config.tol_dual = 1e-4;
    MiniSolver<QuadrotorNavModel, 80> warmup_solver(quadrotor_nav_horizon, Backend::CPU_SERIAL, config);
    warmup_solver.set_dt(quadrotor_nav_tf / static_cast<double>(quadrotor_nav_horizon));
    initialize_quad_solver(warmup_solver, quadrotor_nav_init_zeta);
    warmup_solver.set_model_update_callback(update_quad_track_parameters);
    auto warm_state = quadrotor_nav_init_zeta;
    for (int i = 0; i < args.warmup_runs; ++i) {
        set_quad_cost_parameters(warmup_solver, warm_state[0]);
        warmup_solver.solve();
        MSVec<double, QuadrotorNavModel::NP> p;
        p.setZero();
        fill_quad_track_parameter_vector(p, warm_state[0]);
        const auto next = QuadrotorNavModel::integrate(
            to_msvec<QuadrotorNavModel::NX>(warm_state),
            to_msvec<QuadrotorNavModel::NU>(warmup_solver.get_control(0)),
            p,
            quadrotor_nav_tf / static_cast<double>(quadrotor_nav_horizon),
            IntegratorType::RK4_EXPLICIT);
        warm_state = to_array(next);
        shift_primal_guess(warmup_solver);
        warmup_solver.set_initial_state(std::vector<double>(warm_state.begin(), warm_state.end()));
    }

    MiniSolver<QuadrotorNavModel, 80> solver(quadrotor_nav_horizon, Backend::CPU_SERIAL, config);
    solver.set_dt(quadrotor_nav_tf / static_cast<double>(quadrotor_nav_horizon));
    initialize_quad_solver(solver, quadrotor_nav_init_zeta);
    solver.set_model_update_callback(update_quad_track_parameters);

    std::vector<QuadStep> results;
    results.reserve(static_cast<size_t>(args.steps));
    auto current_state = quadrotor_nav_init_zeta;
    for (int step = 0; step < args.steps; ++step) {
        if (current_state[0] >= quadrotor_nav_s_max) {
            break;
        }

        set_quad_cost_parameters(solver, current_state[0]);
        QuadStep result;
        result.step = step;
        const auto start = std::chrono::steady_clock::now();
        result.status = solver.solve();
        const auto end = std::chrono::steady_clock::now();
        result.time_ms = std::chrono::duration<double, std::milli>(end - start).count();
        result.iterations = solver.get_iteration_count();
        result.deriv_ms = solver.get_profile_time_ms("Derivatives");
        result.riccati_ms = solver.get_profile_time_ms("Linear Solve");
        result.line_search_ms = solver.get_profile_time_ms("Line Search");
        result.total_cost_value = total_cost(solver);
        result.max_constraint_violation_value = max_positive_constraint(solver);

        MSVec<double, QuadrotorNavModel::NP> p;
        p.setZero();
        fill_quad_track_parameter_vector(p, current_state[0]);
        const auto next = QuadrotorNavModel::integrate(
            to_msvec<QuadrotorNavModel::NX>(current_state),
            to_msvec<QuadrotorNavModel::NU>(solver.get_control(0)),
            p,
            quadrotor_nav_tf / static_cast<double>(quadrotor_nav_horizon),
            IntegratorType::RK4_EXPLICIT);
        current_state = to_array(next);
        result.abs_n = std::abs(current_state[1]);
        result.abs_b = std::abs(current_state[2]);
        results.push_back(result);

        shift_primal_guess(solver);
        solver.set_initial_state(std::vector<double>(current_state.begin(), current_state.end()));
    }
    return results;
}

void write_quad_outputs(const Args& args, const std::vector<QuadStep>& results) {
    const std::filesystem::path csv_path(args.output_csv);
    const std::filesystem::path summary_path =
        csv_path.parent_path() / (csv_path.stem().string() + "_summary.csv");
    std::filesystem::create_directories(csv_path.parent_path());

    std::ofstream out(csv_path);
    out << "step,backend,status,time_ms,deriv_ms,riccati_ms,line_search_ms,iterations,total_cost,max_constraint_violation,abs_n,abs_b\n";
    for (const auto& result : results) {
        print_csv_line(
            out,
            result.step,
            kBackendTag,
            status_to_string(result.status),
            result.time_ms,
            result.deriv_ms,
            result.riccati_ms,
            result.line_search_ms,
            result.iterations,
            result.total_cost_value,
            result.max_constraint_violation_value,
            result.abs_n,
            result.abs_b);
    }

    std::vector<double> times;
    int success_count = 0;
    int total_iters = 0;
    double max_viol = 0.0;
    double avg_abs_n = 0.0;
    double avg_abs_b = 0.0;
    for (const auto& result : results) {
        times.push_back(result.time_ms);
        total_iters += result.iterations;
        max_viol = std::max(max_viol, result.max_constraint_violation_value);
        avg_abs_n += result.abs_n;
        avg_abs_b += result.abs_b;
        if (success_status(result.status)) {
            success_count++;
        }
    }
    if (!results.empty()) {
        avg_abs_n /= static_cast<double>(results.size());
        avg_abs_b /= static_cast<double>(results.size());
    }

    std::ofstream summary(summary_path);
    summary << "case_name,backend,steps,success_rate,avg_iterations,median_ms,p95_ms,max_ms,"
               "max_constraint_violation,avg_abs_n,avg_abs_b,avg_deriv_ms,avg_riccati_ms,avg_line_search_ms\n";
    print_csv_line(
        summary,
        "quadrotor_nav",
        kBackendTag,
        results.size(),
        results.empty() ? 0.0 : static_cast<double>(success_count) / results.size(),
        results.empty() ? 0.0 : static_cast<double>(total_iters) / results.size(),
        percentile(times, 0.5),
        percentile(times, 0.95),
        times.empty() ? 0.0 : *std::max_element(times.begin(), times.end()),
        max_viol,
        avg_abs_n,
        avg_abs_b,
        results.empty() ? 0.0 : std::accumulate(results.begin(), results.end(), 0.0, [](double acc, const QuadStep& record) { return acc + record.deriv_ms; }) / results.size(),
        results.empty() ? 0.0 : std::accumulate(results.begin(), results.end(), 0.0, [](double acc, const QuadStep& record) { return acc + record.riccati_ms; }) / results.size(),
        results.empty() ? 0.0 : std::accumulate(results.begin(), results.end(), 0.0, [](double acc, const QuadStep& record) { return acc + record.line_search_ms; }) / results.size());

    print_common_summary("quadrotor_nav", results, csv_path, summary_path);
    print_profile_breakdown(results);
}
#endif

#if defined(MINISOLVER_CASE_CHAIN_MASS)
void set_chain_cost_parameters(MiniSolver<ChainMassModel, 64>& solver) {
    std::array<double, ChainMassModel::NX> xref{};
    std::copy(chain_mass_xrest.begin(), chain_mass_xrest.end(), xref.begin());

    std::array<double, ChainMassModel::NX> stage_wx{};
    stage_wx.fill(2.0);
    const double strong_penalty = static_cast<double>(ChainMassModel::M + 1);
    stage_wx[3 * ChainMassModel::M + 0] = 2.0 * strong_penalty;
    stage_wx[3 * ChainMassModel::M + 1] = 2.0 * strong_penalty;
    stage_wx[3 * ChainMassModel::M + 2] = 2.0 * strong_penalty;
    std::array<double, ChainMassModel::NU> stage_wu = {0.02, 0.02, 0.02};
    std::array<double, ChainMassModel::NU> terminal_wu = {0.0, 0.0, 0.0};
    std::array<double, ChainMassModel::DIST_DIM> zero_dist{};
    const int N = solver.get_horizon();

    for (int k = 0; k < N; ++k) {
        set_stage_array_parameters(solver, k, ChainMassModel::P_DIST, zero_dist);
        set_stage_array_parameters(solver, k, ChainMassModel::P_XREF, xref);
        set_stage_array_parameters(solver, k, ChainMassModel::P_WX, stage_wx);
        set_stage_array_parameters(solver, k, ChainMassModel::P_WU, stage_wu);
    }

    set_stage_array_parameters(solver, N, ChainMassModel::P_DIST, zero_dist);
    set_stage_array_parameters(solver, N, ChainMassModel::P_XREF, xref);
    set_stage_array_parameters(solver, N, ChainMassModel::P_WX, stage_wx);
    set_stage_array_parameters(solver, N, ChainMassModel::P_WU, terminal_wu);
}

void initialize_chain_solver(MiniSolver<ChainMassModel, 64>& solver, const std::array<double, ChainMassModel::NX>& x0) {
    solver.set_initial_state(std::vector<double>(x0.begin(), x0.end()));
    const int N = solver.get_horizon();
    for (int k = 0; k < N; ++k) {
        for (int j = 0; j < ChainMassModel::NU; ++j) {
            solver.set_control_guess(k, j, 0.0);
        }
    }
    solver.rollout_dynamics();
}

template <size_t N>
std::array<double, N> sample_uniform_ellipsoid(std::mt19937& rng, double scale) {
    std::normal_distribution<double> normal(0.0, 1.0);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::array<double, N> direction{};
    double norm_sq = 0.0;
    for (size_t i = 0; i < N; ++i) {
        direction[i] = normal(rng);
        norm_sq += direction[i] * direction[i];
    }
    const double norm = std::sqrt(std::max(norm_sq, 1e-12));
    const double radius = std::pow(unit(rng), 1.0 / static_cast<double>(N));
    const double scale_sqrt = std::sqrt(scale);
    for (size_t i = 0; i < N; ++i) {
        direction[i] = direction[i] / norm * radius * scale_sqrt;
    }
    return direction;
}

double chain_wall_distance(const std::array<double, ChainMassModel::NX>& state) {
    double min_dist = std::numeric_limits<double>::infinity();
    for (int i = 0; i <= ChainMassModel::M; ++i) {
        min_dist = std::min(min_dist, state[3 * i + 1] - chain_mass_y_pos_wall);
    }
    return min_dist;
}

std::vector<ChainStep> run_chain_case(const Args& args) {
    SolverConfig config = make_solver_config();
    config.max_iters = 60;
    MiniSolver<ChainMassModel, 64> warmup_solver(40, Backend::CPU_SERIAL, config);
    warmup_solver.set_dt(chain_mass_ts);
    std::array<double, ChainMassModel::NX> warm_state{};
    std::copy(chain_mass_xrest.begin(), chain_mass_xrest.end(), warm_state.begin());
    MSVec<double, ChainMassModel::NP> zero_p;
    zero_p.setZero();
    for (int i = 0; i < 5; ++i) {
        const auto next = ChainMassModel::integrate(
            to_msvec<ChainMassModel::NX>(warm_state),
            to_msvec<ChainMassModel::NU>(chain_mass_u_init),
            zero_p,
            chain_mass_ts,
            IntegratorType::RK4_EXPLICIT);
        warm_state = to_array(next);
    }
    initialize_chain_solver(warmup_solver, warm_state);
    set_chain_cost_parameters(warmup_solver);
    for (int i = 0; i < args.warmup_runs; ++i) {
        warmup_solver.solve();
        shift_primal_guess(warmup_solver);
        warmup_solver.set_initial_state(std::vector<double>(warm_state.begin(), warm_state.end()));
        warmup_solver.reset(ResetOption::ALG_STATE);
        set_chain_cost_parameters(warmup_solver);
    }

    MiniSolver<ChainMassModel, 64> solver(40, Backend::CPU_SERIAL, config);
    solver.set_dt(chain_mass_ts);
    std::array<double, ChainMassModel::NX> current_state{};
    std::copy(chain_mass_xrest.begin(), chain_mass_xrest.end(), current_state.begin());
    for (int i = 0; i < 5; ++i) {
        const auto next = ChainMassModel::integrate(
            to_msvec<ChainMassModel::NX>(current_state),
            to_msvec<ChainMassModel::NU>(chain_mass_u_init),
            zero_p,
            chain_mass_ts,
            IntegratorType::RK4_EXPLICIT);
        current_state = to_array(next);
    }
    initialize_chain_solver(solver, current_state);
    set_chain_cost_parameters(solver);

    std::vector<ChainStep> results;
    results.reserve(static_cast<size_t>(args.steps));
    std::mt19937 rng(static_cast<uint32_t>(generated::chain_mass_seed));
    for (int step = 0; step < args.steps; ++step) {
        set_chain_cost_parameters(solver);

        ChainStep result;
        result.step = step;
        const auto start = std::chrono::steady_clock::now();
        result.status = solver.solve();
        const auto end = std::chrono::steady_clock::now();
        result.time_ms = std::chrono::duration<double, std::milli>(end - start).count();
        result.iterations = solver.get_iteration_count();
        result.deriv_ms = solver.get_profile_time_ms("Derivatives");
        result.riccati_ms = solver.get_profile_time_ms("Linear Solve");
        result.line_search_ms = solver.get_profile_time_ms("Line Search");
        result.total_cost_value = total_cost(solver);
        result.max_constraint_violation_value = max_positive_constraint(solver);

        auto disturbance = sample_uniform_ellipsoid<ChainMassModel::DIST_DIM>(
            rng,
            generated::chain_mass_perturb_scale);
        MSVec<double, ChainMassModel::NP> plant_p;
        plant_p.setZero();
        for (int i = 0; i < ChainMassModel::DIST_DIM; ++i) {
            plant_p(i) = disturbance[static_cast<size_t>(i)];
        }
        const auto next = ChainMassModel::integrate(
            to_msvec<ChainMassModel::NX>(current_state),
            to_msvec<ChainMassModel::NU>(solver.get_control(0)),
            plant_p,
            chain_mass_ts,
            IntegratorType::RK4_EXPLICIT);
        current_state = to_array(next);
        result.wall_dist = chain_wall_distance(current_state);
        results.push_back(result);

        shift_primal_guess(solver);
        solver.set_initial_state(std::vector<double>(current_state.begin(), current_state.end()));
        solver.reset(ResetOption::ALG_STATE);
    }
    return results;
}

void write_chain_outputs(const Args& args, const std::vector<ChainStep>& results) {
    const std::filesystem::path csv_path(args.output_csv);
    const std::filesystem::path summary_path =
        csv_path.parent_path() / (csv_path.stem().string() + "_summary.csv");
    std::filesystem::create_directories(csv_path.parent_path());

    std::ofstream out(csv_path);
    out << "step,backend,status,time_ms,deriv_ms,riccati_ms,line_search_ms,iterations,total_cost,max_constraint_violation,wall_dist\n";
    for (const auto& result : results) {
        print_csv_line(
            out,
            result.step,
            kBackendTag,
            status_to_string(result.status),
            result.time_ms,
            result.deriv_ms,
            result.riccati_ms,
            result.line_search_ms,
            result.iterations,
            result.total_cost_value,
            result.max_constraint_violation_value,
            result.wall_dist);
    }

    std::vector<double> times;
    int success_count = 0;
    int total_iters = 0;
    double max_viol = 0.0;
    double min_wall_dist = std::numeric_limits<double>::infinity();
    for (const auto& result : results) {
        times.push_back(result.time_ms);
        total_iters += result.iterations;
        max_viol = std::max(max_viol, result.max_constraint_violation_value);
        min_wall_dist = std::min(min_wall_dist, result.wall_dist);
        if (success_status(result.status)) {
            success_count++;
        }
    }
    if (!std::isfinite(min_wall_dist)) {
        min_wall_dist = 0.0;
    }

    std::ofstream summary(summary_path);
    summary << "case_name,backend,steps,success_rate,avg_iterations,median_ms,p95_ms,max_ms,"
               "max_constraint_violation,min_wall_dist,avg_deriv_ms,avg_riccati_ms,avg_line_search_ms\n";
    print_csv_line(
        summary,
        "chain_mass",
        kBackendTag,
        results.size(),
        results.empty() ? 0.0 : static_cast<double>(success_count) / results.size(),
        results.empty() ? 0.0 : static_cast<double>(total_iters) / results.size(),
        percentile(times, 0.5),
        percentile(times, 0.95),
        times.empty() ? 0.0 : *std::max_element(times.begin(), times.end()),
        max_viol,
        min_wall_dist,
        results.empty() ? 0.0 : std::accumulate(results.begin(), results.end(), 0.0, [](double acc, const ChainStep& record) { return acc + record.deriv_ms; }) / results.size(),
        results.empty() ? 0.0 : std::accumulate(results.begin(), results.end(), 0.0, [](double acc, const ChainStep& record) { return acc + record.riccati_ms; }) / results.size(),
        results.empty() ? 0.0 : std::accumulate(results.begin(), results.end(), 0.0, [](double acc, const ChainStep& record) { return acc + record.line_search_ms; }) / results.size());

    print_common_summary("chain_mass", results, csv_path, summary_path);
    print_profile_breakdown(results);
}
#endif

}  // namespace

int run_current_case(const Args& args) {
#if defined(MINISOLVER_CASE_PENDULUM_ON_CART)
    const auto results = run_pendulum_case(args);
    write_pendulum_outputs(args, results);
    return 0;
#elif defined(MINISOLVER_CASE_RACE_CARS)
    const auto results = run_race_case(args);
    write_race_outputs(args, results);
    return 0;
#elif defined(MINISOLVER_CASE_QUADROTOR_NAV)
    const auto results = run_quad_case(args);
    write_quad_outputs(args, results);
    return 0;
#elif defined(MINISOLVER_CASE_CHAIN_MASS)
    const auto results = run_chain_case(args);
    write_chain_outputs(args, results);
    return 0;
#else
    std::cerr << "No benchmark case selected at compile time.\n";
    return 1;
#endif
}

}  // namespace minisolver_bench

int main(int argc, char** argv) {
    minisolver_bench::Args args;
    if (!minisolver_bench::parse_args(argc, argv, args)) {
        return argc > 1 ? 1 : 0;
    }
    return minisolver_bench::run_current_case(args);
}
