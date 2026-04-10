#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <string>
#include <type_traits>
#include <vector>

#include "minisolver/solver/solver.h"
#include "official_case_data.h"

#include "pendulummodel.h"

#if defined(MINISOLVER_CASE_RACE_CARS)
extern "C" {
typedef double real_t;
#include "Spatialbicycle_model_model.h"
#include "Spatialbicycle_model_constraints.h"
}
#endif

#if defined(MINISOLVER_CASE_QUADROTOR_NAV)
extern "C" {
typedef double real_t;
#include "drone_FrenSer_model.h"
#include "drone_FrenSer_constraints.h"
}
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
    const auto& traj = solver.trajectory.active();
    for (int k = 0; k <= solver.N; ++k) {
        result += traj[k].cost;
    }
    return result;
}

template <typename Model, int MAX_N>
double max_positive_constraint(const MiniSolver<Model, MAX_N>& solver) {
    double result = 0.0;
    const auto& traj = solver.trajectory.active();
    for (int k = 0; k <= solver.N; ++k) {
        for (int i = 0; i < Model::NC; ++i) {
            result = std::max(result, std::max(0.0, traj[k].g_val(i)));
        }
    }
    return result;
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

struct CasadiWorkspace {
    std::vector<const double*> arg;
    std::vector<double*> res;
    std::vector<int> iw;
    std::vector<double> w;

    explicit CasadiWorkspace(int (*work_fn)(int*, int*, int*, int*)) {
        int sz_arg = 0;
        int sz_res = 0;
        int sz_iw = 0;
        int sz_w = 0;
        work_fn(&sz_arg, &sz_res, &sz_iw, &sz_w);
        arg.resize(static_cast<size_t>(sz_arg));
        res.resize(static_cast<size_t>(sz_res));
        iw.resize(static_cast<size_t>(sz_iw));
        w.resize(static_cast<size_t>(sz_w));
    }
};

int casadi_nz_count(const int* sparsity) {
    return sparsity[2 + sparsity[1]];
}

template <int R, int C>
void decompress_csc(const int* sparsity, const double* nz_values, MSMat<double, R, C>& dense) {
    static_assert(R >= 0 && C >= 0, "invalid matrix size");
    set_zero_matrix(dense);
    const int rows = sparsity[0];
    const int cols = sparsity[1];
    const int* col_ptr = sparsity + 2;
    const int* row_ind = col_ptr + cols + 1;
    for (int col = 0; col < cols; ++col) {
        for (int k = col_ptr[col]; k < col_ptr[col + 1]; ++k) {
            dense(row_ind[k], col) = nz_values[k];
        }
    }
}

template <int N>
std::array<double, N * N> identity_seed() {
    std::array<double, N * N> seed{};
    for (int i = 0; i < N; ++i) {
        seed[static_cast<size_t>(i + N * i)] = 1.0;
    }
    return seed;
}

using CasadiFun = int (*)(const double**, double**, int*, double*, void*);
using CasadiWork = int (*)(int*, int*, int*, int*);
using CasadiSparsity = const int* (*)(int);

template <int NX, int NU, CasadiFun VDE_FUN, CasadiWork VDE_WORK, CasadiSparsity VDE_SPARSITY_OUT>
struct ExplicitGeneratedContinuousApi {
    static void eval(
        const MSVec<double, NX>& x,
        const MSVec<double, NU>& u,
        MSVec<double, NX>& f,
        MSMat<double, NX, NX>& fx,
        MSMat<double, NX, NU>& fu) {
        static thread_local CasadiWorkspace workspace(VDE_WORK);
        static thread_local const std::array<double, NX * NX> seed_x = identity_seed<NX>();
        static thread_local const std::array<double, NX * NU> seed_u = {};
        static thread_local const int* fx_sparsity = VDE_SPARSITY_OUT(1);
        static thread_local const int* fu_sparsity = VDE_SPARSITY_OUT(2);
        static thread_local std::vector<double> fx_nz(static_cast<size_t>(casadi_nz_count(fx_sparsity)));
        static thread_local std::vector<double> fu_nz(static_cast<size_t>(casadi_nz_count(fu_sparsity)));
        std::array<double, NX> f_out{};

        workspace.arg[0] = raw_ptr(x);
        workspace.arg[1] = seed_x.data();
        workspace.arg[2] = seed_u.data();
        workspace.arg[3] = raw_ptr(u);
        workspace.arg[4] = nullptr;
        workspace.res[0] = f_out.data();
        workspace.res[1] = fx_nz.data();
        workspace.res[2] = fu_nz.data();
        VDE_FUN(workspace.arg.data(), workspace.res.data(), workspace.iw.data(), workspace.w.data(), 0);

        for (int i = 0; i < NX; ++i) {
            f(i) = f_out[static_cast<size_t>(i)];
        }
        decompress_csc(fx_sparsity, fx_nz.data(), fx);
        decompress_csc(fu_sparsity, fu_nz.data(), fu);
    }
};

template <int NG, int NX, int NU, CasadiFun CONSTR_FUN, CasadiWork CONSTR_WORK, CasadiSparsity CONSTR_SPARSITY_OUT>
struct GeneratedConstraintApi {
    static void eval(
        const MSVec<double, NX>& x,
        const MSVec<double, NU>& u,
        MSVec<double, NG>& h,
        MSMat<double, NG, NX>& C,
        MSMat<double, NG, NU>& D) {
        static thread_local CasadiWorkspace workspace(CONSTR_WORK);
        static thread_local const int* jac_sparsity = CONSTR_SPARSITY_OUT(1);
        static thread_local std::vector<double> jac_nz(static_cast<size_t>(casadi_nz_count(jac_sparsity)));
        static thread_local MSMat<double, NX + NU, NG> jac_uxt;
        std::array<double, NG> h_out{};

        workspace.arg[0] = raw_ptr(x);
        workspace.arg[1] = raw_ptr(u);
        workspace.arg[2] = nullptr;
        workspace.arg[3] = nullptr;
        workspace.res[0] = h_out.data();
        workspace.res[1] = jac_nz.data();
        workspace.res[2] = nullptr;
        CONSTR_FUN(workspace.arg.data(), workspace.res.data(), workspace.iw.data(), workspace.w.data(), 0);

        for (int i = 0; i < NG; ++i) {
            h(i) = h_out[static_cast<size_t>(i)];
        }

        decompress_csc(jac_sparsity, jac_nz.data(), jac_uxt);
        set_zero_matrix(C);
        set_zero_matrix(D);
        for (int g = 0; g < NG; ++g) {
            for (int j = 0; j < NU; ++j) {
                D(g, j) = jac_uxt(j, g);
            }
            for (int j = 0; j < NX; ++j) {
                C(g, j) = jac_uxt(NU + j, g);
            }
        }
    }
};

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

#if defined(MINISOLVER_CASE_RACE_CARS)
using RaceContinuousApi = ExplicitGeneratedContinuousApi<
    6,
    2,
    &Spatialbicycle_model_expl_vde_forw,
    &Spatialbicycle_model_expl_vde_forw_work,
    &Spatialbicycle_model_expl_vde_forw_sparsity_out>;
using RaceConstraintApi = GeneratedConstraintApi<
    5,
    6,
    2,
    &Spatialbicycle_model_constr_h_fun_jac_uxt_zt,
    &Spatialbicycle_model_constr_h_fun_jac_uxt_zt_work,
    &Spatialbicycle_model_constr_h_fun_jac_uxt_zt_sparsity_out>;

struct RaceCarsModel {
    static constexpr int NX = 6;
    static constexpr int NU = 2;
    static constexpr int NC = 14;
    static constexpr int NP = 16;

    static constexpr std::array<const char*, NX> state_names = {"s", "n", "alpha", "v", "D", "delta"};
    static constexpr std::array<const char*, NU> control_names = {"derD", "derDelta"};
    static constexpr std::array<const char*, NP> param_names = filled_names<NP>("param");
    static constexpr std::array<double, NC> constraint_weights = {
        100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
        100.0, 100.0, 100.0, 0.0, 0.0, 0.0, 0.0,
    };
    static constexpr std::array<int, NC> constraint_types = {
        1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 0, 0, 0, 0,
    };

    static constexpr double along_min = -4.0;
    static constexpr double along_max = 4.0;
    static constexpr double alat_min = -4.0;
    static constexpr double alat_max = 4.0;
    static constexpr double n_min = -0.12;
    static constexpr double n_max = 0.12;
    static constexpr double throttle_min = -1.0;
    static constexpr double throttle_max = 1.0;
    static constexpr double delta_min = -0.40;
    static constexpr double delta_max = 0.40;
    static constexpr double dthrottle_min = -10.0;
    static constexpr double dthrottle_max = 10.0;
    static constexpr double ddelta_min = -2.0;
    static constexpr double ddelta_max = 2.0;

    static constexpr int P_XREF = 0;
    static constexpr int P_UREF = P_XREF + NX;
    static constexpr int P_WX = P_UREF + NU;
    static constexpr int P_WU = P_WX + NX;

    template <typename T>
    static MSVec<T, NX> dynamics_continuous(
        const MSVec<T, NX>& x,
        const MSVec<T, NU>& u,
        const MSVec<T, NP>& p) {
        static_assert(std::is_same_v<T, double>, "RaceCarsModel only supports double evaluation");
        MSVec<double, NX> f;
        MSMat<double, NX, NX> fx;
        MSMat<double, NX, NU> fu;
        RaceContinuousApi::eval(x, u, f, fx, fu);
        (void)p;
        return f;
    }

    template <typename T>
    static MSVec<T, NX> integrate(
        const MSVec<T, NX>& x,
        const MSVec<T, NU>& u,
        const MSVec<T, NP>& p,
        double dt,
        IntegratorType /*type*/) {
        static_assert(std::is_same_v<T, double>, "RaceCarsModel only supports double evaluation");
        auto eval = [](const auto& xv, const auto& uv, const auto& /*pv*/, auto& f, auto& fx, auto& fu) {
            RaceContinuousApi::eval(xv, uv, f, fx, fu);
        };
        return rk4_step<NX, NU, NP>(x, u, p, dt, eval, nullptr, nullptr);
    }

    template <typename T>
    static void compute_dynamics(KnotPoint<T, NX, NU, NC, NP>& kp, IntegratorType /*type*/, double dt) {
        static_assert(std::is_same_v<T, double>, "RaceCarsModel only supports double evaluation");
        auto eval = [](const auto& xv, const auto& uv, const auto& /*pv*/, auto& f, auto& fx, auto& fu) {
            RaceContinuousApi::eval(xv, uv, f, fx, fu);
        };
        kp.f_resid = rk4_step<NX, NU, NP>(kp.x, kp.u, kp.p, dt, eval, &kp.A, &kp.B);
    }

    template <typename T>
    static void compute_constraints(KnotPoint<T, NX, NU, NC, NP>& kp) {
        static_assert(std::is_same_v<T, double>, "RaceCarsModel only supports double evaluation");
        MSVec<double, 5> h;
        MSMat<double, 5, NX> C_h;
        MSMat<double, 5, NU> D_h;
        RaceConstraintApi::eval(kp.x, kp.u, h, C_h, D_h);

        kp.g_val(0) = h(0) - along_max;
        kp.g_val(1) = along_min - h(0);
        kp.g_val(2) = h(1) - alat_max;
        kp.g_val(3) = alat_min - h(1);
        kp.g_val(4) = h(2) - n_max;
        kp.g_val(5) = n_min - h(2);
        kp.g_val(6) = h(3) - throttle_max;
        kp.g_val(7) = throttle_min - h(3);
        kp.g_val(8) = h(4) - delta_max;
        kp.g_val(9) = delta_min - h(4);
        kp.g_val(10) = kp.u(0) - dthrottle_max;
        kp.g_val(11) = dthrottle_min - kp.u(0);
        kp.g_val(12) = kp.u(1) - ddelta_max;
        kp.g_val(13) = ddelta_min - kp.u(1);

        set_zero_matrix(kp.C);
        set_zero_matrix(kp.D);
        for (int j = 0; j < NX; ++j) {
            kp.C(0, j) = C_h(0, j);
            kp.C(1, j) = -C_h(0, j);
            kp.C(2, j) = C_h(1, j);
            kp.C(3, j) = -C_h(1, j);
            kp.C(4, j) = C_h(2, j);
            kp.C(5, j) = -C_h(2, j);
            kp.C(6, j) = C_h(3, j);
            kp.C(7, j) = -C_h(3, j);
            kp.C(8, j) = C_h(4, j);
            kp.C(9, j) = -C_h(4, j);
        }
        for (int j = 0; j < NU; ++j) {
            kp.D(0, j) = D_h(0, j);
            kp.D(1, j) = -D_h(0, j);
            kp.D(2, j) = D_h(1, j);
            kp.D(3, j) = -D_h(1, j);
            kp.D(4, j) = D_h(2, j);
            kp.D(5, j) = -D_h(2, j);
            kp.D(6, j) = D_h(3, j);
            kp.D(7, j) = -D_h(3, j);
            kp.D(8, j) = D_h(4, j);
            kp.D(9, j) = -D_h(4, j);
        }
        kp.D(10, 0) = 1.0;
        kp.D(11, 0) = -1.0;
        kp.D(12, 1) = 1.0;
        kp.D(13, 1) = -1.0;
    }

    template <typename T>
    static void compute_cost_exact(KnotPoint<T, NX, NU, NC, NP>& kp) {
        static_assert(std::is_same_v<T, double>, "RaceCarsModel only supports double evaluation");
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
            const double ref = kp.p(P_UREF + i);
            const double weight = kp.p(P_WU + i);
            const double diff = kp.u(i) - ref;
            kp.cost += 0.5 * weight * diff * diff;
            kp.r(i) = weight * diff;
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

#if defined(MINISOLVER_CASE_QUADROTOR_NAV)
using QuadContinuousApi = ExplicitGeneratedContinuousApi<
    20,
    4,
    &drone_FrenSer_expl_vde_forw,
    &drone_FrenSer_expl_vde_forw_work,
    &drone_FrenSer_expl_vde_forw_sparsity_out>;
using QuadConstraintApi = GeneratedConstraintApi<
    1,
    20,
    4,
    &drone_FrenSer_constr_h_fun_jac_uxt_zt,
    &drone_FrenSer_constr_h_fun_jac_uxt_zt_work,
    &drone_FrenSer_constr_h_fun_jac_uxt_zt_sparsity_out>;

struct QuadrotorNavModel {
    static constexpr int NX = 20;
    static constexpr int NU = 4;
    static constexpr int NC = 1;
    static constexpr int NP = 48;

    static constexpr std::array<const char*, NX> state_names = {
        "s", "n", "b",
        "q1", "q2", "q3", "q4",
        "sDot", "nDot", "bDot",
        "wr", "wp", "wy",
        "vx", "vy", "vz",
        "ohm1", "ohm2", "ohm3", "ohm4",
    };
    static constexpr std::array<const char*, NU> control_names = {"alpha1", "alpha2", "alpha3", "alpha4"};
    static constexpr std::array<const char*, NP> param_names = filled_names<NP>("param");
    static constexpr std::array<double, NC> constraint_weights = {0.0};
    static constexpr std::array<int, NC> constraint_types = {0};

    static constexpr int P_XREF = 0;
    static constexpr int P_UREF = P_XREF + NX;
    static constexpr int P_WX = P_UREF + NU;
    static constexpr int P_WU = P_WX + NX;

    template <typename T>
    static MSVec<T, NX> dynamics_continuous(
        const MSVec<T, NX>& x,
        const MSVec<T, NU>& u,
        const MSVec<T, NP>& p) {
        static_assert(std::is_same_v<T, double>, "QuadrotorNavModel only supports double evaluation");
        MSVec<double, NX> f;
        MSMat<double, NX, NX> fx;
        MSMat<double, NX, NU> fu;
        QuadContinuousApi::eval(x, u, f, fx, fu);
        (void)p;
        return f;
    }

    template <typename T>
    static MSVec<T, NX> integrate(
        const MSVec<T, NX>& x,
        const MSVec<T, NU>& u,
        const MSVec<T, NP>& p,
        double dt,
        IntegratorType /*type*/) {
        static_assert(std::is_same_v<T, double>, "QuadrotorNavModel only supports double evaluation");
        auto eval = [](const auto& xv, const auto& uv, const auto& /*pv*/, auto& f, auto& fx, auto& fu) {
            QuadContinuousApi::eval(xv, uv, f, fx, fu);
        };
        return rk4_step<NX, NU, NP>(x, u, p, dt, eval, nullptr, nullptr);
    }

    template <typename T>
    static void compute_dynamics(KnotPoint<T, NX, NU, NC, NP>& kp, IntegratorType /*type*/, double dt) {
        static_assert(std::is_same_v<T, double>, "QuadrotorNavModel only supports double evaluation");
        auto eval = [](const auto& xv, const auto& uv, const auto& /*pv*/, auto& f, auto& fx, auto& fu) {
            QuadContinuousApi::eval(xv, uv, f, fx, fu);
        };
        kp.f_resid = rk4_step<NX, NU, NP>(kp.x, kp.u, kp.p, dt, eval, &kp.A, &kp.B);
    }

    template <typename T>
    static void compute_constraints(KnotPoint<T, NX, NU, NC, NP>& kp) {
        static_assert(std::is_same_v<T, double>, "QuadrotorNavModel only supports double evaluation");
        MSVec<double, 1> h;
        MSMat<double, 1, NX> C_h;
        MSMat<double, 1, NU> D_h;
        QuadConstraintApi::eval(kp.x, kp.u, h, C_h, D_h);
        kp.g_val(0) = h(0) - 1.0;
        kp.C.row(0) = C_h.row(0);
        kp.D.row(0) = D_h.row(0);
    }

    template <typename T>
    static void compute_cost_exact(KnotPoint<T, NX, NU, NC, NP>& kp) {
        static_assert(std::is_same_v<T, double>, "QuadrotorNavModel only supports double evaluation");
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
            const double ref = kp.p(P_UREF + i);
            const double weight = kp.p(P_WU + i);
            const double diff = kp.u(i) - ref;
            kp.cost += 0.5 * weight * diff * diff;
            kp.r(i) = weight * diff;
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
    double total_cost_value = 0.0;
    double max_constraint_violation_value = 0.0;
    double s = 0.0;
    double v = 0.0;
};

struct QuadStep {
    int step = 0;
    SolverStatus status = SolverStatus::UNSOLVED;
    int iterations = 0;
    double time_ms = 0.0;
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

void initialize_pendulum_solver(MiniSolver<PendulumModel, 80>& solver) {
    solver.set_initial_state("x", 0.0);
    solver.set_initial_state("theta", M_PI);
    solver.set_initial_state("v", 0.0);
    solver.set_initial_state("omega", 0.0);
    for (int k = 0; k < solver.N; ++k) {
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
    result.iterations = solver.current_iter;
    result.force = solver.get_control(0, 0);
    result.state = to_array<4>(solver.get_state(0));
    result.total_cost_value = total_cost(solver);
    result.max_constraint_violation_value = max_positive_constraint(solver);
    return result;
}

std::vector<PendulumStep> run_pendulum_case(const Args& args) {
    SolverConfig config = make_solver_config();
    config.enable_rti = true;
    config.max_iters = 1;
    config.tol_con = 1e-2;
    config.tol_dual = 1e-2;
    MiniSolver<PendulumModel, 80> warmup_solver(pendulum_horizon, Backend::CPU_SERIAL, config);
    warmup_solver.set_dt(pendulum_tf / static_cast<double>(pendulum_horizon));
    initialize_pendulum_solver(warmup_solver);
    for (int i = 0; i < args.warmup_runs; ++i) {
        warmup_solver.solve();
        warmup_solver.shift_trajectory();
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

        solver.shift_trajectory();
        solver.set_initial_state(std::vector<double>(current_state.begin(), current_state.end()));
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
    out << "step,backend,status,time_ms,iterations,force,total_cost,max_constraint_violation,"
           "cart_x,pole_theta,cart_v,pole_omega\n";
    for (const auto& result : results) {
        print_csv_line(
            out,
            result.step,
            kBackendTag,
            status_to_string(result.status),
            result.time_ms,
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
               "max_constraint_violation,final_theta_abs\n";
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
        std::abs(final_theta));

    print_common_summary("pendulum_on_cart", results, csv_path, summary_path);
}

#if defined(MINISOLVER_CASE_RACE_CARS)
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

    for (int k = 0; k < solver.N; ++k) {
        const double sref_k = s0 + (sref - s0) * static_cast<double>(k) / static_cast<double>(solver.N);
        std::array<double, RaceCarsModel::NX> xref = {sref_k, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::array<double, RaceCarsModel::NU> uref = {0.0, 0.0};
        set_stage_array_parameters(solver, k, race_p_xref, xref);
        set_stage_array_parameters(solver, k, race_p_uref, uref);
        set_stage_array_parameters(solver, k, race_p_wx, stage_wx);
        set_stage_array_parameters(solver, k, race_p_wu, stage_wu);
    }

    std::array<double, RaceCarsModel::NX> xref_terminal = {sref, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, RaceCarsModel::NU> uref_terminal = {0.0, 0.0};
    set_stage_array_parameters(solver, solver.N, race_p_xref, xref_terminal);
    set_stage_array_parameters(solver, solver.N, race_p_uref, uref_terminal);
    set_stage_array_parameters(solver, solver.N, race_p_wx, terminal_wx);
    set_stage_array_parameters(solver, solver.N, race_p_wu, terminal_wu);
}

void initialize_race_solver(MiniSolver<RaceCarsModel, 80>& solver, const std::array<double, 6>& x0) {
    solver.set_initial_state(std::vector<double>(x0.begin(), x0.end()));
    for (int k = 0; k < solver.N; ++k) {
        solver.set_control_guess(k, 0, 0.0);
        solver.set_control_guess(k, 1, 0.0);
    }
    solver.rollout_dynamics();
}

std::vector<RaceStep> run_race_case(const Args& args) {
    SolverConfig config = make_solver_config();
    config.enable_rti = true;
    config.max_iters = 1;
    config.tol_con = 1e-2;
    config.tol_dual = 1e-2;
    MiniSolver<RaceCarsModel, 80> warmup_solver(race_cars_horizon, Backend::CPU_SERIAL, config);
    warmup_solver.set_dt(race_cars_tf / static_cast<double>(race_cars_horizon));
    initialize_race_solver(warmup_solver, race_cars_x0);
    auto warm_state = race_cars_x0;
    for (int i = 0; i < args.warmup_runs; ++i) {
        set_race_cost_parameters(warmup_solver, warm_state[0]);
        warmup_solver.solve();
        warm_state = to_array<6>(warmup_solver.get_state(1));
        warmup_solver.shift_trajectory();
        warmup_solver.set_initial_state(std::vector<double>(warm_state.begin(), warm_state.end()));
        warmup_solver.reset(ResetOption::ALG_STATE);
    }

    MiniSolver<RaceCarsModel, 80> solver(race_cars_horizon, Backend::CPU_SERIAL, config);
    solver.set_dt(race_cars_tf / static_cast<double>(race_cars_horizon));
    initialize_race_solver(solver, race_cars_x0);

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
        result.iterations = solver.current_iter;
        result.total_cost_value = total_cost(solver);
        result.max_constraint_violation_value = max_positive_constraint(solver);
        result.s = solver.get_state(0, 0);
        result.v = solver.get_state(0, 3);
        results.push_back(result);

        current_state = to_array<6>(solver.get_state(1));
        solver.shift_trajectory();
        solver.set_initial_state(std::vector<double>(current_state.begin(), current_state.end()));
        solver.reset(ResetOption::ALG_STATE);

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
    out << "step,backend,status,time_ms,iterations,total_cost,max_constraint_violation,s,v\n";
    for (const auto& result : results) {
        print_csv_line(
            out,
            result.step,
            kBackendTag,
            status_to_string(result.status),
            result.time_ms,
            result.iterations,
            result.total_cost_value,
            result.max_constraint_violation_value,
            result.s,
            result.v);
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
               "max_constraint_violation,avg_speed\n";
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
        avg_speed);

    print_common_summary("race_cars", results, csv_path, summary_path);
}
#endif

#if defined(MINISOLVER_CASE_QUADROTOR_NAV)
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
    for (int k = 0; k < solver.N; ++k) {
        const double sref_k =
            s0 + (sref - s0) * static_cast<double>(k) / static_cast<double>(solver.N);
        std::array<double, QuadrotorNavModel::NX> xref = {
            sref_k, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            sref_dot, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            quadrotor_nav_u_hov, quadrotor_nav_u_hov, quadrotor_nav_u_hov, quadrotor_nav_u_hov,
        };
        std::array<double, QuadrotorNavModel::NU> uref = {0.0, 0.0, 0.0, 0.0};
        set_stage_array_parameters(solver, k, quad_p_xref, xref);
        set_stage_array_parameters(solver, k, quad_p_uref, uref);
        set_stage_array_parameters(solver, k, quad_p_wx, stage_wx);
        set_stage_array_parameters(solver, k, quad_p_wu, stage_wu);
    }

    const double terminal_sref =
        s0 + (sref - s0) * static_cast<double>(solver.N - 1) / static_cast<double>(solver.N);
    std::array<double, QuadrotorNavModel::NX> xref_terminal = {
        terminal_sref, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
        sref_dot, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        quadrotor_nav_u_hov, quadrotor_nav_u_hov, quadrotor_nav_u_hov, quadrotor_nav_u_hov,
    };
    std::array<double, QuadrotorNavModel::NU> uref_terminal = {0.0, 0.0, 0.0, 0.0};
    set_stage_array_parameters(solver, solver.N, quad_p_xref, xref_terminal);
    set_stage_array_parameters(solver, solver.N, quad_p_uref, uref_terminal);
    set_stage_array_parameters(solver, solver.N, quad_p_wx, terminal_wx);
    set_stage_array_parameters(solver, solver.N, quad_p_wu, terminal_wu);
}

void initialize_quad_solver(MiniSolver<QuadrotorNavModel, 80>& solver, const std::array<double, 20>& x0) {
    solver.set_initial_state(std::vector<double>(x0.begin(), x0.end()));
    for (int k = 0; k < solver.N; ++k) {
        for (int j = 0; j < QuadrotorNavModel::NU; ++j) {
            solver.set_control_guess(k, j, 0.0);
        }
    }
    solver.rollout_dynamics();
}

std::vector<QuadStep> run_quad_case(const Args& args) {
    SolverConfig config = make_solver_config();
    config.enable_rti = true;
    config.max_iters = 1;
    config.tol_con = 1e-2;
    config.tol_dual = 1e-2;
    MiniSolver<QuadrotorNavModel, 80> warmup_solver(quadrotor_nav_horizon, Backend::CPU_SERIAL, config);
    warmup_solver.set_dt(quadrotor_nav_tf / static_cast<double>(quadrotor_nav_horizon));
    initialize_quad_solver(warmup_solver, quadrotor_nav_init_zeta);
    auto warm_state = quadrotor_nav_init_zeta;
    for (int i = 0; i < args.warmup_runs; ++i) {
        set_quad_cost_parameters(warmup_solver, warm_state[0]);
        warmup_solver.solve();
        MSVec<double, QuadrotorNavModel::NP> p;
        p.setZero();
        const auto next = QuadrotorNavModel::integrate(
            to_msvec<QuadrotorNavModel::NX>(warm_state),
            to_msvec<QuadrotorNavModel::NU>(warmup_solver.get_control(0)),
            p,
            quadrotor_nav_tf / static_cast<double>(quadrotor_nav_horizon),
            IntegratorType::RK4_EXPLICIT);
        warm_state = to_array(next);
        warmup_solver.shift_trajectory();
        warmup_solver.set_initial_state(std::vector<double>(warm_state.begin(), warm_state.end()));
        warmup_solver.reset(ResetOption::ALG_STATE);
    }

    MiniSolver<QuadrotorNavModel, 80> solver(quadrotor_nav_horizon, Backend::CPU_SERIAL, config);
    solver.set_dt(quadrotor_nav_tf / static_cast<double>(quadrotor_nav_horizon));
    initialize_quad_solver(solver, quadrotor_nav_init_zeta);

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
        result.iterations = solver.current_iter;
        result.total_cost_value = total_cost(solver);
        result.max_constraint_violation_value = max_positive_constraint(solver);

        MSVec<double, QuadrotorNavModel::NP> p;
        p.setZero();
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

        solver.shift_trajectory();
        solver.set_initial_state(std::vector<double>(current_state.begin(), current_state.end()));
        solver.reset(ResetOption::ALG_STATE);
    }
    return results;
}

void write_quad_outputs(const Args& args, const std::vector<QuadStep>& results) {
    const std::filesystem::path csv_path(args.output_csv);
    const std::filesystem::path summary_path =
        csv_path.parent_path() / (csv_path.stem().string() + "_summary.csv");
    std::filesystem::create_directories(csv_path.parent_path());

    std::ofstream out(csv_path);
    out << "step,backend,status,time_ms,iterations,total_cost,max_constraint_violation,abs_n,abs_b\n";
    for (const auto& result : results) {
        print_csv_line(
            out,
            result.step,
            kBackendTag,
            status_to_string(result.status),
            result.time_ms,
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
               "max_constraint_violation,avg_abs_n,avg_abs_b\n";
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
        avg_abs_b);

    print_common_summary("quadrotor_nav", results, csv_path, summary_path);
}
#endif

#if defined(MINISOLVER_CASE_CHAIN_MASS)
void set_chain_cost_parameters(MiniSolver<ChainMassModel, 64>& solver) {
    std::array<double, ChainMassModel::NX> xref{};
    std::copy(chain_mass_xrest.begin(), chain_mass_xrest.end(), xref.begin());

    std::array<double, ChainMassModel::NX> stage_wx{};
    stage_wx.fill(2.0);
    const double strong_penalty = static_cast<double>(chain_mass_m + 1);
    stage_wx[3 * chain_mass_m + 0] = 2.0 * strong_penalty;
    stage_wx[3 * chain_mass_m + 1] = 2.0 * strong_penalty;
    stage_wx[3 * chain_mass_m + 2] = 2.0 * strong_penalty;
    std::array<double, ChainMassModel::NU> stage_wu = {0.02, 0.02, 0.02};
    std::array<double, ChainMassModel::NU> terminal_wu = {0.0, 0.0, 0.0};
    std::array<double, chain_mass_dist_dim> zero_dist{};

    for (int k = 0; k < solver.N; ++k) {
        set_stage_array_parameters(solver, k, chain_mass_p_dist, zero_dist);
        set_stage_array_parameters(solver, k, chain_mass_p_xref, xref);
        set_stage_array_parameters(solver, k, chain_mass_p_wx, stage_wx);
        set_stage_array_parameters(solver, k, chain_mass_p_wu, stage_wu);
    }

    set_stage_array_parameters(solver, solver.N, chain_mass_p_dist, zero_dist);
    set_stage_array_parameters(solver, solver.N, chain_mass_p_xref, xref);
    set_stage_array_parameters(solver, solver.N, chain_mass_p_wx, stage_wx);
    set_stage_array_parameters(solver, solver.N, chain_mass_p_wu, terminal_wu);
}

void initialize_chain_solver(MiniSolver<ChainMassModel, 64>& solver, const std::array<double, ChainMassModel::NX>& x0) {
    solver.set_initial_state(std::vector<double>(x0.begin(), x0.end()));
    for (int k = 0; k < solver.N; ++k) {
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
    for (int i = 0; i < chain_mass_m; ++i) {
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
        warmup_solver.shift_trajectory();
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
        result.iterations = solver.current_iter;
        result.total_cost_value = total_cost(solver);
        result.max_constraint_violation_value = max_positive_constraint(solver);

        auto disturbance = sample_uniform_ellipsoid<chain_mass_dist_dim>(
            rng,
            generated::chain_mass_perturb_scale);
        MSVec<double, ChainMassModel::NP> plant_p;
        plant_p.setZero();
        for (int i = 0; i < chain_mass_dist_dim; ++i) {
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

        solver.shift_trajectory();
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
    out << "step,backend,status,time_ms,iterations,total_cost,max_constraint_violation,wall_dist\n";
    for (const auto& result : results) {
        print_csv_line(
            out,
            result.step,
            kBackendTag,
            status_to_string(result.status),
            result.time_ms,
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
               "max_constraint_violation,min_wall_dist\n";
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
        min_wall_dist);

    print_common_summary("chain_mass", results, csv_path, summary_path);
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
