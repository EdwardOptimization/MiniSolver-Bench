#pragma once
#include "minisolver/core/types.h"
#include "minisolver/core/solver_options.h"
#include "minisolver/matrix/matrix_defs.h"
#include "minisolver/integrator/numerical_jacobian.h"
#include <cstdint>
#include <cmath>
#include <string>
#include <array>
#include <stdexcept>

namespace minisolver {

struct DoubleIntegrator3DTrackingModel {
    // --- Constants ---
    static const int NX=6;
    static const int NU=3;
    static const int NC=6;
    static const int NP=6;

    static constexpr std::uint64_t model_fingerprint = 0x9f664c9e7d7936cbull;

    static constexpr IntegratorType generated_integrator = IntegratorType::RK4_EXPLICIT;

    static constexpr std::array<bool, NC> constraint_has_l1 = {false, false, false, false, false, false};
    static constexpr std::array<bool, NC> constraint_has_l2 = {false, false, false, false, false, false};
    static constexpr bool any_l1_constraints = false;
    static constexpr bool any_l2_constraints = false;


    // --- Name Arrays (for Map Construction) ---
    static constexpr std::array<const char*, NX> state_names = {
        "x",
        "y",
        "z",
        "vx",
        "vy",
        "vz",
    };

    static constexpr std::array<const char*, NU> control_names = {
        "ax",
        "ay",
        "az",
    };

    static constexpr std::array<const char*, NP> param_names = {
        "x_ref",
        "y_ref",
        "z_ref",
        "vx_ref",
        "vy_ref",
        "vz_ref",
    };


    // --- Continuous Dynamics ---
    template<typename T>
    static MSVec<T, NX> dynamics_continuous(
        const MSVec<T, NX>& x_in,
        const MSVec<T, NU>& u_in,
        const MSVec<T, NP>& p_in)
    {
        T vx = x_in(3);
        T vy = x_in(4);
        T vz = x_in(5);
        T ax = u_in(0);
        T ay = u_in(1);
        T az = u_in(2);
        (void)p_in;

        MSVec<T, NX> xdot;
        xdot(0) = vx;
        xdot(1) = vy;
        xdot(2) = vz;
        xdot(3) = ax;
        xdot(4) = ay;
        xdot(5) = az;
        return xdot;

    }

    // --- Continuous Dynamics Jacobians (for implicit integrators) ---
    template<typename T>
    static ContinuousJacobians<T, NX, NU> jacobian_continuous(
        const MSVec<T, NX>& x_in,
        const MSVec<T, NU>& u_in,
        const MSVec<T, NP>& p_in)
    {
        (void)x_in;
        (void)u_in;
        (void)p_in;

        ContinuousJacobians<T, NX, NU> jac;

        // Clear continuous Jacobian packets; nonzero entries are assigned below.
        jac.Jx.setZero();
        jac.Ju.setZero();

        // Jx = df/dx
        jac.Jx(0,3) = 1;
        jac.Jx(1,4) = 1;
        jac.Jx(2,5) = 1;

        // Ju = df/du
        jac.Ju(3,0) = 1;
        jac.Ju(4,1) = 1;
        jac.Ju(5,2) = 1;

        return jac;

    }

    // --- Integrator Interface ---
    template<typename T>
    static MSVec<T, NX> integrate(
        const MSVec<T, NX>& x_in,
        const MSVec<T, NU>& u_in,
        const MSVec<T, NP>& p_in,
        double dt,
        IntegratorType type)
    {
        switch(type) {
            case IntegratorType::EULER_EXPLICIT:
                return x_in + dynamics_continuous(x_in, u_in, p_in) * dt;

            case IntegratorType::RK2_EXPLICIT:
            {
               auto k1 = dynamics_continuous(x_in, u_in, p_in);
               auto k2 = dynamics_continuous<T>(x_in + k1 * (0.5 * dt), u_in, p_in);
               return x_in + k2 * dt;
            }

            case IntegratorType::EULER_IMPLICIT:
            {
                // Simple Fixed-Point Iteration for x_next = x + f(x_next, u) * dt
                MSVec<T, NX> x_next = x_in; // Guess
                for(int i=0; i<5; ++i) {
                    x_next = x_in + dynamics_continuous(x_next, u_in, p_in) * dt;
                }
                return x_next;
            }

            case IntegratorType::RK2_IMPLICIT:
            {
                // Implicit Midpoint: k = f(x + 0.5*dt*k). x_next = x + dt*k
                MSVec<T, NX> k = dynamics_continuous(x_in, u_in, p_in); // Guess k0
                for(int i=0; i<5; ++i) {
                    k = dynamics_continuous<T>(x_in + k * (0.5 * dt), u_in, p_in);
                }
                return x_in + k * dt;
            }

            case IntegratorType::RK4_EXPLICIT:
            case IntegratorType::RK4_IMPLICIT:
            {
               auto k1 = dynamics_continuous(x_in, u_in, p_in);
               auto k2 = dynamics_continuous<T>(x_in + k1 * (0.5 * dt), u_in, p_in);
               auto k3 = dynamics_continuous<T>(x_in + k2 * (0.5 * dt), u_in, p_in);
               auto k4 = dynamics_continuous<T>(x_in + k3 * dt, u_in, p_in);
               return x_in + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);
            }

            case IntegratorType::DISCRETE:
                throw std::invalid_argument("DISCRETE integrator requires Next(state) dynamics");
        }
        throw std::invalid_argument("Unsupported integrator type");

    }

    // --- 1. Compute Dynamics (f_resid, A, B) ---
    template<typename T>
    static void compute_dynamics(KnotPoint<T,NX,NU,NC,NP>& kp, IntegratorType type, double dt) {
        T x = kp.x(0);
        T y = kp.x(1);
        T z = kp.x(2);
        T vx = kp.x(3);
        T vy = kp.x(4);
        T vz = kp.x(5);
        T ax = kp.u(0);
        T ay = kp.u(1);
        T az = kp.u(2);

        switch(type) {
            case IntegratorType::EULER_EXPLICIT:
            case IntegratorType::EULER_IMPLICIT:
            {
                kp.f_resid(0) = dt*vx + x;
                kp.f_resid(1) = dt*vy + y;
                kp.f_resid(2) = dt*vz + z;
                kp.f_resid(3) = ax*dt + vx;
                kp.f_resid(4) = ay*dt + vy;
                kp.f_resid(5) = az*dt + vz;

                // Clear dynamics Jacobian A; nonzero entries are assigned below.
                kp.A.setZero();
                kp.A(0,0) = 1;
                kp.A(0,3) = dt;
                kp.A(1,1) = 1;
                kp.A(1,4) = dt;
                kp.A(2,2) = 1;
                kp.A(2,5) = dt;
                kp.A(3,3) = 1;
                kp.A(4,4) = 1;
                kp.A(5,5) = 1;

                // Clear dynamics Jacobian B; nonzero entries are assigned below.
                kp.B.setZero();
                kp.B(3,0) = dt;
                kp.B(4,1) = dt;
                kp.B(5,2) = dt;
                break;
            }
            case IntegratorType::RK2_EXPLICIT:
            case IntegratorType::RK2_IMPLICIT:
            {
                T tmp_d0 = ax*dt;
                T tmp_d1 = ay*dt;
                T tmp_d2 = az*dt;
                T tmp_d3 = 0.5*pow(dt, 2);
                kp.f_resid(0) = dt*(0.5*tmp_d0 + vx) + x;
                kp.f_resid(1) = dt*(0.5*tmp_d1 + vy) + y;
                kp.f_resid(2) = dt*(0.5*tmp_d2 + vz) + z;
                kp.f_resid(3) = tmp_d0 + vx;
                kp.f_resid(4) = tmp_d1 + vy;
                kp.f_resid(5) = tmp_d2 + vz;

                // Clear dynamics Jacobian A; nonzero entries are assigned below.
                kp.A.setZero();
                kp.A(0,0) = 1;
                kp.A(0,3) = dt;
                kp.A(1,1) = 1;
                kp.A(1,4) = dt;
                kp.A(2,2) = 1;
                kp.A(2,5) = dt;
                kp.A(3,3) = 1;
                kp.A(4,4) = 1;
                kp.A(5,5) = 1;

                // Clear dynamics Jacobian B; nonzero entries are assigned below.
                kp.B.setZero();
                kp.B(0,0) = tmp_d3;
                kp.B(1,1) = tmp_d3;
                kp.B(2,2) = tmp_d3;
                kp.B(3,0) = dt;
                kp.B(4,1) = dt;
                kp.B(5,2) = dt;
                break;
            }
            case IntegratorType::RK4_EXPLICIT:
            case IntegratorType::RK4_IMPLICIT:
            {
                T tmp_d0 = 3.0*dt;
                T tmp_d1 = 0.16666666666666666*dt;
                T tmp_d2 = 1.0*dt;
                T tmp_d3 = 0.5*pow(dt, 2);
                kp.f_resid(0) = tmp_d1*(ax*tmp_d0 + 6*vx) + x;
                kp.f_resid(1) = tmp_d1*(ay*tmp_d0 + 6*vy) + y;
                kp.f_resid(2) = tmp_d1*(az*tmp_d0 + 6*vz) + z;
                kp.f_resid(3) = ax*tmp_d2 + vx;
                kp.f_resid(4) = ay*tmp_d2 + vy;
                kp.f_resid(5) = az*tmp_d2 + vz;

                // Clear dynamics Jacobian A; nonzero entries are assigned below.
                kp.A.setZero();
                kp.A(0,0) = 1;
                kp.A(0,3) = tmp_d2;
                kp.A(1,1) = 1;
                kp.A(1,4) = tmp_d2;
                kp.A(2,2) = 1;
                kp.A(2,5) = tmp_d2;
                kp.A(3,3) = 1;
                kp.A(4,4) = 1;
                kp.A(5,5) = 1;

                // Clear dynamics Jacobian B; nonzero entries are assigned below.
                kp.B.setZero();
                kp.B(0,0) = tmp_d3;
                kp.B(1,1) = tmp_d3;
                kp.B(2,2) = tmp_d3;
                kp.B(3,0) = tmp_d2;
                kp.B(4,1) = tmp_d2;
                kp.B(5,2) = tmp_d2;
                break;
            }
            case IntegratorType::DISCRETE:
                throw std::invalid_argument("DISCRETE integrator requires Next(state) dynamics");
        }
    }

    // --- 1.5 Update Soft Constraint Weights ---
    template<typename T>
    static void update_soft_constraint_weights(KnotPoint<T,NX,NU,NC,NP>& kp) {
        (void)kp;
    }


    // --- 2. Compute QP/IPM Constraints (g_val, C, D) ---
    template<typename T>
    static void compute_qp_constraints(KnotPoint<T,NX,NU,NC,NP>& kp) {
        T ax = kp.u(0);
        T ay = kp.u(1);
        T az = kp.u(2);

        // --- Special Constraints Pre-Calculation ---


        // Clear generated output packets; nonzero entries are assigned below.
        kp.C.setZero();
        kp.D.setZero();

        // g_val
        kp.g_val(0,0) = ax - 15.0;
        kp.g_val(1,0) = -ax - 15.0;
        kp.g_val(2,0) = ay - 15.0;
        kp.g_val(3,0) = -ay - 15.0;
        kp.g_val(4,0) = az - 15.0;
        kp.g_val(5,0) = -az - 15.0;

        // C

        // D
        kp.D(0,0) = 1;
        kp.D(1,0) = -1;
        kp.D(2,1) = 1;
        kp.D(3,1) = -1;
        kp.D(4,2) = 1;
        kp.D(5,2) = -1;

    }

    // Legacy alias for hand-written code that still calls compute_constraints().
    template<typename T>
    static void compute_constraints(KnotPoint<T,NX,NU,NC,NP>& kp) {
        compute_qp_constraints(kp);
    }

    // --- 2.1 Compute True Constraints (g_true) ---
    template<typename T>
    static void compute_true_constraints(KnotPoint<T,NX,NU,NC,NP>& kp) {
        T ax = kp.u(0);
        T ay = kp.u(1);
        T az = kp.u(2);


        // g_true
        kp.g_true(0,0) = ax - 15.0;
        kp.g_true(1,0) = -ax - 15.0;
        kp.g_true(2,0) = ay - 15.0;
        kp.g_true(3,0) = -ay - 15.0;
        kp.g_true(4,0) = az - 15.0;
        kp.g_true(5,0) = -az - 15.0;

    }

    // --- 2.5 Terminal Stage: x-only projection of QP/IPM constraints ---
    template<typename T>
    static void compute_terminal_qp_constraints(KnotPoint<T,NX,NU,NC,NP>& kp) {

        // --- Special Constraints Pre-Calculation ---


        // Clear generated output packets; nonzero entries are assigned below.
        kp.C.setZero();
        kp.D.setZero();

        // g_val
        kp.g_val(0,0) = -15.0;
        kp.g_val(1,0) = -15.0;
        kp.g_val(2,0) = -15.0;
        kp.g_val(3,0) = -15.0;
        kp.g_val(4,0) = -15.0;
        kp.g_val(5,0) = -15.0;

        // C

        // D

    }

    // Legacy alias for hand-written code that still calls compute_terminal_constraints().
    template<typename T>
    static void compute_terminal_constraints(KnotPoint<T,NX,NU,NC,NP>& kp) {
        compute_terminal_qp_constraints(kp);
    }

    // --- 2.5.1 Terminal Stage: true x-only constraint residuals ---
    template<typename T>
    static void compute_terminal_true_constraints(KnotPoint<T,NX,NU,NC,NP>& kp) {


        // g_true
        kp.g_true(0,0) = -15.0;
        kp.g_true(1,0) = -15.0;
        kp.g_true(2,0) = -15.0;
        kp.g_true(3,0) = -15.0;
        kp.g_true(4,0) = -15.0;
        kp.g_true(5,0) = -15.0;

    }

    // --- 2.6 SOC correction constraints ---
    template<typename T>
    static void compute_soc_constraints(
        const KnotPoint<T,NX,NU,NC,NP>& active_kp,
        KnotPoint<T,NX,NU,NC,NP>& trial_kp) {
        compute_qp_constraints(trial_kp);
        compute_true_constraints(trial_kp);
        (void)active_kp;

    }

    // --- 3. Compute Cost (Implemented via template for Exact/GN) ---
    template<typename T, int Mode>
    static void compute_cost_impl(KnotPoint<T,NX,NU,NC,NP>& kp) {
        T x = kp.x(0);
        T y = kp.x(1);
        T z = kp.x(2);
        T vx = kp.x(3);
        T vy = kp.x(4);
        T vz = kp.x(5);
        T ax = kp.u(0);
        T ay = kp.u(1);
        T az = kp.u(2);
        T x_ref = kp.p(0);
        T y_ref = kp.p(1);
        T z_ref = kp.p(2);
        T vx_ref = kp.p(3);
        T vy_ref = kp.p(4);
        T vz_ref = kp.p(5);


        // q
        kp.q(0,0) = 30.0*x - 30.0*x_ref;
        kp.q(1,0) = 30.0*y - 30.0*y_ref;
        kp.q(2,0) = 30.0*z - 30.0*z_ref;
        kp.q(3,0) = 5.0*vx - 5.0*vx_ref;
        kp.q(4,0) = 5.0*vy - 5.0*vy_ref;
        kp.q(5,0) = 5.0*vz - 5.0*vz_ref;

        // r
        kp.r(0,0) = 0.10000000000000001*ax;
        kp.r(1,0) = 0.10000000000000001*ay;
        kp.r(2,0) = 0.10000000000000001*az;

        // Clear Hessian packets; nonzero entries are assigned below.
        kp.Q.setZero();
        kp.R.setZero();
        kp.H.setZero();

        // Q (Mode 0=GN, 1=Exact)
        kp.Q(0,0) = 30.0;
        kp.Q(1,1) = 30.0;
        kp.Q(2,2) = 30.0;
        kp.Q(3,3) = 5.0;
        kp.Q(4,4) = 5.0;
        kp.Q(5,5) = 5.0;

        // R (Mode 0=GN, 1=Exact)
        kp.R(0,0) = 0.10000000000000001;
        kp.R(1,1) = 0.10000000000000001;
        kp.R(2,2) = 0.10000000000000001;

        // H (Mode 0=GN, 1=Exact)

        kp.cost = 0.050000000000000003*pow(ax, 2) + 0.050000000000000003*pow(ay, 2) + 0.050000000000000003*pow(az, 2) + 2.5*pow(vx - vx_ref, 2) + 2.5*pow(vy - vy_ref, 2) + 2.5*pow(vz - vz_ref, 2) + 15.0*pow(x - x_ref, 2) + 15.0*pow(y - y_ref, 2) + 15.0*pow(z - z_ref, 2);
    }

template<typename T>
    static void compute_cost_gn(KnotPoint<T,NX,NU,NC,NP>& kp) {
        compute_cost_impl<T, 0>(kp);
    }

    template<typename T>
    static void compute_cost_exact(KnotPoint<T,NX,NU,NC,NP>& kp) {
        compute_cost_impl<T, 1>(kp);
    }

    template<typename T>
    static void compute_cost(KnotPoint<T,NX,NU,NC,NP>& kp) {
        compute_cost_impl<T, 1>(kp);
    }


    // --- 3.5 Terminal Cost (u projected to zero) ---
    template<typename T, int Mode>
    static void compute_terminal_cost_impl(KnotPoint<T,NX,NU,NC,NP>& kp) {
        T x = kp.x(0);
        T y = kp.x(1);
        T z = kp.x(2);
        T vx = kp.x(3);
        T vy = kp.x(4);
        T vz = kp.x(5);
        T x_ref = kp.p(0);
        T y_ref = kp.p(1);
        T z_ref = kp.p(2);
        T vx_ref = kp.p(3);
        T vy_ref = kp.p(4);
        T vz_ref = kp.p(5);


        // Clear generated output packets; nonzero entries are assigned below.
        kp.r.setZero();

        // q
        kp.q(0,0) = 30.0*x - 30.0*x_ref;
        kp.q(1,0) = 30.0*y - 30.0*y_ref;
        kp.q(2,0) = 30.0*z - 30.0*z_ref;
        kp.q(3,0) = 5.0*vx - 5.0*vx_ref;
        kp.q(4,0) = 5.0*vy - 5.0*vy_ref;
        kp.q(5,0) = 5.0*vz - 5.0*vz_ref;

        // r

        // Clear Hessian packets; nonzero entries are assigned below.
        kp.Q.setZero();
        kp.R.setZero();
        kp.H.setZero();

        // terminal Q (Mode 0=GN, 1=Exact)
        kp.Q(0,0) = 30.0;
        kp.Q(1,1) = 30.0;
        kp.Q(2,2) = 30.0;
        kp.Q(3,3) = 5.0;
        kp.Q(4,4) = 5.0;
        kp.Q(5,5) = 5.0;

        // terminal R (Mode 0=GN, 1=Exact)

        // terminal H (Mode 0=GN, 1=Exact)

        kp.cost = 2.5*pow(vx - vx_ref, 2) + 2.5*pow(vy - vy_ref, 2) + 2.5*pow(vz - vz_ref, 2) + 15.0*pow(x - x_ref, 2) + 15.0*pow(y - y_ref, 2) + 15.0*pow(z - z_ref, 2);
    }

    template<typename T>
    static void compute_terminal_cost_gn(KnotPoint<T,NX,NU,NC,NP>& kp) {
        compute_terminal_cost_impl<T, 0>(kp);
    }

    template<typename T>
    static void compute_terminal_cost_exact(KnotPoint<T,NX,NU,NC,NP>& kp) {
        compute_terminal_cost_impl<T, 1>(kp);
    }


    // --- 4. Compute All (Convenience) ---
    template<typename T>
    static void compute(KnotPoint<T,NX,NU,NC,NP>& kp, IntegratorType type, double dt) {
        update_soft_constraint_weights(kp);
        compute_dynamics(kp, type, dt);
        compute_qp_constraints(kp);
        compute_true_constraints(kp);
        compute_cost(kp); // Default exact Hessian for backward compatibility.
    }

    template<typename T>
    static void compute_exact(KnotPoint<T,NX,NU,NC,NP>& kp, IntegratorType type, double dt) {
        update_soft_constraint_weights(kp);
        compute_dynamics(kp, type, dt);
        compute_qp_constraints(kp);
        compute_true_constraints(kp);
        compute_cost_exact(kp); // Exact Hessian
    }

    // --- 5. Sparse Kernels (Generated) ---

    // --- 6. Fused Riccati Kernel (Generated) ---
    // Updates kp.Q_bar, R_bar, H_bar, q_bar, r_bar in one go.
    // Uses Vxx, Vx from next step.
    template<typename T>
    static void compute_fused_riccati_step(
        const MSMat<T, NX, NX>& Vxx,
        const MSVec<T, NX>& Vx,
        KnotPoint<T,NX,NU,NC,NP>& kp)
    {
        T P_0_0 = Vxx(0,0);
        T P_0_1 = Vxx(0,1);
        T P_0_2 = Vxx(0,2);
        T P_0_3 = Vxx(0,3);
        T P_0_4 = Vxx(0,4);
        T P_0_5 = Vxx(0,5);
        T P_1_1 = Vxx(1,1);
        T P_1_2 = Vxx(1,2);
        T P_1_3 = Vxx(1,3);
        T P_1_4 = Vxx(1,4);
        T P_1_5 = Vxx(1,5);
        T P_2_2 = Vxx(2,2);
        T P_2_3 = Vxx(2,3);
        T P_2_4 = Vxx(2,4);
        T P_2_5 = Vxx(2,5);
        T P_3_3 = Vxx(3,3);
        T P_3_4 = Vxx(3,4);
        T P_3_5 = Vxx(3,5);
        T P_4_4 = Vxx(4,4);
        T P_4_5 = Vxx(4,5);
        T P_5_5 = Vxx(5,5);
        T p_0 = Vx(0);
        T p_1 = Vx(1);
        T p_2 = Vx(2);
        T p_3 = Vx(3);
        T p_4 = Vx(4);
        T p_5 = Vx(5);
        T A_0_0 = kp.A(0,0);
        T A_0_3 = kp.A(0,3);
        T A_1_1 = kp.A(1,1);
        T A_1_4 = kp.A(1,4);
        T A_2_2 = kp.A(2,2);
        T A_2_5 = kp.A(2,5);
        T A_3_3 = kp.A(3,3);
        T A_4_4 = kp.A(4,4);
        T A_5_5 = kp.A(5,5);
        T B_0_0 = kp.B(0,0);
        T B_1_1 = kp.B(1,1);
        T B_2_2 = kp.B(2,2);
        T B_3_0 = kp.B(3,0);
        T B_4_1 = kp.B(4,1);
        T B_5_2 = kp.B(5,2);

        // CSE Intermediate Variables
        T tmp_ric0 = A_0_0*P_0_1;
        T tmp_ric1 = A_0_0*P_0_2;
        T tmp_ric2 = A_0_3*P_0_0;
        T tmp_ric3 = A_3_3*P_0_3;
        T tmp_ric4 = A_1_1*P_1_2;
        T tmp_ric5 = A_0_3*P_0_1;
        T tmp_ric6 = A_3_3*P_1_3;
        T tmp_ric7 = A_1_4*P_1_1;
        T tmp_ric8 = A_4_4*P_1_4;
        T tmp_ric9 = A_0_3*P_0_2;
        T tmp_ric10 = A_3_3*P_2_3;
        T tmp_ric11 = A_1_4*P_1_2;
        T tmp_ric12 = A_4_4*P_2_4;
        T tmp_ric13 = A_2_5*P_2_2;
        T tmp_ric14 = A_5_5*P_2_5;
        T tmp_ric15 = B_0_0*P_0_0 + B_3_0*P_0_3;
        T tmp_ric16 = B_0_0*P_0_3 + B_3_0*P_3_3;
        T tmp_ric17 = B_0_0*P_0_1 + B_3_0*P_1_3;
        T tmp_ric18 = B_0_0*P_0_4 + B_3_0*P_3_4;
        T tmp_ric19 = B_0_0*P_0_2 + B_3_0*P_2_3;
        T tmp_ric20 = B_0_0*P_0_5 + B_3_0*P_3_5;
        T tmp_ric21 = B_1_1*P_1_1 + B_4_1*P_1_4;
        T tmp_ric22 = B_1_1*P_1_4 + B_4_1*P_4_4;
        T tmp_ric23 = B_1_1*P_1_2 + B_4_1*P_2_4;
        T tmp_ric24 = B_1_1*P_1_5 + B_4_1*P_4_5;
        T tmp_ric25 = B_2_2*P_2_2 + B_5_2*P_2_5;
        T tmp_ric26 = B_2_2*P_2_5 + B_5_2*P_5_5;
        T tmp_ric27 = B_1_1*P_0_1 + B_4_1*P_0_4;
        T tmp_ric28 = B_2_2*P_0_2 + B_5_2*P_0_5;
        T tmp_ric29 = B_2_2*P_1_2 + B_5_2*P_1_5;

        // Accumulate Results
        kp.Q_bar(0,0) += pow(A_0_0, 2)*P_0_0;
        kp.Q_bar(0,1) += A_1_1*tmp_ric0;
        kp.Q_bar(0,2) += A_2_2*tmp_ric1;
        kp.Q_bar(0,3) += A_0_0*tmp_ric2 + A_0_0*tmp_ric3;
        kp.Q_bar(0,4) += A_0_0*A_4_4*P_0_4 + A_1_4*tmp_ric0;
        kp.Q_bar(0,5) += A_0_0*A_5_5*P_0_5 + A_2_5*tmp_ric1;
        kp.Q_bar(1,1) += pow(A_1_1, 2)*P_1_1;
        kp.Q_bar(1,2) += A_2_2*tmp_ric4;
        kp.Q_bar(1,3) += A_1_1*tmp_ric5 + A_1_1*tmp_ric6;
        kp.Q_bar(1,4) += A_1_1*tmp_ric7 + A_1_1*tmp_ric8;
        kp.Q_bar(1,5) += A_1_1*A_5_5*P_1_5 + A_2_5*tmp_ric4;
        kp.Q_bar(2,2) += pow(A_2_2, 2)*P_2_2;
        kp.Q_bar(2,3) += A_2_2*tmp_ric10 + A_2_2*tmp_ric9;
        kp.Q_bar(2,4) += A_2_2*tmp_ric11 + A_2_2*tmp_ric12;
        kp.Q_bar(2,5) += A_2_2*tmp_ric13 + A_2_2*tmp_ric14;
        kp.Q_bar(3,3) += A_0_3*(tmp_ric2 + tmp_ric3) + A_3_3*(A_0_3*P_0_3 + A_3_3*P_3_3);
        kp.Q_bar(3,4) += A_1_4*(tmp_ric5 + tmp_ric6) + A_4_4*(A_0_3*P_0_4 + A_3_3*P_3_4);
        kp.Q_bar(3,5) += A_2_5*(tmp_ric10 + tmp_ric9) + A_5_5*(A_0_3*P_0_5 + A_3_3*P_3_5);
        kp.Q_bar(4,4) += A_1_4*(tmp_ric7 + tmp_ric8) + A_4_4*(A_1_4*P_1_4 + A_4_4*P_4_4);
        kp.Q_bar(4,5) += A_2_5*(tmp_ric11 + tmp_ric12) + A_5_5*(A_1_4*P_1_5 + A_4_4*P_4_5);
        kp.Q_bar(5,5) += A_2_5*(tmp_ric13 + tmp_ric14) + A_5_5*(A_2_5*P_2_5 + A_5_5*P_5_5);
        kp.R_bar(0,0) += B_0_0*tmp_ric15 + B_3_0*tmp_ric16;
        kp.R_bar(0,1) += B_1_1*tmp_ric17 + B_4_1*tmp_ric18;
        kp.R_bar(0,2) += B_2_2*tmp_ric19 + B_5_2*tmp_ric20;
        kp.R_bar(1,1) += B_1_1*tmp_ric21 + B_4_1*tmp_ric22;
        kp.R_bar(1,2) += B_2_2*tmp_ric23 + B_5_2*tmp_ric24;
        kp.R_bar(2,2) += B_2_2*tmp_ric25 + B_5_2*tmp_ric26;
        kp.H_bar(0,0) += A_0_0*tmp_ric15;
        kp.H_bar(0,1) += A_1_1*tmp_ric17;
        kp.H_bar(0,2) += A_2_2*tmp_ric19;
        kp.H_bar(0,3) += A_0_3*tmp_ric15 + A_3_3*tmp_ric16;
        kp.H_bar(0,4) += A_1_4*tmp_ric17 + A_4_4*tmp_ric18;
        kp.H_bar(0,5) += A_2_5*tmp_ric19 + A_5_5*tmp_ric20;
        kp.H_bar(1,0) += A_0_0*tmp_ric27;
        kp.H_bar(1,1) += A_1_1*tmp_ric21;
        kp.H_bar(1,2) += A_2_2*tmp_ric23;
        kp.H_bar(1,3) += A_0_3*tmp_ric27 + A_3_3*(B_1_1*P_1_3 + B_4_1*P_3_4);
        kp.H_bar(1,4) += A_1_4*tmp_ric21 + A_4_4*tmp_ric22;
        kp.H_bar(1,5) += A_2_5*tmp_ric23 + A_5_5*tmp_ric24;
        kp.H_bar(2,0) += A_0_0*tmp_ric28;
        kp.H_bar(2,1) += A_1_1*tmp_ric29;
        kp.H_bar(2,2) += A_2_2*tmp_ric25;
        kp.H_bar(2,3) += A_0_3*tmp_ric28 + A_3_3*(B_2_2*P_2_3 + B_5_2*P_3_5);
        kp.H_bar(2,4) += A_1_4*tmp_ric29 + A_4_4*(B_2_2*P_2_4 + B_5_2*P_4_5);
        kp.H_bar(2,5) += A_2_5*tmp_ric25 + A_5_5*tmp_ric26;
        kp.q_bar(0,0) += A_0_0*p_0;
        kp.q_bar(1,0) += A_1_1*p_1;
        kp.q_bar(2,0) += A_2_2*p_2;
        kp.q_bar(3,0) += A_0_3*p_0 + A_3_3*p_3;
        kp.q_bar(4,0) += A_1_4*p_1 + A_4_4*p_4;
        kp.q_bar(5,0) += A_2_5*p_2 + A_5_5*p_5;
        kp.r_bar(0,0) += B_0_0*p_0 + B_3_0*p_3;
        kp.r_bar(1,0) += B_1_1*p_1 + B_4_1*p_4;
        kp.r_bar(2,0) += B_2_2*p_2 + B_5_2*p_5;

        // Fill Lower Triangles (Symmetry)
        kp.Q_bar(1,0) = kp.Q_bar(0,1);
        kp.Q_bar(2,0) = kp.Q_bar(0,2);
        kp.Q_bar(3,0) = kp.Q_bar(0,3);
        kp.Q_bar(4,0) = kp.Q_bar(0,4);
        kp.Q_bar(5,0) = kp.Q_bar(0,5);
        kp.Q_bar(2,1) = kp.Q_bar(1,2);
        kp.Q_bar(3,1) = kp.Q_bar(1,3);
        kp.Q_bar(4,1) = kp.Q_bar(1,4);
        kp.Q_bar(5,1) = kp.Q_bar(1,5);
        kp.Q_bar(3,2) = kp.Q_bar(2,3);
        kp.Q_bar(4,2) = kp.Q_bar(2,4);
        kp.Q_bar(5,2) = kp.Q_bar(2,5);
        kp.Q_bar(4,3) = kp.Q_bar(3,4);
        kp.Q_bar(5,3) = kp.Q_bar(3,5);
        kp.Q_bar(5,4) = kp.Q_bar(4,5);
        kp.R_bar(1,0) = kp.R_bar(0,1);
        kp.R_bar(2,0) = kp.R_bar(0,2);
        kp.R_bar(2,1) = kp.R_bar(1,2);

    }

};
}
