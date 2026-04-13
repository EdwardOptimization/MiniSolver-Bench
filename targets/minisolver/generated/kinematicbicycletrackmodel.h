#pragma once
#include "minisolver/core/types.h"
#include "minisolver/core/solver_options.h"
#include "minisolver/core/matrix_defs.h"
#include <cmath>
#include <string>
#include <array>

namespace minisolver {

struct KinematicBicycleTrackModel {
    // --- Constants ---
    static const int NX=5;
    static const int NU=2;
    static const int NC=10;
    static const int NP=10;

    static constexpr std::array<double, NC> constraint_weights = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    static constexpr std::array<int, NC> constraint_types = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


    // --- Name Arrays (for Map Construction) ---
    static constexpr std::array<const char*, NX> state_names = {
        "x",
        "y",
        "psi",
        "v",
        "delta",
    };

    static constexpr std::array<const char*, NU> control_names = {
        "a",
        "delta_rate",
    };

    static constexpr std::array<const char*, NP> param_names = {
        "x_ref",
        "y_ref",
        "psi_ref",
        "v_ref",
        "n_x",
        "n_y",
        "w_left",
        "w_right",
        "wheelbase",
        "delta_max",
    };


    // --- Continuous Dynamics ---
    template<typename T>
    static MSVec<T, NX> dynamics_continuous(
        const MSVec<T, NX>& x_in,
        const MSVec<T, NU>& u_in,
        const MSVec<T, NP>& p_in) 
    {
        T psi = x_in(2);
        T v = x_in(3);
        T delta = x_in(4);
        T a = u_in(0);
        T delta_rate = u_in(1);
        T wheelbase = p_in(8);

        MSVec<T, NX> xdot;
        xdot(0) = v*cos(psi);
        xdot(1) = v*sin(psi);
        xdot(2) = v*tan(delta)/wheelbase;
        xdot(3) = a;
        xdot(4) = delta_rate;
        return xdot;

    }

    // --- Integrator Interface ---
    template<typename T>
    static MSVec<T, NX> integrate(
        const MSVec<T, NX>& x,
        const MSVec<T, NU>& u,
        const MSVec<T, NP>& p,
        double dt,
        IntegratorType type)
    {
        switch(type) {
            case IntegratorType::EULER_EXPLICIT: 
                return x + dynamics_continuous(x, u, p) * dt;
                
            case IntegratorType::RK2_EXPLICIT: 
            {
               auto k1 = dynamics_continuous(x, u, p);
               auto k2 = dynamics_continuous<T>(x + k1 * (0.5 * dt), u, p);
               return x + k2 * dt;
            }

            case IntegratorType::EULER_IMPLICIT:
            {
                // Simple Fixed-Point Iteration for x_next = x + f(x_next, u) * dt
                MSVec<T, NX> x_next = x; // Guess
                for(int i=0; i<5; ++i) {
                    x_next = x + dynamics_continuous(x_next, u, p) * dt;
                }
                return x_next;
            }

            case IntegratorType::RK2_IMPLICIT:
            {
                // Implicit Midpoint: k = f(x + 0.5*dt*k). x_next = x + dt*k
                MSVec<T, NX> k = dynamics_continuous(x, u, p); // Guess k0
                for(int i=0; i<5; ++i) {
                    k = dynamics_continuous<T>(x + k * (0.5 * dt), u, p);
                }
                return x + k * dt;
            }

            // Fallback for others to RK4 or appropriate handling
            default: // RK4 Explicit (Default)
            {
               auto k1 = dynamics_continuous(x, u, p);
               auto k2 = dynamics_continuous<T>(x + k1 * (0.5 * dt), u, p);
               auto k3 = dynamics_continuous<T>(x + k2 * (0.5 * dt), u, p);
               auto k4 = dynamics_continuous<T>(x + k3 * dt, u, p);
               return x + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);
            }
        }
    }

    // --- 1. Compute Dynamics (f_resid, A, B) ---
    template<typename T>
    static void compute_dynamics(KnotPoint<T,NX,NU,NC,NP>& kp, IntegratorType type, double dt) {
        T x = kp.x(0);
        T y = kp.x(1);
        T psi = kp.x(2);
        T v = kp.x(3);
        T delta = kp.x(4);
        T a = kp.u(0);
        T delta_rate = kp.u(1);
        T wheelbase = kp.p(8);

        switch(type) {
            case IntegratorType::EULER_EXPLICIT:
            case IntegratorType::EULER_IMPLICIT:
            {
                T tmp_d0 = dt*cos(psi);
                T tmp_d1 = tmp_d0*v;
                T tmp_d2 = dt*sin(psi);
                T tmp_d3 = tmp_d2*v;
                T tmp_d4 = tan(delta);
                T tmp_d5 = dt/wheelbase;
                T tmp_d6 = tmp_d4*tmp_d5;
                kp.f_resid(0) = tmp_d1 + x;
                kp.f_resid(1) = tmp_d3 + y;
                kp.f_resid(2) = psi + tmp_d6*v;
                kp.f_resid(3) = a*dt + v;
                kp.f_resid(4) = delta + delta_rate*dt;
                kp.A.setZero();
                kp.A(0,0) = 1;
                kp.A(0,2) = -tmp_d3;
                kp.A(0,3) = tmp_d0;
                kp.A(1,1) = 1;
                kp.A(1,2) = tmp_d1;
                kp.A(1,3) = tmp_d2;
                kp.A(2,2) = 1;
                kp.A(2,3) = tmp_d6;
                kp.A(2,4) = tmp_d5*v*(pow(tmp_d4, 2) + 1);
                kp.A(3,3) = 1;
                kp.A(4,4) = 1;
                kp.B.setZero();
                kp.B(3,0) = dt;
                kp.B(4,1) = dt;
                break;
            }
            case IntegratorType::RK2_EXPLICIT:
            case IntegratorType::RK2_IMPLICIT:
            {
                T tmp_d0 = a*dt;
                T tmp_d1 = 0.5*tmp_d0 + v;
                T tmp_d2 = tan(delta);
                T tmp_d3 = 1.0/wheelbase;
                T tmp_d4 = dt*tmp_d3;
                T tmp_d5 = tmp_d1*tmp_d4;
                T tmp_d6 = psi + 0.5*tmp_d2*tmp_d5;
                T tmp_d7 = cos(tmp_d6);
                T tmp_d8 = dt*tmp_d1*tmp_d7;
                T tmp_d9 = sin(tmp_d6);
                T tmp_d10 = dt*tmp_d9;
                T tmp_d11 = tmp_d1*tmp_d10;
                T tmp_d12 = delta_rate*dt;
                T tmp_d13 = tan(delta + 0.5*tmp_d12);
                T tmp_d14 = tmp_d13*tmp_d4;
                T tmp_d15 = pow(dt, 2);
                T tmp_d16 = 0.5*tmp_d15;
                T tmp_d17 = tmp_d16*tmp_d9;
                T tmp_d18 = tmp_d17*tmp_d3;
                T tmp_d19 = tmp_d1*tmp_d2;
                T tmp_d20 = pow(tmp_d1, 2)*(pow(tmp_d2, 2) + 1);
                T tmp_d21 = tmp_d16*tmp_d3*tmp_d7;
                T tmp_d22 = pow(tmp_d13, 2) + 1;
                T tmp_d23 = 0.25*pow(dt, 3)*tmp_d19*tmp_d3;
                T tmp_d24 = tmp_d16*tmp_d3;
                kp.f_resid(0) = tmp_d8 + x;
                kp.f_resid(1) = tmp_d11 + y;
                kp.f_resid(2) = psi + tmp_d1*tmp_d14;
                kp.f_resid(3) = tmp_d0 + v;
                kp.f_resid(4) = delta + tmp_d12;
                kp.A.setZero();
                kp.A(0,0) = 1;
                kp.A(0,2) = -tmp_d11;
                kp.A(0,3) = dt*tmp_d7 - tmp_d18*tmp_d19;
                kp.A(0,4) = -tmp_d18*tmp_d20;
                kp.A(1,1) = 1;
                kp.A(1,2) = tmp_d8;
                kp.A(1,3) = tmp_d10 + tmp_d19*tmp_d21;
                kp.A(1,4) = tmp_d20*tmp_d21;
                kp.A(2,2) = 1;
                kp.A(2,3) = tmp_d14;
                kp.A(2,4) = tmp_d22*tmp_d5;
                kp.A(3,3) = 1;
                kp.A(4,4) = 1;
                kp.B.setZero();
                kp.B(0,0) = 0.5*tmp_d15*tmp_d7 - tmp_d23*tmp_d9;
                kp.B(1,0) = tmp_d17 + tmp_d23*tmp_d7;
                kp.B(2,0) = tmp_d13*tmp_d24;
                kp.B(2,1) = tmp_d1*tmp_d22*tmp_d24;
                kp.B(3,0) = dt;
                kp.B(4,1) = dt;
                break;
            }
            case IntegratorType::RK4_EXPLICIT:
            case IntegratorType::RK4_IMPLICIT:
            {
                T tmp_d0 = cos(psi);
                T tmp_d1 = a*dt;
                T tmp_d2 = 0.5*tmp_d1 + v;
                T tmp_d3 = tan(delta);
                T tmp_d4 = 1.0/wheelbase;
                T tmp_d5 = tmp_d3*tmp_d4;
                T tmp_d6 = tmp_d2*tmp_d5;
                T tmp_d7 = 0.5*tmp_d6;
                T tmp_d8 = dt*tmp_d7 + psi;
                T tmp_d9 = cos(tmp_d8);
                T tmp_d10 = 2*tmp_d9;
                T tmp_d11 = tmp_d1 + v;
                T tmp_d12 = 1.5*tmp_d1 + v;
                T tmp_d13 = delta_rate*dt;
                T tmp_d14 = tan(delta + 0.5*tmp_d13);
                T tmp_d15 = tmp_d14*tmp_d4;
                T tmp_d16 = dt*tmp_d15;
                T tmp_d17 = psi + tmp_d12*tmp_d16;
                T tmp_d18 = cos(tmp_d17);
                T tmp_d19 = tmp_d11*tmp_d18;
                T tmp_d20 = 1.0*tmp_d1 + v;
                T tmp_d21 = 0.5*tmp_d20;
                T tmp_d22 = psi + tmp_d16*tmp_d21;
                T tmp_d23 = cos(tmp_d22);
                T tmp_d24 = 2*tmp_d23;
                T tmp_d25 = 0.16666666666666666*dt;
                T tmp_d26 = tmp_d25*(tmp_d0*v + tmp_d10*tmp_d2 + tmp_d19 + tmp_d2*tmp_d24);
                T tmp_d27 = sin(psi);
                T tmp_d28 = sin(tmp_d8);
                T tmp_d29 = 2*tmp_d28;
                T tmp_d30 = sin(tmp_d17);
                T tmp_d31 = tmp_d11*tmp_d30;
                T tmp_d32 = sin(tmp_d22);
                T tmp_d33 = 2*tmp_d32;
                T tmp_d34 = tmp_d2*tmp_d29 + tmp_d2*tmp_d33 + tmp_d27*v + tmp_d31;
                T tmp_d35 = tan(delta + tmp_d13);
                T tmp_d36 = tmp_d35*tmp_d4;
                T tmp_d37 = 4*tmp_d15;
                T tmp_d38 = 1.0*dt;
                T tmp_d39 = tmp_d28*tmp_d38;
                T tmp_d40 = tmp_d32*tmp_d38;
                T tmp_d41 = tmp_d15*tmp_d2;
                T tmp_d42 = tmp_d4*(pow(tmp_d3, 2) + 1);
                T tmp_d43 = pow(tmp_d2, 2)*tmp_d42;
                T tmp_d44 = tmp_d4*(pow(tmp_d14, 2) + 1);
                T tmp_d45 = tmp_d12*tmp_d44;
                T tmp_d46 = dt*tmp_d45;
                T tmp_d47 = tmp_d2*tmp_d44;
                T tmp_d48 = tmp_d20*tmp_d47;
                T tmp_d49 = tmp_d38*tmp_d9;
                T tmp_d50 = tmp_d23*tmp_d38;
                T tmp_d51 = tmp_d11*tmp_d4*(pow(tmp_d35, 2) + 1);
                T tmp_d52 = pow(dt, 2);
                T tmp_d53 = tmp_d52*tmp_d7;
                T tmp_d54 = tmp_d31*tmp_d52;
                T tmp_d55 = 1.5*tmp_d15;
                T tmp_d56 = tmp_d32*tmp_d52;
                T tmp_d57 = 1.0*tmp_d41;
                T tmp_d58 = 0.5*tmp_d45;
                T tmp_d59 = tmp_d21*tmp_d47;
                T tmp_d60 = tmp_d19*tmp_d52;
                T tmp_d61 = tmp_d23*tmp_d52;
                kp.f_resid(0) = tmp_d26 + x;
                kp.f_resid(1) = tmp_d25*tmp_d34 + y;
                kp.f_resid(2) = psi + tmp_d25*(tmp_d11*tmp_d36 + tmp_d2*tmp_d37 + tmp_d5*v);
                kp.f_resid(3) = tmp_d20;
                kp.f_resid(4) = delta + 1.0*tmp_d13;
                kp.A.setZero();
                kp.A(0,0) = 1;
                kp.A(0,2) = -tmp_d25*tmp_d34;
                kp.A(0,3) = tmp_d25*(tmp_d0 + tmp_d10 - tmp_d16*tmp_d31 + tmp_d18 + tmp_d24 - tmp_d39*tmp_d6 - tmp_d40*tmp_d41);
                kp.A(0,4) = tmp_d25*(-tmp_d31*tmp_d46 - tmp_d39*tmp_d43 - tmp_d40*tmp_d48);
                kp.A(1,1) = 1;
                kp.A(1,2) = tmp_d26;
                kp.A(1,3) = tmp_d25*(tmp_d16*tmp_d19 + tmp_d27 + tmp_d29 + tmp_d30 + tmp_d33 + tmp_d41*tmp_d50 + tmp_d49*tmp_d6);
                kp.A(1,4) = tmp_d25*(tmp_d19*tmp_d46 + tmp_d43*tmp_d49 + tmp_d48*tmp_d50);
                kp.A(2,2) = 1;
                kp.A(2,3) = tmp_d25*(tmp_d36 + tmp_d37 + tmp_d5);
                kp.A(2,4) = tmp_d25*(tmp_d42*v + 4*tmp_d47 + tmp_d51);
                kp.A(3,3) = 1;
                kp.A(4,4) = 1;
                kp.B.setZero();
                kp.B(0,0) = tmp_d25*(dt*tmp_d18 + 1.0*dt*tmp_d23 + 1.0*dt*tmp_d9 - tmp_d28*tmp_d53 - tmp_d54*tmp_d55 - tmp_d56*tmp_d57);
                kp.B(0,1) = tmp_d25*(-tmp_d54*tmp_d58 - tmp_d56*tmp_d59);
                kp.B(1,0) = tmp_d25*(dt*tmp_d30 + tmp_d39 + tmp_d40 + tmp_d53*tmp_d9 + tmp_d55*tmp_d60 + tmp_d57*tmp_d61);
                kp.B(1,1) = tmp_d25*(tmp_d58*tmp_d60 + tmp_d59*tmp_d61);
                kp.B(2,0) = tmp_d25*(dt*tmp_d36 + 2.0*tmp_d16);
                kp.B(2,1) = tmp_d25*(2.0*dt*tmp_d47 + dt*tmp_d51);
                kp.B(3,0) = tmp_d38;
                kp.B(4,1) = tmp_d38;
                break;
            }
            default:
                break;
        }
    }

    // --- 2. Compute Constraints (g_val, C, D) ---
    template<typename T>
    static void compute_constraints(KnotPoint<T,NX,NU,NC,NP>& kp) {
        T x = kp.x(0);
        T y = kp.x(1);
        T v = kp.x(3);
        T delta = kp.x(4);
        T a = kp.u(0);
        T delta_rate = kp.u(1);
        T x_ref = kp.p(0);
        T y_ref = kp.p(1);
        T n_x = kp.p(4);
        T n_y = kp.p(5);
        T w_left = kp.p(6);
        T w_right = kp.p(7);
        T delta_max = kp.p(9);

        // --- Special Constraints Pre-Calculation ---

        T tmp_c0 = n_x*(x - x_ref) + n_y*(y - y_ref);

        // g_val
        kp.g_val(0,0) = tmp_c0 - w_left;
        kp.g_val(1,0) = -tmp_c0 - w_right;
        kp.g_val(2,0) = v - 12.0;
        kp.g_val(3,0) = 0.10000000000000001 - v;
        kp.g_val(4,0) = delta - delta_max;
        kp.g_val(5,0) = -delta - delta_max;
        kp.g_val(6,0) = a - 15.0;
        kp.g_val(7,0) = -a - 15.0;
        kp.g_val(8,0) = delta_rate - 6.0;
        kp.g_val(9,0) = -delta_rate - 6.0;

        // C
        kp.C(0,0) = n_x;
        kp.C(0,1) = n_y;
        kp.C(0,2) = 0;
        kp.C(0,3) = 0;
        kp.C(0,4) = 0;
        kp.C(1,0) = -n_x;
        kp.C(1,1) = -n_y;
        kp.C(1,2) = 0;
        kp.C(1,3) = 0;
        kp.C(1,4) = 0;
        kp.C(2,0) = 0;
        kp.C(2,1) = 0;
        kp.C(2,2) = 0;
        kp.C(2,3) = 1;
        kp.C(2,4) = 0;
        kp.C(3,0) = 0;
        kp.C(3,1) = 0;
        kp.C(3,2) = 0;
        kp.C(3,3) = -1;
        kp.C(3,4) = 0;
        kp.C(4,0) = 0;
        kp.C(4,1) = 0;
        kp.C(4,2) = 0;
        kp.C(4,3) = 0;
        kp.C(4,4) = 1;
        kp.C(5,0) = 0;
        kp.C(5,1) = 0;
        kp.C(5,2) = 0;
        kp.C(5,3) = 0;
        kp.C(5,4) = -1;
        kp.C(6,0) = 0;
        kp.C(6,1) = 0;
        kp.C(6,2) = 0;
        kp.C(6,3) = 0;
        kp.C(6,4) = 0;
        kp.C(7,0) = 0;
        kp.C(7,1) = 0;
        kp.C(7,2) = 0;
        kp.C(7,3) = 0;
        kp.C(7,4) = 0;
        kp.C(8,0) = 0;
        kp.C(8,1) = 0;
        kp.C(8,2) = 0;
        kp.C(8,3) = 0;
        kp.C(8,4) = 0;
        kp.C(9,0) = 0;
        kp.C(9,1) = 0;
        kp.C(9,2) = 0;
        kp.C(9,3) = 0;
        kp.C(9,4) = 0;

        // D
        kp.D(0,0) = 0;
        kp.D(0,1) = 0;
        kp.D(1,0) = 0;
        kp.D(1,1) = 0;
        kp.D(2,0) = 0;
        kp.D(2,1) = 0;
        kp.D(3,0) = 0;
        kp.D(3,1) = 0;
        kp.D(4,0) = 0;
        kp.D(4,1) = 0;
        kp.D(5,0) = 0;
        kp.D(5,1) = 0;
        kp.D(6,0) = 1;
        kp.D(6,1) = 0;
        kp.D(7,0) = -1;
        kp.D(7,1) = 0;
        kp.D(8,0) = 0;
        kp.D(8,1) = 1;
        kp.D(9,0) = 0;
        kp.D(9,1) = -1;

    }

    // --- 3. Compute Cost (Implemented via template for Exact/GN) ---
    template<typename T, int Mode>
    static void compute_cost_impl(KnotPoint<T,NX,NU,NC,NP>& kp) {
        T x = kp.x(0);
        T y = kp.x(1);
        T psi = kp.x(2);
        T v = kp.x(3);
        T delta = kp.x(4);
        T a = kp.u(0);
        T delta_rate = kp.u(1);
        T x_ref = kp.p(0);
        T y_ref = kp.p(1);
        T psi_ref = kp.p(2);
        T v_ref = kp.p(3);

        T tmp_j0 = psi - psi_ref;
        T tmp_j1 = sin(tmp_j0);
        T tmp_j2 = cos(tmp_j0);
        T tmp_j3 = atan2(tmp_j1, tmp_j2);
        T tmp_j4 = pow(tmp_j1, 2);
        T tmp_j5 = pow(tmp_j2, 2);
        T tmp_j6 = 1.0/(tmp_j4 + tmp_j5);
        T tmp_j7 = tmp_j4*tmp_j6;
        T tmp_j8 = tmp_j5*tmp_j6;
        T tmp_j9 = 5.0*tmp_j7 + 5.0*tmp_j8;

        // q
        kp.q(0,0) = 20.0*x - 20.0*x_ref;
        kp.q(1,0) = 20.0*y - 20.0*y_ref;
        kp.q(2,0) = tmp_j3*tmp_j9;
        kp.q(3,0) = 2*v - 2*v_ref;
        kp.q(4,0) = 0.5*delta;

        // r
        kp.r(0,0) = 0.050000000000000003*a;
        kp.r(1,0) = 0.050000000000000003*delta_rate;

        // Q (Mode 0=GN, 1=Exact)
        kp.Q(0,0) = 20.0;
        kp.Q(0,1) = 0;
        kp.Q(0,2) = 0;
        kp.Q(0,3) = 0;
        kp.Q(0,4) = 0;
        kp.Q(1,0) = 0;
        kp.Q(1,1) = 20.0;
        kp.Q(1,2) = 0;
        kp.Q(1,3) = 0;
        kp.Q(1,4) = 0;
        kp.Q(2,0) = 0;
        kp.Q(2,1) = 0;
        kp.Q(2,2) = tmp_j9*(tmp_j7 + tmp_j8);
        kp.Q(2,3) = 0;
        kp.Q(2,4) = 0;
        kp.Q(3,0) = 0;
        kp.Q(3,1) = 0;
        kp.Q(3,2) = 0;
        kp.Q(3,3) = 2;
        kp.Q(3,4) = 0;
        kp.Q(4,0) = 0;
        kp.Q(4,1) = 0;
        kp.Q(4,2) = 0;
        kp.Q(4,3) = 0;
        kp.Q(4,4) = 0.5;

        // R (Mode 0=GN, 1=Exact)
        kp.R(0,0) = 0.050000000000000003;
        kp.R(0,1) = 0;
        kp.R(1,0) = 0;
        kp.R(1,1) = 0.050000000000000003;

        // H (Mode 0=GN, 1=Exact)
        kp.H(0,0) = 0;
        kp.H(0,1) = 0;
        kp.H(0,2) = 0;
        kp.H(0,3) = 0;
        kp.H(0,4) = 0;
        kp.H(1,0) = 0;
        kp.H(1,1) = 0;
        kp.H(1,2) = 0;
        kp.H(1,3) = 0;
        kp.H(1,4) = 0;

        kp.cost = 0.025000000000000001*pow(a, 2) + 0.25*pow(delta, 2) + 0.025000000000000001*pow(delta_rate, 2) + 2.5*pow(tmp_j3, 2) + pow(v - v_ref, 2) + 10.0*pow(x - x_ref, 2) + 10.0*pow(y - y_ref, 2);
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


    // --- 4. Compute All (Convenience) ---
    template<typename T>
    static void compute(KnotPoint<T,NX,NU,NC,NP>& kp, IntegratorType type, double dt) {
        compute_dynamics(kp, type, dt);
        compute_constraints(kp);
        compute_cost(kp); // Default GN
    }

    template<typename T>
    static void compute_exact(KnotPoint<T,NX,NU,NC,NP>& kp, IntegratorType type, double dt) {
        compute_dynamics(kp, type, dt);
        compute_constraints(kp);
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
        T P_1_1 = Vxx(1,1);
        T P_1_2 = Vxx(1,2);
        T P_1_3 = Vxx(1,3);
        T P_1_4 = Vxx(1,4);
        T P_2_2 = Vxx(2,2);
        T P_2_3 = Vxx(2,3);
        T P_2_4 = Vxx(2,4);
        T P_3_3 = Vxx(3,3);
        T P_3_4 = Vxx(3,4);
        T P_4_4 = Vxx(4,4);
        T p_0 = Vx(0);
        T p_1 = Vx(1);
        T p_2 = Vx(2);
        T p_3 = Vx(3);
        T p_4 = Vx(4);
        T A_0_0 = kp.A(0,0);
        T A_0_2 = kp.A(0,2);
        T A_0_3 = kp.A(0,3);
        T A_0_4 = kp.A(0,4);
        T A_1_1 = kp.A(1,1);
        T A_1_2 = kp.A(1,2);
        T A_1_3 = kp.A(1,3);
        T A_1_4 = kp.A(1,4);
        T A_2_2 = kp.A(2,2);
        T A_2_3 = kp.A(2,3);
        T A_2_4 = kp.A(2,4);
        T A_3_3 = kp.A(3,3);
        T A_4_4 = kp.A(4,4);
        T B_0_0 = kp.B(0,0);
        T B_0_1 = kp.B(0,1);
        T B_1_0 = kp.B(1,0);
        T B_1_1 = kp.B(1,1);
        T B_2_0 = kp.B(2,0);
        T B_2_1 = kp.B(2,1);
        T B_3_0 = kp.B(3,0);
        T B_4_1 = kp.B(4,1);

        // CSE Intermediate Variables
        T tmp_ric0 = A_0_2*P_0_0;
        T tmp_ric1 = A_1_2*P_0_1;
        T tmp_ric2 = A_2_2*P_0_2;
        T tmp_ric3 = A_0_3*P_0_0;
        T tmp_ric4 = A_1_3*P_0_1;
        T tmp_ric5 = A_2_3*P_0_2;
        T tmp_ric6 = A_3_3*P_0_3;
        T tmp_ric7 = A_0_4*P_0_0;
        T tmp_ric8 = A_1_4*P_0_1;
        T tmp_ric9 = A_2_4*P_0_2;
        T tmp_ric10 = A_4_4*P_0_4;
        T tmp_ric11 = A_0_2*P_0_1;
        T tmp_ric12 = A_1_2*P_1_1;
        T tmp_ric13 = A_2_2*P_1_2;
        T tmp_ric14 = A_0_3*P_0_1;
        T tmp_ric15 = A_1_3*P_1_1;
        T tmp_ric16 = A_2_3*P_1_2;
        T tmp_ric17 = A_3_3*P_1_3;
        T tmp_ric18 = A_0_4*P_0_1;
        T tmp_ric19 = A_1_4*P_1_1;
        T tmp_ric20 = A_2_4*P_1_2;
        T tmp_ric21 = A_4_4*P_1_4;
        T tmp_ric22 = tmp_ric0 + tmp_ric1 + tmp_ric2;
        T tmp_ric23 = tmp_ric11 + tmp_ric12 + tmp_ric13;
        T tmp_ric24 = A_0_2*P_0_2 + A_1_2*P_1_2 + A_2_2*P_2_2;
        T tmp_ric25 = tmp_ric3 + tmp_ric4 + tmp_ric5 + tmp_ric6;
        T tmp_ric26 = tmp_ric14 + tmp_ric15 + tmp_ric16 + tmp_ric17;
        T tmp_ric27 = A_0_3*P_0_2 + A_1_3*P_1_2 + A_2_3*P_2_2 + A_3_3*P_2_3;
        T tmp_ric28 = B_0_0*P_0_0 + B_1_0*P_0_1 + B_2_0*P_0_2 + B_3_0*P_0_3;
        T tmp_ric29 = B_0_0*P_0_1 + B_1_0*P_1_1 + B_2_0*P_1_2 + B_3_0*P_1_3;
        T tmp_ric30 = B_0_0*P_0_2 + B_1_0*P_1_2 + B_2_0*P_2_2 + B_3_0*P_2_3;
        T tmp_ric31 = B_0_0*P_0_3 + B_1_0*P_1_3 + B_2_0*P_2_3 + B_3_0*P_3_3;
        T tmp_ric32 = B_0_0*P_0_4 + B_1_0*P_1_4 + B_2_0*P_2_4 + B_3_0*P_3_4;
        T tmp_ric33 = B_0_1*P_0_0 + B_1_1*P_0_1 + B_2_1*P_0_2 + B_4_1*P_0_4;
        T tmp_ric34 = B_0_1*P_0_1 + B_1_1*P_1_1 + B_2_1*P_1_2 + B_4_1*P_1_4;
        T tmp_ric35 = B_0_1*P_0_2 + B_1_1*P_1_2 + B_2_1*P_2_2 + B_4_1*P_2_4;
        T tmp_ric36 = B_0_1*P_0_4 + B_1_1*P_1_4 + B_2_1*P_2_4 + B_4_1*P_4_4;

        // Accumulate Results
        kp.Q_bar(0,0) += pow(A_0_0, 2)*P_0_0;
        kp.Q_bar(0,1) += A_0_0*A_1_1*P_0_1;
        kp.Q_bar(0,2) += A_0_0*tmp_ric0 + A_0_0*tmp_ric1 + A_0_0*tmp_ric2;
        kp.Q_bar(0,3) += A_0_0*tmp_ric3 + A_0_0*tmp_ric4 + A_0_0*tmp_ric5 + A_0_0*tmp_ric6;
        kp.Q_bar(0,4) += A_0_0*tmp_ric10 + A_0_0*tmp_ric7 + A_0_0*tmp_ric8 + A_0_0*tmp_ric9;
        kp.Q_bar(1,1) += pow(A_1_1, 2)*P_1_1;
        kp.Q_bar(1,2) += A_1_1*tmp_ric11 + A_1_1*tmp_ric12 + A_1_1*tmp_ric13;
        kp.Q_bar(1,3) += A_1_1*tmp_ric14 + A_1_1*tmp_ric15 + A_1_1*tmp_ric16 + A_1_1*tmp_ric17;
        kp.Q_bar(1,4) += A_1_1*tmp_ric18 + A_1_1*tmp_ric19 + A_1_1*tmp_ric20 + A_1_1*tmp_ric21;
        kp.Q_bar(2,2) += A_0_2*tmp_ric22 + A_1_2*tmp_ric23 + A_2_2*tmp_ric24;
        kp.Q_bar(2,3) += A_0_3*tmp_ric22 + A_1_3*tmp_ric23 + A_2_3*tmp_ric24 + A_3_3*(A_0_2*P_0_3 + A_1_2*P_1_3 + A_2_2*P_2_3);
        kp.Q_bar(2,4) += A_0_4*tmp_ric22 + A_1_4*tmp_ric23 + A_2_4*tmp_ric24 + A_4_4*(A_0_2*P_0_4 + A_1_2*P_1_4 + A_2_2*P_2_4);
        kp.Q_bar(3,3) += A_0_3*tmp_ric25 + A_1_3*tmp_ric26 + A_2_3*tmp_ric27 + A_3_3*(A_0_3*P_0_3 + A_1_3*P_1_3 + A_2_3*P_2_3 + A_3_3*P_3_3);
        kp.Q_bar(3,4) += A_0_4*tmp_ric25 + A_1_4*tmp_ric26 + A_2_4*tmp_ric27 + A_4_4*(A_0_3*P_0_4 + A_1_3*P_1_4 + A_2_3*P_2_4 + A_3_3*P_3_4);
        kp.Q_bar(4,4) += A_0_4*(tmp_ric10 + tmp_ric7 + tmp_ric8 + tmp_ric9) + A_1_4*(tmp_ric18 + tmp_ric19 + tmp_ric20 + tmp_ric21) + A_2_4*(A_0_4*P_0_2 + A_1_4*P_1_2 + A_2_4*P_2_2 + A_4_4*P_2_4) + A_4_4*(A_0_4*P_0_4 + A_1_4*P_1_4 + A_2_4*P_2_4 + A_4_4*P_4_4);
        kp.R_bar(0,0) += B_0_0*tmp_ric28 + B_1_0*tmp_ric29 + B_2_0*tmp_ric30 + B_3_0*tmp_ric31;
        kp.R_bar(0,1) += B_0_1*tmp_ric28 + B_1_1*tmp_ric29 + B_2_1*tmp_ric30 + B_4_1*tmp_ric32;
        kp.R_bar(1,1) += B_0_1*tmp_ric33 + B_1_1*tmp_ric34 + B_2_1*tmp_ric35 + B_4_1*tmp_ric36;
        kp.H_bar(0,0) += A_0_0*tmp_ric28;
        kp.H_bar(0,1) += A_1_1*tmp_ric29;
        kp.H_bar(0,2) += A_0_2*tmp_ric28 + A_1_2*tmp_ric29 + A_2_2*tmp_ric30;
        kp.H_bar(0,3) += A_0_3*tmp_ric28 + A_1_3*tmp_ric29 + A_2_3*tmp_ric30 + A_3_3*tmp_ric31;
        kp.H_bar(0,4) += A_0_4*tmp_ric28 + A_1_4*tmp_ric29 + A_2_4*tmp_ric30 + A_4_4*tmp_ric32;
        kp.H_bar(1,0) += A_0_0*tmp_ric33;
        kp.H_bar(1,1) += A_1_1*tmp_ric34;
        kp.H_bar(1,2) += A_0_2*tmp_ric33 + A_1_2*tmp_ric34 + A_2_2*tmp_ric35;
        kp.H_bar(1,3) += A_0_3*tmp_ric33 + A_1_3*tmp_ric34 + A_2_3*tmp_ric35 + A_3_3*(B_0_1*P_0_3 + B_1_1*P_1_3 + B_2_1*P_2_3 + B_4_1*P_3_4);
        kp.H_bar(1,4) += A_0_4*tmp_ric33 + A_1_4*tmp_ric34 + A_2_4*tmp_ric35 + A_4_4*tmp_ric36;
        kp.q_bar(0,0) += A_0_0*p_0;
        kp.q_bar(1,0) += A_1_1*p_1;
        kp.q_bar(2,0) += A_0_2*p_0 + A_1_2*p_1 + A_2_2*p_2;
        kp.q_bar(3,0) += A_0_3*p_0 + A_1_3*p_1 + A_2_3*p_2 + A_3_3*p_3;
        kp.q_bar(4,0) += A_0_4*p_0 + A_1_4*p_1 + A_2_4*p_2 + A_4_4*p_4;
        kp.r_bar(0,0) += B_0_0*p_0 + B_1_0*p_1 + B_2_0*p_2 + B_3_0*p_3;
        kp.r_bar(1,0) += B_0_1*p_0 + B_1_1*p_1 + B_2_1*p_2 + B_4_1*p_4;

        // Fill Lower Triangles (Symmetry)
        kp.Q_bar(1,0) = kp.Q_bar(0,1);
        kp.Q_bar(2,0) = kp.Q_bar(0,2);
        kp.Q_bar(3,0) = kp.Q_bar(0,3);
        kp.Q_bar(4,0) = kp.Q_bar(0,4);
        kp.Q_bar(2,1) = kp.Q_bar(1,2);
        kp.Q_bar(3,1) = kp.Q_bar(1,3);
        kp.Q_bar(4,1) = kp.Q_bar(1,4);
        kp.Q_bar(3,2) = kp.Q_bar(2,3);
        kp.Q_bar(4,2) = kp.Q_bar(2,4);
        kp.Q_bar(4,3) = kp.Q_bar(3,4);
        kp.R_bar(1,0) = kp.R_bar(0,1);

    }
    
};
}
