#pragma once
#include "minisolver/core/types.h"
#include "minisolver/core/solver_options.h"
#include "minisolver/core/matrix_defs.h"
#include <cmath>
#include <string>
#include <array>

namespace minisolver {

struct PendulumModel {
    // --- Constants ---
    static const int NX=4;
    static const int NU=1;
    static const int NC=2;
    static const int NP=0;

    static constexpr std::array<double, NC> constraint_weights = {0.0, 0.0};
    static constexpr std::array<int, NC> constraint_types = {0, 0};


    // --- Name Arrays (for Map Construction) ---
    static constexpr std::array<const char*, NX> state_names = {
        "x",
        "theta",
        "v",
        "omega",
    };

    static constexpr std::array<const char*, NU> control_names = {
        "force",
    };

    static constexpr std::array<const char*, NP> param_names = {
    };


    // --- Continuous Dynamics ---
    template<typename T>
    static MSVec<T, NX> dynamics_continuous(
        const MSVec<T, NX>& x_in,
        const MSVec<T, NU>& u_in,
        const MSVec<T, NP>& p_in) 
    {
        T theta = x_in(1);
        T v = x_in(2);
        T omega = x_in(3);
        T force = u_in(0);
        (void)p_in;

        MSVec<T, NX> xdot;
        xdot(0) = v;
        xdot(1) = omega;
        xdot(2) = (force - 0.080000000000000016*pow(omega, 2)*sin(theta) + 0.98100000000000009*sin(theta)*cos(theta))/(1.1000000000000001 - 0.10000000000000001*pow(cos(theta), 2));
        xdot(3) = (force*cos(theta) - 0.080000000000000016*pow(omega, 2)*sin(theta)*cos(theta) + 10.791000000000002*sin(theta))/(0.88000000000000012 - 0.080000000000000016*pow(cos(theta), 2));
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
        T theta = kp.x(1);
        T v = kp.x(2);
        T omega = kp.x(3);
        T force = kp.u(0);

        switch(type) {
            case IntegratorType::EULER_EXPLICIT:
            case IntegratorType::EULER_IMPLICIT:
            {
                T tmp_d0 = dt*omega;
                T tmp_d1 = sin(theta);
                T tmp_d2 = cos(theta);
                T tmp_d3 = tmp_d1*tmp_d2;
                T tmp_d4 = pow(omega, 2);
                T tmp_d5 = 0.080000000000000016*tmp_d4;
                T tmp_d6 = force - tmp_d1*tmp_d5 + 0.98100000000000009*tmp_d3;
                T tmp_d7 = pow(tmp_d2, 2);
                T tmp_d8 = 1.1000000000000001 - 0.10000000000000001*tmp_d7;
                T tmp_d9 = 1.0/tmp_d8;
                T tmp_d10 = dt*tmp_d9;
                T tmp_d11 = force*tmp_d2 + 10.791000000000002*tmp_d1 - tmp_d3*tmp_d5;
                T tmp_d12 = 0.080000000000000016*tmp_d7;
                T tmp_d13 = 0.88000000000000012 - tmp_d12;
                T tmp_d14 = 1.0/tmp_d13;
                T tmp_d15 = dt*tmp_d14;
                T tmp_d16 = pow(tmp_d1, 2);
                T tmp_d17 = dt*tmp_d3;
                T tmp_d18 = 0.16000000000000003*tmp_d0;
                kp.f_resid(0) = dt*v + x;
                kp.f_resid(1) = theta + tmp_d0;
                kp.f_resid(2) = tmp_d10*tmp_d6 + v;
                kp.f_resid(3) = omega + tmp_d11*tmp_d15;
                kp.A.setZero();
                kp.A(0,0) = 1;
                kp.A(0,2) = dt;
                kp.A(1,1) = 1;
                kp.A(1,3) = dt;
                kp.A(2,1) = tmp_d10*(-0.98100000000000009*tmp_d16 - tmp_d2*tmp_d5 + 0.98100000000000009*tmp_d7) - 0.20000000000000001*tmp_d17*tmp_d6/pow(tmp_d8, 2);
                kp.A(2,2) = 1;
                kp.A(2,3) = -tmp_d1*tmp_d18*tmp_d9;
                kp.A(3,1) = -0.16000000000000003*tmp_d11*tmp_d17/pow(tmp_d13, 2) + tmp_d15*(-force*tmp_d1 - tmp_d12*tmp_d4 + 0.080000000000000016*tmp_d16*tmp_d4 + 10.791000000000002*tmp_d2);
                kp.A(3,3) = -tmp_d14*tmp_d18*tmp_d3 + 1;
                kp.B.setZero();
                kp.B(2,0) = tmp_d10;
                kp.B(3,0) = tmp_d15*tmp_d2;
                break;
            }
            case IntegratorType::RK2_EXPLICIT:
            case IntegratorType::RK2_IMPLICIT:
            {
                T tmp_d0 = cos(theta);
                T tmp_d1 = sin(theta);
                T tmp_d2 = tmp_d0*tmp_d1;
                T tmp_d3 = pow(omega, 2);
                T tmp_d4 = 0.080000000000000016*tmp_d3;
                T tmp_d5 = force - tmp_d1*tmp_d4 + 0.98100000000000009*tmp_d2;
                T tmp_d6 = pow(tmp_d0, 2);
                T tmp_d7 = 1.1000000000000001 - 0.10000000000000001*tmp_d6;
                T tmp_d8 = 1.0/tmp_d7;
                T tmp_d9 = 0.5*dt;
                T tmp_d10 = tmp_d8*tmp_d9;
                T tmp_d11 = omega*tmp_d9 + theta;
                T tmp_d12 = sin(tmp_d11);
                T tmp_d13 = cos(tmp_d11);
                T tmp_d14 = tmp_d12*tmp_d13;
                T tmp_d15 = force*tmp_d13 + 10.791000000000002*tmp_d12;
                T tmp_d16 = -tmp_d14*tmp_d4 + tmp_d15;
                T tmp_d17 = pow(tmp_d13, 2);
                T tmp_d18 = 0.080000000000000016*tmp_d17;
                T tmp_d19 = 0.88000000000000012 - tmp_d18;
                T tmp_d20 = 1.0/tmp_d19;
                T tmp_d21 = tmp_d20*tmp_d9;
                T tmp_d22 = omega + tmp_d16*tmp_d21;
                T tmp_d23 = dt*tmp_d22;
                T tmp_d24 = pow(tmp_d22, 2);
                T tmp_d25 = 0.080000000000000016*tmp_d24;
                T tmp_d26 = force - tmp_d12*tmp_d25 + 0.98100000000000009*tmp_d14;
                T tmp_d27 = 1.1000000000000001 - 0.10000000000000001*tmp_d17;
                T tmp_d28 = 1.0/tmp_d27;
                T tmp_d29 = dt*tmp_d28;
                T tmp_d30 = -tmp_d14*tmp_d25 + tmp_d15;
                T tmp_d31 = dt*tmp_d20;
                T tmp_d32 = pow(dt, 2);
                T tmp_d33 = 0.080000000000000016*tmp_d32;
                T tmp_d34 = pow(tmp_d12, 2);
                T tmp_d35 = force*tmp_d12;
                T tmp_d36 = -10.791000000000002*tmp_d13 + tmp_d35;
                T tmp_d37 = -tmp_d18*tmp_d3 + 0.080000000000000016*tmp_d3*tmp_d34 - tmp_d36;
                T tmp_d38 = dt*tmp_d14;
                T tmp_d39 = pow(tmp_d19, -2);
                T tmp_d40 = tmp_d16*tmp_d39;
                T tmp_d41 = tmp_d14*tmp_d40;
                T tmp_d42 = 0.040000000000000008*dt;
                T tmp_d43 = 0.16000000000000003*tmp_d14;
                T tmp_d44 = -5.3955000000000011*dt*tmp_d13 + tmp_d35*tmp_d9;
                T tmp_d45 = 0.040000000000000008*dt*tmp_d3*tmp_d34 - omega*tmp_d43 - tmp_d17*tmp_d3*tmp_d42 - tmp_d44;
                T tmp_d46 = tmp_d26/pow(tmp_d27, 2);
                T tmp_d47 = 1.0*tmp_d31;
                T tmp_d48 = dt*tmp_d43;
                T tmp_d49 = tmp_d37*tmp_d47 - tmp_d40*tmp_d48;
                T tmp_d50 = 0.080000000000000016*tmp_d22;
                T tmp_d51 = tmp_d12*tmp_d50;
                T tmp_d52 = -tmp_d33*tmp_d41 + tmp_d45*tmp_d47 + 2;
                T tmp_d53 = tmp_d30*tmp_d39;
                T tmp_d54 = tmp_d14*tmp_d50;
                T tmp_d55 = 0.5*tmp_d32;
                T tmp_d56 = tmp_d20*tmp_d23;
                kp.f_resid(0) = dt*(tmp_d10*tmp_d5 + v) + x;
                kp.f_resid(1) = theta + tmp_d23;
                kp.f_resid(2) = tmp_d26*tmp_d29 + v;
                kp.f_resid(3) = omega + tmp_d30*tmp_d31;
                kp.A.setZero();
                kp.A(0,0) = 1;
                kp.A(0,1) = dt*(-0.10000000000000001*dt*tmp_d2*tmp_d5/pow(tmp_d7, 2) + tmp_d10*(-tmp_d0*tmp_d4 - 0.98100000000000009*pow(tmp_d1, 2) + 0.98100000000000009*tmp_d6));
                kp.A(0,2) = dt;
                kp.A(0,3) = -omega*tmp_d1*tmp_d33*tmp_d8;
                kp.A(1,1) = dt*(tmp_d21*tmp_d37 - 0.080000000000000016*tmp_d38*tmp_d40) + 1;
                kp.A(1,3) = dt*(tmp_d21*tmp_d45 - 0.040000000000000008*tmp_d32*tmp_d41 + 1);
                kp.A(2,1) = tmp_d29*(-tmp_d13*tmp_d25 + 0.98100000000000009*tmp_d17 - 0.98100000000000009*tmp_d34 - tmp_d49*tmp_d51) - 0.20000000000000001*tmp_d38*tmp_d46;
                kp.A(2,2) = 1;
                kp.A(2,3) = dt*tmp_d28*(-0.040000000000000008*dt*tmp_d13*tmp_d24 + 0.49050000000000005*dt*tmp_d17 - 0.49050000000000005*dt*tmp_d34 - tmp_d51*tmp_d52) - 0.10000000000000001*tmp_d14*tmp_d32*tmp_d46;
                kp.A(3,1) = tmp_d31*(-tmp_d18*tmp_d24 + 0.080000000000000016*tmp_d24*tmp_d34 - tmp_d36 - tmp_d49*tmp_d54) - tmp_d48*tmp_d53;
                kp.A(3,3) = -tmp_d14*tmp_d33*tmp_d53 + tmp_d31*(0.040000000000000008*dt*tmp_d24*tmp_d34 - tmp_d17*tmp_d24*tmp_d42 - tmp_d44 - tmp_d52*tmp_d54) + 1;
                kp.B.setZero();
                kp.B(0,0) = tmp_d55*tmp_d8;
                kp.B(1,0) = tmp_d13*tmp_d20*tmp_d55;
                kp.B(2,0) = tmp_d29*(-0.080000000000000016*tmp_d14*tmp_d56 + 1);
                kp.B(3,0) = tmp_d31*(-tmp_d12*tmp_d18*tmp_d56 + tmp_d13);
                break;
            }
            case IntegratorType::RK4_EXPLICIT:
            case IntegratorType::RK4_IMPLICIT:
            {
                T tmp_d0 = cos(theta);
                T tmp_d1 = pow(tmp_d0, 2);
                T tmp_d2 = 1.1000000000000001 - 0.10000000000000001*tmp_d1;
                T tmp_d3 = 1.0/tmp_d2;
                T tmp_d4 = sin(theta);
                T tmp_d5 = tmp_d0*tmp_d4;
                T tmp_d6 = pow(omega, 2);
                T tmp_d7 = 0.080000000000000016*tmp_d6;
                T tmp_d8 = force - tmp_d4*tmp_d7 + 0.98100000000000009*tmp_d5;
                T tmp_d9 = tmp_d3*tmp_d8;
                T tmp_d10 = 1.0*dt;
                T tmp_d11 = 0.5*dt;
                T tmp_d12 = omega*tmp_d11 + theta;
                T tmp_d13 = sin(tmp_d12);
                T tmp_d14 = cos(tmp_d12);
                T tmp_d15 = tmp_d13*tmp_d14;
                T tmp_d16 = pow(tmp_d14, 2);
                T tmp_d17 = 0.080000000000000016*tmp_d16;
                T tmp_d18 = 0.88000000000000012 - tmp_d17;
                T tmp_d19 = 1.0/tmp_d18;
                T tmp_d20 = force*tmp_d14;
                T tmp_d21 = 10.791000000000002*tmp_d13 + tmp_d20;
                T tmp_d22 = -tmp_d15*tmp_d7 + tmp_d21;
                T tmp_d23 = tmp_d19*tmp_d22;
                T tmp_d24 = omega + tmp_d11*tmp_d23;
                T tmp_d25 = pow(tmp_d24, 2);
                T tmp_d26 = 0.080000000000000016*tmp_d25;
                T tmp_d27 = force - tmp_d13*tmp_d26 + 0.98100000000000009*tmp_d15;
                T tmp_d28 = 1.1000000000000001 - 0.10000000000000001*tmp_d16;
                T tmp_d29 = 1.0/tmp_d28;
                T tmp_d30 = tmp_d10*tmp_d29;
                T tmp_d31 = tmp_d11*tmp_d24;
                T tmp_d32 = theta + tmp_d31;
                T tmp_d33 = cos(tmp_d32);
                T tmp_d34 = pow(tmp_d33, 2);
                T tmp_d35 = 1.1000000000000001 - 0.10000000000000001*tmp_d34;
                T tmp_d36 = 1.0/tmp_d35;
                T tmp_d37 = sin(tmp_d32);
                T tmp_d38 = tmp_d33*tmp_d37;
                T tmp_d39 = tmp_d12 + tmp_d31;
                T tmp_d40 = sin(tmp_d39);
                T tmp_d41 = cos(tmp_d39);
                T tmp_d42 = tmp_d40*tmp_d41;
                T tmp_d43 = force*tmp_d41 + 10.791000000000002*tmp_d40;
                T tmp_d44 = -tmp_d42*tmp_d7 + tmp_d43;
                T tmp_d45 = pow(tmp_d41, 2);
                T tmp_d46 = 0.080000000000000016*tmp_d45;
                T tmp_d47 = 0.88000000000000012 - tmp_d46;
                T tmp_d48 = 1.0/tmp_d47;
                T tmp_d49 = tmp_d11*tmp_d48;
                T tmp_d50 = omega + tmp_d44*tmp_d49;
                T tmp_d51 = pow(tmp_d50, 2);
                T tmp_d52 = -0.080000000000000016*tmp_d42*tmp_d51 + tmp_d43;
                T tmp_d53 = omega + tmp_d49*tmp_d52;
                T tmp_d54 = pow(tmp_d53, 2);
                T tmp_d55 = 0.080000000000000016*tmp_d54;
                T tmp_d56 = force - tmp_d37*tmp_d55 + 0.98100000000000009*tmp_d38;
                T tmp_d57 = tmp_d36*tmp_d56;
                T tmp_d58 = 0.16666666666666666*dt;
                T tmp_d59 = tmp_d10*tmp_d48;
                T tmp_d60 = dt*tmp_d53;
                T tmp_d61 = tmp_d12 + tmp_d60;
                T tmp_d62 = sin(tmp_d61);
                T tmp_d63 = cos(tmp_d61);
                T tmp_d64 = tmp_d62*tmp_d63;
                T tmp_d65 = force*tmp_d63 + 10.791000000000002*tmp_d62 - tmp_d64*tmp_d7;
                T tmp_d66 = 0.080000000000000016*pow(tmp_d63, 2);
                T tmp_d67 = 0.88000000000000012 - tmp_d66;
                T tmp_d68 = 1.0/tmp_d67;
                T tmp_d69 = tmp_d11*tmp_d68;
                T tmp_d70 = tmp_d11*(omega + tmp_d65*tmp_d69);
                T tmp_d71 = theta + tmp_d60;
                T tmp_d72 = tmp_d70 + tmp_d71;
                T tmp_d73 = sin(tmp_d72);
                T tmp_d74 = cos(tmp_d72);
                T tmp_d75 = tmp_d61 + tmp_d70;
                T tmp_d76 = sin(tmp_d75);
                T tmp_d77 = cos(tmp_d75);
                T tmp_d78 = tmp_d76*tmp_d77;
                T tmp_d79 = force*tmp_d77 + 10.791000000000002*tmp_d76;
                T tmp_d80 = -tmp_d7*tmp_d78 + tmp_d79;
                T tmp_d81 = 0.080000000000000016*pow(tmp_d77, 2);
                T tmp_d82 = 0.88000000000000012 - tmp_d81;
                T tmp_d83 = 1.0/tmp_d82;
                T tmp_d84 = tmp_d11*tmp_d83;
                T tmp_d85 = omega + tmp_d80*tmp_d84;
                T tmp_d86 = pow(tmp_d85, 2);
                T tmp_d87 = -0.080000000000000016*tmp_d78*tmp_d86 + tmp_d79;
                T tmp_d88 = omega + tmp_d84*tmp_d87;
                T tmp_d89 = pow(tmp_d88, 2);
                T tmp_d90 = tmp_d73*tmp_d74;
                T tmp_d91 = force*tmp_d74 + 10.791000000000002*tmp_d73 - 0.080000000000000016*tmp_d89*tmp_d90;
                T tmp_d92 = 0.080000000000000016*pow(tmp_d74, 2);
                T tmp_d93 = 0.88000000000000012 - tmp_d92;
                T tmp_d94 = dt/tmp_d93;
                T tmp_d95 = tmp_d91*tmp_d94;
                T tmp_d96 = 2*tmp_d29;
                T tmp_d97 = cos(tmp_d71);
                T tmp_d98 = pow(tmp_d97, 2);
                T tmp_d99 = 1.1000000000000001 - 0.10000000000000001*tmp_d98;
                T tmp_d100 = 1.0/tmp_d99;
                T tmp_d101 = sin(tmp_d71);
                T tmp_d102 = tmp_d101*tmp_d97;
                T tmp_d103 = omega + tmp_d95;
                T tmp_d104 = pow(tmp_d103, 2);
                T tmp_d105 = 0.080000000000000016*tmp_d104;
                T tmp_d106 = force - tmp_d101*tmp_d105 + 0.98100000000000009*tmp_d102;
                T tmp_d107 = 0.080000000000000016*tmp_d1;
                T tmp_d108 = 0.88000000000000012 - tmp_d107;
                T tmp_d109 = 1.0/tmp_d108;
                T tmp_d110 = force*tmp_d0 + 10.791000000000002*tmp_d4 - tmp_d5*tmp_d7;
                T tmp_d111 = -tmp_d15*tmp_d26 + tmp_d21;
                T tmp_d112 = 2*tmp_d19;
                T tmp_d113 = force*tmp_d33 + 10.791000000000002*tmp_d37 - tmp_d38*tmp_d55;
                T tmp_d114 = 0.080000000000000016*tmp_d34;
                T tmp_d115 = 0.88000000000000012 - tmp_d114;
                T tmp_d116 = 1.0/tmp_d115;
                T tmp_d117 = 2*tmp_d116;
                T tmp_d118 = 0.080000000000000016*tmp_d98;
                T tmp_d119 = 0.88000000000000012 - tmp_d118;
                T tmp_d120 = 1.0/tmp_d119;
                T tmp_d121 = force*tmp_d97 + 10.791000000000002*tmp_d101 - tmp_d102*tmp_d105;
                T tmp_d122 = 0.20000000000000001*tmp_d5*tmp_d8/pow(tmp_d2, 2);
                T tmp_d123 = pow(tmp_d4, 2);
                T tmp_d124 = tmp_d3*(-tmp_d0*tmp_d7 + 0.98100000000000009*tmp_d1 - 0.98100000000000009*tmp_d123);
                T tmp_d125 = tmp_d15*tmp_d27/pow(tmp_d28, 2);
                T tmp_d126 = 0.20000000000000001*dt;
                T tmp_d127 = tmp_d125*tmp_d126;
                T tmp_d128 = pow(tmp_d13, 2);
                T tmp_d129 = force*tmp_d13;
                T tmp_d130 = tmp_d129 - 10.791000000000002*tmp_d14;
                T tmp_d131 = tmp_d19*(0.080000000000000016*tmp_d128*tmp_d6 - tmp_d130 - tmp_d17*tmp_d6);
                T tmp_d132 = tmp_d15/pow(tmp_d18, 2);
                T tmp_d133 = tmp_d132*tmp_d22;
                T tmp_d134 = 0.16000000000000003*dt;
                T tmp_d135 = tmp_d10*tmp_d131 - tmp_d133*tmp_d134;
                T tmp_d136 = 0.080000000000000016*tmp_d24;
                T tmp_d137 = tmp_d13*tmp_d136;
                T tmp_d138 = -0.98100000000000009*tmp_d128 - tmp_d135*tmp_d137 - tmp_d14*tmp_d26 + 0.98100000000000009*tmp_d16;
                T tmp_d139 = 0.080000000000000016*dt;
                T tmp_d140 = tmp_d11*(tmp_d11*tmp_d131 - tmp_d133*tmp_d139) + 1;
                T tmp_d141 = tmp_d140*tmp_d38;
                T tmp_d142 = tmp_d56/pow(tmp_d35, 2);
                T tmp_d143 = tmp_d141*tmp_d142;
                T tmp_d144 = pow(tmp_d37, 2);
                T tmp_d145 = pow(tmp_d47, -2);
                T tmp_d146 = tmp_d140*tmp_d145;
                T tmp_d147 = tmp_d134*tmp_d42;
                T tmp_d148 = tmp_d146*tmp_d147;
                T tmp_d149 = tmp_d140*tmp_d46;
                T tmp_d150 = pow(tmp_d40, 2);
                T tmp_d151 = force*tmp_d40;
                T tmp_d152 = tmp_d140*tmp_d151 - 10.791000000000002*tmp_d140*tmp_d41;
                T tmp_d153 = 0.080000000000000016*tmp_d42;
                T tmp_d154 = tmp_d153*tmp_d50;
                T tmp_d155 = 0.080000000000000016*tmp_d140*tmp_d150*tmp_d51 - tmp_d149*tmp_d51 - tmp_d152 - tmp_d154*(-tmp_d148*tmp_d44 + tmp_d59*(0.080000000000000016*tmp_d140*tmp_d150*tmp_d6 - tmp_d149*tmp_d6 - tmp_d152));
                T tmp_d156 = -tmp_d148*tmp_d52 + tmp_d155*tmp_d59;
                T tmp_d157 = 0.080000000000000016*tmp_d53;
                T tmp_d158 = tmp_d157*tmp_d37;
                T tmp_d159 = -0.98100000000000009*tmp_d140*tmp_d144 - tmp_d140*tmp_d33*tmp_d55 + 0.98100000000000009*tmp_d140*tmp_d34 - tmp_d156*tmp_d158;
                T tmp_d160 = dt*tmp_d36;
                T tmp_d161 = 0.16000000000000003*omega;
                T tmp_d162 = tmp_d161*tmp_d4;
                T tmp_d163 = tmp_d162*tmp_d3;
                T tmp_d164 = pow(dt, 2);
                T tmp_d165 = 0.10000000000000001*tmp_d164;
                T tmp_d166 = 0.49050000000000005*dt;
                T tmp_d167 = 0.040000000000000008*dt;
                T tmp_d168 = -5.3955000000000011*dt*tmp_d14 + tmp_d11*tmp_d129;
                T tmp_d169 = tmp_d19*(0.040000000000000008*dt*tmp_d128*tmp_d6 - tmp_d15*tmp_d161 - tmp_d16*tmp_d167*tmp_d6 - tmp_d168);
                T tmp_d170 = tmp_d133*tmp_d164;
                T tmp_d171 = tmp_d10*tmp_d169 - 0.080000000000000016*tmp_d170;
                T tmp_d172 = tmp_d171 + 2;
                T tmp_d173 = -0.040000000000000008*dt*tmp_d14*tmp_d25 + 0.49050000000000005*dt*tmp_d16 - tmp_d128*tmp_d166 - tmp_d137*tmp_d172;
                T tmp_d174 = tmp_d11*tmp_d169 - 0.040000000000000008*tmp_d170 + 1;
                T tmp_d175 = tmp_d174*tmp_d38;
                T tmp_d176 = tmp_d142*tmp_d175;
                T tmp_d177 = tmp_d167*tmp_d54;
                T tmp_d178 = tmp_d11*tmp_d174;
                T tmp_d179 = tmp_d11 + tmp_d178;
                T tmp_d180 = tmp_d179*tmp_d46;
                T tmp_d181 = tmp_d145*tmp_d179;
                T tmp_d182 = tmp_d147*tmp_d181;
                T tmp_d183 = tmp_d151*tmp_d179 - 10.791000000000002*tmp_d179*tmp_d41;
                T tmp_d184 = 0.080000000000000016*tmp_d150*tmp_d179*tmp_d51 - tmp_d154*(-tmp_d182*tmp_d44 + tmp_d59*(0.080000000000000016*tmp_d150*tmp_d179*tmp_d6 - tmp_d161*tmp_d42 - tmp_d180*tmp_d6 - tmp_d183) + 2) - tmp_d180*tmp_d51 - tmp_d183;
                T tmp_d185 = -tmp_d182*tmp_d52 + tmp_d184*tmp_d59;
                T tmp_d186 = tmp_d185 + 2;
                T tmp_d187 = 0.49050000000000005*dt*tmp_d174*tmp_d34 - tmp_d144*tmp_d166*tmp_d174 - tmp_d158*tmp_d186 - tmp_d174*tmp_d177*tmp_d33;
                T tmp_d188 = dt*tmp_d153*tmp_d52;
                T tmp_d189 = dt*(-tmp_d146*tmp_d188 + tmp_d155*tmp_d49) + 1;
                T tmp_d190 = tmp_d64*tmp_d65/pow(tmp_d67, 2);
                T tmp_d191 = tmp_d139*tmp_d190;
                T tmp_d192 = force*tmp_d62;
                T tmp_d193 = pow(tmp_d62, 2);
                T tmp_d194 = tmp_d6*tmp_d66;
                T tmp_d195 = tmp_d11*(-tmp_d189*tmp_d191 + tmp_d69*(-tmp_d189*tmp_d192 + 0.080000000000000016*tmp_d189*tmp_d193*tmp_d6 - tmp_d189*tmp_d194 + 10.791000000000002*tmp_d189*tmp_d63)) + tmp_d189;
                T tmp_d196 = force*tmp_d195;
                T tmp_d197 = pow(tmp_d73, 2);
                T tmp_d198 = tmp_d89*tmp_d92;
                T tmp_d199 = tmp_d134*tmp_d195;
                T tmp_d200 = tmp_d78/pow(tmp_d82, 2);
                T tmp_d201 = tmp_d199*tmp_d200;
                T tmp_d202 = tmp_d195*tmp_d81;
                T tmp_d203 = pow(tmp_d76, 2);
                T tmp_d204 = -10.791000000000002*tmp_d195*tmp_d77 + tmp_d196*tmp_d76;
                T tmp_d205 = tmp_d10*tmp_d83;
                T tmp_d206 = 0.080000000000000016*tmp_d78*tmp_d85;
                T tmp_d207 = 0.080000000000000016*tmp_d88*tmp_d90;
                T tmp_d208 = tmp_d94*(0.080000000000000016*tmp_d195*tmp_d197*tmp_d89 - tmp_d195*tmp_d198 + 10.791000000000002*tmp_d195*tmp_d74 - tmp_d196*tmp_d73 - tmp_d207*(-tmp_d201*tmp_d87 + tmp_d205*(0.080000000000000016*tmp_d195*tmp_d203*tmp_d86 - tmp_d202*tmp_d86 - tmp_d204 - tmp_d206*(-tmp_d201*tmp_d80 + tmp_d205*(0.080000000000000016*tmp_d195*tmp_d203*tmp_d6 - tmp_d202*tmp_d6 - tmp_d204)))));
                T tmp_d209 = tmp_d90*tmp_d91/pow(tmp_d93, 2);
                T tmp_d210 = -tmp_d181*tmp_d188 + tmp_d184*tmp_d49 + 1;
                T tmp_d211 = dt*tmp_d210;
                T tmp_d212 = tmp_d11 + tmp_d211;
                T tmp_d213 = tmp_d11*(-tmp_d191*tmp_d212 + tmp_d69*(-tmp_d161*tmp_d64 - tmp_d192*tmp_d212 + 0.080000000000000016*tmp_d193*tmp_d212*tmp_d6 - tmp_d194*tmp_d212 + 10.791000000000002*tmp_d212*tmp_d63) + 1);
                T tmp_d214 = tmp_d211 + tmp_d213;
                T tmp_d215 = force*tmp_d73;
                T tmp_d216 = tmp_d212 + tmp_d213;
                T tmp_d217 = tmp_d134*tmp_d200;
                T tmp_d218 = tmp_d216*tmp_d217;
                T tmp_d219 = tmp_d216*tmp_d81;
                T tmp_d220 = force*tmp_d76;
                T tmp_d221 = tmp_d216*tmp_d220 - 10.791000000000002*tmp_d216*tmp_d77;
                T tmp_d222 = tmp_d94*(0.080000000000000016*tmp_d197*tmp_d214*tmp_d89 - tmp_d198*tmp_d214 - tmp_d207*(tmp_d205*(0.080000000000000016*tmp_d203*tmp_d216*tmp_d86 - tmp_d206*(tmp_d205*(-tmp_d161*tmp_d78 + 0.080000000000000016*tmp_d203*tmp_d216*tmp_d6 - tmp_d219*tmp_d6 - tmp_d221) - tmp_d218*tmp_d80 + 2) - tmp_d219*tmp_d86 - tmp_d221) - tmp_d218*tmp_d87 + 2) - tmp_d214*tmp_d215 + 10.791000000000002*tmp_d214*tmp_d74);
                T tmp_d223 = tmp_d209*tmp_d214;
                T tmp_d224 = 2*tmp_d36;
                T tmp_d225 = tmp_d102*tmp_d189;
                T tmp_d226 = 0.20000000000000001*tmp_d106/pow(tmp_d99, 2);
                T tmp_d227 = pow(tmp_d101, 2);
                T tmp_d228 = 0.32000000000000006*dt;
                T tmp_d229 = tmp_d209*tmp_d228;
                T tmp_d230 = -tmp_d195*tmp_d229 + 2*tmp_d208;
                T tmp_d231 = 0.080000000000000016*tmp_d103;
                T tmp_d232 = tmp_d101*tmp_d231;
                T tmp_d233 = tmp_d102*tmp_d211;
                T tmp_d234 = tmp_d105*tmp_d97;
                T tmp_d235 = 2*tmp_d222 - tmp_d223*tmp_d228 + 2;
                T tmp_d236 = tmp_d111*tmp_d132;
                T tmp_d237 = tmp_d136*tmp_d15;
                T tmp_d238 = tmp_d113/pow(tmp_d115, 2);
                T tmp_d239 = force*tmp_d37;
                T tmp_d240 = tmp_d157*tmp_d38;
                T tmp_d241 = 0.16000000000000003*tmp_d121/pow(tmp_d119, 2);
                T tmp_d242 = force*tmp_d101;
                T tmp_d243 = tmp_d104*tmp_d118;
                T tmp_d244 = tmp_d102*tmp_d231;
                T tmp_d245 = tmp_d0*tmp_d109;
                T tmp_d246 = dt*tmp_d19;
                T tmp_d247 = -tmp_d237*tmp_d246 + 1;
                T tmp_d248 = tmp_d14*tmp_d19;
                T tmp_d249 = pow(dt, 3)*tmp_d248;
                T tmp_d250 = tmp_d142*tmp_d38;
                T tmp_d251 = tmp_d164*tmp_d248;
                T tmp_d252 = 0.020000000000000004*tmp_d251;
                T tmp_d253 = tmp_d252*tmp_d54;
                T tmp_d254 = tmp_d145*tmp_d249*tmp_d42;
                T tmp_d255 = 0.040000000000000008*tmp_d254;
                T tmp_d256 = tmp_d255*tmp_d52;
                T tmp_d257 = 0.25*tmp_d164*tmp_d19*tmp_d20;
                T tmp_d258 = tmp_d257*tmp_d40;
                T tmp_d259 = tmp_d252*tmp_d6;
                T tmp_d260 = 0.020000000000000004*tmp_d14*tmp_d150*tmp_d164*tmp_d19*tmp_d51 + 2.6977500000000005*tmp_d14*tmp_d164*tmp_d19*tmp_d41 - tmp_d154*(1.0*dt*tmp_d48*(tmp_d150*tmp_d259 + 2.6977500000000005*tmp_d251*tmp_d41 - tmp_d258 - tmp_d259*tmp_d45 + tmp_d41) - tmp_d255*tmp_d44) - tmp_d252*tmp_d45*tmp_d51 - tmp_d258 + tmp_d41;
                T tmp_d261 = 1.0*dt*tmp_d260*tmp_d48 - tmp_d256;
                T tmp_d262 = 0.24525000000000002*tmp_d14*tmp_d164*tmp_d19*tmp_d34 - 0.24525000000000002*tmp_d144*tmp_d251 - tmp_d158*tmp_d261 - tmp_d253*tmp_d33 + 1;
                T tmp_d263 = 0.5*dt*tmp_d260*tmp_d48 - 0.020000000000000004*tmp_d254*tmp_d52;
                T tmp_d264 = dt*tmp_d263;
                T tmp_d265 = tmp_d11*(0.5*dt*tmp_d68*(-tmp_d192*tmp_d264 + tmp_d193*tmp_d264*tmp_d7 - tmp_d194*tmp_d264 + 10.791000000000002*tmp_d264*tmp_d63 + tmp_d63) - 0.080000000000000016*tmp_d164*tmp_d190*tmp_d263) + tmp_d264;
                T tmp_d266 = tmp_d217*tmp_d265;
                T tmp_d267 = tmp_d220*tmp_d265;
                T tmp_d268 = tmp_d265*tmp_d81;
                T tmp_d269 = tmp_d94*(0.080000000000000016*tmp_d197*tmp_d265*tmp_d89 - tmp_d198*tmp_d265 - tmp_d207*(tmp_d205*(0.080000000000000016*tmp_d203*tmp_d265*tmp_d86 - tmp_d206*(tmp_d205*(tmp_d203*tmp_d265*tmp_d7 + 10.791000000000002*tmp_d265*tmp_d77 - tmp_d267 - tmp_d268*tmp_d6 + tmp_d77) - tmp_d266*tmp_d80) + 10.791000000000002*tmp_d265*tmp_d77 - tmp_d267 - tmp_d268*tmp_d86 + tmp_d77) - tmp_d266*tmp_d87) - tmp_d215*tmp_d265 + 10.791000000000002*tmp_d265*tmp_d74 + tmp_d74);
                T tmp_d270 = tmp_d102*tmp_d264;
                T tmp_d271 = -tmp_d229*tmp_d265 + 2*tmp_d269;
                kp.f_resid(0) = tmp_d58*(dt*tmp_d57 + tmp_d10*tmp_d9 + tmp_d27*tmp_d30 + 6*v) + x;
                kp.f_resid(1) = theta + tmp_d58*(6*omega + tmp_d10*tmp_d23 + tmp_d52*tmp_d59 + tmp_d95);
                kp.f_resid(2) = tmp_d58*(tmp_d100*tmp_d106 + tmp_d27*tmp_d96 + 2*tmp_d57 + tmp_d9) + v;
                kp.f_resid(3) = omega + tmp_d58*(tmp_d109*tmp_d110 + tmp_d111*tmp_d112 + tmp_d113*tmp_d117 + tmp_d120*tmp_d121);
                kp.A.setZero();
                kp.A(0,0) = 1;
                kp.A(0,1) = tmp_d58*(-dt*tmp_d122 + tmp_d10*tmp_d124 - tmp_d126*tmp_d143 - tmp_d127 + tmp_d138*tmp_d30 + tmp_d159*tmp_d160);
                kp.A(0,2) = tmp_d10;
                kp.A(0,3) = tmp_d58*(-dt*tmp_d163 + 1.0*dt*tmp_d173*tmp_d29 + dt*tmp_d187*tmp_d36 - tmp_d125*tmp_d165 - tmp_d165*tmp_d176);
                kp.A(1,1) = tmp_d58*(tmp_d135 + tmp_d156 - tmp_d199*tmp_d209 + tmp_d208) + 1;
                kp.A(1,3) = tmp_d58*(-tmp_d134*tmp_d223 + tmp_d171 + tmp_d185 + tmp_d222 + 6);
                kp.A(2,1) = tmp_d58*(tmp_d100*(-tmp_d105*tmp_d189*tmp_d97 - 0.98100000000000009*tmp_d189*tmp_d227 + 0.98100000000000009*tmp_d189*tmp_d98 - tmp_d230*tmp_d232) - tmp_d122 + tmp_d124 - 0.40000000000000002*tmp_d125 + tmp_d138*tmp_d96 - 0.40000000000000002*tmp_d143 + tmp_d159*tmp_d224 - tmp_d225*tmp_d226);
                kp.A(2,2) = 1;
                kp.A(2,3) = tmp_d58*(tmp_d100*(0.98100000000000009*dt*tmp_d210*tmp_d98 - 0.98100000000000009*tmp_d211*tmp_d227 - tmp_d211*tmp_d234 - tmp_d232*tmp_d235) - tmp_d126*tmp_d176 - tmp_d127 - tmp_d163 + 2*tmp_d173*tmp_d29 + 2*tmp_d187*tmp_d36 - tmp_d226*tmp_d233);
                kp.A(3,1) = tmp_d58*(tmp_d109*(-force*tmp_d4 + 10.791000000000002*tmp_d0 - tmp_d107*tmp_d6 + 0.080000000000000016*tmp_d123*tmp_d6) + tmp_d112*(0.080000000000000016*tmp_d128*tmp_d25 - tmp_d130 - tmp_d135*tmp_d237 - tmp_d17*tmp_d25) + tmp_d117*(-tmp_d114*tmp_d140*tmp_d54 + 0.080000000000000016*tmp_d140*tmp_d144*tmp_d54 - tmp_d140*tmp_d239 + 10.791000000000002*tmp_d140*tmp_d33 - tmp_d156*tmp_d240) + tmp_d120*(0.080000000000000016*tmp_d104*tmp_d189*tmp_d227 - tmp_d189*tmp_d242 - tmp_d189*tmp_d243 + 10.791000000000002*tmp_d189*tmp_d97 - tmp_d230*tmp_d244) - 0.32000000000000006*tmp_d141*tmp_d238 - tmp_d225*tmp_d241 - 0.32000000000000006*tmp_d236 - 0.16000000000000003*tmp_d110*tmp_d5/pow(tmp_d108, 2));
                kp.A(3,3) = tmp_d58*(2*tmp_d116*(0.040000000000000008*dt*tmp_d144*tmp_d174*tmp_d54 + 5.3955000000000011*dt*tmp_d174*tmp_d33 - tmp_d174*tmp_d177*tmp_d34 - tmp_d178*tmp_d239 - tmp_d186*tmp_d240) + tmp_d120*(0.080000000000000016*dt*tmp_d104*tmp_d210*tmp_d227 + 10.791000000000002*dt*tmp_d210*tmp_d97 - tmp_d211*tmp_d242 - tmp_d211*tmp_d243 - tmp_d235*tmp_d244) - tmp_d134*tmp_d175*tmp_d238 - tmp_d134*tmp_d236 - tmp_d162*tmp_d245 + 2*tmp_d19*(0.040000000000000008*dt*tmp_d128*tmp_d25 - tmp_d16*tmp_d167*tmp_d25 - tmp_d168 - tmp_d172*tmp_d237) - tmp_d233*tmp_d241) + 1;
                kp.B.setZero();
                kp.B(0,0) = tmp_d58*(tmp_d10*tmp_d3 + tmp_d160*tmp_d262 + tmp_d247*tmp_d30 - 0.050000000000000003*tmp_d249*tmp_d250);
                kp.B(1,0) = tmp_d58*(tmp_d10*tmp_d248 - tmp_d134*tmp_d209*tmp_d265 - tmp_d256 + tmp_d260*tmp_d59 + tmp_d269);
                kp.B(2,0) = tmp_d58*(tmp_d100*(0.98100000000000009*dt*tmp_d263*tmp_d98 - 0.98100000000000009*tmp_d227*tmp_d264 - tmp_d232*tmp_d271 - tmp_d234*tmp_d264 + 1) + tmp_d224*tmp_d262 - tmp_d226*tmp_d270 + tmp_d247*tmp_d96 - 0.10000000000000001*tmp_d250*tmp_d251 + tmp_d3);
                kp.B(3,0) = tmp_d58*(tmp_d112*(-tmp_d13*tmp_d17*tmp_d24*tmp_d246 + tmp_d14) + tmp_d117*(0.020000000000000004*tmp_d14*tmp_d144*tmp_d164*tmp_d19*tmp_d54 + 2.6977500000000005*tmp_d14*tmp_d164*tmp_d19*tmp_d33 - tmp_d240*tmp_d261 - tmp_d253*tmp_d34 - tmp_d257*tmp_d37 + tmp_d33) + tmp_d120*(0.080000000000000016*dt*tmp_d104*tmp_d227*tmp_d263 + 10.791000000000002*dt*tmp_d263*tmp_d97 - tmp_d242*tmp_d264 - tmp_d243*tmp_d264 - tmp_d244*tmp_d271 + tmp_d97) - 0.080000000000000016*tmp_d238*tmp_d251*tmp_d38 - tmp_d241*tmp_d270 + tmp_d245);
                break;
            }
            default:
                break;
        }
    }

    // --- 2. Compute Constraints (g_val, C, D) ---
    template<typename T>
    static void compute_constraints(KnotPoint<T,NX,NU,NC,NP>& kp) {
        T force = kp.u(0);

        // --- Special Constraints Pre-Calculation ---


        // g_val
        kp.g_val(0,0) = force - 80.0;
        kp.g_val(1,0) = -force - 80.0;

        // C
        kp.C(0,0) = 0;
        kp.C(0,1) = 0;
        kp.C(0,2) = 0;
        kp.C(0,3) = 0;
        kp.C(1,0) = 0;
        kp.C(1,1) = 0;
        kp.C(1,2) = 0;
        kp.C(1,3) = 0;

        // D
        kp.D(0,0) = 1;
        kp.D(1,0) = -1;

    }

    // --- 3. Compute Cost (Implemented via template for Exact/GN) ---
    template<typename T, int Mode>
    static void compute_cost_impl(KnotPoint<T,NX,NU,NC,NP>& kp) {
        T x = kp.x(0);
        T theta = kp.x(1);
        T v = kp.x(2);
        T omega = kp.x(3);
        T force = kp.u(0);


        // q
        kp.q(0,0) = 4000.0*x;
        kp.q(1,0) = 4000.0*theta;
        kp.q(2,0) = 0.040000000000000001*v;
        kp.q(3,0) = 0.040000000000000001*omega;

        // r
        kp.r(0,0) = 0.040000000000000001*force;

        // Q (Mode 0=GN, 1=Exact)
        kp.Q(0,0) = 4000.0;
        kp.Q(0,1) = 0;
        kp.Q(0,2) = 0;
        kp.Q(0,3) = 0;
        kp.Q(1,0) = 0;
        kp.Q(1,1) = 4000.0;
        kp.Q(1,2) = 0;
        kp.Q(1,3) = 0;
        kp.Q(2,0) = 0;
        kp.Q(2,1) = 0;
        kp.Q(2,2) = 0.040000000000000001;
        kp.Q(2,3) = 0;
        kp.Q(3,0) = 0;
        kp.Q(3,1) = 0;
        kp.Q(3,2) = 0;
        kp.Q(3,3) = 0.040000000000000001;

        // R (Mode 0=GN, 1=Exact)
        kp.R(0,0) = 0.040000000000000001;

        // H (Mode 0=GN, 1=Exact)
        kp.H(0,0) = 0;
        kp.H(0,1) = 0;
        kp.H(0,2) = 0;
        kp.H(0,3) = 0;

        kp.cost = 0.02*pow(force, 2) + 0.02*pow(omega, 2) + 2000.0*pow(theta, 2) + 0.02*pow(v, 2) + 2000.0*pow(x, 2);
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
        T P_1_1 = Vxx(1,1);
        T P_1_2 = Vxx(1,2);
        T P_1_3 = Vxx(1,3);
        T P_2_2 = Vxx(2,2);
        T P_2_3 = Vxx(2,3);
        T P_3_3 = Vxx(3,3);
        T p_0 = Vx(0);
        T p_1 = Vx(1);
        T p_2 = Vx(2);
        T p_3 = Vx(3);
        T A_0_0 = kp.A(0,0);
        T A_0_1 = kp.A(0,1);
        T A_0_2 = kp.A(0,2);
        T A_0_3 = kp.A(0,3);
        T A_1_1 = kp.A(1,1);
        T A_1_3 = kp.A(1,3);
        T A_2_1 = kp.A(2,1);
        T A_2_2 = kp.A(2,2);
        T A_2_3 = kp.A(2,3);
        T A_3_1 = kp.A(3,1);
        T A_3_3 = kp.A(3,3);
        T B_0_0 = kp.B(0,0);
        T B_1_0 = kp.B(1,0);
        T B_2_0 = kp.B(2,0);
        T B_3_0 = kp.B(3,0);

        // CSE Intermediate Variables
        T tmp_ric0 = A_0_1*P_0_0;
        T tmp_ric1 = A_1_1*P_0_1;
        T tmp_ric2 = A_2_1*P_0_2;
        T tmp_ric3 = A_3_1*P_0_3;
        T tmp_ric4 = A_0_2*P_0_0;
        T tmp_ric5 = A_2_2*P_0_2;
        T tmp_ric6 = A_0_3*P_0_0;
        T tmp_ric7 = A_1_3*P_0_1;
        T tmp_ric8 = A_2_3*P_0_2;
        T tmp_ric9 = A_3_3*P_0_3;
        T tmp_ric10 = tmp_ric0 + tmp_ric1 + tmp_ric2 + tmp_ric3;
        T tmp_ric11 = A_0_1*P_0_1 + A_1_1*P_1_1 + A_2_1*P_1_2 + A_3_1*P_1_3;
        T tmp_ric12 = A_0_1*P_0_2 + A_1_1*P_1_2 + A_2_1*P_2_2 + A_3_1*P_2_3;
        T tmp_ric13 = A_0_1*P_0_3 + A_1_1*P_1_3 + A_2_1*P_2_3 + A_3_1*P_3_3;
        T tmp_ric14 = tmp_ric4 + tmp_ric5;
        T tmp_ric15 = A_0_2*P_0_2 + A_2_2*P_2_2;
        T tmp_ric16 = B_0_0*P_0_0 + B_1_0*P_0_1 + B_2_0*P_0_2 + B_3_0*P_0_3;
        T tmp_ric17 = B_0_0*P_0_1 + B_1_0*P_1_1 + B_2_0*P_1_2 + B_3_0*P_1_3;
        T tmp_ric18 = B_0_0*P_0_2 + B_1_0*P_1_2 + B_2_0*P_2_2 + B_3_0*P_2_3;
        T tmp_ric19 = B_0_0*P_0_3 + B_1_0*P_1_3 + B_2_0*P_2_3 + B_3_0*P_3_3;

        // Accumulate Results
        kp.Q_bar(0,0) += pow(A_0_0, 2)*P_0_0;
        kp.Q_bar(0,1) += A_0_0*tmp_ric0 + A_0_0*tmp_ric1 + A_0_0*tmp_ric2 + A_0_0*tmp_ric3;
        kp.Q_bar(0,2) += A_0_0*tmp_ric4 + A_0_0*tmp_ric5;
        kp.Q_bar(0,3) += A_0_0*tmp_ric6 + A_0_0*tmp_ric7 + A_0_0*tmp_ric8 + A_0_0*tmp_ric9;
        kp.Q_bar(1,1) += A_0_1*tmp_ric10 + A_1_1*tmp_ric11 + A_2_1*tmp_ric12 + A_3_1*tmp_ric13;
        kp.Q_bar(1,2) += A_0_2*tmp_ric10 + A_2_2*tmp_ric12;
        kp.Q_bar(1,3) += A_0_3*tmp_ric10 + A_1_3*tmp_ric11 + A_2_3*tmp_ric12 + A_3_3*tmp_ric13;
        kp.Q_bar(2,2) += A_0_2*tmp_ric14 + A_2_2*tmp_ric15;
        kp.Q_bar(2,3) += A_0_3*tmp_ric14 + A_1_3*(A_0_2*P_0_1 + A_2_2*P_1_2) + A_2_3*tmp_ric15 + A_3_3*(A_0_2*P_0_3 + A_2_2*P_2_3);
        kp.Q_bar(3,3) += A_0_3*(tmp_ric6 + tmp_ric7 + tmp_ric8 + tmp_ric9) + A_1_3*(A_0_3*P_0_1 + A_1_3*P_1_1 + A_2_3*P_1_2 + A_3_3*P_1_3) + A_2_3*(A_0_3*P_0_2 + A_1_3*P_1_2 + A_2_3*P_2_2 + A_3_3*P_2_3) + A_3_3*(A_0_3*P_0_3 + A_1_3*P_1_3 + A_2_3*P_2_3 + A_3_3*P_3_3);
        kp.R_bar(0,0) += B_0_0*tmp_ric16 + B_1_0*tmp_ric17 + B_2_0*tmp_ric18 + B_3_0*tmp_ric19;
        kp.H_bar(0,0) += A_0_0*tmp_ric16;
        kp.H_bar(0,1) += A_0_1*tmp_ric16 + A_1_1*tmp_ric17 + A_2_1*tmp_ric18 + A_3_1*tmp_ric19;
        kp.H_bar(0,2) += A_0_2*tmp_ric16 + A_2_2*tmp_ric18;
        kp.H_bar(0,3) += A_0_3*tmp_ric16 + A_1_3*tmp_ric17 + A_2_3*tmp_ric18 + A_3_3*tmp_ric19;
        kp.q_bar(0,0) += A_0_0*p_0;
        kp.q_bar(1,0) += A_0_1*p_0 + A_1_1*p_1 + A_2_1*p_2 + A_3_1*p_3;
        kp.q_bar(2,0) += A_0_2*p_0 + A_2_2*p_2;
        kp.q_bar(3,0) += A_0_3*p_0 + A_1_3*p_1 + A_2_3*p_2 + A_3_3*p_3;
        kp.r_bar(0,0) += B_0_0*p_0 + B_1_0*p_1 + B_2_0*p_2 + B_3_0*p_3;

        // Fill Lower Triangles (Symmetry)
        kp.Q_bar(1,0) = kp.Q_bar(0,1);
        kp.Q_bar(2,0) = kp.Q_bar(0,2);
        kp.Q_bar(3,0) = kp.Q_bar(0,3);
        kp.Q_bar(2,1) = kp.Q_bar(1,2);
        kp.Q_bar(3,1) = kp.Q_bar(1,3);
        kp.Q_bar(3,2) = kp.Q_bar(2,3);

    }
    
};
}
