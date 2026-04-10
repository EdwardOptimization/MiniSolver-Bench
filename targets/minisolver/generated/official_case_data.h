#pragma once

#include <array>

namespace minisolver_bench::generated {

inline constexpr int pendulum_horizon = 40;
inline constexpr int pendulum_steps = 100;
inline constexpr double pendulum_tf = 0.80000000000000004;
inline constexpr std::array<double, 4> pendulum_x0 = {0, 3.1415926535897931, 0, 0};

inline constexpr int race_cars_horizon = 50;
inline constexpr int race_cars_steps = 500;
inline constexpr double race_cars_tf = 1;
inline constexpr double race_cars_pathlength = 8.7104967000000002;
inline constexpr double race_cars_sref_n = 3;
inline constexpr std::array<double, 6> race_cars_x0 = {-2, 0, 0, 0, 0, 0};

inline constexpr int quadrotor_nav_horizon = 50;
inline constexpr int quadrotor_nav_steps = 2250;
inline constexpr double quadrotor_nav_tf = 1;
inline constexpr double quadrotor_nav_u_hov = 15.292;
inline constexpr double quadrotor_nav_s_ref = 0.1875;
inline constexpr double quadrotor_nav_s_max = 5.9000000000000004;
inline constexpr std::array<double, 20> quadrotor_nav_init_zeta = {0.050000000000000003, 0, 0, 1, 0, 0, 0, 0.02, 0, 0, 0, 0, 0, 0, 0, 0, 15.292, 15.292, 15.292, 15.292};

inline constexpr int chain_mass_steps = 25;
inline constexpr int chain_mass_n_mass = 5;
inline constexpr int chain_mass_wall_count = 4;
inline constexpr double chain_mass_ts = 0.20000000000000001;
inline constexpr double chain_mass_tf = 8;
inline constexpr double chain_mass_y_pos_wall = -0.050000000000000003;
inline constexpr double chain_mass_perturb_scale = 0.01;
inline constexpr int chain_mass_seed = 50;
inline constexpr std::array<double, 3> chain_mass_u_init = {-1, 1, 1};
inline constexpr std::array<double, 21> chain_mass_xrest = {0.19146386167133261, 0, -0.51455499131876781, 0.39600000000000002, 0, -0.69761511454571834, 0.60053613832866748, 0, -0.51455499131876781, 0.79200000000000004, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

}  // namespace minisolver_bench::generated
