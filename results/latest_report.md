# Latest Benchmark Report

Date: 2026-04-10

## Provenance

- `nmpc-bench`: `uncommitted-workspace`
- `acados`: `third_party/acados` at `47b961bcaf493de59048918ffaa585181466ea65`
- `MiniSolver`: local checkout `/home/quyaonan/workspace/MiniSolver`
  - branch: `master`
  - commit: `f541d49d967edd5a8d50d931c6ae7ca6873a2c1b`

## Summary

| Backend | Case | Steps | Success | Median ms | P95 ms | Max ms | Extra |
|---|---:|---:|---:|---:|---:|---:|---|
| `minisolver` | `pendulum_on_cart` | 100 | 0.75 | 0.033861 | 0.037074 | 0.046271 | `avg_iterations=1`, `max_constraint_violation=0`, `final_theta_abs=0.0038049` |
| `acados` | `pendulum_on_cart` | 100 | 1 | 0.5385 | 0.86675 | 1.434 | `avg_iterations=1`, `final_theta_abs=0.0247401` |
| `casadi` | `pendulum_on_cart` | 100 | 0.98 | 1.7181 | 284.457 | 1671.62 | `avg_iterations=3.64`, `final_theta_abs=0.0016704` |
| `minisolver` | `race_cars` | 500 | 0.906 | 0.902315 | 0.942156 | 1.25639 | `avg_iterations=1`, `max_constraint_violation=0.0430696`, `avg_speed=1.44975` |
| `acados` | `race_cars` | 344 | 1 | 0.721 | 0.9129 | 1.458 | `avg_speed=1.35207` |
| `casadi` | `race_cars` | 500 | 0 | 2.4762 | 2.52565 | 2937.7 | `avg_iterations=0.06`, `avg_speed=nan` |
| `minisolver` | `quadrotor_nav` | 1559 | 1 | 8.39711 | 8.63614 | 9.46972 | `avg_iterations=1`, `max_constraint_violation=0`, `avg_abs_n=0.0627374`, `avg_abs_b=0.000744301` |
| `acados` | `quadrotor_nav` | 1566 | 1 | 4.332 | 4.442 | 6.548 | `avg_abs_n=5.23977e-05`, `avg_abs_b=1.48661e-05` |
| `minisolver` | `chain_mass` | 25 | 1 | 2.1442 | 3.83935 | 5.73771 | `avg_iterations=3.56`, `max_constraint_violation=0`, `min_wall_dist=0.00112253` |
| `acados` | `chain_mass` | 25 | 1 | 6.002 | 9.3336 | 9.352 | `min_wall_dist=-0.00841888` |
| `casadi` | `chain_mass` | 25 | 1 | 373.413 | 400.758 | 412.369 | `avg_iterations=2.32`, `min_wall_dist=-0.000475765` |
| `minisolver` | `data_driven_mpc_lemniscate_tracking` | 500 | 1 | 0.0837195 | 0.0878028 | 0.164849 | `avg_iterations=2.002`, `max_constraint_violation=0`, `avg_tracking_error=5.64856`, `final_tracking_error=9.64606` |
| `minisolver` | `data_driven_mpc_loop_tracking` | 500 | 1 | 0.0838015 | 0.0915315 | 0.136348 | `avg_iterations=2.002`, `max_constraint_violation=0`, `avg_tracking_error=6.18116`, `final_tracking_error=10.4907` |
| `minisolver` | `mpcc_track_following` | 400 | 0.02 | 5.52489 | 6.22106 | 6.49809 | `avg_iterations=59.3725`, `max_constraint_violation=40.7789`, `avg_speed=9.35694`, `avg_tracking_error=2.10695`, `final_tracking_error=2.40594` |
| `minisolver` | `nonlinear_mpcc_fssim_following` | 1200 | 0.988 | 6.56949 | 7.73193 | 9.12065 | `avg_iterations=55.265`, `max_constraint_violation=0.691586`, `avg_speed=9.37842`, `avg_tracking_error=0.61476`, `final_tracking_error=0.877518` |
| `minisolver` | `nonlinear_mpcc_porto_following` | 600 | 0.997 | 5.64281 | 6.42677 | 7.84063 | `avg_iterations=57.545`, `max_constraint_violation=0.133782`, `avg_speed=11.2065`, `avg_tracking_error=0.565746`, `final_tracking_error=0.571822` |
| `acados` | `data_driven_mpc_lemniscate_tracking` | 500 | 1 | 0.091 | 0.182 | 0.245 | `avg_iterations=0.994`, `max_constraint_violation=0`, `avg_tracking_error=4.17021`, `final_tracking_error=7.32787` |
| `acados` | `data_driven_mpc_loop_tracking` | 500 | 1 | 0.091 | 0.113 | 0.179 | `avg_iterations=0.996`, `max_constraint_violation=0`, `avg_tracking_error=4.61694`, `final_tracking_error=7.32531` |
| `acados` | `mpcc_track_following` | 400 | 0 | 0.61 | 0.66805 | 0.989 | `avg_iterations=1`, `max_constraint_violation=47.6196`, `avg_speed=2.5`, `avg_tracking_error=23.0953`, `final_tracking_error=47.9929` |
| `acados` | `nonlinear_mpcc_fssim_following` | 1200 | 0.848 | 0.068 | 7.41605 | 8.969 | `avg_iterations=3.43333`, `max_constraint_violation=3.57817`, `avg_speed=0.778856`, `avg_tracking_error=2.61523`, `final_tracking_error=2.24323` |
| `acados` | `nonlinear_mpcc_porto_following` | 600 | 0.967 | 0.447 | 1.97215 | 3.121 | `avg_iterations=5.06167`, `max_constraint_violation=14.669`, `avg_speed=7.43766`, `avg_tracking_error=12.0808`, `final_tracking_error=14.3333` |
| `casadi` | `data_driven_mpc_lemniscate_tracking` | 500 | 1 | 6.32731 | 7.63235 | 13.3216 | `avg_iterations=1.002`, `max_constraint_violation=1.77636e-15`, `avg_tracking_error=0.10448`, `final_tracking_error=0.00197783` |
| `casadi` | `data_driven_mpc_loop_tracking` | 500 | 1 | 6.21193 | 6.37218 | 8.08882 | `avg_iterations=1`, `max_constraint_violation=0`, `avg_tracking_error=0.0273858`, `final_tracking_error=0.00232964` |
| `casadi` | `mpcc_track_following` | 400 | 0.953 | 148.59 | 864.913 | 3794.16 | `avg_iterations=2.04`, `max_constraint_violation=0.322332`, `avg_speed=10.7955`, `avg_tracking_error=0.281649`, `final_tracking_error=0.125879` |
| `casadi` | `nonlinear_mpcc_fssim_following` | 1200 | 0.968 | 222.877 | 230.646 | 7805.82 | `avg_iterations=1.72`, `max_constraint_violation=0`, `avg_speed=11.9311`, `avg_tracking_error=0.413769`, `final_tracking_error=0.416136` |
| `casadi` | `nonlinear_mpcc_porto_following` | 600 | 0.983 | 185.873 | 193.42 | 4761.42 | `avg_iterations=1.44167`, `max_constraint_violation=0`, `avg_speed=0.268256`, `avg_tracking_error=0.422826`, `final_tracking_error=0.271951` |

## Fairness Notes

- `acados` is benchmarked through official Python export plus compiled C runtime. Python codegen is not included in runtime timings.
- `MiniSolver` is benchmarked as a native in-process C++ target, not through an adapter layer.
- `CasADi` is benchmarked through a pre-built `nlpsol('sqpmethod')` graph with `qrqp` as the QP backend. Model construction is excluded from the per-step timings; the timed region is the repeated SQP solve call in the closed loop.
- `pendulum_on_cart`, `race_cars`, and `quadrotor_nav` use RTI-style single-iteration closed-loop runs on both sides.
- `chain_mass` uses full SQP-style solves on both sides, matching the official acados setup.
- `race_cars` and `quadrotor_nav` are official closed-loop cases with solver-dependent early termination. `steps` therefore measure executed closed-loop steps, not a fixed common horizon count.

## Pairwise Comparison

- `pendulum_on_cart`: `acados/MiniSolver` latency ratio is `median 15.90x`, `p95 23.38x`.
- `pendulum_on_cart`: success is `MiniSolver 75.00%` vs `acados 100.00%`.
- `pendulum_on_cart`: `CasADi/MiniSolver` latency ratio is `median 50.74x`, `p95 7672.68x`.
- `pendulum_on_cart`: success is `MiniSolver 75.00%` vs `CasADi 98.00%`.
- `race_cars`: `acados/MiniSolver` latency ratio is `median 0.80x`, `p95 0.97x`.
- `race_cars`: success is `MiniSolver 90.60%` vs `acados 100.00%`.
- `race_cars`: `CasADi/MiniSolver` latency ratio is `median 2.74x`, `p95 2.68x`.
- `race_cars`: success is `MiniSolver 90.60%` vs `CasADi 0.00%`.
- `quadrotor_nav`: `acados/MiniSolver` latency ratio is `median 0.52x`, `p95 0.51x`.
- `quadrotor_nav`: success is `MiniSolver 100.00%` vs `acados 100.00%`.
- `chain_mass`: `acados/MiniSolver` latency ratio is `median 2.80x`, `p95 2.43x`.
- `chain_mass`: success is `MiniSolver 100.00%` vs `acados 100.00%`.
- `chain_mass`: `CasADi/MiniSolver` latency ratio is `median 174.15x`, `p95 104.38x`.
- `chain_mass`: success is `MiniSolver 100.00%` vs `CasADi 100.00%`.

## Files

- Raw data: `results/raw/`
- Aggregate CSV: `results/summary.csv`
- Aggregate JSON: `results/summary.json`
