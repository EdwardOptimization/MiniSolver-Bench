# Latest Benchmark Report

Date: 2026-04-10

## Provenance

- `nmpc-bench`: `d244384539a019401c3feca17893ddb2eddba1b5`
- `acados`: `third_party/acados` at `47b961bcaf493de59048918ffaa585181466ea65`
- `MiniSolver`: local checkout `/home/quyaonan/workspace/MiniSolver`
  - branch: `master`
  - commit: `21d71b7d2a83a216cc8d454b2cc92f24cac9eda2`

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
| `minisolver` | `data_driven_mpc_lemniscate_tracking` | 500 | 1 | 0.04279 | 0.0829758 | 0.170776 | `avg_iterations=3.392`, `max_constraint_violation=0`, `avg_tracking_error=0.10448`, `final_tracking_error=0.00197805` |
| `minisolver` | `data_driven_mpc_loop_tracking` | 500 | 1 | 0.030502 | 0.042611 | 0.082827 | `avg_iterations=2.898`, `max_constraint_violation=0`, `avg_tracking_error=0.0273859`, `final_tracking_error=0.00232988` |
| `minisolver` | `mpcc_track_following` | 400 | 0.0225 | 0.516871 | 3.92812 | 6.22722 | `avg_iterations=30.8325`, `max_constraint_violation=6.08021`, `avg_speed=6.73929`, `avg_tracking_error=1.29962`, `final_tracking_error=0.764616` |
| `minisolver` | `nonlinear_mpcc_fssim_following` | 1200 | 1 | 0.124634 | 0.160365 | 0.846379 | `avg_iterations=3.81833`, `max_constraint_violation=0`, `avg_speed=8.01675`, `avg_tracking_error=0.0685242`, `final_tracking_error=0.116547` |
| `minisolver` | `nonlinear_mpcc_porto_following` | 600 | 1 | 0.077778 | 0.108051 | 0.98224 | `avg_iterations=3.17833`, `max_constraint_violation=0`, `avg_speed=6.00057`, `avg_tracking_error=0.026356`, `final_tracking_error=0.011208` |
| `acados` | `data_driven_mpc_lemniscate_tracking` | 500 | 1 | 0.0947706 | 0.188829 | 1.5078 | `avg_iterations=0.994`, `max_constraint_violation=0`, `avg_tracking_error=4.17021`, `final_tracking_error=7.32787` |
| `acados` | `data_driven_mpc_loop_tracking` | 500 | 1 | 0.092665 | 0.118436 | 0.169756 | `avg_iterations=0.996`, `max_constraint_violation=0`, `avg_tracking_error=4.61694`, `final_tracking_error=7.32531` |
| `acados` | `mpcc_track_following` | 400 | 0 | 0.584725 | 0.641001 | 0.98426 | `avg_iterations=1`, `max_constraint_violation=47.6196`, `avg_speed=2.5`, `avg_tracking_error=23.0953`, `final_tracking_error=47.9929` |
| `acados` | `nonlinear_mpcc_fssim_following` | 1200 | 1 | 0.070071 | 0.43564 | 0.546143 | `avg_iterations=0.499167`, `max_constraint_violation=34.3269`, `avg_speed=7.99096`, `avg_tracking_error=27.9709`, `final_tracking_error=32.0262` |
| `acados` | `nonlinear_mpcc_porto_following` | 600 | 1 | 0.365769 | 0.405735 | 0.560773 | `avg_iterations=1.625`, `max_constraint_violation=3.79406`, `avg_speed=5.97223`, `avg_tracking_error=2.6298`, `final_tracking_error=1.9865` |
| `casadi` | `data_driven_mpc_lemniscate_tracking` | 500 | 1 | 6.32731 | 7.63235 | 13.3216 | `avg_iterations=1.002`, `max_constraint_violation=1.77636e-15`, `avg_tracking_error=0.10448`, `final_tracking_error=0.00197783` |
| `casadi` | `data_driven_mpc_loop_tracking` | 500 | 1 | 6.21193 | 6.37218 | 8.08882 | `avg_iterations=1`, `max_constraint_violation=0`, `avg_tracking_error=0.0273858`, `final_tracking_error=0.00232964` |
| `casadi` | `mpcc_track_following` | 400 | 0.953 | 148.59 | 864.913 | 3794.16 | `avg_iterations=2.04`, `max_constraint_violation=0.322332`, `avg_speed=10.7955`, `avg_tracking_error=0.281649`, `final_tracking_error=0.125879` |
| `casadi` | `nonlinear_mpcc_fssim_following` | 1200 | 0.998 | 0.58985 | 53.7348 | 5065.9 | `avg_iterations=1.3`, `max_constraint_violation=0`, `avg_speed=11.3022`, `avg_tracking_error=0.511541`, `final_tracking_error=0.524476` |
| `casadi` | `nonlinear_mpcc_porto_following` | 600 | 0.985 | 194.203 | 924.837 | 4095.2 | `avg_iterations=1.94667`, `max_constraint_violation=1.02624e+08`, `avg_speed=11.8928`, `avg_tracking_error=4.97621e+07`, `final_tracking_error=6.00939e+07` |

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
