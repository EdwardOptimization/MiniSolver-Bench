# Latest Benchmark Report

Date: 2026-04-10

## Provenance

- `nmpc-bench`: `a8f1ed0ded50749a521592869c6c1e9cb276afc1`
- `acados`: `third_party/acados` at `47b961bcaf493de59048918ffaa585181466ea65`
- `MiniSolver`: local checkout `/home/quyaonan/workspace/MiniSolver`
  - branch: `master`
  - commit: `21d71b7d2a83a216cc8d454b2cc92f24cac9eda2`

## Summary

| Backend | Case | Steps | Success | Median ms | P95 ms | Max ms | Extra |
|---|---:|---:|---:|---:|---:|---:|---|
| `minisolver` | `pendulum_on_cart` | 100 | 0.75 | 0.033861 | 0.037074 | 0.046271 | `avg_iterations=1`, `max_constraint_violation=0`, `final_theta_abs=0.0038049` |
| `acados` | `pendulum_on_cart` | 100 | 1 | 0.5385 | 0.86675 | 1.434 | `avg_iterations=1`, `final_theta_abs=0.0247401` |
| `casadi` | `pendulum_on_cart` | 100 | 0.98 | 1.7181 | 284.457 | 1671.62 | `avg_iterations=2.89796`, `final_theta_abs=0.0016704` |
| `minisolver` | `race_cars` | 500 | 0.906 | 0.902315 | 0.942156 | 1.25639 | `avg_iterations=1`, `max_constraint_violation=0.0430696`, `avg_speed=1.44975` |
| `acados` | `race_cars` | 344 | 1 | 0.721 | 0.9129 | 1.458 | `avg_speed=1.35207` |
| `casadi` | `race_cars` | 500 | 0 | 2.4762 | 2.52565 | 2937.7 | `avg_speed=nan` |
| `minisolver` | `quadrotor_nav` | 1559 | 1 | 8.39711 | 8.63614 | 9.46972 | `avg_iterations=1`, `max_constraint_violation=0`, `avg_abs_n=0.0627374`, `avg_abs_b=0.000744301` |
| `acados` | `quadrotor_nav` | 1566 | 1 | 4.332 | 4.442 | 6.548 | `avg_abs_n=5.23977e-05`, `avg_abs_b=1.48661e-05` |
| `minisolver` | `chain_mass` | 25 | 1 | 2.1442 | 3.83935 | 5.73771 | `avg_iterations=3.56`, `max_constraint_violation=0`, `min_wall_dist=0.00112253` |
| `acados` | `chain_mass` | 25 | 1 | 6.002 | 9.3336 | 9.352 | `min_wall_dist=-0.00841888` |
| `casadi` | `chain_mass` | 25 | 1 | 373.413 | 400.758 | 412.369 | `avg_iterations=2.32`, `min_wall_dist=-0.000475765` |
| `minisolver` | `data_driven_mpc_lemniscate_tracking` | 500 | 1 | 0.041193 | 0.0800572 | 0.125611 | `avg_iterations=3.392`, `max_constraint_violation=0`, `avg_tracking_error=0.10448`, `final_tracking_error=0.00197805` |
| `minisolver` | `data_driven_mpc_loop_tracking` | 500 | 1 | 0.031126 | 0.0434231 | 0.073778 | `avg_iterations=2.898`, `max_constraint_violation=0`, `avg_tracking_error=0.0273859`, `final_tracking_error=0.00232988` |
| `minisolver` | `mpcc_track_following` | 400 | 0.0225 | 0.516871 | 3.92812 | 6.22722 | `avg_iterations=38.8889`, `max_constraint_violation=6.08021`, `avg_speed=6.73929`, `avg_tracking_error=1.29962`, `final_tracking_error=0.764616` |
| `minisolver` | `nonlinear_mpcc_fssim_following` | 1200 | 1 | 0.090179 | 0.143948 | 0.853933 | `avg_iterations=2.72`, `max_constraint_violation=0`, `avg_speed=7.99931`, `avg_tracking_error=0.00106131`, `final_tracking_error=0.000630828` |
| `minisolver` | `nonlinear_mpcc_porto_following` | 600 | 1 | 0.076063 | 0.0839592 | 1.02719 | `avg_iterations=2.875`, `max_constraint_violation=0`, `avg_speed=5.99918`, `avg_tracking_error=0.00208937`, `final_tracking_error=6.99941e-05` |
| `acados` | `data_driven_mpc_lemniscate_tracking` | 500 | 1 | 0.089954 | 0.139534 | 1.491 | `avg_iterations=1`, `max_constraint_violation=0`, `avg_tracking_error=0.0951679`, `final_tracking_error=0.00164302` |
| `acados` | `data_driven_mpc_loop_tracking` | 500 | 1 | 0.090739 | 0.115713 | 0.290099 | `avg_iterations=1`, `max_constraint_violation=0`, `avg_tracking_error=0.0214657`, `final_tracking_error=0.00193837` |
| `acados` | `mpcc_track_following` | 400 | 0 | 0.584725 | 0.641001 | 0.98426 | `max_constraint_violation=47.6196`, `avg_speed=2.5`, `avg_tracking_error=23.0953`, `final_tracking_error=47.9929` |
| `acados` | `nonlinear_mpcc_fssim_following` | 1200 | 0.172 | 0.633175 | 0.916617 | 2.6332 | `avg_iterations=1.03865`, `max_constraint_violation=49.713`, `avg_speed=7.99151`, `avg_tracking_error=25.765`, `final_tracking_error=31.1793` |
| `acados` | `nonlinear_mpcc_porto_following` | 600 | 0.03 | 0.683246 | 5.18267 | 7.34456 | `avg_iterations=1.44444`, `max_constraint_violation=20.7884`, `avg_speed=5.99336`, `avg_tracking_error=11.4278`, `final_tracking_error=12.545` |
| `casadi` | `data_driven_mpc_lemniscate_tracking` | 500 | 1 | 6.38007 | 9.8141 | 16.2217 | `avg_iterations=1.002`, `max_constraint_violation=1.77636e-15`, `avg_tracking_error=0.10448`, `final_tracking_error=0.00197783` |
| `casadi` | `data_driven_mpc_loop_tracking` | 500 | 1 | 6.33781 | 7.29299 | 8.09844 | `avg_iterations=1`, `max_constraint_violation=0`, `avg_tracking_error=0.0273858`, `final_tracking_error=0.00232964` |
| `casadi` | `nonlinear_mpcc_fssim_following` | 1200 | 0.921 | 53.3818 | 4711.21 | 8008.45 | `avg_iterations=1.84072`, `max_constraint_violation=6.91704e+06`, `avg_speed=5.44575`, `avg_tracking_error=797602`, `final_tracking_error=6.64423e+06` |
| `casadi` | `nonlinear_mpcc_porto_following` | 600 | 0.97 | 0.485091 | 200.965 | 6913.27 | `avg_iterations=1.22165`, `max_constraint_violation=3.13215e+07`, `avg_speed=0.493289`, `avg_tracking_error=2.50103e+07`, `final_tracking_error=2.84008e+07` |

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
