# Latest Benchmark Report

Date: 2026-04-11

## Provenance

- `nmpc-bench`: `4ae93db87f724565ad09925268484304cac4cb3a`
- `acados`: `third_party/acados` at `47b961bcaf493de59048918ffaa585181466ea65`
- `MiniSolver`: local checkout `/home/quyaonan/workspace/MiniSolver`
  - branch: `master`
  - commit: `6179e901532686e32935398bf66569eabb35a946`

## Summary

| Backend | Case | Steps | Success | Median ms | P95 ms | Max ms | Extra |
|---|---:|---:|---:|---:|---:|---:|---|
| `minisolver` | `pendulum_on_cart` | 100 | 0.77 | 0.035509 | 0.0391555 | 0.071648 | `avg_iterations=1`, `max_constraint_violation=0`, `final_theta_abs=0.00280468` |
| `acados` | `pendulum_on_cart` | 100 | 1 | 0.563 | 0.90265 | 2.373 | `avg_iterations=1`, `final_theta_abs=0.0247401` |
| `casadi` | `pendulum_on_cart` | 100 | 0.98 | 1.72047 | 282.507 | 1675.19 | `avg_iterations=2.89796`, `final_theta_abs=0.0016704` |
| `minisolver` | `race_cars` | 500 | 0.906 | 0.928484 | 0.953599 | 1.44094 | `avg_iterations=1`, `max_constraint_violation=0.133403`, `avg_speed=1.54277` |
| `acados` | `race_cars` | 344 | 1 | 0.759 | 0.96625 | 1.547 | `avg_speed=1.35207` |
| `casadi` | `race_cars` | 500 | 0 | 2.67814 | 2.78088 | 2986.72 | `avg_speed=nan` |
| `minisolver` | `quadrotor_nav` | 1559 | 1 | 8.83362 | 8.99354 | 11.0323 | `avg_iterations=1`, `max_constraint_violation=0`, `avg_abs_n=0.0623786`, `avg_abs_b=0.000746707` |
| `acados` | `quadrotor_nav` | 1566 | 1 | 4.568 | 4.72825 | 5.995 | `avg_abs_n=5.23977e-05`, `avg_abs_b=1.48661e-05` |
| `casadi` | `quadrotor_nav` | 1566 | 1 | 272.597 | 280.83 | 504.72 | `avg_iterations=1.00702`, `avg_abs_n=4.41991e-05`, `avg_abs_b=1.51695e-05` |
| `minisolver` | `chain_mass` | 25 | 1 | 2.27594 | 4.05462 | 6.11807 | `avg_iterations=3.56`, `max_constraint_violation=0`, `min_wall_dist=0.00112253` |
| `acados` | `chain_mass` | 25 | 1 | 6.235 | 9.8582 | 9.923 | `min_wall_dist=-0.00841888` |
| `casadi` | `chain_mass` | 25 | 1 | 386.28 | 414.196 | 435.488 | `avg_iterations=2.32`, `min_wall_dist=-0.000475765` |

## Fairness Notes

- `acados` is benchmarked through official Python export plus compiled C runtime. Python codegen is not included in runtime timings.
- `MiniSolver` is benchmarked as a native in-process C++ target, not through an adapter layer.
- `CasADi` is benchmarked through a pre-built `nlpsol('sqpmethod')` graph with `qrqp` as the QP backend. Model construction is excluded from the per-step timings; the timed region is the repeated SQP solve call in the closed loop.
- `pendulum_on_cart`, `race_cars`, and `quadrotor_nav` use RTI-style single-iteration closed-loop runs on both sides.
- `chain_mass` uses full SQP-style solves on both sides, matching the official acados setup.
- `race_cars` and `quadrotor_nav` are official closed-loop cases with solver-dependent early termination. `steps` therefore measure executed closed-loop steps, not a fixed common horizon count.

## Pairwise Comparison

- `pendulum_on_cart`: `acados/MiniSolver` latency ratio is `median 15.86x`, `p95 23.05x`.
- `pendulum_on_cart`: success is `MiniSolver 77.00%` vs `acados 100.00%`.
- `pendulum_on_cart`: `CasADi/MiniSolver` latency ratio is `median 48.45x`, `p95 7215.01x`.
- `pendulum_on_cart`: success is `MiniSolver 77.00%` vs `CasADi 98.00%`.
- `race_cars`: `acados/MiniSolver` latency ratio is `median 0.82x`, `p95 1.01x`.
- `race_cars`: success is `MiniSolver 90.60%` vs `acados 100.00%`.
- `race_cars`: `CasADi/MiniSolver` latency ratio is `median 2.88x`, `p95 2.92x`.
- `race_cars`: success is `MiniSolver 90.60%` vs `CasADi 0.00%`.
- `quadrotor_nav`: `acados/MiniSolver` latency ratio is `median 0.52x`, `p95 0.53x`.
- `quadrotor_nav`: success is `MiniSolver 100.00%` vs `acados 100.00%`.
- `quadrotor_nav`: `CasADi/MiniSolver` latency ratio is `median 30.86x`, `p95 31.23x`.
- `quadrotor_nav`: success is `MiniSolver 100.00%` vs `CasADi 100.00%`.
- `chain_mass`: `acados/MiniSolver` latency ratio is `median 2.74x`, `p95 2.43x`.
- `chain_mass`: success is `MiniSolver 100.00%` vs `acados 100.00%`.
- `chain_mass`: `CasADi/MiniSolver` latency ratio is `median 169.72x`, `p95 102.15x`.
- `chain_mass`: success is `MiniSolver 100.00%` vs `CasADi 100.00%`.

## Files

- Raw data: `results/raw/`
- Aggregate CSV: `results/summary.csv`
- Aggregate JSON: `results/summary.json`
