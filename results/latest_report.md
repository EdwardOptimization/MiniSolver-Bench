# Latest Official Compatibility Report

Date: 2026-05-30

This report covers official acados-derived compatibility/reference cases. It is not the default headline ranking report.

For same-asset, same-model-family, same-`dt`, same-horizon, same-requested-step comparisons, use `results/asset_benchmark_report.md`.

## Provenance

- `nmpc-bench`: `70fa6cac9864422fa3693e02b719b7e0edb4d4c4`
- `acados`: `third_party/acados` at `47b961bcaf493de59048918ffaa585181466ea65`
- `MiniSolver`: `third_party/MiniSolver` at `5816a5b44318b6f97ef7076c3384216bb97454e5`
  - branch: `detached`

## Summary

| Backend | Case | Steps | Success | Median ms | P95 ms | Max ms | Extra |
|---|---:|---:|---:|---:|---:|---:|---|
| `minisolver` | `pendulum_on_cart` | 100 | 1 | 0.0366505 | 0.0401524 | 0.080501 | `avg_iterations=1`, `max_constraint_violation=0`, `final_theta_abs=0.00102476` |
| `acados` | `pendulum_on_cart` | 100 | 1 | 0.5495 | 0.8693 | 1.485 | `avg_iterations=1`, `final_theta_abs=0.0247401` |
| `casadi` | `pendulum_on_cart` | 100 | 0.98 | 1.70865 | 278.006 | 1619.77 | `avg_iterations=2.89796`, `final_theta_abs=0.0016704` |
| `minisolver` | `race_cars` | 266 | 1 | 1.74239 | 3.31109 | 5.28262 | `avg_iterations=7.71805`, `max_constraint_violation=0.0001059`, `avg_speed=1.39569` |
| `acados` | `race_cars` | 344 | 1 | 0.724 | 0.92685 | 2.129 | `avg_iterations=10.593`, `avg_speed=1.35207` |
| `casadi` | `race_cars` | 500 | 0 | 2.4517 | 2.51766 | 2851.95 | `avg_speed=nan` |
| `minisolver` | `quadrotor_nav` | 1566 | 1 | 1.53335 | 1.57866 | 3.60234 | `avg_iterations=2.00192`, `max_constraint_violation=0`, `avg_abs_n=0.000126632`, `avg_abs_b=1.54681e-05` |
| `acados` | `quadrotor_nav` | 1566 | 1 | 2.5 | 2.6305 | 3.597 | `avg_abs_n=5.49982e-05`, `avg_abs_b=1.654e-05` |
| `casadi` | `quadrotor_nav` | 1566 | 1 | 277.058 | 285.742 | 504.462 | `avg_iterations=1.00702`, `avg_abs_n=4.41991e-05`, `avg_abs_b=1.51695e-05` |
| `minisolver` | `chain_mass` | 25 | 1 | 2.02788 | 3.72461 | 5.642 | `avg_iterations=3.52`, `max_constraint_violation=0`, `min_wall_dist=0.00112113` |
| `acados` | `chain_mass` | 25 | 1 | 5.974 | 9.4886 | 9.653 | `min_wall_dist=-0.00841888` |
| `casadi` | `chain_mass` | 25 | 1 | 360.629 | 387.148 | 408.922 | `avg_iterations=2.32`, `min_wall_dist=-0.000475765` |

## Fairness Notes

- This official-case report is compatibility/reference evidence. Use `results/asset_benchmark_report.md` for the current same-condition, end-to-end solver ranking.
- `acados` is benchmarked through official Python export plus compiled C runtime. Python codegen is not included in runtime timings.
- `MiniSolver` is benchmarked as a native in-process C++ target, not through an adapter layer.
- `CasADi` is benchmarked through a pre-built `nlpsol('sqpmethod')` graph with `qrqp` as the QP backend. Model construction is excluded from the per-step timings; the timed region is the repeated SQP solve call in the closed loop.
- `pendulum_on_cart` keeps the official RTI-style single-iteration setup. `quadrotor_nav` remains an RTI-style official closed-loop baseline. `race_cars` uses a quality-first MiniSolver `ACCEPTABLE_NMPC` multi-iteration profile with `tol_con=tol_dual=1e-4` and rollout globalization; acados keeps the official `SQP_RTI` setup.
- `chain_mass` uses full SQP-style solves on both sides, matching the official acados setup.
- `race_cars` and `quadrotor_nav` contain piecewise track functions. MiniSolver runs them through generated C++ models with dcol-style model-update callbacks. `race_cars` refreshes a local ppoly segment window so RK substeps evaluate the track at their current state; `quadrotor_nav` refreshes smooth local spline jets.
- `race_cars` and `quadrotor_nav` are official closed-loop cases with solver-dependent early termination. `steps` therefore measure executed closed-loop steps, not a fixed common horizon count.

## Pairwise Comparison

- `pendulum_on_cart`: `acados/MiniSolver` latency ratio is `median 14.99x`, `p95 21.65x`.
- `pendulum_on_cart`: success is `MiniSolver 100.00%` vs `acados 100.00%`.
- `pendulum_on_cart`: `CasADi/MiniSolver` latency ratio is `median 46.62x`, `p95 6923.77x`.
- `pendulum_on_cart`: success is `MiniSolver 100.00%` vs `CasADi 98.00%`.
- `race_cars`: `acados/MiniSolver` latency ratio is `median 0.42x`, `p95 0.28x`.
- `race_cars`: success is `MiniSolver 100.00%` vs `acados 100.00%`.
- `race_cars`: `CasADi/MiniSolver` latency ratio is `median 1.41x`, `p95 0.76x`.
- `race_cars`: success is `MiniSolver 100.00%` vs `CasADi 0.00%`.
- `quadrotor_nav`: `acados/MiniSolver` latency ratio is `median 1.63x`, `p95 1.67x`.
- `quadrotor_nav`: success is `MiniSolver 100.00%` vs `acados 100.00%`.
- `quadrotor_nav`: `CasADi/MiniSolver` latency ratio is `median 180.69x`, `p95 181.00x`.
- `quadrotor_nav`: success is `MiniSolver 100.00%` vs `CasADi 100.00%`.
- `chain_mass`: `acados/MiniSolver` latency ratio is `median 2.95x`, `p95 2.55x`.
- `chain_mass`: success is `MiniSolver 100.00%` vs `acados 100.00%`.
- `chain_mass`: `CasADi/MiniSolver` latency ratio is `median 177.84x`, `p95 103.94x`.
- `chain_mass`: success is `MiniSolver 100.00%` vs `CasADi 100.00%`.

## Files

- Raw data: `results/raw/`
- Aggregate CSV: `results/summary.csv`
- Aggregate JSON: `results/summary.json`
