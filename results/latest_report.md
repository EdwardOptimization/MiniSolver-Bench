# Latest Official Compatibility Report

Date: 2026-05-27

This report covers official acados-derived compatibility/reference cases. It is not the default headline ranking report.

For same-asset, same-model-family, same-`dt`, same-horizon, same-requested-step comparisons, use `results/asset_benchmark_report.md`.

## Provenance

- `nmpc-bench`: `b4d558320cbce9851d46728cacec386bdc15d1b9`
- `acados`: `third_party/acados` at `47b961bcaf493de59048918ffaa585181466ea65`
- `MiniSolver`: `third_party/MiniSolver` at `dfb1861de4ca580b35288cab5bdc25857ec2f26e`
  - branch: `master`

## Summary

| Backend | Case | Steps | Success | Median ms | P95 ms | Max ms | Extra |
|---|---:|---:|---:|---:|---:|---:|---|
| `minisolver` | `pendulum_on_cart` | 100 | 1 | 0.0366205 | 0.0403815 | 0.077421 | `avg_iterations=1`, `max_constraint_violation=0`, `final_theta_abs=0.00160175` |
| `acados` | `pendulum_on_cart` | 100 | 1 | 0.5365 | 0.8654 | 1.471 | `avg_iterations=1`, `final_theta_abs=0.0247401` |
| `casadi` | `pendulum_on_cart` | 100 | 0.98 | 1.67175 | 276.881 | 1622.91 | `avg_iterations=2.89796`, `final_theta_abs=0.0016704` |
| `minisolver` | `race_cars` | 500 | 0.906 | 0.401139 | 0.428082 | 0.511706 | `avg_iterations=1`, `max_constraint_violation=0.0700106`, `avg_speed=1.54816` |
| `acados` | `race_cars` | 344 | 1 | 0.742 | 0.9635 | 1.761 | `avg_speed=1.35207` |
| `casadi` | `race_cars` | 500 | 0 | 2.51439 | 2.55963 | 2891.64 | `avg_speed=nan` |
| `minisolver` | `quadrotor_nav` | 1560 | 1 | 1.30837 | 1.35031 | 1.63718 | `avg_iterations=1`, `max_constraint_violation=0`, `avg_abs_n=0.0399726`, `avg_abs_b=0.000869766` |
| `acados` | `quadrotor_nav` | 1566 | 1 | 2.254 | 2.32075 | 3.715 | `avg_abs_n=5.49982e-05`, `avg_abs_b=1.654e-05` |
| `casadi` | `quadrotor_nav` | 1566 | 1 | 265.961 | 283.131 | 477.64 | `avg_iterations=1.00702`, `avg_abs_n=4.41991e-05`, `avg_abs_b=1.51695e-05` |
| `minisolver` | `chain_mass` | 25 | 1 | 2.27817 | 3.92546 | 5.93721 | `avg_iterations=3.52`, `max_constraint_violation=0`, `min_wall_dist=0.00112253` |
| `acados` | `chain_mass` | 25 | 1 | 6.137 | 9.636 | 9.764 | `min_wall_dist=-0.00841888` |
| `casadi` | `chain_mass` | 25 | 1 | 368.493 | 391.51 | 410.568 | `avg_iterations=2.32`, `min_wall_dist=-0.000475765` |

## Fairness Notes

- This official-case report is compatibility/reference evidence. Use `results/asset_benchmark_report.md` for the current same-condition, end-to-end solver ranking.
- `acados` is benchmarked through official Python export plus compiled C runtime. Python codegen is not included in runtime timings.
- `MiniSolver` is benchmarked as a native in-process C++ target, not through an adapter layer.
- `CasADi` is benchmarked through a pre-built `nlpsol('sqpmethod')` graph with `qrqp` as the QP backend. Model construction is excluded from the per-step timings; the timed region is the repeated SQP solve call in the closed loop.
- `pendulum_on_cart`, `race_cars`, and `quadrotor_nav` use RTI-style single-iteration closed-loop runs on both sides.
- `chain_mass` uses full SQP-style solves on both sides, matching the official acados setup.
- `race_cars` and `quadrotor_nav` contain piecewise track functions. MiniSolver runs them through generated C++ models with dcol-style model-update callbacks that refresh smooth local spline jets from the official track geometry.
- `race_cars` and `quadrotor_nav` are official closed-loop cases with solver-dependent early termination. `steps` therefore measure executed closed-loop steps, not a fixed common horizon count.

## Pairwise Comparison

- `pendulum_on_cart`: `acados/MiniSolver` latency ratio is `median 14.65x`, `p95 21.43x`.
- `pendulum_on_cart`: success is `MiniSolver 100.00%` vs `acados 100.00%`.
- `pendulum_on_cart`: `CasADi/MiniSolver` latency ratio is `median 45.65x`, `p95 6856.64x`.
- `pendulum_on_cart`: success is `MiniSolver 100.00%` vs `CasADi 98.00%`.
- `race_cars`: `acados/MiniSolver` latency ratio is `median 1.85x`, `p95 2.25x`.
- `race_cars`: success is `MiniSolver 90.60%` vs `acados 100.00%`.
- `race_cars`: `CasADi/MiniSolver` latency ratio is `median 6.27x`, `p95 5.98x`.
- `race_cars`: success is `MiniSolver 90.60%` vs `CasADi 0.00%`.
- `quadrotor_nav`: `acados/MiniSolver` latency ratio is `median 1.72x`, `p95 1.72x`.
- `quadrotor_nav`: success is `MiniSolver 100.00%` vs `acados 100.00%`.
- `quadrotor_nav`: `CasADi/MiniSolver` latency ratio is `median 203.28x`, `p95 209.68x`.
- `quadrotor_nav`: success is `MiniSolver 100.00%` vs `CasADi 100.00%`.
- `chain_mass`: `acados/MiniSolver` latency ratio is `median 2.69x`, `p95 2.45x`.
- `chain_mass`: success is `MiniSolver 100.00%` vs `acados 100.00%`.
- `chain_mass`: `CasADi/MiniSolver` latency ratio is `median 161.75x`, `p95 99.74x`.
- `chain_mass`: success is `MiniSolver 100.00%` vs `CasADi 100.00%`.

## Files

- Raw data: `results/raw/`
- Aggregate CSV: `results/summary.csv`
- Aggregate JSON: `results/summary.json`
