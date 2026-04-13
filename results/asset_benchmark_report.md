# Asset Benchmark Report

Cross-solver benchmark results for public-asset-derived case candidates.

## Current Baseline Cases

These are the cases that currently behave like usable benchmark baselines under the present harness.

| Case | Backend | Steps | Success | Median ms | P95 ms | Max ms | Avg Tracking Error | Max Constraint Violation |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| `data_driven_mpc_lemniscate_tracking` | `minisolver` | 500 | 1 | 0.0405095 | 0.0800385 | 0.11498 | 0.10448 | 0 |
| `data_driven_mpc_lemniscate_tracking` | `acados` | 500 | 1 | 0.069346 | 0.12093 | 1.48778 | 0.0951679 | 0 |
| `data_driven_mpc_lemniscate_tracking` | `casadi` | 500 | 1 | 5.95695 | 7.11261 | 7.56597 | 0.10448 | 1.77636e-15 |
| `data_driven_mpc_lemniscate_tracking` | `clarabel` | 500 | 1 | 0.333311 | 0.521438 | 0.625294 | 0.10448 | 0 |
| `data_driven_mpc_loop_tracking` | `minisolver` | 500 | 1 | 0.040943 | 0.0433315 | 0.113808 | 0.0273859 | 0 |
| `data_driven_mpc_loop_tracking` | `acados` | 500 | 1 | 0.063682 | 0.080475 | 0.140465 | 0.0214657 | 0 |
| `data_driven_mpc_loop_tracking` | `casadi` | 500 | 1 | 5.94653 | 6.01878 | 7.59994 | 0.0273858 | 0 |
| `data_driven_mpc_loop_tracking` | `clarabel` | 500 | 1 | 0.305645 | 0.393768 | 0.422171 | 0.0273858 | 0 |
| `nonlinear_mpcc_fssim_following` | `minisolver` | 1200 | 1 | 0.110797 | 0.176224 | 1.82018 | 0.00106131 | 0 |
| `nonlinear_mpcc_fssim_following` | `acados` | 1200 | 0.999 | 1.94643 | 2.65358 | 2.85423 | 29.5741 | 68.0295 |
| `nonlinear_mpcc_fssim_following` | `casadi` | 1200 | 0.957 | 0.633244 | 270.268 | 4605.41 | 0.79742 | 0.036368 |
| `nonlinear_mpcc_fssim_following` | `clarabel` | - | - | - | - | - | - | - |
| `nonlinear_mpcc_porto_following` | `minisolver` | 600 | 1 | 0.115451 | 0.143681 | 2.04868 | 0.00208937 | 0 |
| `nonlinear_mpcc_porto_following` | `acados` | 600 | 0.998 | 0.626061 | 1.47274 | 1.82751 | 2.56204 | 9.40125 |
| `nonlinear_mpcc_porto_following` | `casadi` | 600 | 0.8 | 0.510582 | 3112.7 | 4977.53 | 1.87096 | 13.7211 |
| `nonlinear_mpcc_porto_following` | `clarabel` | - | - | - | - | - | - | - |

## Stress Case

`mpcc_track_following` is kept separate because it is still a stress case, not a stable baseline.

| Case | Backend | Steps | Success | Median ms | P95 ms | Max ms | Avg Tracking Error | Max Constraint Violation |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| `mpcc_track_following` | `minisolver` | 400 | 1 | 0.106131 | 0.142878 | 0.19018 | 0.0100655 | 0 |
| `mpcc_track_following` | `acados` | 400 | 0.998 | 0.579828 | 0.822239 | 1.16371 | 1.15544 | 3.99598 |
| `mpcc_track_following` | `casadi` | 400 | 0.637 | 1.32756 | 2476.24 | 4258.24 | 1.00218 | 4.2645 |
| `mpcc_track_following` | `clarabel` | - | - | - | - | - | - | - |

## Notes

- `MiniSolver` asset benchmarks use MiniSolver Python model generation plus native C++ solve runners.
- `acados` asset benchmarks use Python export/codegen and compiled C closed-loop runners. Python export time is excluded.
- `CasADi` asset benchmarks use native C++ runners with `nlpsol('sqpmethod')`; one-time graph construction is excluded from the per-step timing.
- `time_ms` is external wall-clock time around each compiled solve call for all asset backends in this report.
- `max_constraint_violation` is the current closed-loop step violation, not the maximum predicted violation over the full horizon.
- `Clarabel` is included only for convex robotics QP cases. It is not run on the nonconvex driving NMPC cases.
- The two robotics cases are the cleanest current baselines. MiniSolver and acados are in the same sub-millisecond class there; CasADi is materially slower.
- `nonlinear_mpcc_porto_following` and `nonlinear_mpcc_fssim_following` are the current driving baselines. Interpret success, tracking, and latency together; none of the three solvers dominates every metric.
- `mpcc_track_following` should not be used for headline conclusions yet. It is still exposing model/setup mismatch rather than a stable solver ranking.

## Files

- Raw CSVs: `results/raw/minisolver/`, `results/raw/acados/`, `results/raw/casadi/`, `results/raw/clarabel/`
- Aggregate CSV: `results/summary.csv`
- Aggregate JSON: `results/summary.json`
