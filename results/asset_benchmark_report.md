# Asset Benchmark Report

Cross-solver benchmark results for public-asset-derived case candidates.

## Current Baseline Cases

These are the cases that currently behave like usable benchmark baselines under the present harness.

| Case | Backend | Steps | Success | Median ms | P95 ms | Max ms | Avg Tracking Error | Max Constraint Violation |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| `data_driven_mpc_lemniscate_tracking` | `minisolver` | 500 | 1 | 0.041193 | 0.0800572 | 0.125611 | 0.10448 | 0 |
| `data_driven_mpc_lemniscate_tracking` | `acados` | 500 | 1 | 0.089954 | 0.139534 | 1.491 | 0.0951679 | 0 |
| `data_driven_mpc_lemniscate_tracking` | `casadi` | 500 | 1 | 6.38007 | 9.8141 | 16.2217 | 0.10448 | 1.77636e-15 |
| `data_driven_mpc_lemniscate_tracking` | `clarabel` | 500 | 1 | 0.333989 | 0.559852 | 0.655285 | 0.168999 | 0 |
| `data_driven_mpc_loop_tracking` | `minisolver` | 500 | 1 | 0.031126 | 0.0434231 | 0.073778 | 0.0273859 | 0 |
| `data_driven_mpc_loop_tracking` | `acados` | 500 | 1 | 0.090739 | 0.115713 | 0.290099 | 0.0214657 | 0 |
| `data_driven_mpc_loop_tracking` | `casadi` | 500 | 1 | 6.33781 | 7.29299 | 8.09844 | 0.0273858 | 0 |
| `data_driven_mpc_loop_tracking` | `clarabel` | 500 | 1 | 0.318799 | 0.560733 | 0.625605 | 0.0291629 | 0 |
| `nonlinear_mpcc_fssim_following` | `minisolver` | 1200 | 1 | 0.090179 | 0.143948 | 0.853933 | 0.00106131 | 0 |
| `nonlinear_mpcc_fssim_following` | `acados` | 1200 | 0.172 | 0.633175 | 0.916617 | 2.6332 | 25.765 | 49.713 |
| `nonlinear_mpcc_fssim_following` | `casadi` | 1200 | 0.921 | 53.3818 | 4711.21 | 8008.45 | 797602 | 6.91704e+06 |
| `nonlinear_mpcc_fssim_following` | `clarabel` | - | - | - | - | - | - | - |
| `nonlinear_mpcc_porto_following` | `minisolver` | 600 | 1 | 0.076063 | 0.0839592 | 1.02719 | 0.00208937 | 0 |
| `nonlinear_mpcc_porto_following` | `acados` | 600 | 0.03 | 0.683246 | 5.18267 | 7.34456 | 11.4278 | 20.7884 |
| `nonlinear_mpcc_porto_following` | `casadi` | 600 | 0.97 | 0.485091 | 200.965 | 6913.27 | 2.50103e+07 | 3.13215e+07 |
| `nonlinear_mpcc_porto_following` | `clarabel` | - | - | - | - | - | - | - |

## Stress Case

`mpcc_track_following` is kept separate because it is still a stress case, not a stable baseline.

| Case | Backend | Steps | Success | Median ms | P95 ms | Max ms | Avg Tracking Error | Max Constraint Violation |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| `mpcc_track_following` | `minisolver` | 400 | 0.0225 | 0.516871 | 3.92812 | 6.22722 | 1.29962 | 6.08021 |
| `mpcc_track_following` | `acados` | 400 | 0 | 0.584725 | 0.641001 | 0.98426 | 23.0953 | 47.6196 |
| `mpcc_track_following` | `casadi` | - | - | - | - | - | - | - |
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
