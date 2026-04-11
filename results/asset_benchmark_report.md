# Asset Benchmark Report

Cross-solver benchmark results for public-asset-derived case candidates.

## Current Baseline Cases

These are the cases that currently behave like usable benchmark baselines under the present harness.

| Case | Backend | Steps | Success | Median ms | P95 ms | Max ms | Avg Tracking Error | Max Constraint Violation |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| `data_driven_mpc_lemniscate_tracking` | `minisolver` | 500 | 1 | 0.04279 | 0.0829758 | 0.170776 | 0.10448 | 0 |
| `data_driven_mpc_lemniscate_tracking` | `acados` | 500 | 1 | 0.0947706 | 0.188829 | 1.5078 | 4.17021 | 0 |
| `data_driven_mpc_lemniscate_tracking` | `casadi` | 500 | 1 | 6.32731 | 7.63235 | 13.3216 | 0.10448 | 1.77636e-15 |
| `data_driven_mpc_lemniscate_tracking` | `clarabel` | 500 | 1 | 0.333989 | 0.559852 | 0.655285 | 0.168999 | 0 |
| `data_driven_mpc_loop_tracking` | `minisolver` | 500 | 1 | 0.030502 | 0.042611 | 0.082827 | 0.0273859 | 0 |
| `data_driven_mpc_loop_tracking` | `acados` | 500 | 1 | 0.092665 | 0.118436 | 0.169756 | 4.61694 | 0 |
| `data_driven_mpc_loop_tracking` | `casadi` | 500 | 1 | 6.21193 | 6.37218 | 8.08882 | 0.0273858 | 0 |
| `data_driven_mpc_loop_tracking` | `clarabel` | 500 | 1 | 0.318799 | 0.560733 | 0.625605 | 0.0291629 | 0 |
| `nonlinear_mpcc_fssim_following` | `minisolver` | 1200 | 1 | 0.124634 | 0.160365 | 0.846379 | 0.0685242 | 0 |
| `nonlinear_mpcc_fssim_following` | `acados` | 1200 | 1 | 0.070071 | 0.43564 | 0.546143 | 27.9709 | 34.3269 |
| `nonlinear_mpcc_fssim_following` | `casadi` | 1200 | 0.998 | 0.58985 | 53.7348 | 5065.9 | 0.511541 | 0 |
| `nonlinear_mpcc_fssim_following` | `clarabel` | - | - | - | - | - | - | - |
| `nonlinear_mpcc_porto_following` | `minisolver` | 600 | 1 | 0.077778 | 0.108051 | 0.98224 | 0.026356 | 0 |
| `nonlinear_mpcc_porto_following` | `acados` | 600 | 1 | 0.365769 | 0.405735 | 0.560773 | 2.6298 | 3.79406 |
| `nonlinear_mpcc_porto_following` | `casadi` | 600 | 0.985 | 194.203 | 924.837 | 4095.2 | 4.97621e+07 | 1.02624e+08 |
| `nonlinear_mpcc_porto_following` | `clarabel` | - | - | - | - | - | - | - |

## Stress Case

`mpcc_track_following` is kept separate because it is still a stress case, not a stable baseline.

| Case | Backend | Steps | Success | Median ms | P95 ms | Max ms | Avg Tracking Error | Max Constraint Violation |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| `mpcc_track_following` | `minisolver` | 400 | 0.0225 | 0.516871 | 3.92812 | 6.22722 | 1.29962 | 6.08021 |
| `mpcc_track_following` | `acados` | 400 | 0 | 0.584725 | 0.641001 | 0.98426 | 23.0953 | 47.6196 |
| `mpcc_track_following` | `casadi` | 400 | 0.953 | 148.59 | 864.913 | 3794.16 | 0.281649 | 0.322332 |
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
