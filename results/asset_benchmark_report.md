# Asset Benchmark Report

Cross-solver benchmark results for public-asset-derived case candidates.

## Stable Cases

These are the 4 cases that currently behave like usable benchmark baselines.

| Case | Backend | Steps | Success | Median ms | P95 ms | Max ms | Avg Tracking Error | Max Constraint Violation |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| `data_driven_mpc_lemniscate_tracking` | `minisolver` | 500 | 1 | 0.0837195 | 0.0878028 | 0.164849 | 5.64856 | 0 |
| `data_driven_mpc_lemniscate_tracking` | `acados` | 500 | 1 | 0.091 | 0.182 | 0.245 | 4.17021 | 0 |
| `data_driven_mpc_lemniscate_tracking` | `casadi` | 500 | 1 | 6.32731 | 7.63235 | 13.3216 | 0.10448 | 1.77636e-15 |
| `data_driven_mpc_lemniscate_tracking` | `clarabel` | 500 | 1 | 0.333989 | 0.559852 | 0.655285 | 0.168999 | 0 |
| `data_driven_mpc_loop_tracking` | `minisolver` | 500 | 1 | 0.0838015 | 0.0915315 | 0.136348 | 6.18116 | 0 |
| `data_driven_mpc_loop_tracking` | `acados` | 500 | 1 | 0.091 | 0.113 | 0.179 | 4.61694 | 0 |
| `data_driven_mpc_loop_tracking` | `casadi` | 500 | 1 | 6.21193 | 6.37218 | 8.08882 | 0.0273858 | 0 |
| `data_driven_mpc_loop_tracking` | `clarabel` | 500 | 1 | 0.318799 | 0.560733 | 0.625605 | 0.0291629 | 0 |
| `nonlinear_mpcc_fssim_following` | `minisolver` | 1200 | 0.988 | 6.56949 | 7.73193 | 9.12065 | 0.61476 | 0.691586 |
| `nonlinear_mpcc_fssim_following` | `acados` | 1200 | 0.848 | 0.068 | 7.41605 | 8.969 | 2.61523 | 3.57817 |
| `nonlinear_mpcc_fssim_following` | `casadi` | 1200 | 0.968 | 222.877 | 230.646 | 7805.82 | 0.413769 | 0 |
| `nonlinear_mpcc_fssim_following` | `clarabel` | - | - | - | - | - | - | - |
| `nonlinear_mpcc_porto_following` | `minisolver` | 600 | 0.997 | 5.64281 | 6.42677 | 7.84063 | 0.565746 | 0.133782 |
| `nonlinear_mpcc_porto_following` | `acados` | 600 | 0.967 | 0.447 | 1.97215 | 3.121 | 12.0808 | 14.669 |
| `nonlinear_mpcc_porto_following` | `casadi` | 600 | 0.983 | 185.873 | 193.42 | 4761.42 | 0.422826 | 0 |
| `nonlinear_mpcc_porto_following` | `clarabel` | - | - | - | - | - | - | - |

## Stress Case

`mpcc_track_following` is kept separate because it is still a stress case, not a stable baseline.

| Case | Backend | Steps | Success | Median ms | P95 ms | Max ms | Avg Tracking Error | Max Constraint Violation |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| `mpcc_track_following` | `minisolver` | 400 | 0.02 | 5.52489 | 6.22106 | 6.49809 | 2.10695 | 40.7789 |
| `mpcc_track_following` | `acados` | 400 | 0 | 0.61 | 0.66805 | 0.989 | 23.0953 | 47.6196 |
| `mpcc_track_following` | `casadi` | 400 | 0.953 | 148.59 | 864.913 | 3794.16 | 0.281649 | 0.322332 |
| `mpcc_track_following` | `clarabel` | - | - | - | - | - | - | - |

## Notes

- `MiniSolver` asset benchmarks use MiniSolver Python model generation plus native C++ solve runners.
- `acados` asset benchmarks use Python export/codegen and compiled C closed-loop runners. Python export time is excluded.
- `CasADi` asset benchmarks use native C++ runners with `nlpsol('sqpmethod')`; one-time graph construction is excluded from the per-step timing.
- `Clarabel` is included only for convex robotics QP cases. It is not run on the nonconvex driving NMPC cases.
- The two robotics cases are the cleanest current baselines. MiniSolver and acados are in the same sub-millisecond class there; CasADi is about two orders slower.
- `nonlinear_mpcc_porto_following` and `nonlinear_mpcc_fssim_following` are currently the best driving baselines. acados is much faster there; MiniSolver is competitive on success rate; CasADi is much slower with heavy tail latency.
- `mpcc_track_following` should not be used for headline conclusions yet. It is still exposing model/setup mismatch rather than a stable solver ranking.

## Files

- Raw CSVs: `results/raw/minisolver/`, `results/raw/acados/`, `results/raw/casadi/`, `results/raw/clarabel/`
- Aggregate CSV: `results/summary.csv`
- Aggregate JSON: `results/summary.json`
