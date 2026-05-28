# End-to-End Asset Solver Comparison

Same-case, closed-loop, end-to-end solver comparison for public-asset-derived candidates.

This is the repository's current end-to-end closed-loop asset report, not a strict solver-core full-solve ranking. It covers public-asset-derived cases; official `race_cars` and `quadrotor_nav` live in `results/latest_report.md` and use MiniSolver callback-refreshed track data.

Each solver row uses the same asset, model family, `dt`, horizon, and requested closed-loop step count for that candidate. The timed region is the per-step solver call inside the closed-loop runner; one-time code generation, CMake configure/build, Python import time, and report aggregation are excluded.

The asset candidates do not rely on Python MiniModel piecewise modeling. Robotics candidates use native reference-tracking data, and driving candidates use runner-side track sampling plus stage parameters. For callback-backed track models, the runner/model contract owns smooth references and first-derivative behavior.

## Fairness Contract

| Case | Domain | Model Family | dt | Horizon | Requested Steps | Comparable Backends | Contract Status |
|---|---|---|---:|---:|---:|---|---|
| `data_driven_mpc_lemniscate_tracking` | `robotics` | `double_integrator_3d_tracking` | 0.04 | 25 | 500 | `minisolver`, `acados`, `acados_sqp`, `casadi`, `clarabel`, `altro` | complete |
| `data_driven_mpc_loop_tracking` | `robotics` | `double_integrator_3d_tracking` | 0.04 | 25 | 500 | `minisolver`, `acados`, `acados_sqp`, `casadi`, `clarabel`, `altro` | complete |
| `nonlinear_mpcc_fssim_following` | `autonomous_driving` | `kinematic_bicycle_track_following` | 0.05 | 60 | 1200 | `minisolver`, `acados`, `acados_sqp`, `casadi` | complete |
| `nonlinear_mpcc_porto_following` | `autonomous_driving` | `kinematic_bicycle_track_following` | 0.05 | 50 | 600 | `minisolver`, `acados`, `acados_sqp`, `casadi` | complete |
| `mpcc_track_following` | `autonomous_driving` | `kinematic_bicycle_track_following` | 0.05 | 40 | 400 | `minisolver`, `acados`, `acados_sqp`, `casadi` | complete |

## Current Baseline Cases

These cases currently provide usable same-condition cross-solver comparisons under the present harness.

| Case | Backend | Contract | Requested Steps | Observed Steps | Success | Avg Iterations | Avg SQP Iterations | Median ms | P95 ms | Max ms | Avg Tracking Error | Max Constraint Violation |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `data_driven_mpc_lemniscate_tracking` | `minisolver` | comparable | 500 | 500 | 1 | 3.14 | - | 0.0525455 | 0.097711 | 0.180607 | 0.10448 | 0 |
| `data_driven_mpc_lemniscate_tracking` | `acados` | comparable | 500 | 500 | 1 | 2.54 | 1 | 0.063951 | 0.0986925 | 0.179833 | 0.0951679 | 0 |
| `data_driven_mpc_lemniscate_tracking` | `acados_sqp` | comparable | 500 | 500 | 1 | 2.54 | 1 | 0.0874864 | 0.137135 | 0.187522 | 0.0951679 | 0 |
| `data_driven_mpc_lemniscate_tracking` | `casadi` | comparable | 500 | 500 | 1 | 1.002 | - | 6.01327 | 7.34338 | 12.179 | 0.10448 | 1.77636e-15 |
| `data_driven_mpc_lemniscate_tracking` | `clarabel` | comparable | 500 | 500 | 1 | 7.452 | - | 0.326664 | 0.504898 | 0.592107 | 0.10448 | 0 |
| `data_driven_mpc_lemniscate_tracking` | `altro` | comparable | 500 | 500 | 0.388 | 2.27835 | - | 9.42932 | 9.71721 | 13.6595 | 195.26 | 3.53392e-05 |
| `data_driven_mpc_loop_tracking` | `minisolver` | comparable | 500 | 500 | 1 | 2.644 | - | 0.052616 | 0.0567311 | 0.104211 | 0.0273858 | 0 |
| `data_driven_mpc_loop_tracking` | `acados` | comparable | 500 | 500 | 1 | 2.078 | 1 | 0.063881 | 0.080922 | 0.195982 | 0.0214657 | 0 |
| `data_driven_mpc_loop_tracking` | `acados_sqp` | comparable | 500 | 500 | 1 | 2.078 | 1 | 0.087091 | 0.108361 | 0.253083 | 0.0214657 | 0 |
| `data_driven_mpc_loop_tracking` | `casadi` | comparable | 500 | 500 | 1 | 1 | - | 6.03558 | 6.10692 | 7.21056 | 0.0273858 | 0 |
| `data_driven_mpc_loop_tracking` | `clarabel` | comparable | 500 | 500 | 1 | 6.596 | - | 0.309244 | 0.39291 | 0.428415 | 0.0273858 | 0 |
| `data_driven_mpc_loop_tracking` | `altro` | comparable | 500 | 500 | 1 | 2 | - | 0.144421 | 0.165051 | 0.269874 | 0.0273858 | 0 |
| `nonlinear_mpcc_fssim_following` | `minisolver` | comparable | 1200 | 1200 | 1 | 2.10167 | - | 0.130951 | 0.167207 | 0.435546 | 0.00106131 | 0 |
| `nonlinear_mpcc_fssim_following` | `acados` | comparable | 1200 | 1200 | 0.999 | 5.58966 | 1 | 0.36434 | 0.410245 | 0.758859 | 0.00110226 | 0 |
| `nonlinear_mpcc_fssim_following` | `acados_sqp` | comparable | 1200 | 1200 | 1 | 5.90333 | 1.0525 | 0.405945 | 0.770262 | 0.931571 | 0.0010614 | 0 |
| `nonlinear_mpcc_fssim_following` | `casadi` | comparable | 1200 | 1200 | 0.969 | 1.42734 | - | 0.634703 | 138.899 | 4587.83 | 0.265533 | 0 |
| `nonlinear_mpcc_fssim_following` | `clarabel` | unsupported | 1200 | - | - | - | - | - | - | - | - | - |
| `nonlinear_mpcc_fssim_following` | `altro` | unsupported | 1200 | - | - | - | - | - | - | - | - | - |
| `nonlinear_mpcc_porto_following` | `minisolver` | comparable | 600 | 600 | 1 | 2.32833 | - | 0.112266 | 0.142581 | 0.333784 | 0.00208937 | 0 |
| `nonlinear_mpcc_porto_following` | `acados` | comparable | 600 | 600 | 0.998 | 6.03172 | 1 | 0.311229 | 0.358031 | 0.637907 | 0.00216734 | 0 |
| `nonlinear_mpcc_porto_following` | `acados_sqp` | comparable | 600 | 600 | 1 | 7.64667 | 1.26833 | 0.36701 | 0.694369 | 0.81313 | 0.00209014 | 0 |
| `nonlinear_mpcc_porto_following` | `casadi` | comparable | 600 | 600 | 0.8 | 1.55208 | - | 0.512161 | 3181.76 | 5099.49 | 1.87096 | 13.7211 |
| `nonlinear_mpcc_porto_following` | `clarabel` | unsupported | 600 | - | - | - | - | - | - | - | - | - |
| `nonlinear_mpcc_porto_following` | `altro` | unsupported | 600 | - | - | - | - | - | - | - | - | - |

## Stress Case

`mpcc_track_following` is shown separately as the retained harder driving track. Use it for ranking only when the compared rows pass the same success, tracking, and violation gate.

| Case | Backend | Contract | Requested Steps | Observed Steps | Success | Avg Iterations | Avg SQP Iterations | Median ms | P95 ms | Max ms | Avg Tracking Error | Max Constraint Violation |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `mpcc_track_following` | `minisolver` | comparable | 400 | 400 | 1 | 2.915 | - | 0.112421 | 0.150185 | 0.313634 | 0.0100655 | 0 |
| `mpcc_track_following` | `acados` | comparable | 400 | 400 | 0.998 | 7.03008 | 1 | 0.287478 | 0.324008 | 0.369775 | 0.0100741 | 0 |
| `mpcc_track_following` | `acados_sqp` | comparable | 400 | 400 | 1 | 12.8425 | 1.83 | 0.593323 | 0.934811 | 1.19007 | 0.0100745 | 0 |
| `mpcc_track_following` | `casadi` | comparable | 400 | 400 | 0.637 | 2.91765 | - | 1.35733 | 2559.7 | 4447.16 | 1.00218 | 4.2645 |
| `mpcc_track_following` | `clarabel` | unsupported | 400 | - | - | - | - | - | - | - | - | - |
| `mpcc_track_following` | `altro` | unsupported | 400 | - | - | - | - | - | - | - | - | - |

## Notes

- `time_ms` is end-to-end wall-clock time around each solver call inside the closed-loop runner.
- `Avg Iterations` is the comparable inner iteration count. For acados asset runners it is the summed HPIPM QP/IPM iterations per closed-loop step; `Avg SQP Iterations` is reported separately.
- One-time export, graph construction, CMake configure/build, and report aggregation are excluded from timing.
- `comparable` means that backend supports the candidate and produced the candidate's requested closed-loop step count. It does not by itself certify an equivalent full-solve optimality budget.
- `missing` means the backend should be comparable for that candidate but no raw result is present in `results/raw/`.
- `unsupported` means the backend does not implement that model family in this harness and is excluded from ranking for that case.
- `MiniSolver` uses generated models plus native C++ closed-loop runners with `tol_con=tol_dual=1e-4` for asset candidates.
- `acados` uses Python export/codegen plus compiled C closed-loop runners; export time is excluded. `acados` rows use `SQP_RTI`, so treat them as RTI closed-loop baselines and do not mix them into strict full-solve rankings when their quality metrics diverge.
- `acados_sqp` uses the same acados asset models with `nlp_solver_type=SQP`, `tol=qp_tol=1e-4`, and `nlp_solver_max_iter=60`; use these rows for the stricter MiniSolver-vs-acados full-solve comparison when their quality metrics pass the same gate.
- `CasADi` uses native C++ runners with a pre-built `nlpsol('sqpmethod')` graph.
- `ALTRO` uses ALTRO's C++ API and is currently limited to the convex robotics reference-tracking cases.
- `max_constraint_violation` is the current closed-loop step violation, not the maximum predicted violation over the full horizon.
- `Clarabel` is included only for convex robotics QP cases. It is not run on the nonconvex driving NMPC cases.
- `ALTRO` is included only for the robotics reference-tracking cases. It is not run on the nonconvex driving NMPC cases.
- `data_driven_mpc_loop_tracking` is the cleanest current all-backend robotics baseline, including ALTRO.
- `data_driven_mpc_lemniscate_tracking` remains a useful robotics benchmark, but ALTRO currently hits `MaxInnerIterations` on the later closed-loop steps. Treat that row as robustness/tuning evidence, not as a pure ALTRO latency comparison.
- Current `acados_sqp` driving rows pass the closed-loop quality gate used here: success rate is 1.0, max constraint violation is 0, and tracking is comparable to MiniSolver.
- `nonlinear_mpcc_porto_following` and `nonlinear_mpcc_fssim_following` are the current primary driving baselines. Interpret success, tracking, and latency together.
- `mpcc_track_following` is now quality-aligned for `minisolver` vs `acados_sqp`, but remains separated from the primary baseline section because it is the harder legacy track.

## Files

- Raw CSVs: `results/raw/minisolver/`, `results/raw/acados/`, `results/raw/acados_sqp/`, `results/raw/casadi/`, `results/raw/clarabel/`, `results/raw/altro/`
- Aggregate CSV: `results/summary.csv`
- Aggregate JSON: `results/summary.json`
