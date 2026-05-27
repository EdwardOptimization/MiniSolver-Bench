# End-to-End Asset Solver Comparison

Same-case, closed-loop, end-to-end solver comparison for public-asset-derived candidates.

This is the repository's current headline report for solver ranking. It excludes official `race_cars` and `quadrotor_nav` because those cases require piecewise track functions; MiniSolver currently runs them from checked-in generated C++ models when Python MiniModel lacks `ppoly`.

Each solver row uses the same asset, model family, `dt`, horizon, and requested closed-loop step count for that candidate. The timed region is the per-step solver call inside the closed-loop runner; one-time code generation, CMake configure/build, Python import time, and report aggregation are excluded.

The asset candidates do not rely on Python MiniModel piecewise modeling. Robotics candidates use native reference-tracking data, and driving candidates use runner-side track sampling plus stage parameters. For any callback-backed track model, the benchmark owner must provide smooth references and first-derivative behavior before treating the case as a headline comparison.

## Fairness Contract

| Case | Domain | Model Family | dt | Horizon | Requested Steps | Comparable Backends | Contract Status |
|---|---|---|---:|---:|---:|---|---|
| `data_driven_mpc_lemniscate_tracking` | `robotics` | `double_integrator_3d_tracking` | 0.04 | 25 | 500 | `minisolver`, `acados`, `casadi`, `clarabel`, `altro` | complete |
| `data_driven_mpc_loop_tracking` | `robotics` | `double_integrator_3d_tracking` | 0.04 | 25 | 500 | `minisolver`, `acados`, `casadi`, `clarabel`, `altro` | complete |
| `nonlinear_mpcc_fssim_following` | `autonomous_driving` | `kinematic_bicycle_track_following` | 0.05 | 60 | 1200 | `minisolver`, `acados`, `casadi` | complete |
| `nonlinear_mpcc_porto_following` | `autonomous_driving` | `kinematic_bicycle_track_following` | 0.05 | 50 | 600 | `minisolver`, `acados`, `casadi` | complete |
| `mpcc_track_following` | `autonomous_driving` | `kinematic_bicycle_track_following` | 0.05 | 40 | 400 | `minisolver`, `acados`, `casadi` | complete |

## Current Baseline Cases

These cases currently provide usable same-condition cross-solver comparisons under the present harness.

| Case | Backend | Contract | Requested Steps | Observed Steps | Success | Median ms | P95 ms | Max ms | Avg Tracking Error | Max Constraint Violation |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `data_driven_mpc_lemniscate_tracking` | `minisolver` | comparable | 500 | 500 | 1 | 0.041137 | 0.0787125 | 0.189789 | 0.10448 | 0 |
| `data_driven_mpc_lemniscate_tracking` | `acados` | comparable | 500 | 500 | 1 | 0.065299 | 0.107706 | 0.179569 | 0.0951679 | 0 |
| `data_driven_mpc_lemniscate_tracking` | `casadi` | comparable | 500 | 500 | 1 | 6.4841 | 7.78326 | 8.29397 | 0.10448 | 1.77636e-15 |
| `data_driven_mpc_lemniscate_tracking` | `clarabel` | comparable | 500 | 500 | 1 | 0.33702 | 0.577829 | 0.738871 | 0.10448 | 0 |
| `data_driven_mpc_lemniscate_tracking` | `altro` | comparable | 500 | 500 | 0.388 | 9.72021 | 10.0043 | 12.9042 | 195.26 | 3.53392e-05 |
| `data_driven_mpc_loop_tracking` | `minisolver` | comparable | 500 | 500 | 1 | 0.041172 | 0.0437485 | 0.095101 | 0.0273859 | 0 |
| `data_driven_mpc_loop_tracking` | `acados` | comparable | 500 | 500 | 1 | 0.0651225 | 0.0822591 | 0.148161 | 0.0214657 | 0 |
| `data_driven_mpc_loop_tracking` | `casadi` | comparable | 500 | 500 | 1 | 6.42517 | 6.50445 | 7.78572 | 0.0273858 | 0 |
| `data_driven_mpc_loop_tracking` | `clarabel` | comparable | 500 | 500 | 1 | 0.323274 | 0.407224 | 0.442793 | 0.0273858 | 0 |
| `data_driven_mpc_loop_tracking` | `altro` | comparable | 500 | 500 | 1 | 0.148477 | 0.170358 | 0.272993 | 0.0273858 | 0 |
| `nonlinear_mpcc_fssim_following` | `minisolver` | comparable | 1200 | 1200 | 1 | 0.114802 | 0.178778 | 1.46696 | 0.00106131 | 0 |
| `nonlinear_mpcc_fssim_following` | `acados` | comparable | 1200 | 1200 | 0.999 | 1.87401 | 2.43573 | 2.85635 | 29.5273 | 53.5924 |
| `nonlinear_mpcc_fssim_following` | `casadi` | comparable | 1200 | 1200 | 0.969 | 0.604417 | 135.907 | 4651.08 | 0.265533 | 0 |
| `nonlinear_mpcc_fssim_following` | `clarabel` | unsupported | 1200 | - | - | - | - | - | - | - |
| `nonlinear_mpcc_fssim_following` | `altro` | unsupported | 1200 | - | - | - | - | - | - | - |
| `nonlinear_mpcc_porto_following` | `minisolver` | comparable | 600 | 600 | 1 | 0.123479 | 0.143619 | 1.81116 | 0.00208937 | 0 |
| `nonlinear_mpcc_porto_following` | `acados` | comparable | 600 | 600 | 0.998 | 1.04805 | 1.48404 | 1.94464 | 6.01463 | 15.4147 |
| `nonlinear_mpcc_porto_following` | `casadi` | comparable | 600 | 600 | 0.8 | 0.566308 | 3162.63 | 5046.1 | 1.87096 | 13.7211 |
| `nonlinear_mpcc_porto_following` | `clarabel` | unsupported | 600 | - | - | - | - | - | - | - |
| `nonlinear_mpcc_porto_following` | `altro` | unsupported | 600 | - | - | - | - | - | - | - |

## Stress Case

`mpcc_track_following` is kept separate because it is still a stress case, not a stable baseline.

| Case | Backend | Contract | Requested Steps | Observed Steps | Success | Median ms | P95 ms | Max ms | Avg Tracking Error | Max Constraint Violation |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `mpcc_track_following` | `minisolver` | comparable | 400 | 400 | 1 | 0.101953 | 0.136815 | 0.162967 | 0.0100655 | 0 |
| `mpcc_track_following` | `acados` | comparable | 400 | 400 | 0.998 | 0.572381 | 0.828174 | 1.24253 | 1.15544 | 3.99598 |
| `mpcc_track_following` | `casadi` | comparable | 400 | 400 | 0.637 | 1.3366 | 2497.13 | 4424.52 | 1.00218 | 4.2645 |
| `mpcc_track_following` | `clarabel` | unsupported | 400 | - | - | - | - | - | - | - |
| `mpcc_track_following` | `altro` | unsupported | 400 | - | - | - | - | - | - | - |

## Notes

- `time_ms` is end-to-end wall-clock time around each solver call inside the closed-loop runner.
- One-time export, graph construction, CMake configure/build, and report aggregation are excluded from timing.
- `comparable` means that backend supports the candidate and produced the candidate's requested closed-loop step count.
- `missing` means the backend should be comparable for that candidate but no raw result is present in `results/raw/`.
- `unsupported` means the backend does not implement that model family in this harness and is excluded from ranking for that case.
- `MiniSolver` uses generated models plus native C++ closed-loop runners.
- `acados` uses Python export/codegen plus compiled C closed-loop runners; export time is excluded.
- `CasADi` uses native C++ runners with a pre-built `nlpsol('sqpmethod')` graph.
- `ALTRO` uses ALTRO's C++ API and is currently limited to the convex robotics reference-tracking cases.
- `max_constraint_violation` is the current closed-loop step violation, not the maximum predicted violation over the full horizon.
- `Clarabel` is included only for convex robotics QP cases. It is not run on the nonconvex driving NMPC cases.
- `ALTRO` is included only for the robotics reference-tracking cases. It is not run on the nonconvex driving NMPC cases.
- `data_driven_mpc_loop_tracking` is the cleanest current all-backend robotics baseline, including ALTRO.
- `data_driven_mpc_lemniscate_tracking` remains a useful robotics benchmark, but ALTRO currently hits `MaxInnerIterations` on the later closed-loop steps. Treat that row as robustness/tuning evidence, not as a pure ALTRO latency comparison.
- `nonlinear_mpcc_porto_following` and `nonlinear_mpcc_fssim_following` are the current driving baselines. Interpret success, tracking, and latency together; none of the three solvers dominates every metric.
- `mpcc_track_following` should not be used for headline conclusions yet. It is still exposing model/setup mismatch rather than a stable solver ranking.

## Files

- Raw CSVs: `results/raw/minisolver/`, `results/raw/acados/`, `results/raw/casadi/`, `results/raw/clarabel/`, `results/raw/altro/`
- Aggregate CSV: `results/summary.csv`
- Aggregate JSON: `results/summary.json`
