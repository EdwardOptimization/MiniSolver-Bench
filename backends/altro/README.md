# ALTRO Backend

This backend adapts ALTRO's C++ API into the asset-derived benchmark harness.

Current scope:

- supported model family: `double_integrator_3d_tracking`
- supported candidates:
  - `data_driven_mpc_loop_tracking`
  - `data_driven_mpc_lemniscate_tracking`

It is intentionally limited to the convex robotics reference-tracking assets.
The nonconvex driving candidates use kinematic-bicycle track-following models
and are not implemented for ALTRO in this harness.

Timing follows the repository's same-case end-to-end policy: the measured value
is wall-clock time around each ALTRO solve call inside the closed-loop runner.
CMake configure/build, dependency fetching, and report aggregation are excluded.

## Prerequisites

- `third_party/ALTRO` initialized as a git submodule
- CMake 3.23 or newer, matching ALTRO's own CMake requirement
- network access during the first configure, because ALTRO fetches some CMake
  dependencies through `FetchContent`

Initialize dependencies from a fresh clone with:

```bash
git submodule update --init --recursive
```

## Run One Candidate

```bash
python3 backends/altro/run_candidate.py \
  --candidate data_driven_mpc_loop_tracking
```

Useful flags:

- `--altro-source-dir`
- `--build-dir`
- `--binary`
- `--steps`
- `--output`

## Run All Supported Candidates

```bash
python3 backends/altro/run_candidates.py
```

Generated build and result files are written under ignored paths:

- `.build/altro_assets/`
- `results/raw/altro/`
