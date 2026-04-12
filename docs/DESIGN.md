# Design: Real-World NMPC Scenarios

## Problem

MiniSolver currently lacks **credible, real-world scenarios** to validate:
- Correctness (does the solution match a high-accuracy reference?)
- Robustness (success rate across varied trajectories/initializations)
- Performance (time/iters under realistic targets and constraints)

Hand-made tutorial examples are useful for development but weak as evidence.

## Goals

- Provide a small set of **dataset-driven** NMPC scenarios covering:
  - Autonomous driving (2D pose + speed tracking with road/lane constraints)
  - Robotics (3D position/velocity tracking)
- Keep the benchmark **fair**:
  - Same dynamics, discretization, horizon, weights, and bounds across solvers.
  - No Python runtime runner for timing; use **C/C++** runners.
- Keep this repo **lightweight**:
  - No full dataset vendoring.
  - Import scripts download/parse datasets externally.
  - Commit only short, derived scenario segments (CSV/JSON).

## Non-Goals (Now)

- Full closed-loop simulation stacks (CARLA/Gazebo/PX4/Autoware).
- GPU acceleration evaluation (CPU only first).
- Supporting every dataset format or every solver backend on day 1.

## Scenario Contract

A scenario directory contains:
- `scenario.json`: metadata and solver-independent problem definition
- `ref.csv`: time-indexed reference targets (dataset-derived)
- `gt.json` (optional): high-accuracy ground truth (offline generated)

### `scenario.json` (v0)

Minimal schema (subject to change, no backward-compat promise):
- `name`: string
- `domain`: `"driving"` or `"robotics"`
- `model`: string id, e.g. `"kinematic_bicycle_v1"`, `"double_integrator_3d_v1"`
- `dt`: seconds
- `N`: horizon length
- `x0`: initial state array
- `weights`: dict of named weights (model-specific)
- `bounds`: dict of named bounds (model-specific)
- `ref`: relative path to `ref.csv`
- `gt`: relative path to `gt.json` (optional)

### `ref.csv` (v0)

Driving (example columns):
- `t, x_ref, y_ref, yaw_ref, v_ref, w_left, w_right`

Robotics (example columns):
- `t, x_ref, y_ref, z_ref, vx_ref, vy_ref, vz_ref`

Notes:
- The runner is allowed to resample/interpolate to match `dt`.
- Reference is **targets only**. Dynamics remain solver-defined via the `model` id.

### `gt.json` (v0)

Store a small set of verifiable numbers (not full trajectories):
- `objective`
- `terminal_state`
- `first_control`
- `status` + `solver` + `tolerance`

This is used for correctness gates, not for benchmarking speed.

## Architecture

### Offline Pipeline (Python)

- `tools/import_<dataset>.py`
  - Downloads or reads a dataset (user-provided path)
  - Extracts short segments
  - Writes `scenarios/<name>/{scenario.json, ref.csv}`
- `tools/generate_gt.py`
  - Uses CasADi + IPOPT to solve the open-loop NMPC at high precision
  - Writes `gt.json`

### Online Runners (C/C++)

- `src/minisolver_runner.cpp`
  - Loads `scenario.json` + `ref.csv`
  - Instantiates a MiniSolver model matching `model` id
  - Solves and reports:
    - runtime (wall time)
    - iterations
    - objective + constraint residuals
    - error vs `gt.json` (if present)

Future (optional):
- `src/acados_runner.c`
- `src/casadi_runner.cpp` (C++ API)

## Metrics

Correctness (must have, gate):
- Objective relative error vs ground-truth
- Terminal state error
- First control error

Performance (benchmark):
- Solve time (p50/p90)
- Iterations (avg/p90)
- Success rate

## Repository Layout

```
nmpc-bench/
  docs/
    DESIGN.md
    PLAN.md
  tools/
    requirements.txt
    import_ngsim.py
    import_tum_rgbd.py
    generate_gt.py
  scenarios/
    <scenario_name>/
      scenario.json
      ref.csv
      gt.json
  src/
    minisolver_runner.cpp
  CMakeLists.txt
```
