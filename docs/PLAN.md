# Plan (v0)

## Phase 0: Scaffolding (1 day)

- Add a CMake target `minisolver_real_scenarios` (C++ runner) to the existing harness build.
- Add a tiny JSON/CSV loader (header-only, no heavy deps).
- Add `tools/requirements.txt`.

## Phase 1: Two Real-World Scenario Families (2-3 days)

- Driving:
  - Choose an open dataset with 2D trajectories (e.g. NGSIM or similar).
  - Provide `tools/import_ngsim.py` to extract a few short segments.
  - Output `ref.csv` with pose/speed references and simple lane width bounds.
- Robotics:
  - Use TUM RGB-D ground-truth trajectories.
  - Provide `tools/import_tum_rgbd.py` to extract a few short segments.
  - Output `ref.csv` with position/velocity references.

Commit only derived segments (small CSV) + clear attribution.

## Phase 2: Ground Truth (1 day)

- Implement `tools/generate_gt.py`:
  - Solve per-scenario open-loop NMPC with CasADi + IPOPT at high precision.
  - Store compact `gt.json` (objective, terminal state, first control).

## Phase 3: MiniSolver Runner (1-2 days)

- Implement `src/minisolver_runner.cpp`:
  - Support `kinematic_bicycle_v1` and `double_integrator_3d_v1`.
  - Run single-shot open-loop solve (no MPC loop first).
  - Print JSON lines with metrics for easy aggregation.

## Exit Criteria (v0)

- `cmake && build` works on Ubuntu.
- `--list` prints scenarios.
- At least 2 driving + 2 robotics scenarios exist.
- For scenarios with `gt.json`, MiniSolver matches within tolerance.
