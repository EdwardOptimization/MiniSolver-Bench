# Reference Source Survey

This note records which public MPC-related repositories are currently useful as
benchmark asset sources.

## Autonomous Driving

### `alexliniger/MPCC`

- status: imported
- useful assets:
  - `C++/Params/track.json`
  - `C++/Params/bounds.json`
- why it matters:
  - compact closed-track benchmark
  - direct centerline and inner/outer boundaries
  - low-friction import path into canonical CSV

### `nirajbasnet/Nonlinear_MPCC_for_autonomous_racing`

- status: imported
- useful assets:
  - `nonlinear_mpc_casadi/scripts/porto/*.csv`
  - `nonlinear_mpc_casadi/scripts/fssim/*.csv`
- why it matters:
  - different track geometry from MPCC
  - direct centerline plus left/right width files
  - good candidate for multiple racing-style benchmark cases

### `argoverse-api`

- status: imported sample only
- useful assets:
  - `tests/test_data/forecasting/0.csv`
- why it matters:
  - very small trajectory sample
  - good for testing the import/normalization path
  - not yet a serious NMPC benchmark by itself

## Robotics

### `uzh-rpg/data_driven_mpc`

- status: imported reconstructed references
- useful assets:
  - `trajectory_test.py`
  - `trajectories.py`
- why it matters:
  - public quadrotor reference families with clear formulas
  - gives a robotics-domain benchmark seed without depending on ROS bags
- caveat:
  - the repository does not ship a standard benchmark dataset
  - current CSVs are reconstructed from the public trajectory formulas and
    experiment defaults, not exported from the original ROS stack

## Repositories Reviewed But Not Promoted To Canonical Assets

### `tud-amr/mpc_planner`

- contains configs, worlds, and scenario logic
- more suitable for simulator-based scenario generation than direct asset import

### `uzh-rpg/rpg_mpc`

- contains controller parameters but not enough reusable public benchmark data

### `HKUST-Aerial-Robotics/Fast-Planner`

- contains maps, planning utilities, and simulator support
- more useful for environment generation than for a compact benchmark asset set

## Practical Recommendation

If the next step is to add more formal benchmark cases, prioritize:

1. `mpcc_track`
2. `nonlinear_mpcc_porto`
3. `nonlinear_mpcc_fssim`
4. `data_driven_mpc_loop`
5. `data_driven_mpc_lemniscate`

This gives both autonomous driving and robotics coverage, with multiple model
families and low dependency overhead.
