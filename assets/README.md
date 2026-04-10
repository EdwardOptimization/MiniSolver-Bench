# Reference Assets

This directory stores small, canonical reference assets used to seed new
benchmark cases before a solver-specific runner exists.

The current intent is pragmatic:

- pull public driving tracks and trajectories from MPC paper repositories
- pull light-weight sample trajectories from public robotics MPC repositories
- convert them into stable CSV schemas that `MiniSolver`, `acados`, and other
  backends can consume later without depending on the original repository layout

## Import

```bash
python3 scripts/import_reference_assets.py
```

The importer writes:

- `assets/reference/driving/*.csv`
- `assets/reference/robotics/*.csv`
- sidecar metadata files `*.json`
- aggregate manifest `assets/catalog.json`

## Schemas

Driving track with explicit boundaries:

- `index`
- `s`
- `x_center`, `y_center`
- `x_inner`, `y_inner`
- `x_outer`, `y_outer`
- `width_inner`, `width_outer`

Driving track with centerline and widths:

- `index`
- `s`
- `x_center`, `y_center`
- `width_left`, `width_right`

XY trajectory:

- `timestamp`
- `track_id`
- `object_type`
- `x`, `y`
- `city_name`

Robot reference trajectory:

- `index`
- `t`
- `x`, `y`, `z`
- `vx`, `vy`, `vz`
- `ax`, `ay`, `az`
