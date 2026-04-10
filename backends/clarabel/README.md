# Clarabel Backend

This backend is intentionally limited to **convex asset-derived cases**.

Current scope:

- supported model family: `double_integrator_3d_tracking`
- supported candidates:
  - `data_driven_mpc_loop_tracking`
  - `data_driven_mpc_lemniscate_tracking`

It uses the Python `clarabel` package and solves the condensed convex QP
directly. It is not used for the nonconvex driving NMPC cases.

Timing policy:

- matrix structure setup is done once
- per-step timing includes data update plus `solve()`
- Python import/install time is excluded
