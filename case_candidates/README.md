# Case Candidates

These files are the asset-derived specifications used by the formal
same-condition solver comparison report in `results/asset_benchmark_report.md`.

Use them to:

- keep a stable suggested `dt`, `horizon`, and metric set per asset
- avoid repeating benchmark design decisions every time a new solver backend is added
- declare which model family a backend must implement before a result is
  considered comparable

The current candidates intentionally cover both:

- autonomous driving
- robotics

Modeling notes:

- `double_integrator_3d_tracking` candidates pass reference data through native
  closed-loop runners and do not need Python MiniModel piecewise modeling.
- `kinematic_bicycle_track_following` candidates use runner-side track sampling
  and stage parameters. Treat that as the callback-like/parametric path: the
  runner/model contract owns smooth references and first-derivative behavior.
- Official `race_cars` and `quadrotor_nav` are not duplicated here because their
  MiniSolver models depend on piecewise track functions and currently use
  checked-in generated C++ headers when Python MiniModel lacks `ppoly`.
