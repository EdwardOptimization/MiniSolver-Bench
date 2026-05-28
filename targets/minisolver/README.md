# MiniSolver Native Target

This target builds a native benchmark app directly against a `MiniSolver` checkout.
It is not an adapter layer: the benchmark case links MiniSolver as an embedded C++ library and runs the solver in-process.

By default it uses the sibling `../MiniSolver` checkout when present, otherwise `third_party/MiniSolver`.
It does not modify the `MiniSolver` repository and does not require benchmark-specific files to live there.

## Example

```bash
python3 run.py --case pendulum_on_cart
```

For an active local branch:

```bash
python3 run.py --case pendulum_on_cart --minisolver-source-dir /path/to/MiniSolver
```

Asset-derived benchmark candidates use separate runners:

```bash
python3 run_case_candidate.py \
  --candidate ../../case_candidates/nonlinear_mpcc_porto_following.json \
  --minisolver-source-dir /path/to/MiniSolver
```

## Race Cars Tuning

The checked-in `race_cars` default is a quality-first multi-iteration MiniSolver
profile, not an RTI fixed-step profile. For local sweeps, the native runner also
accepts `MINISOLVER_RACE_*` environment overrides, for example:

```bash
MINISOLVER_RACE_MAX_ITERS=40 \
MINISOLVER_RACE_OBJECTIVE_SCALING=hessian \
python3 run.py --case race_cars
```

Useful values include `MINISOLVER_RACE_PROFILE=strict|acceptable`,
`MINISOLVER_RACE_LINE_SEARCH=filter|merit|none`,
`MINISOLVER_RACE_BARRIER=mehrotra|adaptive|monotone`,
`MINISOLVER_RACE_ROLLOUT=0|1`,
`MINISOLVER_RACE_OBJECTIVE_SCALING=hessian|none`, and
`MINISOLVER_RACE_PROBLEM_SCALING=ruiz|none`.
