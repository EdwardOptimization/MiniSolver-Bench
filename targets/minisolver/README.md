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
