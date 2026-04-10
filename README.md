# nmpc-bench

Cross-solver NMPC benchmark harness.

This repository is intentionally solver-neutral. It keeps:

- canonical benchmark case specs
- native targets for embedded solvers and adapters for external solver stacks
- git submodules under `third_party/` for external solver code
- result directories
- plotting utilities
- CI for validating the harness itself

It does **not** vendor solver source trees or generated benchmark outputs.
External solvers are pulled in as git submodules instead.

Fairness rules in this repository:

- official `acados` examples define the benchmark problems
- `acados` timings measure compiled C closed-loop runners, not Python export/codegen
- `MiniSolver` runs as a native embedded C++ target linked directly into the benchmark app
- `CasADi` timings measure repeated solve calls on a pre-built CasADi SQP graph; one-time graph construction is excluded
- case-specific termination follows the official closed-loop logic for that case

## Layout

- `cases/`: benchmark case definitions
- `targets/minisolver/`: MiniSolver native benchmark target and local runner source
- `backends/acados/`: acados export/build/run adapter
- `backends/casadi/`: CasADi SQP adapter
- `assets/`: canonical external reference assets imported from public MPC repositories
- `third_party/`: solver checkouts managed as git submodules
- `results/raw/`: ignored raw benchmark output
- `results/figures/`: ignored generated plots
- `plots/`: result aggregation and plotting helpers
- `run.py`: unified entry point

## Current cases

- `pendulum_on_cart`
- `race_cars`
- `quadrotor_nav`
- `chain_mass`

## Current backends

- `minisolver`
- `acados`
- `casadi`
- `clarabel` for convex asset cases only

## External Reference Assets

Besides the official `acados` solver cases, this repository now keeps a small
asset library for future benchmark expansion across different model families.

Current imported sources:

- autonomous driving:
  - `alexliniger/MPCC`
  - `nirajbasnet/Nonlinear_MPCC_for_autonomous_racing`
  - `argoverse-api` sample trajectories
- robotics:
  - `uzh-rpg/data_driven_mpc` loop and lemniscate references

Import or refresh them with:

```bash
python3 scripts/import_reference_assets.py
```

The importer writes canonical CSVs under `assets/reference/` plus provenance
metadata in `assets/catalog.json`.

## Examples

Initialize external solvers:

```bash
git submodule update --init --recursive
./scripts/bootstrap_acados.sh
```

Run MiniSolver on the official pendulum case:

```bash
python3 run.py --backend minisolver --case pendulum_on_cart
```

Run MiniSolver against a local development checkout instead of the pinned submodule:

```bash
python3 run.py --backend minisolver --case pendulum_on_cart --minisolver-source-dir /path/to/MiniSolver
```

Run acados on the same case:

```bash
python3 run.py --backend acados --case pendulum_on_cart
```

Run CasADi SQP on the same case:

```bash
python3 run.py --backend casadi --case pendulum_on_cart
```

Aggregate the latest raw data:

```bash
./scripts/summarize_results.py
./scripts/write_report.py
```

Run all four MiniSolver cases from a local development checkout:

```bash
python3 targets/minisolver/run_all.py \
  --minisolver-source-dir /path/to/MiniSolver \
  --solver-backend cpu \
  --refresh-assets
```

Run all public-asset-derived MiniSolver benchmark candidates:

```bash
python3 targets/minisolver/run_case_candidates.py \
  --minisolver-source-dir /path/to/MiniSolver
```

Run all public-asset-derived acados benchmark candidates:

```bash
python3 backends/acados/run_candidates.py \
  --acados-repo third_party/acados
```

Run all public-asset-derived CasADi benchmark candidates:

```bash
python3 backends/casadi/run_candidates.py
```

Run all convex Clarabel asset candidates:

```bash
python3 backends/clarabel/run_candidates.py
```

Run a single asset-derived candidate:

```bash
python3 targets/minisolver/run_case_candidate.py \
  --candidate case_candidates/nonlinear_mpcc_porto_following.json \
  --minisolver-source-dir /path/to/MiniSolver
```

Refresh summary and reports after asset benchmark runs:

```bash
python3 scripts/summarize_results.py
python3 scripts/write_report.py
python3 scripts/write_asset_benchmark_report.py
```

Run all four acados cases:

```bash
python3 backends/acados/run_all.py \
  --acados-repo third_party/acados
```

Run all four CasADi cases:

```bash
python3 backends/casadi/run_all.py \
  --acados-repo third_party/acados
```
