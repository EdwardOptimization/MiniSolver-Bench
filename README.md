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
- `backends/altro/`: ALTRO C++ adapter for robotics asset candidates
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
- `altro` for convex robotics asset cases only

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

Run all ALTRO robotics asset candidates:

```bash
python3 backends/altro/run_candidates.py
```

Refresh the same-case end-to-end solver comparison report:

```bash
python3 scripts/summarize_results.py
python3 scripts/write_asset_benchmark_report.py
```

Run a single asset-derived candidate:

```bash
python3 targets/minisolver/run_case_candidate.py \
  --candidate case_candidates/nonlinear_mpcc_porto_following.json \
  --minisolver-source-dir /path/to/MiniSolver
```

Refresh all summary and asset reports after benchmark runs:

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

## Dataset-Driven Scenarios (CPU, C++ Runner)

This repo also includes a tiny set of **dataset-driven** NMPC scenarios under `scenarios/`.
They use public datasets as sources of *reference trajectories*, while dynamics/constraints remain solver-defined.

Design notes live under `docs/` (see `docs/DESIGN.md`).

Build the C++ runner:

```bash
cmake -S . -B build -DMINISOLVER_SOURCE_DIR=/path/to/MiniSolver
cmake --build build -j
./build/minisolver_real_scenarios --list
./build/minisolver_real_scenarios --scenario scenarios/ngsim_i80_v36/scenario.json --check-gt
```

Import datasets (offline):

```bash
python3 tools/import_ngsim.py --vehicle-id 36 --location i-80 --start-time-ms 1113433135300 --duration-s 5.0 --N 50 --dt 0.1 --name ngsim_i80_v36 --output-root scenarios
python3 tools/import_tum_rgbd.py --url https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz-groundtruth.txt --start-time 1305031098.6659 --duration-s 6.0 --N 50 --dt 0.1 --name tum_freiburg1_xyz --output-root scenarios
```

Regenerate ground-truth (offline):

```bash
python3 -m pip install -r tools/requirements.txt
python3 tools/generate_gt.py --scenario-root scenarios
```
