# acados Official Runner

This backend adapts official `acados` examples into a compiled benchmark path:

1. Python is used only for solver export.
2. Runtime benchmarking is done by a compiled C executable built from the exported solver code.
3. The generated `main_<model>.c` is replaced with a closed-loop benchmark runner per case.

Supported official cases:

- `pendulum_on_cart`
- `race_cars`
- `quadrotor_nav`
- `chain_mass`

## Prerequisites

- an `acados` checkout, for example `/tmp/acados-official`
- Python dependencies available to the export step:
  - `casadi`
  - `numpy`
  - `scipy`
- an `acados` build or install that provides headers and libraries for the generated CMake project

Generated code and build artifacts are written under ignored paths:

- `out/acados/`
- `results/raw/acados/`

## Run one case

```bash
python3 backends/acados/run.py \
  --case pendulum_on_cart \
  --acados-repo /path/to/acados
```

Useful flags:

- `--skip-export`
- `--skip-build`
- `--steps`
- `--output`
- `--output-root`

## Run all four

```bash
python3 backends/acados/run_all.py \
  --acados-repo /path/to/acados
```
