# Third-Party Solvers

External solvers are kept as git submodules so the benchmark harness stays solver-neutral.

Current submodules:

- `MiniSolver`
- `acados`
- `ALTRO`

Runtime dependencies that are not vendored as submodules:

- `casadi` Python package for the `casadi` backend
- CMake/FetchContent dependencies pulled by `ALTRO` during the first configure

Initialize everything with:

```bash
git submodule update --init --recursive
```
