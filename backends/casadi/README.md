# CasADi Backend

This backend uses CasADi's `nlpsol("sqpmethod")` plugin with `qrqp` as the QP backend.

Benchmarking policy:

1. The NLP graph is built once per case.
2. Closed-loop timings exclude graph construction.
3. The timed region is the repeated CasADi SQP solve call inside the benchmark loop.

Supported official cases:

- `pendulum_on_cart`
- `race_cars`
- `quadrotor_nav`
- `chain_mass`

Run one case:

```bash
python3 backends/casadi/run.py --case pendulum_on_cart
```

Run all four:

```bash
python3 backends/casadi/run_all.py
```
