# Reference Asset Summary

| Name | Domain | Kind | Points | Main Stats |
|---|---|---|---:|---|
| `mpcc_track` | `autonomous_driving` | `closed_track` | 489 | track_length=17.800, mean_width=0.370 |
| `nonlinear_mpcc_porto` | `autonomous_driving` | `closed_track` | 525 | track_length=56.202, mean_width=3.041 |
| `nonlinear_mpcc_fssim` | `autonomous_driving` | `closed_track` | 1124 | track_length=307.507, mean_width=3.886 |
| `argoverse_sample_av` | `autonomous_driving` | `trajectory` | 2 | duration=1.000, path_length=5.000 |
| `argoverse_sample_agent` | `autonomous_driving` | `trajectory` | 2 | duration=1.000, path_length=5.000 |
| `data_driven_mpc_loop` | `robotics` | `reference_trajectory` | 500 | duration=19.960, path_length=80.423, max_speed=8.273, max_accel=13.689 |
| `data_driven_mpc_lemniscate` | `robotics` | `reference_trajectory` | 500 | duration=19.960, path_length=77.963, max_speed=11.688, max_accel=28.782 |

## Notes

- Driving assets with explicit track geometry are the best candidates for near-term solver benchmarks.
- The Argoverse sample is only a tiny import sanity check, not yet a serious benchmark case.
- The robotics loop and lemniscate assets are reconstructed from public trajectory formulas and defaults in `data_driven_mpc`.
