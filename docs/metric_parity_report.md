# S2 metric parity report

## Scope

This report validates the frozen S2 planning-level contract against all nine
accepted S1 raw trajectories.  It does not rerun the S1 matrix and does not
claim benchmark, controller, dynamics, wind, or paper results.

Official source commit: `c87fc76044b350a37fcea7afc468c13c8371a237`.
Project baseline head: `b68212d5f5e7ede4d001d68a40378d2cdf4647fa`.

## Results

| check | result |
|---|---:|
| raw S1 runs evaluated | 9/9 |
| candidate index | 0 for 9/9 |
| state/position/velocity shape | 9/9 |
| direct official vs evaluator collision boolean | 9/9, mismatch 0 |
| path-length parity | PASS |
| maximum path-length absolute error | `9.5367431640625e-07` |
| acceleration parity | PASS |
| maximum acceleration absolute error | `3.5762786865234375e-07` |
| runtime parse parity | PASS |

The path-length and acceleration tolerances were fixed at `1e-6` before the
test; they were not widened to obtain PASS.

## Definitions checked

- Candidate zero is selected directly from `raw_paths[0]`.
- Position is the first `2*N` state values and velocity is the remaining
  `2*N` values, each decoded to `(N,T,2)`.
- Collision is the official `is_collision.py::check_paths_ok` function with
  radius `0.05` and threshold `1e-3`.
- Path length calls official
  `compute_path_length_from_pos`; the independent reference sums adjacent
  position Euclidean distances.
- Acceleration calls official
  `compute_average_acceleration_from_pos_vel`; the independent reference
  averages adjacent velocity-change magnitudes and does not divide by `dt`.
- Runtime parsing reads the official `Planning times:` line from the one
  permitted 3-agent runtime parity retry.  The parsed planning time was
  `16.81274962425232 s`, and the external wrapper wall time was
  `26.1729535 s`; `planning_time <= wall_runtime`.

## Signed-clearance caveat

Signed geometric clearance and the official collision boolean are separate
metrics.  S1 contains valid official collision-free trajectories with
negative signed obstacle clearance because the official test uses squared
distance compared with `(radius_sum)^2 - 1e-3`, not a signed-clearance cutoff
of `-0.001 m`.

## Failure and retry audit

The first runtime parity wrapper attempt created raw output but failed to save
stdout because of a wrapper-relative log path.  It is retained as an
`INFRASTRUCTURE_FAILURE`; one identical retry produced the complete stdout
and is the only runtime parity result used.  The two historical S1 dense
retries remain preserved in the S1 evidence and are linked from the baseline
CSV via `retry_of`.

