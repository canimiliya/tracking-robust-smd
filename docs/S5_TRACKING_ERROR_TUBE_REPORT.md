# S5 Tracking Error Tube Report

## Decision

`S5_TRACKING_ERROR_TUBE_READY` is submitted for review. The preregistered D1
holdout had zero violations in 110/110 trajectory trials, and all six S4 D1
collision references were covered. This is validated conditional coverage, not
a formal guarantee and not evidence that the unmodified planner is safe.

## Method and Calibration

The frozen 13-state nonlinear CF2X/controller/RK4 stack is propagated directly;
the local diagnostic error is `[delta_p, delta_v, delta_theta, delta_omega]`
with quaternion-log attitude error. For each agent/reference, 21 bounded-force
scenarios form a time-varying scalar position envelope. The model component is
their pointwise maximum. Across 26 calibration references (208 trials) and 18
validation references (144 trials), the largest unseen-schedule/base ratio was
1.0190495. The preregistered 10% safety factor froze the multiplier at
1.1209545; the measured additive residual was zero, leaving only `1e-9 m`
numerical inflation. Holdout data were never used for calibration.

The pre-holdout runtime verified the Windows CRLF manifest hash `1895D3...`;
Git's LF-normalized blob at commit `fa0fc45` reconstructs as `0A000C...`.
Independent comparison found line-ending normalization only. Both hashes are
retained in the final summary; the original calibration freeze was not
rewritten.

## Frozen Holdout Results

The 11 holdout references span all five map families and dense idx 18-24; all
are 3-agent references. The ten schedules include common cardinal directions,
an off-axis fixed direction, an independent fixed direction, and common and
independent 0.25 s piecewise disturbances.

| Metric | Adaptive tube | Fixed global radius |
|---|---:|---:|
| Sample coverage | 100% | 100% |
| Trajectory coverage | 110/110 | 110/110 |
| Mean radius | 0.019807 m | 0.143929 m |
| Maximum radius | 0.064558 m | 0.143929 m |
| Discrete integrated radius | 0.099074 m s | 0.719935 m s |
| Mean/integrated reduction | 86.24% / 86.24% | — |

Mean actual error was 0.012743 m and maximum actual error was 0.057592 m;
the mean radius/error ratio was 1.554. Radius standard deviation averaged
0.005818 m and within-reference range averaged 0.043031 m, so the method is not
a renamed fixed margin. Mean instantaneous radius/acceleration correlation was
only 0.006: accumulated closed-loop response and directional sensitivity, not
instantaneous acceleration alone, dominate the variation. Mean D1 sensitivity
was 3.96 m/N.

## Audits, Stress Test, and Interface

The scheduled simulator is bit-exact with the S4 constant-force path. For dense
3-agent idx22, maximum uninflated model envelopes were 0.043436, 0.057592, and
0.072827 m for W=0, D1, and D2, with exact numerical monotonicity. D2 independently achieved
110/110 stress coverage; it does not redefine D1.

`compute_tracking_tube(...)` returns `rho_dense[agent,2501]`, conservative
`rho_support[agent,64]`, `rho_segment[agent,63]`, and metadata. A representative
20-process computation took 2.98 s (46.6 ms/support point). This is suitable as
per-trajectory preprocessing for S6, but high-throughput inner-loop use may
require caching or surrogate acceleration. Six/nine-agent evidence is limited
to calibration/validation, and arbitrary bounded disturbances remain outside a
formal guarantee reserved for S7.
