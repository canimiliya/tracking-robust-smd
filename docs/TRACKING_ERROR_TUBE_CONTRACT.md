# Tracking Error Tube Contract

## Scope

S5 computes a position tracking-error radius before execution for the frozen
S3/S4 `PositionConsistentHermiteReference`, geometric controller, CF2X model,
100 Hz control loop, and 500 Hz RK4 dynamics. The primary disturbance set is
per-agent `||F_ext,xy||_2 <= 0.005 N`, with zero vertical force and moment and
zero initial error. The 0.010 N set is stress-only.

## Method

The state is executed in its exact 13-state nonlinear representation with a
unit quaternion. Diagnostics use the local 12-vector
`[delta_p, delta_v, delta_theta, delta_omega]`, where `delta_theta` is the
shortest quaternion log error. No long-horizon Euclidean quaternion
linearization is used.

For each reference, a pre-registered bank of zero, eight common fixed-direction,
four independent fixed-direction, four common piecewise, and four independent
piecewise disturbances is simulated through the frozen closed loop. Their
pointwise maximum position error is the model component of `rho_i(t)`. A single
multiplicative factor and near-zero additive residual are fitted using only
calibration and validation trajectories, with a pre-registered 10% safety
inflation. This residual represents finite scenario-bank incompleteness; it is
not a dynamics-parameter correction.

The method may use reference state, controller/dynamics parameters, time, and
the disturbance bound. It must not use obstacle clearance, collision outcomes,
future disturbance realizations, or future actual tracking error.

## Outputs and Claim Boundary

`compute_tracking_tube(...)` returns dense 2 ms radii, conservative 64-point
neighborhood maxima, 63 segment maxima, and provenance/runtime metadata.
Support radii are neighborhood maxima, not point samples. S5 claims only an
empirically validated conditional tube. Formal all-disturbance safety is
explicitly reserved for S7.
