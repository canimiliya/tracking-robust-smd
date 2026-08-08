# S3-R0 Root-Cause Report

This report records the S3-R0 root-cause hypothesis and the R1 remediation
evidence. It does not claim that every future execution failure is explained.

## Evidence chain

```text
R0 tracking symptom
  -> base nonlinear dynamics and frozen-controller synthetic tests passed
  -> projected-position/raw-velocity mismatch observed
  -> official projection source audit
  -> raw velocity retained while position was overwritten
  -> R0 Hermite combined incompatible state components
  -> R1 rebuilt the physical derivative from projected position only
```

The official projection source copies the candidate and overwrites only planar
position dimensions. The nine-run audit confirms this is systemic in the
accepted 3/6/9-agent, simple/dense/connected-room matrix. Raw velocity remains
valid as a retained diffusion/planning state, but is not guaranteed to be the
derivative of the projected geometry.

## R1 result

R1 kept the SMD positions, controller, dynamics, limits, timing, and source
path unchanged. It used position-derived knot velocities, zero endpoint
velocities, analytic Hermite derivatives, and an explicit terminal hold. The
corrected reference was continuous-collision-free and below the total thrust
envelope. With the frozen revision-3 controller, the unique formal 3-agent
rerun passed every frozen S3 tracking gate: mean XY RMSE
`0.004417622834878108 m`, maximum XY error `0.026932576189519285 m`, and Z
RMSE `0.007891901734266044 m`.

These results confirm the R0 reference-interface inconsistency as the cause of
the R0 gate failure for this sample. They do not turn S3 into a closed stage,
and they do not authorize S4; high-level closure review remains required.
