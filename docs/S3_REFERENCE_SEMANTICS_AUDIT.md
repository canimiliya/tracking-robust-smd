# S3-R1 Projected Reference Semantics Audit

This is a baseline implementation-semantics audit, not a TR-SMD innovation
claim and not a planner modification.

## Official projection fact

The audited source is `smd/projection/projection.py` at the frozen official
import baseline. It creates:

```text
x_projected = copy(x_candidate)
```

At the end of simultaneous projection it writes, for every agent and support
point, only `j*2` and `j*2+1`: the planar `x,y` position dimensions. The
remaining state dimensions are inherited from the candidate. No velocity
recomputation occurs in this function.

Therefore:

```text
PROJECTED_POSITION = authoritative output of simultaneous projection
RAW_VELOCITY_STATE = diffusion state retained across position-only projection
RAW_VELOCITY_STATE_IS_DERIVATIVE_OF_PROJECTED_POSITION = NOT GUARANTEED
```

The raw state remains untouched and is still retained for S2 planning-level
metric parity. S3 physical execution uses only projected positions as geometry
and derives its physical velocity and acceleration from those positions. This
does not change any SMD support point.

## Nine-run evidence

The audit reuses the nine accepted S1 raw trajectories: three maps
(`instances_simple`, `instances_dense`, `instances_connected_room`) at 3, 6,
and 9 agents, candidate 0, with `dt_ref=5/64=0.078125 s`. The machine-readable
results are in
`experiments/summaries/s3_r1_velocity_semantics_audit.csv`. All nine runs show
substantial raw-velocity versus position-finite-difference mismatch; the
formal primary 3-agent run has RMSE `0.15395999853895792` and maximum absolute
error `0.3510340690612793`.

The primary raw start and goal velocities are numerically zero (maximum norm
`5.960464477539063e-08`), but S3-R1 binds the actual initial velocity to the
new reference explicitly rather than hard-coding zero.
