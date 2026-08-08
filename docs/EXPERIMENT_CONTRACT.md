# SMD planning-level experiment contract

This contract is frozen at S2 for Official SMD, Fixed-Margin SMD, TR-SMD, and
future closed-loop consumers.  It does not define a benchmark, controller,
dynamics, wind, or paper metric.

## Shared raw-trial contract

- Use the same official maps, starts/goals, instance IDs, checkpoint, and
  algorithm parameters for a fair comparison.
- Read `paths.npy` and `map_info.pkl` without changing the raw trajectory.
- Select candidate zero only: `PRIMARY_CANDIDATE_INDEX = 0`.
- Decode state dimension `4*N` as `2*N` position values followed by `2*N`
  velocity values, giving `(N,T,2)` position and velocity arrays.
- Use `is_collision.py::check_paths_ok` with `robot_radius = 0.05` and
  `threshold = 1e-3` for the primary collision boolean.
- Preserve signed geometric clearances independently from the official
  collision boolean.  Negative signed clearance does not by itself override
  the official squared-distance tolerance semantics.

## Primary planning metrics

```text
PRIMARY_PATH_LENGTH_METRIC = MEAN_PATH_LENGTH_PER_AGENT [m]
PATH_LENGTH_PER_AGENT_i = sum_t ||p_i(t+1)-p_i(t)||
TOTAL_PATH_LENGTH = sum_i PATH_LENGTH_PER_AGENT_i

PRIMARY_ACCELERATION_METRIC = MEAN_PATH_ACCELERATION_PER_AGENT
a_i = mean_t ||v_i(t+1)-v_i(t)||
```

The acceleration metric is explicitly the official discrete velocity-change
metric.  It has no implicit division by `dt`.

Signed diagnostics are:

```text
MIN_INTER_AGENT_CENTER_DISTANCE = min_(i<j,t) ||p_i(t)-p_j(t)||
MIN_INTER_AGENT_CLEARANCE_M = MIN_INTER_AGENT_CENTER_DISTANCE - 2*0.05
MIN_OBSTACLE_CLEARANCE_M = min ||p_i-obstacle_center|| - 0.05 - obstacle_radius
```

Signed values are never clipped to zero.

## Runtime and success

```text
PLANNING_TIME_S = time around planner.plan(...)
WALL_RUNTIME_S = external launcher-to-completion time
```

```text
PLANNING_SUCCESS = scheduled trial AND raw result exists AND finite AND
                    OFFICIAL_COLLISION_FREE
```

All scheduled trials remain in denominators:

```text
PLANNING_SUCCESS_RATE = planning-success trials / scheduled trials
NO_OUTPUT_FAILURE_RATIO = no-output trials / scheduled trials
NONFINITE_FAILURE_RATIO = non-finite trials / scheduled trials
COLLISION_RATIO = valid-trajectory collision trials / scheduled trials
```

## Retry policy

Algorithm failures are retained as failures and are not retried to improve a
headline result.  A clearly independent infrastructure failure may receive
one retry with the same configuration.  Both `parent_attempt_id` and
`retry_attempt_id`, together with the failure class and reason, are retained.

## Evaluator boundary

`scripts/evaluation/baseline_metrics.py` is read-only over raw results.  It
does not smooth, interpolate, repair, replan, alter tolerance, or choose a
different candidate.  Formal S2 metrics are planning-level only; S3 dynamics,
quadrotor, wind, closed-loop, fixed-margin, and TR-SMD work remain frozen.

