# Official SMD metric source audit

This audit freezes planning-level metric definitions for S2.  It is based on
the official source tree at commit
`c87fc76044b350a37fcea7afc468c13c8371a237` (`smd-official-import`).  It is
not a benchmark or paper-metric pipeline.

## Source inventory

| source | official blob at `c87fc76` | audited role |
|---|---|---|
| `is_collision.py` | `834d2e73359ae009eccbe24057e567a1bfcaca16` | collision feasibility |
| `deps/torch_robotics/torch_robotics/trajectory/metrics.py` | `e1dd3da7086f3d6b6e5c3ca6e602bd13a8ee5de3` | path length and discrete velocity-change metric |
| `scripts/inference/inference_multi_agent.py` | `48c67770da865969f33b8413021a8b1c610b13ca` | candidate save layout and planning-time scope |
| `smd/common/experiments/experiments.py` | `14b4e873943b2a99fbdf954c98a556adb7706fbc` | legacy result-field names and result serialization |
| `smd/common/experiments/experiment_utils.py` | `15eb759311770daa3832935d47b48996950429f1` | aggregation denominator semantics |

## Frozen definitions

### Candidate and state layout

The official collision path reads candidate zero (`paths_data[0, ...]`).  The
project contract therefore uses `PRIMARY_CANDIDATE_INDEX = 0` for every
method.  No candidate is selected by shortest length, safety, or success.

For `N` agents, the raw state dimension is `4*N`.  Within candidate zero:

```text
position = raw_paths[0, :, :2*N]       -> (N,T,2)
velocity = raw_paths[0, :, 2*N:4*N]    -> (N,T,2)
```

S1 evidence verifies `(64,64,12)`, `(64,64,24)`, and `(64,64,36)` for 3, 6,
and 9 agents respectively, with decoded position and velocity shapes
`(N,64,2)`.

### Collision

Primary boolean: `is_collision.py::check_paths_ok`.

```text
robot_radius = 0.05
threshold = 1e-3
```

The official obstacle and robot-pair checks compare squared distance against
the squared radius sum minus `threshold`.  This is not equivalent to a signed
clearance threshold in metres.  The evaluator calls the official function
extracted from the pinned source rather than maintaining a second collision
implementation.

### Path length

Source: `compute_path_length_from_pos` in
`deps/torch_robotics/torch_robotics/trajectory/metrics.py`.

For each agent:

```text
L_i = sum_t ||p_i(t+1) - p_i(t)||
```

The primary aggregate is:

```text
MEAN_PATH_LENGTH_PER_AGENT = mean_i L_i   [m]
```

`TOTAL_PATH_LENGTH = sum_i L_i` is retained as a secondary aggregate.

### Acceleration / smoothness

Source: `compute_average_acceleration_from_pos_vel` in the same official
metrics file.  It computes:

```text
delta_v(t) = v(t+1) - v(t)
a_i = mean_t ||delta_v(t)||
```

The primary aggregate is `mean_i a_i`.  The official function does not divide
by `dt`; this is therefore named and interpreted as an official discrete
velocity-change metric, not silently relabelled as physical `m/s^2`.

### Runtime

`inference_multi_agent.py` starts the primary timer immediately before
`planner.plan(...)` and stops it immediately after.  This is
`PLANNING_TIME_S`.  Launcher/configuration/serialization and wrapper overhead
are separately reported as `WALL_RUNTIME_S`; the two values are never mixed.

### Success and aggregation

`PLANNING_SUCCESS` requires a scheduled trial to produce a raw result, pass
finite checks, and pass the official collision boolean.  Its denominator is
all scheduled scientific trials, including no-output and non-finite failures.
The legacy aggregation code normalizes path length and acceleration over
successful results, while success rate is normalized over all trials.  S2
keeps these concepts separate and reports no-output, non-finite, collision,
and planning-success rates independently.

### Failure and retry

Algorithm failures remain failures.  An infrastructure failure may have at
most one retry with identical map, instance, checkpoint, parameters, and
contract.  The original attempt and retry relationship must remain recorded.

