# S4 Gap Statistical Contract

Frozen before confirmation outcomes on 2026-08-09.

## Candidate family

- Map: `instances_dense`
- Agent count: 3
- Disturbance: `D0_CALM` (no external force, moment, or parameter mismatch)
- Planner output: official SMD candidate 0 with the frozen checkpoint and parameters
- Discovery evidence: instance 3 was nominally strict-safe and thrust-feasible but had an actual agent-obstacle collision. Discovery instances 0--4 are excluded from the prospective confirmation estimate.

## Scheduled confirmation

The scheduled official instance IDs are exactly `5, 6, ..., 24` (20 trials). Every scheduled ID remains in the manifest even if planning or execution fails. No replacement instance may be selected based on outcome.

An eligible trial requires official planning success, non-negative strict continuous nominal clearance at 0.002 s, and maximum required total-thrust ratio at most 1. Nominal geometry failures, dynamically infeasible references, nonfinite execution, and infrastructure failures are reported but excluded from the execution-gap denominator.

## Endpoints and analysis

The primary endpoint is the proportion of eligible trials with strict actual-trajectory collision. The report will give the eligible denominator, execution failures, execution success rate, execution-gap rate, and a two-sided 95% Wilson score interval for the gap rate. Collision type is recorded as agent-agent and/or agent-obstacle.

Secondary descriptive endpoints are nominal and actual minimum clearance, clearance loss, tracking position RMSE, maximum tracking error, reference thrust ratio/tilt, actual tilt, and rotor saturation.

The target is 20 scheduled trials. Confirmation meets the task card's minimum evidence threshold only if at least three eligible execution-unsafe cases are observed. A smaller count remains candidate-level evidence and is not promoted by changing the family, instances, endpoint, or disturbance.

## Pre-registered disturbance extension

The prospective calm set produced zero failures among 16 eligible trials, so the same frozen map/agent/instance family is extended to the two nonzero levels in `S4_DISTURBANCE_CONTRACT.md`. All IDs 5--24 are scheduled at both `D1_FX_005` and `D2_FX_010`; nominal eligibility is unchanged and every scheduled ID remains in the taxonomy. Gap rate and its 95% Wilson interval are reported separately at each level.

The primary disturbance conclusion uses the lowest pre-registered level with at least three eligible execution collisions. Both levels are run regardless of the D1 result; amplitudes, direction, timing, family, and endpoint are not modified after outcomes.
