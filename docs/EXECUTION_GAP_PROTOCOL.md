# S4 Execution Gap Protocol

Task: `S4-R0-AUTONOMOUS-EXECUTION-GAP-DISCOVERY-CAMPAIGN-R1`

## Frozen baseline

The campaign starts at S3 commit `65fb9fc87ab0df44b2061709c41cc354ddb1d2fb` and uses candidate 0, the position-consistent Hermite reference, CF2X parameters, controller revision 3, RK4 at 0.002 s, control updates at 0.01 s, and a 0.05 m robot radius. The manifest records SHA-256 fingerprints for the controller configuration, controller source, dynamics source, and reference source.

## Trial eligibility and taxonomy

An official SMD result is gap-eligible only when planning succeeded, the corrected continuous reference is strictly collision-free at 0.002 s, and its maximum required total-thrust ratio is at most 1. An eligible execution is unsafe only if its finite, propagated actual trajectory has negative strict XY clearance.

Every scheduled trial is assigned exactly one class: `ELIGIBLE_EXECUTION_SAFE`, `ELIGIBLE_EXECUTION_UNSAFE`, `NOMINAL_STRICT_GEOMETRY_FAILURE`, `REFERENCE_DYNAMIC_INFEASIBLE`, `NONFINITE_EXECUTION`, or `INFRASTRUCTURE_FAILURE`. Nominally unsafe and dynamically infeasible trials are excluded from the execution-gap denominator.

## Geometry and dynamics

Inter-agent clearance is `||p_i-p_j|| - 0.10 m`. Agent-obstacle clearance is `||p_i-c_o|| - 0.05 m - r_o`. Nominal and actual trajectories use the same 0.002 s samples and map geometry. Dynamic diagnostics include maximum speed, maximum and p99 acceleration, required thrust ratio, and required tilt.

## Progressive campaign

Phase A reuses the accepted S1/S2 3-map by 3/6/9-agent matrix without replanning. Candidates are ranked from objective predeclared quantities: eligibility, actual minimum clearance, and clearance loss. If calm evidence is insufficient, official candidate-0 trajectories from existing maps and instances may be screened by nominal clearance and dynamic feasibility before closed-loop execution. Disturbance outcomes require a frozen disturbance contract; formal multi-instance confirmation requires a frozen statistical contract.

Tracking RMSE is diagnostic. The primary endpoint is strict collision of the actual trajectory after a strictly safe, dynamically feasible nominal reference.
