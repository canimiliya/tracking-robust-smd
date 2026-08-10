# S4 Execution Gap Evidence Report

Task: `S4-R0-AUTONOMOUS-EXECUTION-GAP-DISCOVERY-CAMPAIGN-R1`

Status: `SUBMITTED_FOR_REVIEW`
Final label: `S4_EXECUTION_GAP_CONFIRMED`

## Scientific answers

**Q1 — Calm baseline gap.** The broad calm campaign found one eligible execution gap: `instances_dense / 3 agents / instance 3` had +0.013144 m nominal clearance and -0.007969 m actual clearance. The prospective dense-family calm set had 0 collisions among 16 eligible trials (95% Wilson interval for the gap rate: 0.0000--0.1936), so the calm event was not statistically repeated.

**Q2 — Most dangerous nominal scenario.** Among eligible confirmation references, dense/3-agent instance 22 had the smallest nominal clearance, +0.000368 m. It remained actual-safe in calm but collided under both frozen nonzero disturbance levels.

**Q3 — Margin consumption.** At the lowest confirmed disturbance `D1_FX_005`, the maximum eligible clearance loss was 0.015736 m. The six D1 failures consumed 0.006687--0.015736 m of nominal margin and crossed strict zero clearance.

**Q4 — First reasonable disturbance producing a stable gap.** `D1_FX_005`, a common +x horizontal external force of 0.005 N applied for the complete 5 s trajectory, was the lowest pre-registered level that confirmed the gap. It is an equivalent bounded wind-force disturbance, not a claimed physical wind speed.

**Q5 — Repeatability.** In the frozen `instances_dense / 3 agents` family and prospective instance IDs 5--24, 16 trials were eligible at each level. D1 produced 6/16 collisions (gap rate 0.375; 95% Wilson interval 0.1848--0.6136). D2 produced 15/16 (0.9375; interval 0.7167--0.9889). This exceeds the pre-registered minimum of three failures.

**Q6 — Failure mechanism.** All 22 eligible execution-unsafe outcomes across discovery and confirmation were agent-obstacle collisions; there were no eligible agent-agent collisions. The mechanism is bounded tracking displacement consuming small obstacle clearance, not nominal-reference collision.

**Q7 — Reference validity.** Every execution-gap numerator and denominator entry was first required to have non-negative strict continuous reference clearance at 0.002 s and required thrust ratio at most 1. Nominal geometry failures and dynamically infeasible references were excluded and retained in the taxonomy.

**Q8 — S4 closure readiness.** The execution AI submits S4 for high-level closure review because a reproducible gap is confirmed under the frozen D1 contract. This report does not close S4, merge the branch to main, or authorize S5.

## Integrity and scope

The S3 baseline fingerprints are retained in the manifest. The S4 dynamics source differs only by the permitted external force/moment interface. With zero external force and moment, the S3 simple/3-agent raw regression passed with maximum position, velocity, and rotor-thrust differences below `3e-16`; collision status and metrics are unchanged within floating tolerance. Controller configuration, controller source, and reference source hashes are unchanged.

The campaign used 9 accepted existing planning runs and 46 new successful official SMD runs. Twenty-six infrastructure attempts without raw outputs are preserved: 6 missing-IPOPT-PATH attempts and 20 Windows path-length attempts. The successful-run soft budget was exceeded to execute the pre-registered 20-instance confirmation after the calm discovery candidate; the family, candidate index, planner, checkpoint, SMD parameters, and scientific endpoints were not changed.
