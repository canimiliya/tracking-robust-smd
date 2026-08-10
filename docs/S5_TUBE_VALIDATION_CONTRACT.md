# S5 Tube Validation Contract

## Frozen Baseline and Primary Endpoint

The baseline is commit `20707c51f1b25652a1dae7bc57d368589cd346d7` with
the approved S4 force/moment interface. Controller gains, CF2X parameters,
integrator, SMD timing/support points, and reference generator are immutable.
The primary endpoint is zero observed D1 holdout violations, where a violation
is `||p_actual-p_ref||_2 > rho_i(t) + 1e-9 m`. Every agent and every 2 ms sample
counts. Whole-trajectory coverage requires every sample of every agent covered.

## Frozen Whole-Reference Split

The exact paths, source hashes, trial IDs, seeds, and disturbance-schedule
hashes are frozen in `experiments/manifests/s5_tube_manifest.json` before
holdout execution. The split rule is:

- Holdout: dense/3-agent idx 18-24, plus simple, connected_room, empty, and
  shelf 3-agent idx 4 (11 references).
- Validation: dense/3-agent idx 9-17; all five map families at 3-agent idx 3;
  empty and shelf 6/9-agent idx 0 (18 references).
- Calibration: the remaining 26 references from the frozen 55-reference S4
  catalog.

No timestep or trajectory crosses splits. Calibration and validation each use
eight disjoint fixed/random/piecewise schedules. Holdout uses ten untouched
common and independent schedules, including `+X`, `-X`, `+Y`, and `-Y` and
0.25 s piecewise switching. All obey the D1 norm bound.

## Frozen Comparators and Decision Rule

The fair fixed baseline is the smallest global radius covering all calibration
and validation observations, followed by the same pre-registered 10% safety
factor. The adaptive method is scientifically meaningful only if frozen
holdout coverage is no worse and mean or integrated radius is reduced by at
least 10%. The post-hoc empirical oracle is diagnostic and non-deployable.

Any primary holdout violation blocks this revision; holdout data cannot be
reused for recalibration. D2 failure does not redefine the D1 tube. S4 D1
collision cases are audited for tube coverage, not replanned or repaired.
