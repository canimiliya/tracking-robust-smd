# S4 Disturbance Contract

Frozen before disturbance outcomes on 2026-08-09.

## Mathematical definition

The actual plant receives a world-frame external force while the nominal reference and controller model remain unchanged:

`m * dv/dt = m * g + R * [0, 0, T] + F_ext`.

No external moment is used (`M_ext = [0, 0, 0]`). The force is common to every drone and constant over the full simulated trajectory, from 0.0 s through 5.0 s.

## Frozen levels

- `D0_CALM`: `F_ext = [0.000, 0.000, 0.000] N`
- `D1_FX_005`: `F_ext = [+0.005, 0.000, 0.000] N`
- `D2_FX_010`: `F_ext = [+0.010, 0.000, 0.000] N`

The positive world-x direction and both amplitudes are frozen before observing disturbance outcomes. For the 0.027 kg CF2X baseline, the nonzero levels are approximately 1.9% and 3.8% of weight and correspond to uncompensated horizontal accelerations of about 0.185 and 0.370 m/s^2. They are intended as bounded persistent loads that test safety-margin consumption without reducing controller frequency, thrust limits, or controller gains.

## Interpretation and limits

These conditions are named `horizontal external force disturbance` or `equivalent bounded wind-force disturbance`. They are not reported as physical wind speeds because no wind-speed-to-aerodynamic-force mapping is implemented.

The disturbance interface must pass an exact zero-force regression against the S3 dynamics before nonzero outcomes are accepted. If that regression fails, the disturbance campaign stops.
