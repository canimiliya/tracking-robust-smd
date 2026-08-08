# S3-R0 Closed-Loop Dynamics Contract

This document defines the nominal, calm-air S3 bridge. It does not claim a
high-fidelity flight model or robust execution guarantee.

## Coordinate and state conventions

- World frame is ENU, with `z` up and gravity `[0, 0, -9.81] m/s^2`.
- Body thrust is along positive body `z`.
- Quaternion order is `[w, x, y, z]`, and `R(q)` maps body vectors to world.
- The state is `[p_x,p_y,p_z,v_x,v_y,v_z,q_w,q_x,q_y,q_z,omega_x,omega_y,omega_z]` (13 states).
- Every complete RK4 step normalizes the quaternion and records the norm error.

## Frozen SMD timing and lift

The accepted SMD candidate 0 is used without planner rerun or waypoint change.
Its 64 support points use `dt_ref = trajectory_duration / n_support_points =
5.0 / 64 = 0.078125 s`; 63 Hermite segments span `4.921875 s`, and the
last support point is held through the 5.0 s execution horizon. SMD `[x,y]`
is lifted to `[x,y,1.0]`, with velocity `[vx,vy,0]` and yaw zero. Piecewise
cubic Hermite interpolation uses the frozen SMD position and velocity at both
ends of each segment; acceleration is its analytic second derivative.

## Dynamics

\[
\dot p=v,\qquad m\dot v=mg+R[0,0,T]^T,
\]
\[
\dot q=\frac12q\otimes[0,\omega],\qquad
J\dot\omega=M-\omega\times(J\omega).
\]

External force and moment interfaces exist but are zero in S3. No wind or
disturbance is added.

## CF2X provenance and rotor model

Parameters come from `learnsyslab/gym-pybullet-drones`, source commit
`e712698a05a80728b06572819dcf044596707754`, file
`gym_pybullet_drones/assets/cf2x.urdf`: mass `0.027 kg`, inertia
`diag(1.4e-5,1.4e-5,2.17e-5) kg m^2`, `arm=0.0397 m`, `kf=3.16e-10`,
`km=7.94e-12`, and thrust-to-weight `2.25`. The allocation rotor positions
are `[(0.028,-0.028,0),(-0.028,-0.028,0),(-0.028,0.028,0),(0.028,0.028,0)]`
and yaw signs `[-1,+1,-1,+1]`. The planning/execution safety radius remains
the S2 value `0.05 m`; it is not replaced by a URDF geometry radius.

For rotor thrusts `f_i`, `0 <= f_i <= T_max/4`,
`u=[T,Mx,My,Mz]^T=A f`, where the rows of `A` are total thrust, the `y_i`
lever arms, the negative `x_i` lever arms, and `spin_i km/kf`. The clipped
thrust vector is reconstructed into `u_actual` before entering dynamics.

## Controller and numerical rates

The controller uses position/velocity error, analytic reference acceleration,
geometric attitude error on SO(3), and angular-rate feedback. The nominal
controller configuration is frozen at revision 3 after hover and synthetic
10 cm x/y tests; SMD smoke results may not change its gains.

Dynamics uses fixed-step RK4 at `0.002 s` (500 Hz). Control updates are at
`0.01 s` (100 Hz), with zero-order hold between updates. Euler integration,
per-frame position resets, reference-delta state updates, repulsive forces,
collision-avoidance control, and inter-agent communication are not used.

## Execution collision and known scope

Collision is checked at every 0.002 s step in XY: inter-agent distance must be
at least `2*0.05 m`; obstacle clearance is center distance minus `0.05 m` and
the obstacle radius. Altitude is not used to hide the inherited 2D collision
problem. S3 excludes motor first-order lag, aerodynamic drag, ground effect,
sensor noise, wind, and mass mismatch. These omissions are recorded rather
than presented as a complete high-fidelity model.

## Projected SMD Position vs Diffusion Velocity State

The official SMD trajectory state contains both position and velocity. In
`smd/projection/projection.py`, simultaneous projection starts from
`x_projected = copy(x_candidate)` and overwrites only each agent's `x,y`
position dimensions. It does not recompute the retained diffusion velocity
state. Therefore the raw velocity state is not guaranteed to be the derivative
of the projected position.

The SMD projected position is the authoritative physical execution geometry.
Raw SMD velocity is preserved unchanged for S2 planning-metric parity, but it
is not used as the physical derivative in S3. S3-R1 constructs
`POSITION_DERIVED_VELOCITY` from projected-position knots using central
differences at interior knots and zero at the start and goal hard-condition
knots. Cubic Hermite interpolation then uses those derived velocities and its
analytic acceleration. The SMD position support points are not moved, smoothed,
retimed, or replanned.

For `4.921875 <= t <= 5.0 s`, the final projected position is held with
`v_ref = 0` and `a_ref = 0`. This prevents a held position from being paired
with a nonzero derivative.
