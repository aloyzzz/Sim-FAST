# PID Control of the Hex-Ring Actuators & Solar-Panel Joint Modeling

This note documents two things requested for the hex-ring simulations:

1. **A PID control scheme** that drives the joint actuators to *counteract
   external forces* and hold the ring near its reference shape.
2. **A 3-D joint study** that models the hexagon-to-hexagon connections as the
   compliant *bar joints* they are (not idealized hinges) and shows the effect
   of an external force on those joints in full 3-D.

---

## 1. PID control of the joint actuators

### 1.1 What the actuators physically are

Every joint in the hex-ring sims is a **bar element** whose stress is

```
Sx = E * (Ex - prestrain)          % Vec_Elements_Bars/Solve_Stress.m
```

`prestrain` is an eigen-strain (a commanded change of rest length). The
existing scripts already use it in *open loop*: `Hex_Center_Ring.m` and
`Three_Hex_YJoint.m` play a pre-baked rest-length trajectory `L0_his` through
`Solver_CAA_Dynamics.actuation`. That is feed-forward shape morphing — it has
no idea what the structure is actually doing, so it cannot react to a
disturbance.

To *counteract external forces* the actuator command must depend on the
measured state, i.e. it must be **closed-loop**.

### 1.2 Control law

Each joint bar gets its own decentralised PID loop (no central solver, no
control-allocation matrix — it scales to any of the hex sims unchanged):

```
measurement   y_k(t) = Ex_k(t)          strain of joint bar k
setpoint      r_k    = 0                 joint at its natural length
error         e_k    = r_k - y_k
command       prestrain_k = Kp*e_k + Ki*∫e_k dt + Kd*de_k/dt
```

**Why the sign works out as negative feedback.** A disturbance that racks
the ring stretches a joint, `Ex_k > 0`, so `e_k < 0` and the commanded
`prestrain_k < 0`. A negative prestrain raises the bar tension
(`Sx = E*(Ex - prestrain)` grows), which pulls the two panels back together
and reduces `Ex_k`. The loop therefore actively opposes the disturbance.

* The **proportional** term gives an immediate stiffening reaction.
* The **integral** term removes the residual offset that a *sustained*
  external force would otherwise leave (zero steady-state strain error).
* The **derivative** term adds damping so the active ring does not ring up.

This is equivalent to making each compliant joint behave like a much stiffer,
*actively regulated* joint — exactly the behaviour you want for disturbance
rejection / vibration suppression of a deployed panel array.

### 1.3 Where it lives in the code

The controller is integrated directly into the dynamic solver so it sees the
true state at every step:

* `00_SourceCode_Solver/@Solver_CAA_Dynamics/Solver_CAA_Dynamics.m`
  — new `control` property (struct: `bar_ids, Kp, Ki, Kd, target_strain,
  prestrain_limit, t_on`).
* `00_SourceCode_Solver/@Solver_CAA_Dynamics/Solve.m`
  — at each step it measures the actuated-bar strains
  (`bar.Solve_Strain`), evaluates the discrete PID law, applies the
  command through `bar.prestrain_vec`, and logs the histories. It also
  returns an optional second output `ctrlLog`.

Robustness details that matter for a real actuator model:

* **Stroke saturation** — `prestrain_limit` clamps `|prestrain|` to a finite
  actuator stroke.
* **Anti-windup** — conditional integration stops the integral term winding
  up while the command is saturated.
* **Soft start** — `t_on` lets the controller switch on after the disturbance
  so the before/after effect is visible.

The change is fully backward compatible: leave `control` empty and the solver
behaves exactly as before; the open-loop `actuation` path is untouched.

### 1.4 Demo

`Hex_Ring_PID_Control.m` runs the *same* sustained 80 N disturbance on the
6-panel A-ring twice — controller **off** (passive) and **on** (active PID) —
and produces:

* `hex_ring_pid_control.mp4` — side-by-side animation (joint bars on the
  active side are coloured by command: red = contract, blue = extend),
* a time-history figure of panel-1 deflection (passive vs. controlled) and
  the six actuator commands, plus a steady-state deflection-reduction summary
  printed to the console.

> Gains in the demo (`Kp=3, Ki=80, Kd=0.05`, 5 % stroke) are illustrative.
> Because the actuator force authority is `E_joint·A·prestrain`, the joint
> modulus must be stiff enough (here `E_joint = 1e8` Pa) for the actuators to
> out-muscle the disturbance; with the very soft joints used in
> `Hex_Ring_Dynamic.m` (`5e5` Pa) you would raise `E_joint`, the area, or the
> stroke limit, then re-tune.

### 1.5 Variations the same hook supports

* **Position/shape feedback instead of strain** — set `target_strain` per bar
  from a desired-shape map, or (a small extension) feed measured nodal
  displacements as the error to hold a *specific panel pose*.
* **Pure feed-forward + feedback** — keep `actuation` (open-loop morph) for the
  commanded motion and add `control` for disturbance rejection about it.
* The identical `control` struct works for `Three_Hex_YJoint.m` and
  `Hex_Center_Ring.m` because they already expose joint-bar indices
  (`act_ids`).

---

## 2. Modeling the joints in 3-D (NOT hinged) and the effect of a force on them

The array is **not** treated as a hinged/folding mechanism. The panels stay
panels and the connections between them are modelled as the **compliant bar
joints** they already are in the hex sims — we simply want to (a) let the
structure move in full **3-D** and (b) see what an external force does to
those joints (how far each joint stretches, how much axial force it carries).

### 2.1 What had to change to make it 3-D

The original hex scripts are *planar by constraint*, not by physics — every
node has its out-of-plane DOF removed:

```matlab
supp(:,3) = 1;     % Y fixed for ALL nodes  (forces the ring to stay flat)
```

The element library is already fully 3-D (bars use 3-D Green strain;
`Solve_Strain`/`FindMassMat` are 3-D; supports are per-axis). So the only
thing standing between the existing model and a 3-D study is that one line.
Removing the global `Y` constraint — and clamping just one panel fully
(X, Y, Z) to ground the structure — lets the joints respond out of plane
under a 3-D load, with **no solver or element changes required**.

The bar joints carry the out-of-plane load through **geometric (tension)
stiffening**: a flat connector loaded transversely develops axial tension as
it deflects, and that tension provides the restoring force — the natural,
physically-correct behaviour of a thin connector (think guy-wire / membrane),
not an artificial rotational hinge. The constant-average-acceleration solver
is unconditionally stable and the lumped-mass term regularises the otherwise
slack initial out-of-plane direction, so the dynamic 3-D run is well posed.

### 2.2 Demo: `Hex_Ring_3D_Joints.m`

Built on the 1-center + 6-ring geometry (12 explicit bar joints — the truest
"joint" representation, since panels do **not** share nodes):

* leaves the out-of-plane DOF free (3-D),
* clamps ring panel 4 only,
* applies an **out-of-plane (+Y) impulse** to ring panel 1 for `t_force`, then
  releases it,
* produces a **3-D animation** (`hex_ring_3d_joints.mp4`) with the panels as
  3-D patches and **joint bars coloured by axial force** (blue = compression,
  red = tension), and
* reports, per joint and over time, the **elongation ΔL** and **axial force**
  (computed with the same `Solve_Strain` the solver uses, so the diagnostics
  are model-consistent), plus the loaded panel's out-of-plane deflection.

Switch `joint_type` between `'triangle'` (one bar per gap) and `'Y'` (hub +
two prong bars) to compare joint topologies. Set `use_control = true` to also
engage the §1 joint PID and watch the active joints reject the same 3-D
disturbance — i.e. the controller from Part 1 works unchanged in 3-D, since it
acts on bar strain which is already a 3-D quantity.

### 2.3 If you ever *do* want true hinge behaviour (optional aside)

Should the goal later shift to panels that genuinely **fold** about their
edges, Sim-FAST already has the right primitive — the **4-node rotational
spring** (`@Vec_Elements_RotSprings_4N`), whose energy depends on the dihedral
fold angle and whose rest angle `theta_stress_free_vec` is directly actuated
by the dynamic solver's `rotSprTargetAngle` input. The
`MicroMirror.m` example and the `Assembly_Origami`/`Assembly_MEMS` pattern
(whose `Solve_FK` simply sums `Tbar + Trs`) are ready templates. That is a
*different* model from the bar-joint study above and is noted only for
completeness.
