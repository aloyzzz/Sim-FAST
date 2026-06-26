# PID Control of the Hex-Ring Actuators & Solar-Panel Joint Modeling

This note documents two things requested for the hex-ring simulations:

1. **A PID control scheme** that drives the joint actuators to *counteract
   external forces* and hold the ring near its reference shape.
2. **An exploration** of whether a different Sim-FAST element/assembly is a
   better fit when the goal is to model a solar-panel array of hexagons
   connected by *joints* (hinges).

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

## 2. Is a bar mesh the best model for a *hinged* solar-panel array?

The hex-ring scripts model each panel as a **stiff triangulated bar mesh** and
each joint as a **soft bar**. That is a fine, fast way to get an in-plane
*compliant-membrane* response, but it has real limitations for a hinged
solar array:

* A bar only carries **axial** force. A physical panel-to-panel hinge is a
  **rotational** connection — it should be soft in *bending/folding* about the
  hinge line but stiff in-plane. A single soft bar cannot represent that
  anisotropy; it is soft along one direction only and says nothing about
  out-of-plane folding.
* The whole hex-ring setup is constrained to a plane (`Y` fixed for all
  nodes). The defining motion of a deployable solar array — panels **folding
  out of plane** about their hinges — cannot happen in that model at all.
* "Stiffness" of a bar joint is an axial modulus, not a hinge stiffness or a
  fold angle, so it does not map cleanly onto a real hinge spec or onto a
  *deployment angle* you would want to actuate.

### 2.1 The better-fit Sim-FAST primitives

Sim-FAST already ships the elements that model hinges directly:

| Need | Element | File |
|------|---------|------|
| Panel as a stiff, true 2-D plate | CST triangle | `@Vec_Elements_CST`, `@CD_Elements_CST` |
| Hinge / fold line between panels | 4-node rotational spring | `@Vec_Elements_RotSprings_4N` |
| Hinge with a *commanded* fold angle | 4N rot-spring `theta_stress_free_vec` | same |
| One-sided (thick) hinge, contact-aware | directional 4N rot-spring | `@Vec_Elements_RotSprings_4N_Directional` |

A **4-node rotational spring** is precisely a hinge: its energy depends on the
dihedral fold angle `theta` between the two panels sharing an edge, with
stiffness `rot_spr_K_vec` and rest angle `theta_stress_free_vec`. That gives
you exactly the *soft-in-folding / stiff-in-plane* behaviour a panel hinge
needs, and it lets the array fold in 3-D.

Crucially, **actuation maps onto the natural variable**: the dynamic solver
already actuates `theta_stress_free_vec` through its `rotSprTargetAngle`
input (`Solver_CAA_Dynamics/Solve.m`). So "drive the hinge to a deployment
angle" is native, and the *same PID idea above can command the rest angle*
instead of a bar prestrain — a hinge-angle PID for deployment hold and
vibration suppression.

This is not hypothetical in this repo:

* **`02_Other_Simulation_Examples/MicroMirror.m`** is the canonical analog of
  a panel on a torsional hinge: stiff plates + actuated rotational springs +
  the *same* `Solver_CAA_Dynamics`. It even folds initial angles and drives
  the hinges dynamically.
* **`@Assembly_MEMS` / `@Assembly_Origami`** show the assembly pattern: their
  `Solve_FK` simply **sums** bar and rotational-spring contributions
  (`T = Tbar + Trs; K = Kbar + Krs`). The hex `Assembly_Hex_Origami` only
  sums bars — adding a `rot_spr_4N` member and the extra two lines is all that
  is required to upgrade it.

### 2.2 Recommendation

* **Keep the bar-mesh hex-ring model** when the question is *in-plane*
  compliant-joint mechanics, planar vibration, or quick actuator-strain
  studies. It is the lightest model and the PID controller added here works on
  it directly.
* **Switch to CST panels + 4-node rotational-spring hinges** (the
  MicroMirror / Origami assembly pattern) when the panels should be genuine
  plates and the joints are real **hinges** that fold out of plane — i.e. the
  actual deployable-solar-array problem. Then:
  - model each panel with a few CST triangles (stiff in-plane),
  - put a 4N rotational spring on every panel-to-panel edge (soft fold,
    realistic hinge stiffness),
  - actuate / regulate the hinge via `theta_stress_free_vec` (open loop with
    `rotSprTargetAngle`, or closed loop with the PID approach of §1 applied to
    the rest angle), and
  - drop the global `Y`-fixed constraint so deployment and out-of-plane modes
    appear.

In short: the PID hook is general (it can command either bar prestrain or
hinge rest angle), and for a *hinged* solar-panel array the rotational-spring
hinge model is the better-suited Sim-FAST option, with `MicroMirror.m` and the
`Assembly_Origami`/`Assembly_MEMS` pattern as ready templates.
