# Layer 3/4 — Sensing, Actuation, Control, and Reconfiguration

## 1. Signal chain

```
 plant state ─► SensorStack ─► Estimator ─► Controller ─► Allocator
                (noise,        (optional    (law under      (map task
                 latency,       filter/      test)           space →
                 rate)          observer)                   actuators)
                                                               │
 plant ◄── ActuatorStack (dynamics, stroke/rate/force limits) ◄─┘
              ▲
              └── feedforward from ReconfigurationPlanner (Layer 4)
```

The controller under test is the *only* thing that changes between control
comparison runs. Sensors, actuator limits, allocation, and timing are held
fixed, otherwise the comparison measures the harness rather than the control
law. `orchestration/sweep.py` enforces this by construction: a control sweep
may only vary the `control:` block.

---

## 2. Actuator model

### 2.1 Force law
Truss: `F = k·(L − L0_cmd) + c·L̇`, applied equal-and-opposite along the strut.
Joint: `T = k_θ·(θ − θ0_cmd) + c_θ·θ̇` about the interface axis.

`L0_cmd` is the *commanded rest length* — the control input. This is the
eigenstrain formulation already used in `Hex_Ring_Sim` (`Sx = E(Ex − prestrain)`),
which is what makes the existing MATLAB results a valid regression target.

Never command position directly through an ideal constraint. A prescribed-motion
joint delivers unbounded force and perfectly tracks the command, which would
erase the induced-dynamics effect the paper exists to measure.

### 2.2 Limits (`ActuatorLimits`)
| field | meaning |
|---|---|
| `stroke` | `|L0_cmd − L0_ref| ≤ stroke` (m) |
| `rate` | `|dL0_cmd/dt| ≤ rate` (m/s) — usually the binding constraint |
| `force` | output force saturation (N) |
| `deadband`, `resolution` | commanded-length deadband and quantization step |
| `noise_std` | actuator force noise |

Order of application is fixed and must not be reordered: **rate limit →
stroke clamp → actuator dynamics → force saturation**. Ad-hoc reordering
changes the answer under saturation.

### 2.3 Bandwidth (`ActuatorDynamics`)
Second-order lag between commanded and realized rest length:
`L̈0 + 2ζω_n L̇0 + ω_n² L0 = ω_n² L0_cmd`, default `f_n = 1 Hz`, `ζ = 0.7`.
That is ~20–100× the structural `f_1`, which is the realistic separation and
the reason a badly-tuned controller can still destabilize the structure.

### 2.4 Topologies (the paper's actuator-architecture axis)
| id | description |
|---|---|
| `joint_only` | rotary actuators at inter-module interfaces only |
| `truss_1dof_per_edge` | one variable-length strut per shared edge |
| `truss_3dof_per_edge` | three struts per edge → full relative pose authority |
| `hybrid` | joint actuators + a sparse truss set |
| `intra_module` | actuators inside modules for local figure control |
| `intra_plus_inter` | the full Optical Reef proposal |

Each topology must report its actuator count, total installed force capability,
and mass penalty (`actuators/topology.py`), because "more actuators settle
faster" is not a result unless normalized by cost.

---

## 3. Sensing

| kind | measures | realistic spec |
|---|---|---|
| `edge` | relative displacement/rotation across an interface | 1 nm–1 µm, 1–100 Hz, the segmented-telescope workhorse |
| `metrology_range` | range from a reference structure to module corners | µm-class, sparse, low rate |
| `attitude` | bus attitude/rate | star tracker + gyro class |
| `oracle` | exact state | **baseline only** — every oracle run must be paired with a realistic-sensor run |

Edge sensors give *relative* information only: they cannot observe the global
low-order shape modes (the classic segmented-mirror observability gap). Some
absolute metrology is therefore required for focal-length reconfiguration, and
demonstrating that gap is itself a result. `sensing/observability.py` computes
the observability rank of each sensor suite against the shape modes and must be
reported for every configuration.

---

## 4. Control laws

Common interface:

```python
class ControllerABC(ABC):
    def reset(self, model: ReefModel, cfg: ControlConfig) -> None: ...
    def step(self, t: float, meas: Measurements) -> Commands: ...
    def diagnostics(self) -> dict: ...   # per-step internals for post-analysis
```

| id | description | why it's in the study |
|---|---|---|
| `open_loop` | feedforward only | isolates how much of the settling behavior is pure structural dynamics |
| `pid_decentralized` | per-actuator PID on local error | direct port of the existing Sim-FAST hex-ring controller; the scalable baseline |
| `pid_shape` | PID on global shape error through the allocator | shows the centralized/decentralized trade |
| `lqr_modal` | LQR on a modally-reduced plant, first `n_m` flexible modes | optimal baseline; exposes the spillover problem |
| `lqr_modal_filtered` | + roll-off on unmodeled modes | the standard spillover fix; the honest comparison point |
| `mpc_rate_limited` | MPC with explicit stroke/rate/force constraints | the only law that can *plan around* saturation, which at 31 m of rim travel is the binding issue |
| `passive_damped` | no actuation, structural damping only | lower bound on settling |

Implementation order: `open_loop` → `pid_decentralized` → `pid_shape` →
`lqr_modal` (+filtered) → `mpc_rate_limited`. Each must pass L1 before L2.

**Anti-windup, soft start, and saturation handling** are mandatory in every law
(the existing MATLAB `Solver_CAA_Dynamics` conditional-integration approach is
the reference behavior). A controller that winds up during a 600 s rate-limited
slew will produce a spectacular and entirely artificial instability.

---

## 5. Control allocation

For the centralized laws, the map from desired shape correction `δs` to
actuator commands `δq` uses the shape Jacobian `J = ∂s/∂q` from
`geometry/feasibility.py`:

`δq = J⁺ δs`, with `J⁺` a **damped (Tikhonov) pseudo-inverse**,
`J⁺ = Jᵀ(JJᵀ + λ²I)⁻¹`. The damping `λ` is not cosmetic: `J` is
ill-conditioned at scale (near-null directions correspond to shape changes no
actuator set can produce), and an undamped pseudo-inverse will command enormous
opposing actuator forces in the null space. Report `cond(J)` and the commanded
null-space energy for every run.

`J` is recomputed at the start and at the midpoint of a large maneuver
(configurable), since 31 m of rim travel is well outside the linear range of a
single Jacobian.

---

## 6. Reconfiguration planner (Layer 4)

`control/planner.py` turns (initial surface, target surface) into a
feedforward command trajectory:

1. **Inverse shape solve** — nonlinear least squares for the actuator vector
   `q*` minimizing target surface RMS error subject to stroke limits
   (shared with `geometry/feasibility.py`; one implementation only).
2. **Path** — interpolate `q_ref → q*` with a profile:
   `min_jerk` (default), `trapezoidal`, `bang_coast_bang`, `step` (worst case,
   for excitation studies).
3. **Input shaping** — convolve the profile with a ZV / ZVD / EI shaper tuned
   to `f_1` and `ζ_1` from the eigenanalysis. At `f_1 ≈ 20 mHz` a ZVD shaper
   adds ~50–75 s of maneuver time; whether that beats closed-loop settling is
   exactly the kind of question the framework is built to answer.
4. **Duration sweep** — maneuver duration in units of `1/f_1` (e.g. 0.5, 1, 2,
   5, 20 periods) is a primary independent variable, not a fixed choice.

The planner output is a feedforward term added to the feedback command, with
the split logged separately so "how much work did feedback actually do" is
answerable.

---

## 7. Timing and latency

`control/timing.py` centralizes: control rate (default 1 Hz — appropriate for a
20 mHz structure), sensor rate and latency, one-step actuation delay, and the
integer relationship between `dt`, `control_dt`, and `output_dt`. Both engine
adapters consume this object. Nothing else may hold a timing constant.

## 8. Disturbances (for the shape-maintenance cases)

| case | model |
|---|---|
| `none` | reconfiguration only |
| `srp` | solar radiation pressure, per-module area × cos-incidence, with a shadow-crossing step |
| `gravity_gradient` | quasi-static tidal field about the orbit normal |
| `thermal_snap` | prescribed interface rest-length step at terminator crossing |
| `slew` | bus attitude maneuver as a base-motion input |
| `isam_reaction` | assembly-robot contact force impulse at a specified module |

Disturbances are additive loads applied through `DisturbanceABC`; they do not
participate in control allocation (the controller must reject them through
feedback, which is the point).
