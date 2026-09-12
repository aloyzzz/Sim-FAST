# Optical Reef Reconfiguration Simulator — System Architecture

**Target paper:** *Dynamic Modeling of Active Shape Reconfiguration in a Modular
1000-Meter In-Space Telescope* — J. Hill, D. A. Barnhart (USC SERC).

**Engines:** MBDyn (reference implicit flexible-multibody solver) and
Project Chrono / PyChrono (scalable, visual, modal-reduction solver).

This document is the top-level design. Detailed contracts live in:

| Doc | Contents |
|-----|----------|
| `01_MODEL_SCHEMA.md`      | Engine-neutral model definition (the single source of truth) |
| `02_ENGINE_ADAPTERS.md`   | How the neutral model maps onto MBDyn and Chrono primitives |
| `03_CONTROL_ARCHITECTURE.md` | Actuators, sensors, controllers, co-simulation transport |
| `04_METRICS_AND_EXPERIMENTS.md` | Metric definitions and the experiment matrix |
| `05_WORK_PACKAGES.md`     | Agent-assignable work packages, deps, acceptance criteria |
| `06_VERIFICATION.md`      | V&V plan: cross-engine, analytical, and Sim-FAST regression |

---

## 1. What the simulation has to answer

The paper's claims all reduce to one question:

> When a kilometre-scale modular assembly with a fundamental frequency in the
> tens of millihertz is *commanded to change shape*, what dynamics does the
> maneuver itself excite, and how do actuator topology, control law, and
> length scale trade against settling time, residual surface error, and
> actuator effort?

That drives four hard requirements that the architecture must satisfy up front,
because retrofitting any of them is expensive:

**R1 — Free-free, low-frequency, long-horizon.** The telescope is unconstrained
in orbit: the stiffness matrix is singular with six rigid-body modes. With
`f_1 ~ 10–50 mHz` the periods are 20–100 s and a maneuver plus settling runs
`10^3–10^4 s`. Explicit integration is hopeless; the solver must be implicit,
A-stable, with controllable numerical damping (MBDyn multistep / `ms`,
Chrono `ChTimestepperHHT`). Rigid-body drift must be *separated from shape
error in post-processing*, not suppressed by fake grounding. See §4.

**R2 — Actuation is the excitation source.** Unlike the LFSS vibration-
suppression literature, the disturbance here is *internal and commanded*. The
actuator model is therefore first-class physics, not a boundary condition: it
needs finite stiffness, stroke, rate, force saturation, and bandwidth. A
perfectly-followed prescribed motion would hide exactly the effect being
studied. See `03_CONTROL_ARCHITECTURE.md` §2.

**R3 — Length-scale sweep is a first-class axis.** The same model definition
must instantiate at 7 modules, ~10 m; ~100 modules, ~100 m; and ~9000 modules,
1000 m. That forbids hand-built decks and forces a *generator + model-order
reduction* path. See §3 and §5.

**R4 — Two engines, one model, one controller.** Cross-engine agreement is the
verification argument for a regime with no test data. The engines must consume
the *same* neutral model and drive the *same* controller object, so a
discrepancy can only be the solver. See §2 and `06_VERIFICATION.md`.

---

## 2. Layered architecture

```
              ┌───────────────────────────────────────────────────┐
 Layer 6      │ Orchestration: sweep runner, run DB, figures       │
              └───────────────────────┬───────────────────────────┘
                                      │
              ┌───────────────────────┴───────────────────────────┐
 Layer 5      │ Metrics: surface RMS, Zernike/WFE, settling time,  │
              │ modal participation, actuator effort, momentum     │
              └───────────────────────┬───────────────────────────┘
                                      │  RunResult (engine-neutral)
   ┌──────────────────────────────────┴──────────────────────────────────┐
   │                          SIMULATION CORE                            │
   │                                                                     │
   │  ┌────────────┐   measurements   ┌──────────────────────────────┐   │
   │  │  Engine    │ ───────────────► │ Layer 3: Control stack        │   │
   │  │  adapter   │                  │  sensors → estimator →        │   │
   │  │ (MBDyn |   │ ◄─────────────── │  controller → allocation →    │   │
   │  │  Chrono)   │    commands      │  actuator dynamics/limits     │   │
   │  └─────┬──────┘                  └───────────────┬──────────────┘   │
   │        │                                         │                  │
   │        │                          ┌──────────────┴──────────────┐   │
   │        │                          │ Layer 4: Reconfiguration     │   │
   │        │                          │ commander (target surface →  │   │
   │        │                          │ setpoints → shaped traj.)    │   │
   │        │                          └──────────────────────────────┘   │
   │        │                                                             │
   │  ┌─────┴───────────────────────────────────────────────────────┐    │
   │  │ Layer 2: Engine adapters — neutral model → .mbd deck /       │    │
   │  │ PyChrono system.  Plus: disturbance injection.               │    │
   │  └─────┬───────────────────────────────────────────────────────┘    │
   └────────┼─────────────────────────────────────────────────────────────┘
            │
   ┌────────┴─────────────────────────────────────────────────────────┐
 Layer 1    │ Neutral model (`ReefModel`): bodies, interfaces,         │
            │ actuator ports, sensor ports, reduction data             │
   └────────┬─────────────────────────────────────────────────────────┘
            │
   ┌────────┴─────────────────────────────────────────────────────────┐
 Layer 0    │ Geometry generator: aperture → hex tiling on target      │
            │ surface → module frames → connectivity                   │
   └──────────────────────────────────────────────────────────────────┘
```

**The invariant that makes the whole thing work:** *everything above Layer 2 is
engine-agnostic and everything below Layer 2 is engine-specific.* Layer 2 is the
only place either MBDyn or Chrono is named. Any agent writing controller,
metric, or geometry code that imports an engine has violated the architecture,
and CI enforces it (`tests/test_layering.py`, WP-11).

---

## 3. Fidelity ladder

Four model levels, each a config not a code path. This is how R3 is met and how
agents get fast feedback before the expensive cases.

| Level | Name | Modules | Bodies | Purpose | Wall-clock budget |
|-------|------|---------|--------|---------|--------------------|
| **L0** | Unit cell | 2 | 2 rigid + 1 actuator | Actuator/joint unit tests, analytic checks | < 1 s |
| **L1** | Hex ring | 7 | 7 | Regression vs. existing Sim-FAST `Hex_Ring_Sim`; controller tuning | < 1 min |
| **L2** | Sub-aperture | ~100 (≈100 m) | 100 | Full-fidelity control studies, all algorithms | < 1 h |
| **L3** | Full Reef | ~9000 (1000 m) | reduced | Headline result; scale-law extrapolation | < 24 h |

**L3 is only reachable with model-order reduction.** A 10 m flat-to-flat hex is
≈86.6 m²; a 1000 m aperture is ≈7.85e5 m², so ~9000 modules. At 6 DOF/module
that is ~54k DOF before intra-module flexibility. The plan:

1. Each module is condensed to a **Craig–Bampton superelement**: interface DOF
   at its (up to) six edge connection points and its actuator ports, plus
   `n_fix` fixed-interface normal modes (default 6, converged in WP-09).
2. MBDyn consumes this through its **modal element** (`joint: modal`) with an
   FEM data file; Chrono through **Chrono::Modal** `ChModalAssembly` in
   reduced state. Both are fed by the *same* CB matrices produced in Layer 1,
   so the reduction is not a source of cross-engine disagreement.
3. L2 must be run **both** full and reduced; the reduction is accepted only if
   the L2 reduced/full comparison passes the `06_VERIFICATION.md` §4 gate.

Agents implement L0→L1→L2→L3 strictly in order. Do not start L3 work before the
L2 reduction gate passes.

---

## 4. Handling the free-free structure (R1)

This is the single most common way a simulation like this silently produces
garbage, so the policy is fixed here and is not a per-agent choice.

- **Simulate free-free.** No ground clamp, no soft springs to inertial space,
  no "fix one module" shortcut. Six rigid-body modes at ~0 Hz are expected and
  correct. MBDyn handles the singular system with its implicit multistep
  integrator; Chrono requires `ChSolverPardisoMKL` or `ChSolverSparseQR`
  (iterative solvers will not converge on a free-free stiff system — see
  `02_ENGINE_ADAPTERS.md` §4.3).
- **Remove rigid-body motion in post-processing, not in the physics.** Every
  output frame is reduced to a body-fixed frame by a weighted Procrustes /
  Kabsch fit of module centers to their reference positions. Shape error is
  measured in that frame. `opticalreef/metrics/frames.py` owns this and is the
  *only* place the fit is implemented.
- **Internal actuation must conserve linear and angular momentum.** Net
  external force and torque are zero for every reconfiguration run (the
  disturbance cases excepted). Residual momentum drift is a standing
  correctness check, run on every simulation, not an optional diagnostic —
  see `06_VERIFICATION.md` §3.
- **Attitude control is out of scope but must not be faked.** Runs report the
  net attitude excursion the maneuver causes; a future ACS can be attached at
  the `spacecraft_bus` port reserved in the schema.

---

## 5. Module and interface modeling

Two module fidelities, selected per-run by `module.fidelity`:

- **`rigid`** — module is one rigid body; all compliance is in the inter-module
  interface. Valid when intra-module stiffness ≫ interface stiffness, which is
  the regime the existing Sim-FAST hex-ring studies (`E_panel = 70e9` vs
  `E_joint = 5e5`). Cheapest; the default for L3.
- **`flexible`** — module is a truss/frame of beam elements with its own nodes,
  supporting *intra-module* actuators (the paper's "within modules" actuation)
  and local figure control. Required for at least one L2 case so the paper can
  claim the distinction matters.

**Inter-module interfaces** are 6-DOF compliant bushings (3 translational, 3
rotational stiffness + damping), *not* ideal joints. Idealized hinges would
remove exactly the softness that sets `f_1`. A `joint_type` enum selects
`bushing` (default), `revolute_compliant` (for joint-actuated architectures),
and `rigid_weld` (for stiffness-sensitivity studies only).

Both actuator families from the paper are supported on this substrate:

- **Truss-based actuator** — a variable-rest-length strut spanning two module
  attachment points. Force law `F = k(L − L0_cmd) + c·L̇`. This is a direct
  generalization of the eigenstrain/`prestrain` actuator already used in
  `Hex_Ring_Sim`, which is why the L1 regression against the MATLAB code is
  meaningful rather than decorative.
- **Joint-based actuator** — a commanded rest-angle rotary actuator at an
  inter-module interface, `T = k_θ(θ − θ0_cmd) + c_θ·θ̇`.

Both share one saturation/bandwidth block so architecture comparisons are not
contaminated by differing actuator realism. See `03_CONTROL_ARCHITECTURE.md` §2.

---

## 6. Execution flow of one run

```
config.yaml
   │
   ├─► GeometryGenerator      → module frames, connectivity, target surfaces
   ├─► ModelBuilder           → ReefModel (+ CB reduction if requested)
   │
   ├─► ReconfigurationPlanner → actuator setpoint trajectory q_cmd(t)
   │                            (inverse shape solve + trajectory shaping)
   │
   ├─► EngineAdapter.build()  → .mbd deck  |  PyChrono ChSystem
   │
   └─► EngineAdapter.run()  ──loop──►  every control_dt:
                                        measurements = SensorStack.read(state)
                                        commands     = Controller.step(t, meas)
                                        ActuatorStack.apply(commands)
                                      every output_dt:
                                        recorder.append(state, commands)
                                 ──►  RunResult (HDF5 + manifest)
                                        │
                                        └─► MetricSuite → metrics.json → figures
```

A run is reproducible from `config.yaml` + the git SHA recorded in the
manifest. No run-specific code, ever: if a study needs a knob, the knob goes in
the schema.

---

## 7. Repository layout

```
OpticalReef_Sim/
├── docs/                      # this architecture set
├── schemas/                   # JSON Schema for configs (normative)
├── configs/                   # study configs: L0/L1/L2/L3 + sweeps
├── opticalreef/
│   ├── model/                 # Layer 1: ReefModel dataclasses, CB reduction
│   ├── geometry/              # Layer 0: tiling, target surfaces, IK
│   ├── engines/               # Layer 2: mbdyn/, chrono/, base.py (ABC)
│   ├── actuators/             # actuator models, limits, allocation map
│   ├── control/               # controllers (PID, LQR, MPC, shaping)
│   ├── sensing/               # edge sensors, metrology, noise/latency
│   ├── disturbance/           # SRP, gravity gradient, thermal, slew
│   ├── metrics/               # frames.py, surface.py, modal.py, effort.py
│   ├── orchestration/         # sweep runner, run DB, plotting
│   └── verification/          # cross-engine + analytic harnesses
├── tests/                     # pytest; unit + layering + regression
└── scripts/                   # CLI entry points
```

---

## 8. Technology decisions and their rationale

| Decision | Choice | Why |
|---|---|---|
| Neutral model format | Python dataclasses + JSON-Schema-validated YAML | Schema catches config errors before a 12 h run; dataclasses keep the builder typed |
| MBDyn control coupling | `ExtStructuralForce`, socket transport | MBDyn's supported co-sim path; keeps the controller in Python next to Chrono's |
| Chrono control coupling | In-process Python callback | PyChrono is already Python; a socket would only add latency and a failure mode |
| Actuator realization | Compliant force law with commanded rest length | Portable across both engines *and* faithful to hardware; prescribed-motion joints would mask the excitation being studied |
| Reduction | Craig–Bampton, computed once in Layer 1 | Both engines accept CB data; computing it once removes it as a cross-engine variable |
| Storage | HDF5 per run + JSON manifest + SQLite index | L3 runs produce GB of time history; JSON/CSV will not do |
| Units | SI throughout, radians, no exceptions | Documented once here; unit bugs at this scale are near-undetectable |

---

## 9. Explicit non-goals

Out of scope for this framework; do not let scope creep in:

- Optical ray-tracing / PSF synthesis. We produce surface error and Zernike
  coefficients; an optics tool consumes them.
- Orbital mechanics beyond a quasi-static disturbance environment.
- The assembly sequence itself (robotic ISAM operations). Only the *assembled*
  telescope is modeled; assembly-robot reaction loads enter as a disturbance.
- Thermal FEA. Thermal effects enter as prescribed strain/disturbance profiles.
- Attitude control system design (port reserved, see §4).
