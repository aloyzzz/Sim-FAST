# Layer 2 — Engine Adapters

Layer 2 is the *only* layer allowed to import `pychrono` or emit MBDyn syntax.
Both adapters implement the same ABC (`opticalreef/engines/base.py`):

```python
class EngineAdapter(ABC):
    name: str
    def build(self, model: ReefModel, cfg: RunConfig) -> None: ...
    def attach_controller(self, ctrl: ControllerABC, rate_hz: float) -> None: ...
    def attach_disturbance(self, dist: DisturbanceABC) -> None: ...
    def run(self, recorder: Recorder) -> RunResult: ...
    def eigenanalysis(self, n_modes: int) -> ModalData: ...
```

`RunResult` is engine-neutral. If a metric needs something only one engine can
produce, the metric is wrong or the ABC needs extending — never special-case an
engine upstream.

---

## 1. Primitive mapping table

| Neutral concept | MBDyn | Chrono |
|---|---|---|
| Rigid module | `structural node: dynamic` + `body` | `ChBody` with explicit inertia tensor |
| Flexible module | `beam3` network on dynamic nodes | `ChElementBeamEuler` in a `ChMesh` |
| Reduced module | `joint: modal` + FEM data file | `ChModalAssembly` (reduced state) |
| Bushing interface | `joint: deformable displacement joint` + `deformable hinge` | `ChLinkBushing` (or `ChLinkMateGeneric` + `ChLoadBodyBodyBushingGeneric`) |
| Compliant revolute | `joint: revolute hinge` + `deformable hinge` (about axis) | `ChLinkLockRevolute` + `ChLinkRSDA` |
| Truss actuator | `rod` with driven prestrain, **or** `force: structural, follower` pair | `ChLinkTSDA` with a registered `ForceFunctor` |
| Joint actuator | `joint: deformable hinge` with driven rest angle | `ChLinkRSDA` with commanded rest angle |
| Sensor | node output / `stream output` | direct state read in the callback |
| Controller coupling | `force: external structural`, socket | in-process Python callback |
| Disturbance | `force: structural` with drive callers | `ChLoadBodyForce` / custom `ChLoad` |

---

## 2. MBDyn adapter

### 2.1 Deck generation
`engines/mbdyn/writer.py` emits a `.mbd` deck from `ReefModel` using Jinja2
templates in `engines/mbdyn/templates/`. The deck is **generated, never hand
edited** — a hand edit that is not reflected in the generator is a defect.

Deck structure:

```
begin: data; problem: initial value; end: data;
begin: initial value;
    initial time: 0.;  final time: {{ t_end }};  time step: {{ dt }};
    method: ms, 0.6, 0.6;          # spectral radius -> tunable numerical damping
    tolerance: 1e-6;  max iterations: 50;
    derivatives tolerance: 1e-4;
    linear solver: naive, colamd, mt, 1;   # or umfpack for L2+
    output: iterations;
end: initial value;
begin: control data;
    structural nodes: {{ n_nodes }};
    rigid bodies:     {{ n_bodies }};
    joints:           {{ n_joints }};
    forces:           {{ n_forces }};
    default orientation: orientation matrix;
end: control data;
... nodes / bodies / joints / forces ...
```

**`method: ms, rho, rho`** — the asymptotic spectral radius `rho` sets numerical
dissipation. Set `rho = 0.6` for production and **`rho = 1.0` (no dissipation)
for any run whose result is a modal-participation or energy metric**; otherwise
the integrator itself damps the low-frequency modes the paper is trying to
measure. This is a recurring source of wrong answers; the config schema
requires `solver.rho` to be explicit for MBDyn and the metric layer refuses to
compute modal energy from a `rho < 0.95` run.

### 2.2 Truss actuator in MBDyn — two realizations
- **Preferred (`rod_prestrain`):** a `rod` element whose constitutive law is
  `linear viscoelastic` with a *driven* prestrain fed from the external
  controller stream. Physically identical to the Sim-FAST `prestrain` actuator,
  which makes the L1 regression (`06_VERIFICATION.md` §2) a true apples-to-apples
  comparison.
- **Fallback (`ext_force_pair`):** the controller returns an equal-and-opposite
  follower force pair on the two attachment nodes, computed in Python from
  `F = k(L − L0_cmd) + c·L̇`. Simpler to get right, and guarantees momentum
  conservation by construction, at the cost of moving the actuator stiffness
  outside the Jacobian (which slows Newton convergence on stiff cases).

Implement `ext_force_pair` first (it unblocks the whole control stack), then
`rod_prestrain`, and verify they agree on L0 to 1e-8 before using either at
scale.

### 2.3 Control coupling
Use `force: external structural` with socket communication:

```
force: {{ id }}, external structural,
    socket, create, yes, path, "{{ sock }}",
    coupling, tight,
    {{ n_ext_nodes }}, {{ node_list }},
    orientation, orientation matrix,
    accelerations, yes;
```

`coupling, tight` iterates the controller inside the Newton loop — correct, but
only meaningful if the controller is a pure function of state. Our controllers
are discrete-time with internal integrator state, so **use `coupling, loose`
(explicit) with `control_dt` an integer multiple of `dt`**, and document the
resulting one-step delay as part of the modeled control latency. Do not use
`tight` with a stateful discrete controller: it re-enters the integrator term
several times per step and silently corrupts it.

The Python side is `engines/mbdyn/extforce.py`, implementing MBDyn's external
structural binary protocol (node kinematics out, forces/moments in). Its wire
format is pinned by `tests/test_mbdyn_protocol.py` against a recorded fixture so
an MBDyn version bump is caught immediately rather than showing up as physics.

### 2.4 Eigenanalysis
MBDyn's `eigenanalysis` block at `t=0` (and at the target configuration) gives
free-free modes. Expect **six eigenvalues at ~0**; the adapter must identify and
strip them by threshold (`|f| < 1e-4 Hz`) and *assert that exactly six are
found*. Not six means the model is over- or under-constrained — fail loudly,
do not proceed.

---

## 3. Chrono adapter

### 3.1 Build
`engines/chrono/builder.py` constructs a `ChSystemNSC`? **No — use
`ChSystemSMC`.** NSC's complementarity solver is for contact-dominated problems;
this is a smooth, stiff, constraint-and-force problem. Modules are `ChBody`
with explicit inertia; interfaces are bushings; actuators are `ChLinkTSDA`
with a `ForceFunctor` that reads the commanded rest length from the shared
`ActuatorStack`.

### 3.2 Integrator and solver
```python
sys.SetSolver(chrono.ChSolverPardisoMKL())      # fallback: ChSolverSparseQR
ts = chrono.ChTimestepperHHT(sys)
ts.SetAlpha(-0.2)          # numerical damping; use -0.05..0 for modal runs
ts.SetMaxIters(50); ts.SetAbsTolerances(1e-6)
ts.SetModifiedNewton(False)
sys.SetTimestepper(ts)
```
Iterative solvers (`ChSolverPSOR`, `BARZILAIBORWEIN`, `MINRES`) **will not
converge** on a free-floating structure with 1e7 N/m actuators next to 1e5
N·m/rad bushings. A direct sparse solver is mandatory. The same `rho`/`alpha`
discipline as MBDyn §2.1 applies: HHT `alpha` must be near zero for modal and
energy metrics.

### 3.3 Control coupling
PyChrono is in-process, so the controller is called from a stepping loop in
`engines/chrono/runner.py` — no socket, no serialization:

```python
while sys.GetChTime() < t_end:
    if due(control_dt):
        cmds = controller.step(t, sensors.read(state_view(sys)))
        actuators.apply(cmds)
    sys.DoStepDynamics(dt)
```
To keep the two engines comparable the Chrono loop must apply the **same
one-step command delay** that MBDyn's loose coupling imposes. `ControlTiming`
in `control/timing.py` owns this so it cannot drift between adapters.

### 3.4 Reduction
`ChModalAssembly` in reduced mode consumes the Craig–Bampton basis from Layer 1.
Chrono is the **primary L3 engine** because its modal substructuring scales
better and it gives the visualization the paper needs; MBDyn is the reference
that validates it at L1/L2.

---

## 4. Division of labour between the engines

| | MBDyn | Chrono |
|---|---|---|
| Primary role | Reference / truth | Scale and visualization |
| Levels | L0, L1, L2 (full + reduced) | L0, L1, L2, **L3** |
| Strengths used | Robust implicit multistep, mature flexible multibody, controllable spectral radius | Modal substructuring, Python-native control, VSG/Irrlicht rendering, parallelism |
| Must-run cases | Every case that appears in a verification table | Every case that appears in a results figure |

Every headline result must be produced by Chrono **and** spot-checked by MBDyn
at L2. A result with no cross-engine check does not go in the paper.

---

## 5. Adapter acceptance criteria

An adapter is done when:
1. `build()` on L0/L1/L2 configs produces a system whose mass and COM match the
   `ReefModel` to 1e-10 relative.
2. `eigenanalysis()` returns exactly 6 rigid-body modes below 1e-4 Hz.
3. A zero-command run over 1000 s shows momentum drift below the
   `06_VERIFICATION.md` §3 threshold.
4. The L0 actuator step response matches the analytic 2-body/1-spring solution
   to 1e-6.
5. Cross-engine L1 `f_1..f_10` agree within 1 %.
