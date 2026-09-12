# Work Packages — Agent Assignment Plan

Each WP is sized for one agent working independently. Every WP states its
**inputs**, **deliverables**, **acceptance criteria** (mechanically checkable),
and **dependencies**. An agent may not begin a WP until its dependencies'
acceptance criteria pass in CI.

## Global rules for every agent

1. **Layering is enforced.** Only `opticalreef/engines/**` may import
   `pychrono` or emit MBDyn syntax. `tests/test_layering.py` fails the build
   otherwise.
2. **No run-specific code.** A new study is a new config, never a new script.
   If a study needs a knob, add it to `schemas/reef_config.schema.json` first.
3. **SI units, radians.** No exceptions.
4. **Every WP ships tests.** A WP with no failing-then-passing test is not done.
5. **Don't fake the physics to make a test pass.** If an acceptance criterion
   cannot be met, stop and report *why* — a failed criterion is information
   (see WP-09 / `06_VERIFICATION.md` §4), not an obstacle to route around.
6. **Record provenance.** Every artifact carries the git SHA and resolved config.

---

## Phase 0 — Foundation (must finish before anything else)

### WP-00 · Environment and build
**Deps:** none.
**Inputs:** none.
**Deliverables:** `pyproject.toml`; pinned environment for numpy/scipy/pyyaml/
jsonschema/h5py/matplotlib/jinja2/pytest; MBDyn build or container (with the
`external structural` module enabled); PyChrono install with
`ChSolverPardisoMKL` and the modal module; `scripts/doctor.py` that reports
what is present; CI workflow running `pytest`.
**Acceptance:** `python scripts/doctor.py` reports both engines available and
their versions; `pytest tests/` collects and runs; CI green on a trivial test.
**Note:** neither engine nor numpy is present in the current container — this WP
is real work and blocks everything, so it is staffed first and alone.

### WP-01 · Neutral model and schema
**Deps:** WP-00.
**Deliverables:** `opticalreef/model/` — frozen dataclasses per
`01_MODEL_SCHEMA.md` §3; `schemas/reef_config.schema.json` (normative);
`model/io.py` HDF5 round-trip; `model/validate.py`.
**Acceptance:** schema rejects every fixture in `tests/fixtures/bad_configs/`
with a specific message; `ReefModel` round-trips byte-identically through HDF5
for L0/L1 fixtures; all dataclasses are frozen (test asserts it).

### WP-02 · Geometry generator and target surfaces
**Deps:** WP-01.
**Deliverables:** `geometry/surfaces.py` (all 5 surface kinds + normals),
`geometry/tiling.py` (hex tiling, projection, module frames, connectivity,
**gap field**), `geometry/frames.py`.
**Acceptance:** module count matches the closed-form tiling count for
D ∈ {10, 100, 1000} m; every module center lies on the target surface to 1e-9 m;
module normals match the analytic surface normal to 1e-9; the reported gap
growth for the f/1→f/2 case matches a hand-computed rim value; L3 tiling
generates in < 60 s.

---

## Phase 1 — Physics (parallel after Phase 0)

### WP-03 · MBDyn adapter
**Deps:** WP-01, WP-02.
**Deliverables:** `engines/mbdyn/` — Jinja2 deck templates, `writer.py`,
`runner.py` (subprocess + output parsing), `extforce.py` (socket protocol),
`eigen.py`.
**Acceptance:** `02_ENGINE_ADAPTERS.md` §5 criteria 1–4; `extforce.py` passes
the pinned wire-format fixture test; L1 deck runs to completion in < 60 s.

### WP-04 · Chrono adapter
**Deps:** WP-01, WP-02. *Parallel with WP-03.*
**Deliverables:** `engines/chrono/` — `builder.py`, `runner.py`, `eigen.py`,
`visual.py` (VSG/Irrlicht + offline frame export).
**Acceptance:** `02_ENGINE_ADAPTERS.md` §5 criteria 1–4; solver is a direct
sparse solver (asserted in a test); control delay matches `ControlTiming`.

### WP-05 · Actuator and sensor stacks
**Deps:** WP-01. *Parallel with WP-03/04.*
**Deliverables:** `actuators/` — force laws, limits (fixed application order),
2nd-order dynamics, all 6 topologies, `topology.py` cost accounting;
`sensing/` — 4 sensor kinds with noise/latency/rate/quantization,
`observability.py`.
**Acceptance:** limit-order unit tests (rate→stroke→dynamics→saturation, with a
test that a reordering changes the result and is therefore caught); actuator
step response matches the analytic 2nd-order solution to 1e-8; momentum-neutral
force pair asserted for every truss actuator; `observability.py` correctly
reports the rank deficiency of an `edge_only` suite on a known small case.

### WP-06 · Disturbance models
**Deps:** WP-01. *Parallel.*
**Deliverables:** `disturbance/` — all 6 cases from
`03_CONTROL_ARCHITECTURE.md` §8 behind `DisturbanceABC`.
**Acceptance:** SRP total force on a flat 1000 m aperture matches the analytic
`P_srp·A` to 1 %; gravity-gradient torque matches the closed form for a dumbbell
to 1 %; each case is reproducible from its seed.

---

## Phase 2 — Control (after Phase 1)

### WP-07 · Control framework, planner, allocation
**Deps:** WP-02, WP-05, and one of WP-03/WP-04.
**Deliverables:** `control/base.py` (`ControllerABC`), `control/timing.py`,
`control/planner.py` (inverse shape solve, 4 profiles, ZV/ZVD/EI shaping),
`control/allocation.py` (damped pseudo-inverse, conditioning report),
`geometry/feasibility.py` (shared inverse solve + Jacobian).
**Acceptance:** inverse shape solve on L1 reaches the kinematic floor; ZVD
shaper applied to an SDOF at `f_1` leaves residual vibration < 1 % of the
unshaped case; allocation reproduces a known `J⁺` on a small analytic case;
`cond(J)` reported for L1/L2.

### WP-08 · Controllers
**Deps:** WP-07.
**Deliverables:** all 7 laws from `03_CONTROL_ARCHITECTURE.md` §4, each with
anti-windup, soft start, and saturation handling, in the stated order.
**Acceptance:** each law reaches the L1 target shape within its spec band;
`pid_decentralized` reproduces the `Hex_Ring_PID_Control.m` deflection
reduction within 10 % (WP-10); `lqr_modal` without filtering is *shown* to
exhibit spillover and `lqr_modal_filtered` is shown to fix it — the failure
case is a deliverable, not a bug; `mpc_rate_limited` respects every constraint
(asserted per step).

---

## Phase 3 — Scale, metrics, results

### WP-09 · Craig–Bampton reduction and L3
**Deps:** WP-03, WP-04.
**Deliverables:** `model/reduction.py` (CB matrices, engine-neutral), MBDyn
modal-element export, Chrono `ChModalAssembly` path, L3 config and memory/time
profiling.
**Acceptance:** the full `06_VERIFICATION.md` §4 gate passes at L2; `n_fix`
convergence reported; L3 builds and runs a 3000 s simulation within the 24 h
budget. **If the gate fails, report it as a finding — do not proceed to L3.**

### WP-10 · Metrics suite
**Deps:** WP-03 or WP-04 (needs a `RunResult`). *Parallel with WP-08.*
**Deliverables:** `metrics/` — `frames.py` (Procrustes, both fit variants),
`surface.py`, `zernike.py` (Noll, n≤10), `settling.py` (both bands),
`modal.py` (participation + PSD + spectrogram, **with the numerical-damping
guard**), `deformation.py`, `effort.py`, `conservation.py`.
**Acceptance:** Procrustes recovers a known applied rigid transform to 1e-12;
Zernike fit recovers synthetic coefficients to 1e-10; settling time correct on
a synthetic decaying sinusoid; modal-participation metric *refuses* a run with
`rho < 0.95` (test asserts the refusal); conservation checks flag a
deliberately momentum-violating fixture.

### WP-11 · Orchestration, run DB, figures
**Deps:** WP-10.
**Deliverables:** `orchestration/` — `sweep.py` (declares and enforces the
single varying axis), `rundb.py` (SQLite index), `plots.py` (standard set),
`scripts/run_study.py`, `tests/test_layering.py`.
**Acceptance:** Studies A–F configs exist and validate; a sweep that varies an
undeclared axis is rejected; quarantined runs are excluded from every query;
figures regenerate from the DB alone.

### WP-12 · Verification harness
**Deps:** WP-03, WP-04, WP-10.
**Deliverables:** `verification/` — analytic benchmark suite (§1),
`simfast_comparison.md` + configs (§2), `cross_engine.py` (§5), convergence
study configs (§6).
**Acceptance:** all analytic benchmarks pass on both engines; the Sim-FAST L1
comparison meets its tolerances or the deviations are explained with a
sensitivity run; the cross-engine report generates for L0/L1/L2.

---

## Dependency graph

```
WP-00 ─┬─► WP-01 ─┬─► WP-02 ─┬─► WP-03 ─┬─► WP-09
       │          │          │          │
       │          │          ├─► WP-04 ─┤
       │          ├─► WP-05 ─┤          ├─► WP-10 ─► WP-11
       │          └─► WP-06 ─┘          │
       │                     └─► WP-07 ─┴─► WP-08
       │                                      │
       └──────────────────────────────────────┴─► WP-12
```

**Critical path:** WP-00 → WP-01 → WP-02 → WP-04 → WP-09 → L3 results.
**Max useful parallelism:** 4 agents (WP-03 ∥ WP-04 ∥ WP-05 ∥ WP-06 in Phase 1).

## Suggested agent assignment

| Agent | WPs | Skills emphasized |
|---|---|---|
| A — Infrastructure | WP-00, WP-01, WP-11 | packaging, schema, data plumbing |
| B — Geometry/optics | WP-02, WP-07 (planner), WP-10 (surface/Zernike) | differential geometry, optics |
| C — MBDyn | WP-03, WP-12 | MBDyn decks, socket protocols |
| D — Chrono | WP-04, WP-09 | PyChrono, modal reduction, performance |
| E — Controls | WP-05, WP-08 | control theory, actuator modeling |
| F — Physics/V&V | WP-06, WP-10, WP-12 | structural dynamics, V&V |

## Definition of done for the whole framework

1. All 12 WPs' acceptance criteria pass in CI.
2. All six studies (A–F) produce indexed, non-quarantined runs.
3. The cross-engine report and the Sim-FAST comparison are both green (or
   their deviations are explained).
4. Every figure in the paper regenerates from `rundb` with one command.
