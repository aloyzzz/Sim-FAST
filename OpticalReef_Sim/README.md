# Optical Reef — Active Shape Reconfiguration Simulator

Simulation framework for **"Dynamic Modeling of Active Shape Reconfiguration in a
Modular 1000-Meter In-Space Telescope"** (J. Hill, D. A. Barnhart, USC SERC),
built on **MBDyn** and **Project Chrono**.

> **Status: architecture and contracts only.** No physics is implemented yet.
> This directory defines the design, the normative schema, and the ABCs that the
> implementation work packages fill in. Start at
> [`docs/05_WORK_PACKAGES.md`](docs/05_WORK_PACKAGES.md).

## The problem in one paragraph

A 1000 m modular telescope has a fundamental frequency in the tens of
millihertz. Commanding it to change shape — e.g. an f/1 → f/2 focal-length
reconfiguration, which requires **~31 m of axial travel at the rim** — excites
the structure with the maneuver itself. This framework measures what that
costs: settling time, residual surface error, induced modes, and actuator
effort, across control laws, actuator topologies, and length scales.

## Read in this order

1. [`docs/00_ARCHITECTURE.md`](docs/00_ARCHITECTURE.md) — layering, fidelity ladder, free-free policy
2. [`docs/01_MODEL_SCHEMA.md`](docs/01_MODEL_SCHEMA.md) — the engine-neutral model
3. [`docs/02_ENGINE_ADAPTERS.md`](docs/02_ENGINE_ADAPTERS.md) — MBDyn and Chrono mappings
4. [`docs/03_CONTROL_ARCHITECTURE.md`](docs/03_CONTROL_ARCHITECTURE.md) — actuators, sensors, controllers
5. [`docs/04_METRICS_AND_EXPERIMENTS.md`](docs/04_METRICS_AND_EXPERIMENTS.md) — metric definitions, studies A–F
6. [`docs/06_VERIFICATION.md`](docs/06_VERIFICATION.md) — V&V, incl. regression against `../Hex_Ring_Sim`
7. [`docs/05_WORK_PACKAGES.md`](docs/05_WORK_PACKAGES.md) — who implements what

## Four rules that the rest of the design hangs on

1. **Only `opticalreef/engines/` may import an engine.** Enforced by
   `tests/test_layering.py`. This is what makes cross-engine agreement a
   verification argument rather than a coincidence.
2. **Simulate free-free; remove rigid-body motion in post-processing.** Six
   zero-frequency modes are correct. Grounding the structure to avoid them
   changes the answer.
3. **Actuators are compliant with finite stroke, rate, and bandwidth.** A
   prescribed-motion joint would perfectly track the command and erase the
   induced dynamics the paper exists to measure.
4. **Numerical damping is a configured, declared quantity.** MBDyn `rho` and
   Chrono HHT `alpha` must be near-undamped for any modal or energy metric,
   and the metric layer refuses damped runs rather than returning a plausible
   wrong number.

## Relationship to the rest of this repository

`../Hex_Ring_Sim/` contains the prior MATLAB (Sim-FAST) hex-ring work: compliant
bar joints and PID-controlled `prestrain` actuators. That model is the
**regression target** for L1 (`docs/06_VERIFICATION.md` §2) — the actuator
formulation here (`F = k(L − L0_cmd) + c·L̇`) is the direct continuation of its
eigenstrain actuator, so the comparison is meaningful rather than decorative.

## Quick start (once WP-00 lands)

```bash
python scripts/doctor.py            # verify MBDyn + PyChrono are present
pytest tests/                       # contracts and architectural guards
python scripts/run_study.py configs/L1_hexring_simfast.yaml
```
