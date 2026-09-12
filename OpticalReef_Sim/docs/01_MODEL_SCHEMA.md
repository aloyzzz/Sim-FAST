# Layer 0/1 — Geometry Generation and the Neutral Model

## 1. Target surfaces

All shapes are defined as a `TargetSurface` with an analytic form, its normal
field, and a parameter vector. Reconfiguration is a transition between two
`TargetSurface` instances.

| Kind | Definition | Parameters |
|---|---|---|
| `paraboloid` | `z = r²/(4f)` | `f` (focal length) |
| `sphere` | `z = R − sqrt(R² − r²)` | `R` |
| `flat` | `z = 0` | — |
| `offaxis_paraboloid` | parent paraboloid, aperture decentered by `d` | `f`, `d` |
| `conic` | `z = r²/(R(1+sqrt(1−(1+K)r²/R²)))` | `R`, `K` |

The headline reconfiguration case is a focal-length change on a 1000 m
aperture, e.g. `f: 1000 → 2000 m` (f/1 → f/2). Required sag change at the rim
is `r²/4·(1/f₀ − 1/f₁)` = 250000·(1/1000 − 1/2000) ≈ **31.25 m** of axial
travel at the edge — which is what makes this a *reconfiguration* problem and
not a figure-maintenance problem, and what sets the actuator stroke budget.

## 2. Hex tiling on a curved surface

`geometry/tiling.py` generates the module layout:

1. Tile a hex lattice in the aperture plane with flat-to-flat pitch `p`
   (default 10 m), out to radius `D/2`, optionally with a central obscuration.
2. Project each hex center onto the target surface along `+z`.
3. Build the module frame: origin at the projected center, `z_local` = surface
   normal, `x_local` = projection of global `+x` (with a documented tie-break
   at the pole).
4. Record, for each adjacent pair, the interface location (midpoint of the
   shared edge, projected) and the interface frame.

**Gap growth is the key output.** On a curved surface the flat hexes cannot
stay edge-to-edge: the inter-module gap grows with `r` and with curvature.
`tiling.py` must report the gap field for both the initial and target surface;
`Δgap` per interface *is* the required actuator stroke for a truss-based
architecture, and feeds the feasibility check in §5. An agent who builds the
tiling in the plane and ignores this will produce a model that cannot
reconfigure at all.

## 3. Neutral model object graph

```python
ReefModel
├── meta: ModelMeta              # name, level (L0..L3), git sha, units="SI"
├── bodies:      list[Body]      # rigid or reduced-flexible modules
├── nodes:       list[Node]      # attachment / FEM nodes (flexible fidelity)
├── interfaces:  list[Interface] # inter-module connections
├── actuators:   list[Actuator]  # truss or joint, with limits
├── sensors:     list[Sensor]    # edge, metrology, attitude, oracle
├── reduction:   Reduction|None  # Craig-Bampton data if reduced
└── surfaces:    dict[str, TargetSurface]   # "initial", "target", ...
```

### Body
| field | type | notes |
|---|---|---|
| `id` | int | dense, 0-based |
| `frame` | `Frame` (pos 3, quat 4) | reference configuration |
| `mass` | float | kg |
| `inertia` | 3×3 | about body COM, body axes |
| `fidelity` | `"rigid" \| "flexible" \| "reduced"` | |
| `node_ids` | list[int] | non-empty iff `flexible`/`reduced` |
| `ring` , `tile_ij` | int, (int,int) | provenance for plotting and per-ring metrics |

### Interface
| field | notes |
|---|---|
| `id`, `body_a`, `body_b` | |
| `frame_a`, `frame_b` | attachment frames in each body's local coords |
| `joint_type` | `bushing` \| `revolute_compliant` \| `rigid_weld` |
| `k` | 6-vector stiffness (N/m ×3, N·m/rad ×3) |
| `c` | 6-vector damping |
| `nominal_gap` | m, from the tiling |

### Actuator
| field | notes |
|---|---|
| `id`, `kind` | `truss` \| `joint` \| `intra_module` |
| `attach_a`, `attach_b` | (body_id, local point) for `truss`; interface_id for `joint` |
| `axis` | unit vector in the reference config (`joint` only) |
| `k`, `c` | actuator internal stiffness / damping |
| `L0_ref` / `theta0_ref` | reference rest length / angle |
| `limits` | `ActuatorLimits` — see `03_CONTROL_ARCHITECTURE.md` §2.2 |
| `dynamics` | `ActuatorDynamics` — `wn`, `zeta`, or `none` |

### Sensor
`kind ∈ {edge, metrology_range, attitude, oracle}`, each with
`rate_hz`, `noise_std`, `latency_s`, `bias`, `quantization`.

## 4. Normative rules

1. **SI units, radians, everywhere.** No degrees, no mm, no kN. The schema
   rejects a config with a `_deg` or `_mm` suffix.
2. **IDs are dense 0-based integers** assigned by the builder. Engine adapters
   own their own ID mapping and must never leak engine IDs upward.
3. `ReefModel` is **immutable after `build()`** (frozen dataclasses). Controllers
   and metrics read it; nothing mutates it.
4. Every `ReefModel` must round-trip losslessly to and from HDF5
   (`model/io.py`), so an L3 model is generated once and reused across a sweep
   instead of being regenerated per run.
5. The model carries **no time-varying state**. State lives in the engine;
   histories live in `RunResult`.

## 5. Feasibility pre-check (runs before any engine is touched)

`geometry/feasibility.py` solves the *kinematic* inverse problem: find the
actuator command vector `q*` minimizing RMS surface error against the target,
subject to stroke limits. It reports, without running any dynamics:

- required stroke per actuator vs. available stroke (pass/fail per actuator),
- best achievable residual RMS error (the *kinematic floor* — no controller can
  beat it, and a dynamic run that appears to is a bug),
- the condition number of the shape Jacobian `J = ∂(surface error)/∂q`, which
  predicts control-allocation difficulty and flags under-actuated directions.

This check is mandatory in the runner: a config that fails it must not consume
a compute budget. It is also, on its own, a paper-worthy result — it tells you
the stroke a given architecture needs at a given length scale.

## 6. Config file shape

```yaml
meta: {name: L2_focal_change, level: L2}
geometry:
  aperture_diameter: 100.0
  module_pitch: 10.0
  central_obscuration: 0.0
  surfaces:
    initial: {kind: paraboloid, f: 100.0}
    target:  {kind: paraboloid, f: 200.0}
module:   {fidelity: rigid, mass: 250.0, inertia_model: hex_plate, thickness: 0.05}
interface: {joint_type: bushing, k: [1e6,1e6,1e6,1e5,1e5,1e5], c_ratio: 0.02}
actuation: {topology: truss_3dof_per_edge, k: 1e7, c: 1e4,
            limits: {stroke: 0.5, rate: 0.01, force: 500.0}}
control:  {law: pid_decentralized, rate_hz: 1.0, gains: {kp: 2.0, ki: 0.05, kd: 5.0}}
maneuver: {profile: min_jerk, duration: 600.0, input_shaping: ZVD}
disturbance: {cases: [none]}
solver:   {engine: mbdyn, dt: 0.05, t_end: 3000.0, output_dt: 0.5}
```

The JSON Schema in `schemas/reef_config.schema.json` is **normative**; this
table is illustrative. Validation failure is a hard error, never a warning.
