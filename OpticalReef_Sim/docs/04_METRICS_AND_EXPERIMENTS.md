# Layer 5/6 — Metrics and the Experiment Matrix

The paper's abstract names its evaluation metrics explicitly: *settling time,
structural deformation, induced dynamic modes, residual shape error, and
required actuator force.* Each gets one normative definition here so numbers
are comparable across every run, engine, and length scale.

## 1. Reference frame (prerequisite for every shape metric)

`metrics/frames.py` computes, at each output step, the mass-weighted
Procrustes/Kabsch fit of module centers to their reference positions, yielding
`R(t), d(t)`. All shape metrics are evaluated in that frame. Rigid-body motion
is reported separately as `rigid_body_excursion`.

Two variants, both reported:
- **`fit_rigid`** — remove translation + rotation only. Honest surface error.
- **`fit_rigid_focus`** — additionally remove best-fit focus. This is what an
  optical system with a refocusable secondary actually sees; it is usually far
  smaller and must never be reported without the `fit_rigid` number beside it.

## 2. Metric definitions

### 2.1 Surface error
For each module `i` with center `p_i` and normal `n_i`, let `d_i` be the signed
distance from `p_i` to the target surface along the local surface normal.

- `surface_rms(t) = sqrt( Σ a_i d_i² / Σ a_i )`, area-weighted (`a_i` = module area).
- `surface_pv(t) = max d_i − min d_i`.
- Per-module **piston / tip / tilt** relative to the local target tangent plane
  — the segmented-telescope error budget decomposition.
- **Wavefront error**: `wfe_rms ≈ 2 · surface_rms` at normal incidence; report
  in nm and in waves at `λ = 500 nm`.

### 2.2 Zernike decomposition
Fit `d_i` over the unit-normalized aperture with Noll-ordered Zernikes up to
`n = 10` (`metrics/zernike.py`). Report the coefficient time histories. This is
what makes "the maneuver excited a trefoil residual" a statement rather than an
impression, and it connects directly to optical performance.

### 2.3 Settling time
`t_settle` = the earliest `t` after maneuver end such that `surface_rms(τ)`
stays within the band for all `τ > t`. **Two bands, both reported**:
- **Absolute**: `surface_rms ≤ spec`, with `spec` from the config
  (default `λ/20` surface = 25 nm — the optical requirement).
- **Relative**: within `±5 %` of the final settled value — meaningful even when
  the run never reaches the absolute spec.

If the run ends before settling, report `t_settle = inf` and the value at
end-of-run. Never extrapolate; never silently report the last sample.

### 2.4 Induced dynamic modes
1. Eigenanalysis at the initial configuration gives `Φ` (rigid modes stripped).
2. Project the post-maneuver transient onto `Φ`: `η(t) = Φᵀ M u(t)`.
3. Report **modal energy fraction** per mode, `E_j = ½(η̇_j² + ω_j²η_j²)`,
   normalized — i.e. *which modes the maneuver rang up*.
4. Also report the response PSD and a spectrogram, since `ω_j` shifts as the
   structure changes shape (a real and reportable nonlinear effect: the
   f/1→f/2 reconfiguration changes the structure's own modal frequencies).

**Requires an undamped-integrator run** (`rho ≈ 1` / HHT `alpha ≈ 0`). The
metric layer must refuse to compute this from a numerically-damped run rather
than quietly returning a wrong number.

### 2.5 Structural deformation
- `max_module_deviation(t)` — largest module-center deviation from the
  *quasi-static* reconfiguration path (the same maneuver run with an
  effectively infinite duration). This isolates dynamic overshoot from the
  intended shape change, which a raw displacement plot cannot do.
- `interface_load(t)` — peak and RMS force/moment per interface; feeds the
  structural sizing argument.
- `dynamic_amplification = max|u_dyn| / max|u_quasistatic|`.

### 2.6 Actuator effort
`peak_force`, `rms_force`, `total_stroke_used`, `stroke_margin`,
`energy = ∫ Σ |F_k · L̇_k| dt`, `peak_power`, `saturation_duty` (fraction of
time any actuator is rate- or force-saturated). Normalize by actuator count
when comparing topologies.

### 2.7 Conservation checks (computed on every run, not optional)
`linear_momentum_drift`, `angular_momentum_drift`, `energy_balance_residual`.
These are pass/fail gates — see `06_VERIFICATION.md` §3 — and a run that fails
them is quarantined by the runner and excluded from any results table.

---

## 3. Experiment matrix

The paper's independent variables, as the sweep axes:

| Axis | Levels |
|---|---|
| **Length scale** | L1 (10 m, 7 mod) · L2 (100 m, ~100 mod) · L3 (1000 m, ~9000 mod) |
| **Actuator topology** | `joint_only` · `truss_1dof_per_edge` · `truss_3dof_per_edge` · `hybrid` · `intra_plus_inter` |
| **Control law** | `open_loop` · `pid_decentralized` · `pid_shape` · `lqr_modal(+filtered)` · `mpc_rate_limited` · `passive_damped` |
| **Commanded shape change** | focal `f/1→f/2` · focal `f/2→f/1` · flat→parabolic · parabolic→spherical · off-axis retarget · pure global tilt |
| **Maneuver duration** | `{0.5, 1, 2, 5, 20} × 1/f_1`, with/without ZVD shaping |
| **Sensing** | `oracle` · `edge_only` · `edge + sparse metrology` |
| **Disturbance** | `none` · `srp` · `thermal_snap` · `slew` · `isam_reaction` |

Full crossing is combinatorially absurd. The sweep plan is:

- **Study A — Control comparison** (L2, `truss_3dof_per_edge`, `f/1→f/2`,
  duration `2/f_1`, oracle): all 6 laws. *6 runs.*
- **Study B — Actuator topology** (L2, best law from A, same maneuver): all 5
  topologies. *5 runs.*
- **Study C — Length scale** (L1/L2/L3 × best law × 2 topologies): the scaling
  law for `f_1`, settling time, and required force vs. aperture `D`. *6 runs,
  L3 included — the headline figure.*
- **Study D — Maneuver aggressiveness** (L2, best config): 5 durations × 2
  shaping. *10 runs.*
- **Study E — Sensing realism** (L2, best config): 3 sensor suites. *3 runs.*
- **Study F — Shape maintenance under disturbance** (L2 and L3): 5 disturbance
  cases × {passive, best law}. *20 runs.*

≈50 production runs plus the verification suite. Every study declares its
controlled variables in its config; `orchestration/sweep.py` refuses a sweep
that varies more than the declared axis.

## 4. Scaling law (the transferable result)

Study C exists to produce the relationships that generalize beyond Optical Reef:
`f_1(D)`, `t_settle(D)`, `F_peak(D)`, `E_actuation(D)`, and required stroke
`Δs(D)`, each fitted as a power law with confidence intervals and compared
against the analytic expectation for a plate-like structure (`f_1 ∝ t/D²` for a
homogeneous plate; deviation from that exponent is precisely the effect of the
modular, jointed architecture — and is the framework's most transferable
finding for space-based solar power and other ultra-large assemblies).

## 5. Outputs per run

```
runs/<run_id>/
├── config.yaml          # exact, resolved config
├── manifest.json        # git sha, engine + version, host, wall time, seeds
├── model.h5             # the ReefModel actually simulated
├── history.h5           # state, commands, sensors, diagnostics
├── metrics.json         # every metric in §2
└── figures/             # standard plot set
```

`orchestration/rundb.py` indexes `metrics.json` into SQLite so cross-run
figures are a query, not a directory walk. Figures are regenerated from the DB,
never hand-assembled.
