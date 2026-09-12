# Verification and Validation Plan

There is no experimental data for a 1000 m in-space telescope. The credibility
of every number in the paper therefore rests entirely on this plan. It is not
optional and it is not "later".

## 1. Analytic benchmarks (L0)

| Case | Closed form | Tolerance |
|---|---|---|
| Two masses + one spring, free-free | `ω = sqrt(k/µ)`, `µ` reduced mass | 1e-8 rel. |
| Two masses + actuator step in rest length | exact response; net momentum ≡ 0 | 1e-8 |
| Free-free uniform beam, first 5 bending modes | `β_n L` roots of `cos·cosh=1` | 0.5 % |
| Free-free circular plate, first 5 modes | tabulated `λ²` values | 2 % (discretization) |
| Rigid-body drift under an applied force | `a = F/m`, `α = I⁻¹T` | 1e-10 |
| Damped SDOF actuator dynamics | 2nd-order step response | 1e-8 |

Every one is a pytest, runs in CI, runs on **both** engines.

## 2. Regression against the existing Sim-FAST hex-ring (L1)

`Hex_Ring_Sim/` in this repository already contains a validated MATLAB model of
a 7-hex ring with compliant bar joints and PID-controlled `prestrain`
actuators. It is the only pre-existing, independently-developed artifact
available to check against, so it is used deliberately:

1. Port `Build_Hex_Ring_Model.m` geometry and properties into
   `configs/L1_hexring_simfast.yaml` (same `R`, `barA`, `E_panel`, `E_joint`,
   `m_node`, Rayleigh `alpha_d`/`beta_d`).
2. Compare, MBDyn and Chrono against MATLAB:
   - first 10 free-free frequencies — **within 2 %**;
   - transient response to the `Hex_Ring_Dynamic.m` lateral impulse —
     normalized RMS difference of tip displacement **< 5 %**;
   - the `Hex_Ring_PID_Control.m` closed-loop case with identical gains —
     same steady-state deflection reduction **within 10 %**.
3. Differences beyond tolerance must be *explained*, not tuned away. The
   expected legitimate sources are: Sim-FAST's lumped nodal mass without
   rotational DOF vs. full 6-DOF rigid bodies, and its bar-only (no bending)
   panel representation. Document each in
   `verification/simfast_comparison.md` with a sensitivity run showing the
   discrepancy shrinks as the models are made more alike.

This is the closest thing to validation available and it is worth doing
properly; it also gives the paper a defensible continuity story from the prior
hex-ring work.

## 3. Standing invariants (every production run, automatically)

| Check | Threshold | On failure |
|---|---|---|
| Linear momentum drift (internal actuation only) | `|Δp|/(m·v_char) < 1e-6` | quarantine run |
| Angular momentum drift | `|ΔL|/(‖I‖·ω_char) < 1e-6` | quarantine |
| Energy balance: `ΔE_kin + ΔE_strain − W_act + E_damp ≈ 0` | 1 % of `W_act` | quarantine |
| Exactly 6 rigid-body modes | exact | hard fail at build |
| Newton convergence, all steps | no step at max iterations | warn + flag |
| Time-step convergence | metrics change < 1 % when `dt` halved | rerun at smaller `dt` |

Quarantined runs are excluded from every results table by `rundb.py`. A run
that fails an invariant is never reported with a caveat.

## 4. Model-order-reduction gate (prerequisite for L3)

L2 must be run **both** full and Craig–Bampton-reduced, same config:
- `f_1..f_20` within **1 %**;
- `surface_rms(t)` normalized RMS difference **< 2 %**;
- `t_settle` within **5 %**;
- `peak_force` within **5 %**.

Sweep `n_fix` (fixed-interface modes per module) over `{0, 3, 6, 12}` and report
the convergence. L3 uses the smallest `n_fix` that passes. **L3 results are not
valid before this gate passes** — and if the gate cannot be passed, that is a
reportable finding about reduced-order modeling of jointed modular structures,
not a reason to proceed anyway.

## 5. Cross-engine agreement

Run on both engines for L0, L1, L2:

| Quantity | Tolerance |
|---|---|
| `f_1..f_10` | 1 % |
| `surface_rms(t)` normalized RMS difference | 3 % |
| `t_settle` | 5 % |
| `peak_force` | 5 % |
| Modal energy fractions, top 5 modes | 5 % absolute |

`verification/cross_engine.py` produces a standing report. Systematic
disagreement is almost always one of four things, in decreasing order of
likelihood: mismatched numerical damping (`rho` vs. HHT `alpha`), a different
control-delay convention between the adapters, different bushing damping
conventions, or an inertia-tensor frame error. Check those four before
suspecting either solver.

## 6. Convergence studies

Report in an appendix: time step (`dt` ÷2, ÷4), Craig–Bampton `n_fix`,
intra-module mesh refinement (flexible fidelity), and solver tolerance. Each is
a single sweep config and each is cheap at L1/L2. Do them early — discovering a
`dt` dependence after the L3 runs is a very expensive mistake.
