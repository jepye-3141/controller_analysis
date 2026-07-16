# Plan J2 — Baseline Strength + Mechanism Attribution

**Goal:** defend the headline comparison (0% clipped → 55% handled) against the two standard major-revision demands — *"your baseline is a strawman"* and *"which of your two mechanisms actually does the work?"* — and repair a confirmed correctness defect in the cSMC baseline.
**Status:** proposal — W1/W2 add controller modes (author sign-off on any `dsmc_constraints.m` edit); W3 changes a baseline controller and requires re-running its results.
**Cost profile:** cheapest of the three plans — each new sweep arm ≈ 6–16 min on 12 workers; the envelope re-run is the usual `analysis.m` pass.

## Why this blocks acceptance

- The only saturated comparison is naive per-rotor clipping, which lands 0/140 — reviewers will call it a strawman unless bracketed by stronger alternatives.
- The handler bundles **priority-weighted allocation** and the **auxiliary anti-windup state (ξ)**; the paper attributes the gain to the pair with no ablation.
- The saturated arm recovers only 7/20 envelope samples (35%, vs 20/20 unconstrained, post-2026-07-08 masks) — currently not foregrounded; better to own and analyze it than have a reviewer discover it.
- **Confirmed defect:** cSMC pitch channel uses `Jzz` where the exact Lyapunov cancellation requires `Jyy` — in root.tex eq:smc3 *and* in the block (`zoh_smc_controller.slx`: `uhat_My = (-cY*Jzz/cos(phi))*(…)`). With `Jzz/Jyy = 5/6`, eq:sdot_final is false; the residual `(1−Jzz/Jyy)·c_My·(ω_y cφ − ω_z sφ)` reaches ~75% of the γ·sat reaching term at π-rad/s tumble rates ((8/6)·π√2 ≈ 5.9 vs γ_My = 8). The paper's cSMC equations additionally carry `v_y sθ sφ` (twice) where the EOM's ż has `v_y cθ sφ`, and the block's `uhat_T` leading sign differs from eq:smc1. The cSMC section never got the math-verification treatment the saturation sections received.
- Baseline tuning provenance is undocumented ("manual tuning" for LQR; PID/cSMC gains asserted).

## Workstreams

### W1 — Mechanism ablation (2×2)
Four arms on the identical 140-trial grid (deterministic, paired by construction):

| Arm | Allocation | Aux state ξ | Exists today? |
|---|---|---|---|
| A0 clip-only | – | – | yes (`saturation_on=false`) |
| A1 allocation-only | ✓ | – | **new flag** |
| A2 aux-only (clip + ξ) | – | ✓ | **new flag** |
| A3 full handler | ✓ | ✓ | yes (`saturation_on=true`) |

Implementation: one new bus field (e.g. `aux_on`, default true) read in `dsmc_constraints.m` — A1 = allocation with `xi` frozen at 0 (`tilde_s = s`); A2 = clip branch computing `delta_u` from the clipped command and running the ξ update. Add the field in `constants.m`, thread through `sweep_landing_centroid`'s `traj_params` like `saturation_on`. **Regression gate:** defaults reproduce 77/140 and 0/140 exactly.
Analysis: per-arm reachability + paired comparison on identical trials (McNemar exact test — the grid is paired, so this is the statistically correct claim of superiority), per-clause failure attribution from `results(...).criteria`. Deliverable: one ablation table + a paragraph attributing the gain.

### W2 — One published competitor
Implement a Faessler-2017-style **iterative prioritized mixer** (their Alg.: saturate per-rotor, redistribute preserving Mx/My, sacrifice yaw then thrust) as another dispatch mode in `dsmc_constraints.m` (same interface as `priority_weighted_allocate`, ~60 lines). One sweep arm. This is the fairest available "state of practice" comparator and the paper already cites it as the nearest prior; showing the closed-form projection + ξ beats (or ties) the iterative mixer with a stability certificate is a far stronger claim than beating a clip. *(Optional stretch: discretized Xu2020-style attitude-only auxiliary SMC as a second competitor — only if time allows; it requires more adaptation and design choices that could be disputed.)*

### W3 — cSMC repair + verification pass
1. Fix `Jzz→Jyy` in eq:smc3 and in the `uhat_My` line of the cSMC charts (`smc_controller.slx`, `zoh_smc_controller.slx` — confirm which one(s) `smc_swarm{,_single}.slx` reference before editing); reconcile the `sθ/cθ` and `uhat_T` sign discrepancies between root.tex eqs smc1/sdot-from-eom and the block (code is presumed ground truth per project convention — verify the code's own cancellation symbolically first).
2. Run the cSMC section through the same sympy verification treatment the saturation sections received (2026-07-06 math-verification pass: independent symbolic re-derivation of every displayed identity, PASS/FAIL per identity, exact — no floating-point tolerance except where a numeric value is itself the claim). Small scope here: 4 laws, 1 substitution chain.
3. Re-run cSMC results (envelope + any figures) — expect possible improvement in the cSMC arm; the comparison claim must rest on a *correct* baseline.
4. Document the tuning protocol for all baselines in an appendix (what was tuned, on which case, what the ballistic case reuses). If LQR/PID were tuned only for tracking, say so and add one retuning attempt on STABILIZE to preempt "you didn't even try."

### W4 — Terminal-fallback upgrade (mid-band re-centering remedy) + stage occupancy
1. Instrument the allocation cascade with per-tick stage counters (returned via a diagnostic output or persistent tally): how often stages 1/2/3/4 fire across the sweep. The stage-4 full-motor-cutoff story (root.tex, "the vehicle coasts unactuated") deserves data.
2. Implement the **mid-band re-centering variant** of the stage-4 terminal fallback (banked remedy from the 2026-07-06 saturation math-verification finding F1 + the 2026-07-06 root deconfliction §5; deliberately kept out of the paper-truthing pass because it is a controller behavior change). **What it is:** the current stage-4 fallback returns `ū_k = (0, γ·Mx, γ·My, 0)` — but with `Ω²min = 0` the closed form admits only `γ = 0`, so all four motors are commanded off (full cutoff) and *no* roll/pitch authority is actually retained; the item-4 "guaranteed to find γ ∈ [0,1]" narrative is false, because `v_Mx + v_My = (1/2b)·(−My, −Mx, My, Mx)` has a strictly negative component for any `(Mx, My) ≠ 0`, and any `γ > 0` drives it below `Ω²min = 0`. The remedy re-centers collective inside the feasible band instead of zeroing it: take `base = ½(Ω²min + Ω²max)·1` (equivalently a fixed non-commanded collective `T_c = 2b(Ω²min + Ω²max)`), `dir = v_Mx + v_My`, `γ = λ_max(base, dir)`, and return `ū_k = (T_c, γ·Mx, γ·My, 0)`. Because base is now strictly interior (margin `½(Ω²max − Ω²min) > 0` to both bounds), `γ > 0` *strictly* for any `(Mx, My) ≠ 0` — a true guarantee; physically, differential moments need a nonzero common mode to differentiate about, so maximizing retained roll/pitch authority forces mid-band collective, sacrificing altitude *tracking* but preserving maximal attitude authority. The resulting thrust deficit `T − T_c` is already absorbed by the auxiliary state ξ. **Why deferred:** adopting it requires editing `priority_weighted_allocate()` and re-running the sweep, so root.tex currently *documents* the honest motor-cutoff fallback (matching `dsmc_constraints.m` stage 4) rather than changing behavior. One sweep arm here. If it helps, it strengthens the contribution; if not, the occupancy data justifies the simple fallback.

### W5 — Envelope + intro-promise closure
1. Add the five-arm envelope success table (4/9/10/20/7 of 20) to the paper next to fig 39, with a sentence analyzing *why* the saturated arm loses envelope points (deploy-state energy vs authority — links to J1's T/W curve).
2. Close the dangling intro promise (root.tex L86 "energy efficiency and settling time"): either import the settling/energy comparison from the existing `analysis.m` material (figs 35–38 score matrices) as a short subsection, or cut the promise. Recommend importing — it is finished work that widens the journal delta for free.

## Order and effort

W1 (1–2 d incl. flags + 4 sweeps) → W2 (2–3 d) → W3 (2–4 d incl. re-verification + re-runs) → W4 (2–3 d) → W5 (1 d). Total ≈ 1.5–2 weeks elapsed, compute trivial next to J1/J3.

## Verification

- Regression gates: default-flag runs bit-reproduce 77/140 (sat_on) and 0/140 (sat_off) before any new arm is trusted.
- W1/W2 arms run on the identical grid (paired trials) — assert identical `xi`/`xf_ballistic` per trial index across arms.
- W3: sympy check of the corrected cSMC cancellation; A/B envelope run pre/post fix archived.
- Statistical outputs (McNemar p-values, CIs) computed by script committed alongside the logs.

## Paper deliverables

Ablation table + attribution paragraph (W1); competitor row + related-work sharpening (W2); corrected cSMC section + tuning appendix (W3); stage-occupancy figure + (if adopted) improved fallback description (W4); envelope table + settling/energy subsection or trimmed intro (W5).

## Decision points needing author input

(a) Sign-off on the two new bus flags and the competitor mode living inside `dsmc_constraints.m` vs a separate file. (b) Whether the cSMC fix re-runs everything cSMC-touching or only the paper-cited results. (c) Import the settling/energy material vs trim the intro promise. (d) Adopt mid-band re-centering if it wins, or keep the documented cutoff.

## Deferred reviewer/theory items (from 2026-07-10 journal-readiness audit G4)

The post-deconfliction ISS development is genuinely strong (exact ν/γ, an envelope certificate for M_D). These baseline/theory-attribution-scoped reviewer pokes remain open and ride along with W1/W3:

- **No robustness term for model mismatch.** The reaching identity `Δs = −E·s̃ + D·Δu` presumes exact equivalent-control inversion; the two-step-extrapolated model vs the true plant carries a mismatch the saturated analysis does not bound with a robustness/disturbance term.
- **Clip/ISS interaction not carried through.** The ±2 `s_z`/`s_ψ` sliding-variable clip (canonical, load-bearing) is not threaded through the saturated ISS argument — its interaction with the certificate is left unanalyzed.
- **Selection-biased M_D envelope.** The `M_D = sup_k |D_k| < ∞` envelope is checked empirically only on the successful-landing trials, so the supporting evidence is selection-biased.

(The remaining G4 item — "saturation not permanent" assumed, with stage-4 = full motor cutoff and the mid-band re-centering remedy unimplemented — is handled in W4 above.)
