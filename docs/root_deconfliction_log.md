# Deconfliction log — `root.tex` vs `root_with_alts.tex` (2026-07-06)

**Outcome.** `docs/scitech-paper/root.tex` is now authoritative. It keeps every code-agreement fact it already carried (mixer structure, coefficient values, sign conventions, the 2026-07-03 results, the success criterion with its 1 s grace window) and absorbs every derivation correction from `root_with_alts.tex` (findings F1–F5 and the minor items M3–M10, M12, M13 of `docs/saturation_verification_report.md`), with three small adaptations where the two campaigns interact. Rebuilt clean: 25 pages, no errors, no undefined references, no overfull boxes.

## 1. How the two files diverged

Both files descend from a common pre-2026-07-03 ancestor:

- **`root.tex`** received the **code-agreement campaign** (2026-07-03, documented in `docs/paper_code_agreement_saturation.md` and CLAUDE.md): mixer rewritten to match `dsmc_constraints.m` (`b`, not `bl`, in the moment rows; explicit normalized values `b=5, d=2, Ω²∈[0,2]`), sign corrections for `T_y`/`f₁`/`f₂`, the `s_z`/`s_ψ` clip paragraph, the rotation clause + 1 s grace window in the success criterion, the sweep methodology sentence, and the current results (abstract 0% → 59.3%; Table 4 = 0.593/0.000 etc.).
- **`root_with_alts.tex`** received the **math-verification campaign**: the remedial edits prescribed by `docs/saturation_verification_report.md` (F1–F5 substantive findings, M-series minor findings), applied to the *older* base — so it still carried the ancestor's stale numbers (46.4%/62.9%), the `bl` mixer, the pre-grace-window criterion, and the `+` sign variants.

The two files therefore differ for two unrelated reasons, and each hunk was classified by asking: *is this a root-side code-agreement edit that alts simply lacks, or an alts-side derivation fix that root lacks?* No hunk turned out to be a genuine both-sides-edited collision except the mixer prose (§3, decision K2) and three wording interactions (§4).

## 2. Decision rule

1. **Code is ground truth for structure, coefficients, signs, and results.** Verified directly against `dsmc_constraints.m`, `dsmc_no_constraints.m`, `system_dynamics.m`, `constants.m` — not against either document's claims.
2. **The verification report's corrections are ground truth for derivation validity** (they were machine-checked with sympy, and F1 was later verified against the code per the report's RESOLVED note), so they are adopted — re-expressed in the code-true frame where necessary.

## 3. Keep-root decisions (alts version rejected as stale or code-inconsistent)

| # | Location | root.tex (kept) | root_with_alts (rejected) | Code evidence |
|---|----------|-----------------|---------------------------|---------------|
| K1 | Abstract | 0% (naive per-motor clipping) → **59.3%** | 46.4% → 62.9% | Canonical 2026-07-03 re-run: sat_on 83/140 under the 1 s grace window, sat_off (clipped) 0/140. The 46.4% figure was the *unconstrained* controller, retired per audit finding C1/C3. |
| K2 | `eq:sat-mixing` + prose, `eq:sat-BBT`, `eq:sat-TMinv`, `eq:sat-decomposition` | Moment rows carry **`b`** (no `l`); Gram `diag(4b², 2b², 2b², 4d²)`; contributions `Mx/(2b)`, `My/(2b)`; prose gives `b=5, d=2, Ω²min=0, Ω²max=2`, T/W = 5.10, hover Ω² = 0.392, and explicitly flags these as *effective* values not derived from the physical mixing of Eq. 1 | `bl` in moment rows; `2b²l²` Gram; `Mx/(2bl)`; M1's tie `d = k_MT·b` | `dsmc_constraints.m:126-129` (`TM = [b b b b; 0 -b 0 b; -b 0 b 0; d -d d -d]`), `:101-102` (`b=5, d=2`), `:117-118` (`Omega2_min=0, Omega2_max=2`), `:304-307` (`c_Mx = Mx/(2*b)`). **M1 deliberately not adopted**: the code's `d/b = 0.4 ≠ k_MT = 0.1` and the moment rows omit `l`, so tying `T_M` to Eq. 1 would assert something false; root's "effective values" disclaimer is the honest replacement for M1's intent (it acknowledges the disconnect instead of hiding it). |
| K3 | Nomenclature: `κ` | `≈ 0 (see text)` | `0.1` | `constants.m:4` sets `Jmp = 0`, so the plant's coupling term `(kwt/kmt)·Jmp·Mz·ω` (`system_dynamics.m:41-42`) vanishes identically. Alts' `0.1` conflates κ with `kmt = 0.1` (the yaw-torque arm). |
| K4 | Nomenclature: `x,y,z`; EOM prose | north, east positive; **z upward (altitude)** | north, east, **down** | `system_dynamics.m:53`: `xdot(12) = vx·sθ − vy·cθsφ − vz·cθcφ` is the *negative* of the NED down-rate, i.e. altitude, up positive. |
| K5 | Nomenclature: `Ω²min/max`, `b`, `d` rows | `0, 2 (normalized)`, `5`, `2` | `--` placeholders + TODO comment | Same code lines as K2. This is the M11 fix — root resolved it with real values; alts left it open. |
| K6 | Gyroscopic sentence after `eq:nonlinear-eom` | Present (`J_r ≈ 0`, `κ → 0`, terms retained for generality) | Absent | `Jmp = 0` in `constants.m` — the sentence is code-true and load-bearing for K3. |
| K7 | `s_z`/`s_ψ` clip paragraph after the dSMC laws | Present (clip to ±2 before `s̃` correction) | Absent | `dsmc_constraints.m:156-157`; clip is canonical and load-bearing (MEMORY: removing it lands zero sweep trajectories). |
| K8 | `T_y` definition | `cφ sθ sψ **−** sφ cψ` | `+ sφ cψ` | `dsmc_constraints.m:198` (`g1_k`) has the minus. |
| K9 | `f₁`, `f₂` definitions | `−K₄lφ̇`; `−J_r φ̇Ω_r − K₅lθ̇` | `+K₄lφ̇`; `+J_r φ̇Ω_r + K₅lθ̇` | `dsmc_constraints.m:219-220`. |
| K10 | `tab:gains-scalars` | Rows for `b/d`, `Ω²min/max`, clip ±2, `k_ξα = ηαΔt` | TODO + `[value]` placeholders | Same code lines as K2/K7; `k_xi = [nuz nu3 nu4 nupsi]·dt` at `:123`. |
| K11 | `eq:ballistic-success` | Rotation clause carries `(t > 1 s)` + separation-transient explanation sentence | No grace window | `ballistic_success.m` default `RotGraceT = 1`; canonical numbers (59.3%) depend on it. |
| K12 | Saturated-results narrative | Methodology sentence (8 deploys × 5 cross-track × 4 neighbors = 140 valid, 60 s window); applied `|ū_k| ≈ 40 = 4bΩ²max`; unconstrained spikes >800 (median peak ≈1.2×10³, worst ≈10⁶); clipped capped at 40 but 0/140 lands | Older, vaguer version attributing the >800 spikes to "simple clipping" | Matches `sweep_landing_centroid.m` grid and the 2026-07-03 log data. The >800 spikes belong to the *unconstrained* command; the clipped baseline is capped at 40 by construction — root's attribution is the correct one. |
| K13 | `fig:sat_control_norm_on` caption | "applied-command norm `|ū_k|`" | "command-norm `|u_k|`" | TO_06 plots the post-allocation command (the controller returns `u_bar`). |
| K14 | `tab:sat_cmp_summary` | 0.593 / 0.000; 0.999 / 0.000; 314.63 / --; (580.87, −241.32) / -- | 0.629 / 0.464; 0.999 / 0.882; 318.64 / 387.83; centroids | Canonical 2026-07-03 numbers (grace-window run); alts' column pairs pre-criterion sat_on with the *unconstrained* arm mislabeled as clipping. |

## 4. Take-alts decisions (verification-report fixes ported into root.tex)

| # | Report finding | What changed in root.tex | Adaptation? |
|---|---------------|--------------------------|-------------|
| A1 | M12 | §Saturation intro: "Inside U … Lyapunov stable" now qualified by "provided no saturation has yet occurred", with convergence-back-to-baseline cross-ref to §Reduction | None |
| A2 | M3 | "scales exactly one column of `T_M⁻¹u`" → "one column-contribution … the `v_α` of Eq. (sat-decomposition)" | None |
| A3 | M4 | Gain identification `(η_T, η_Mx, η_My, η_Mz) ≡ (η_z, η_φ, η_θ, η_ψ)` stated once, where `E` is introduced | Verified against code: `nuz=7 (z/T)`, `nu3=14 (φ/Mx)`, `nu4=14 (θ/My)`, `nupsi=7 (ψ/Mz)` — matches |
| A4 | M13 | After `eq:sat-maxscale`: base-feasibility precondition + "trailing feasibility checks are load-bearing" | **Adapted**: "each stage **short of the terminal fallback** returns its candidate only after re-checking" — code re-checks after stages 1–2 (`if all(Om2 …)`) but the terminal stage returns unconditionally |
| A5 | **F1** (must-fix) | Stage-4 item rewritten: states the `Ω²min = 0` assumption, shows the ± pair structure of `v_Mx + v_My`, concludes γ = 0 and `ū = 0` (full motor cutoff), replaces the false "guaranteed to find γ" narrative with the coast-unactuated/recovery story. This is the report's *minimal* remedial plan — matching what the code actually does (`dsmc_constraints.m:340-343` returns exactly this), not the *preferred* mid-band re-centering plan, which would be a behavior change requiring re-simulation | **Adapted**: direction written `1/(2b)·(−My, −Mx, My, Mx)` (alts had `1/(2bl)`, inconsistent with the code mixer per K2). Also "the stage returns no command at all" → "the closed form admits no feasible scale" (tighter). M2 (soften "guaranteed") is subsumed — the word is gone |
| A6 | — | `\label{sec:sat-deficit}` added (needed by A5/A12 cross-refs) | None |
| A7 | M10 | Deficit-section notation harmonized to the dSMC section's conventions: `dz_k → ż_k` (and `ẏ, ẋ, φ̇, ψ̇`), `a_z → α_z`, `a_ψ → α_ψ`, `Δ_{k,k+1} → Δ_{k+1,k}` (10 sites) | Verified the dSMC section uses `Δ_{k+1,k}`/`α_z`/`ż` exclusively (9/11/8 occurrences, zero `Δ_{k,k+1}`) |
| A8 | M5 + M6 | Substitution list un-inverted (now maps roll→pitch: `{a1..a4}→{a5..a8}`, `g1→g2`, `ẏ→ẋ`, `u2→u3`, `Jxx→Jyy`); M6 implementation sentence added (δy must use commanded `u₁`, not `ū₁`) | M6's condition verified TRUE in code: `dsmc_constraints.m:224-227` builds `u_Mx/u_My` from the commanded `u_T` of STEP 4 (single-pass, pre-allocation) |
| A9 | F4 (part) | Positivity sentence itemized per entry; `D₁₁ > 0` now carries the `|φ|,|θ| < 90°` envelope qualifier, forward-ref to §ISS | None |
| A10 | — | `\label{sec:sat-schur}` added (needed by A8) | None |
| A11 | F5 | "Schur stability is parameter-uniform" → "the Schur *condition* is parameter-uniform" + decay-constants caveat; new paragraph bridging the frozen-time/LTV gap (constant diagonal blocks, `D_k` only off-diagonal, `(D_k − D_{k+1})ξ_{k+1}` remainder, ISS as the formal certificate) | None |
| A12 | **F3** | ISS proof reordered: second Young (ν₂, new `eq:sat-nu2-bound`, noting it coincides with the μ bound = 0.35 under matched gains) now precedes the ρ choice; ρ chosen against the *eroded* margin (new `eq:sat-rho-choice`); the "~" order-of-magnitude ν/γ replaced by exact `eq:sat-nu-exact`/`eq:sat-gamma-exact` including the subtracted cross-square bound in the ξ branch | None (pure presentation; no conclusion changes) |
| A13 | **F4** | New envelope block after the ISS conclusions: explicit `D₂₁ = 6Δt T_y/(u_T cψ)`, `D₃₁ = −6Δt T_x/(u_T cφ cψ)` (verified against `tab:a-coeffs` and code `a1/a5/g1/g2`), envelope `eq:sat-envelope` (ε_u, ε_a, attitude bound), Frobenius certificate `eq:sat-MD-bound` (√6 over the six nonzero entries), empirical tie to TO_06, and the attitude-free anti-windup direction `∂u_T/∂ξ_T = −η_zΔt` | **Adapted**: "commanded effort norm" → "applied effort norm sits at the allocation bound" for consistency with K13's corrected caption. The `|T_x|,|T_y| ≤ 1` inner-product argument survives the K8 sign fix unchanged (`(sψ, −cψ)` is still a unit vector) |
| A14 | M7 | "positive semi-definite" → "positive definite in `s`" (§Reduction) | None |
| A15 | M8 | `a₁|g₁ₖ|` → `|a₁||g₁ₖ|` in `eq:sat-residual-cross` (a₁ = 6m/(u_T cψ) flips sign with cψ — confirmed at `dsmc_constraints.m:187`) | None |
| A16 | M9 | "(identically for pitch and yaw)" → "(identically for pitch; Δt/J_zz for yaw)" | None |
| A17 | **F2** (must-fix) | `eq:sat-residual-ratio` gains the nondimensional form `(a₃/l)·(ml²/J_xx)·(η_T/η_Mx)`; "large for any quadrotor with J ≪ ml" (dimensionally ill-posed) replaced by the dimensionless driver `ml²/J_xx = 17.8` and the `ml² ≫ J_αα` thin-rotor-plane criterion (report's remedial option (b)) | Numeric check: 0.8 × 0.2² / 1.8×10⁻³ = 17.8 ✓ |

## 5. Interactions worth remembering

- **F1 vs the code**: the paper now *documents* the motor-cutoff fallback rather than fixing it. The report's preferred remedy (mid-band collective re-centering) remains a candidate controller change; adopting it would require editing `priority_weighted_allocate()` and re-running the sweep, so it was correctly kept out of a paper-truthing pass. The code comment at `dsmc_constraints.m:337-339` ("measure-zero set of inputs in practice") is consistent with the paper's transient-saturation framing.
- **M1 is the one report finding whose remedy was overtaken by events**: the report assumed the `bl` mixer and asked for the `d = k_MT b` link; the code-agreement pass revealed the implementation mixer is an effective/normalized one, so the correct fix was root's explicit disclaimer, not the link.
- The **envelope block (A13) uses commanded `u_T`** in `eq:sat-envelope` while its empirical support cites the *applied* norm figure; this matches the code (the `a_i` are evaluated at commanded `u_T`, `dsmc_constraints.m:178-190`) and the caveat in the code's own STEP 5 comment.

## 6. Verification

- `pdflatex` (×2) + `bibtex` clean: 25 pages, no errors, no undefined references/citations, 0 overfull boxes.
- `grep` confirms zero remaining `Δ_{k,k+1}` / `\mathrm{d}⟨state⟩` tokens; all 8 new labels (`sec:sat-deficit`, `sec:sat-schur`, `eq:sat-nu2-bound`, `eq:sat-rho-choice`, `eq:sat-nu-exact`, `eq:sat-gamma-exact`, `eq:sat-envelope`, `eq:sat-MD-bound`) defined and resolved.
- `root_with_alts.tex` left untouched as the historical artifact of the math-verification pass; it is now fully superseded by `root.tex` and can be archived or deleted at the author's discretion.
