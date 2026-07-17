# Reviewer Response Audit — CDC 2026 Submission 2391 vs. Current Manuscript

**Date:** 2026-07-17
**Subject paper (current):** `docs/scitech-paper/root.tex` — *"Discrete Sliding Mode Control with Actuator Constraints for the Stabilization of a Ballistically Launched Drone"* (AIAA SciTech format, ~1540 lines).
**Source reviews:** `ATLIS Ongoing Research/CommentsToAuthor.txt` (editor) + `Review22212/13/14/15.txt` (Reviewers 2, 3, 4, 8) — CDC 2026 / IEEE L-CSS submission 2391.
**Method:** Full read of the current paper + all reviews, then a 4-lens audit (theory rigor, claims-vs-evidence, consistency/cross-refs, scope/lit/justification), each lens adversarially re-verified by an independent agent against `root.tex` and the codebase. Headline numbers independently reproduced from `constants.m`. Line numbers below refer to the current `root.tex`.

> **Purpose:** map every reviewer concern from the rejected version onto the current manuscript, record what is now addressed vs. still open, and give a prioritized fix list for resubmission. Self-contained by design — safe to delete once acted on.

---

## 1. Context: this is a re-scope, not a revision

The rejected paper (CDC 2026 / L-CSS) was a **multi-controller UAV-swarm comparison** across step/figure-8/spiral trajectories, scored by kinetic-energy and settling-time metrics. The current paper is a **single-vehicle discrete-SMC + actuator-saturation** study centered on stabilizing a ballistically launched drone and on landing-site reachability, **with formal Lyapunov/Schur/ISS stability analysis added**. Two things changed:

1. **Venue:** CDC/L-CSS (6-page controls-theory letter) → AIAA SciTech (longer aerospace paper).
2. **Scope:** the swarm comparison was cut; the corrected dSMC + saturation handling is now the center.

This matters because a large fraction of the specific complaints are **moot by deletion** — the content that carried them is gone. That is a legitimate resolution (those defects cannot recur), but it also means the residual work is concentrated in the *new* material (the stability proofs and their framing).

---

## 2. Bottom line

- The **two concerns that actually sank the paper** are resolved: "no stability analysis" (E2/R4.3) and the energy-efficiency/settling-time overclaims (R2.3/R2.5).
- **Every headline number reproduces from the code** (independently checked — see §7). Numbers are not a risk.
- The residual risk is **framing, not arithmetic**, and it clusters on one point: **the certified quantity ≠ the reported quantity** (the proofs cover the 4 actuated sliding variables; the headline metric is a 10 m *horizontal* bound the proofs never touch). This is cheap to fix — scope the language, don't add theorems.
- **Author dispositions (2026-07-17):** the scope/focus (§4.C), simulation-only (§4.I), and equation-font (§4.L) concerns are set aside — the first two by decision (intentional organization; sim-only acceptable for this venue/stage), the third because the font has already been enlarged to a good size. Recorded, but excluded from the action list.
- Net: the paper is in far better shape than the rejected version. A focused **P1 language pass** (§5) closes the highest-leverage gap; the remaining **P2/P3** strengthen methodology and polish.

---

## 3. Status at a glance

| # | Concern (reviewers) | Status | Basis |
|---|---|---|---|
| A | No stability analysis (E2, R4.3, R3.2) | ✅ Addressed | cSMC/dSMC/Schur/ISS proofs added (552–574, 620–632, 689–701, 1241–47, 1358) |
| B | Overclaiming; theoretical vs empirical (E1, E3, R2.3) | 🟡 Mostly | "global/energy/settling" grep-zero; abstract still calls both SMCs "certified" |
| C | dSMC is the real contribution; baselines overstate breadth (R2.1, R2.5) | ⚪ Dismissed (author) | Current focus intentional; baselines retained by decision (2026-07-17) — see §4.C |
| D | Consistency: 3-tier/4-tier, mass 0.2/0.8, settling xref (R2.4, R3.5) | ✅ Resolved / moot | All removed; mass consistently 0.8 kg; clean label/cite set |
| E | Metrics not defined (R3.4) | 🟡 Partial | New criterion mostly defined; `x_t,y_t,x_f,y_f,v_t` unbound as symbols |
| F | Symmetric assumption unexplained (R2.2) | 🟡 Partial | Asserted (634), not explained; appears **not** load-bearing (laws keep Jₓₓ≠Jᵧᵧ) |
| G | Literature / motivation / title (R4.1, R8.1) | ✅ Addressed | 2024–25 cites; concrete motivation; dSMC-specific title |
| H | Comparison unfair — tuning-dependent (R8.2, R8.3, R4.4) | 🟡 Partial | Structural-failure argument present; no best-effort-tuning statement; narrow 8-vs-5/17 margin |
| I | Justify sim-only vs experiments (E4, R8.4) | ⚪ Dismissed (author) | Sim-only accepted for this venue/stage by decision (2026-07-17) — see §4.I |
| J | UAV model detail (R4.2) | ✅ Addressed | Full nonlinear + linear EOM + nomenclature |
| K | Data detail (R4.5) | ✅ Largely | 4-stat landing table + control-norm/envelope figures |
| L | Equation font (R3.6) | ✅ Resolved | Already enlarged to a good, legible size since the reviewed version (author, 2026-07-17) — see §4.L |
| M | 6-page limit (R3.3) | ⚪ Moot | Venue changed to AIAA SciTech |
| N | iThenticate 17% similarity | ⚪ Note | Report not accessible; under typical thresholds; citation audit complete |

Legend: ✅ addressed · 🟡 partial · 🔴 open · ⚪ moot / dismissed-by-author / informational.

---

## 4. Detailed findings

Each finding: **Evidence** (line-numbered) · **A reviewer could still say** (sharpest surviving counterpoint) · **Action**.

### A. Stability analysis — ✅ ADDRESSED (dominant theme: E2, R4.3, R3.2)

The bare complaint is genuinely closed, not hidden by the scope change.

- **Evidence:** cSMC Lyapunov proof `V=½s²`, `V̇=−γ·s·sat<0` (552–559), boundary-layer `V̇=−ρV` with QED (566–572); discrete dSMC reaching/Lyapunov condition (620–632); the Xiong2016 reaching-law violation is *actually derived* and shown `≠ −η s Δt` (689–701); Schur spectrum condition `0<ηΔt<2, 0<k_ξ<2` (1241–47); ISS-Lyapunov `ΔV ≤ −νV + γ|Δu|²` via Jiang–Wang (1358); reduction to unconstrained (1405–13) and steady-state residuals (1415–71).

**Residual scope caveats (these become P1/P2):**

- **A1 — x/y horizontal-position gap (most important).** The proofs certify only the four actuated sliding variables (roll, pitch, yaw, altitude). The headline success metric is a **10 m horizontal (x/y)** bound, which rides on the *underactuated* channels. dSMC folds y into `s_φ` and x into `s_θ` (710–714) but proves only reaching `Δs=−η s Δt` (762–767); cSMC error is `e = r_PID − [z,φ,θ,ψ]` (375). Grep confirms *"zero dynamics" / "internal dynamics" / "region of attraction" / "basin" appear nowhere*. So horizontal convergence is **neither proven nor acknowledged**.
  - *A reviewer could still say:* the Lyapunov claims silently cover attitude+altitude while the reported metric is horizontal miss — closer to "open" than "partial" on the honesty axis.
  - *Action:* add a zero-dynamics / sliding-manifold (or timescale-separation) argument for x/y, **or** explicitly scope the Lyapunov claims to the four actuated sliding variables and state horizontal-position convergence is empirical only.

- **A2 — frozen-D / slowly-varying-D step in the LTV argument.** Line 1250 honestly raises the "pointwise Schur ≠ LTV stability" caveat and labels the spectrum "diagnostic"; the triangular-cascade rescue is sound. But the `O(Δt)` remainder needs a bound on the **rate** `|D_{k+1}−D_k|`, whereas `eq:sat-envelope` (1388–97) and the `M_D` bound (1399–1402) constrain only the **magnitude** `|D_k|`. `d/dt(1/u_T)` is unbounded even with `|u_T|≥ε_u` under fast thrust slew — and that slew is precisely the reaching transient on a tumbling deploy, the regime the paper exists to handle. (Your own `dsmc_constraints.m:159-168` comment flags exactly this.)
  - *Action:* add a slew bound `|D_{k+1}−D_k| ≤ L·Δt` (or a rate bound on `u_T`, `ψ̇`) to **derive** the `O(Δt)` remainder, **or** carry the remainder as a bounded exogenous input in `eq:sat-ISS` and drop the frozen-spectrum claim to diagnostic-only.

- **A3 — ISS envelope verified only on winners (circularity).** Assumption (iii) `M_D=sup|D_k|<∞` is confirmed only on *successful-landing* trials (1403), which necessarily satisfy the envelope. Failing trials — which may fail *because* they leave the envelope — are never characterized.
  - *Action:* report the fraction of **all** sweep trials (not just successes) inside `eq:sat-envelope`; this kills the selection-bias reading and may even explain the failures.

- **A4 — test-vs-theory bridge (R3.2).** Gap narrowed (theory and test use the same dSMC laws; 632 ties the reaching proof to the ballistic case), but not formally closed: no ROA/basin result, only 8/17 sampled states recover, and assumption (iii) is violated exactly where they fail.
  - *Action:* add a recoverable-set / numerically-estimated-basin statement, or explicitly reframe: certificates are **local** (reaching + ISS under `eq:sat-envelope`) and 8/17 is the empirically observed basin.

- **A5 — Young's-inequality / μ, ν₂, ρ construction is SOUND.** Independently reproduced: with `η_z Δt = η_ψ Δt = 0.14` and `η_φ Δt = η_θ Δt = 0.28`, the binding `(z,ψ)` bound `0.14·(2−0.14)/(1−0.14)² = 0.352` matches the paper's `μ < 0.35` (1335); `ν₂`-bound coincides under matched gains; the "fix μ,ν₂ before ρ" ordering is justified (1349). *Minor:* `ν` (eq:sat-nu-exact) is an existence statement — `W_s, W_ξ` never pinned — so the "exact, not order-of-magnitude" claim (1371) is a certificate *in form* but not instantiated. Optional: report a concrete `(W_s, W_ξ, ρ, μ, ν₂)` tuple and the numeric `ν`.

### B. Overclaiming; theoretical-vs-empirical — 🟡 MOSTLY (E1, E3, R2.3)

- **Evidence the old overclaims are gone:** grep for `global / energy / kinetic / settling / settle` returns **zero hits**. Stability wording is now local/conditional: cSMC "asymptotically stable … exponential rate of convergence to the sliding surface" (574); dSMC "exponentially converges" *conditioned on* "if our control laws satisfy the exponential reaching law" (632); saturated dSMC "weaker input-to-state stability (ISS)" (82). Empirical results are attributed ("We find…" 82; "8 of 17 sampled states" 888; "in our simulation … the vehicle crashes" 1478).
- **B1 — the "certified" oversell (verifier overturned "addressed" → "partial").** The abstract (82) flatly calls **both** SMCs "certified through Lyapunov stability analysis" with **no discretization caveat** — yet the body concedes cSMC's certificate "does not transfer to the sampled-data implementation" (888) and cSMC recovers **0/17**. Future Work (1526) says "the stability certificates of both dSMC demonstrated on the ballistic envelope" when only 8/17 recover.
  - *A reviewer could still say:* an abstract-only reader sees cSMC and dSMC jointly "certified" and never learns cSMC recovers 0/17 or that the ISS certificate is envelope-local — exactly the established-vs-observed conflation R2 raised.
  - *Action:* reword to "two nonlinear controllers for which we derive Lyapunov-based stability conditions," and note in-sentence that the cSMC certificate is continuous-time only; soften 1526 to "certificates hold under the `eq:sat-envelope` conditions on the recovered subset (8/17)."
- **B2 — ISS presented flatly.** Abstract (82) and Contributions (96) present the ISS bound as obtained, while the body makes it conditional on assumption (iii) (1374, 1381). *Action:* scope the abstract/contributions ISS claim to the trajectory envelope.
- **B3 — performance generalization (minor, hedged).** "for off-nominal ICs and aggressive flight **in general**, the dSMC … beats the COTS controllers" (1523) and the Future-Work inference that certificates imply IC/orientation-perturbation robustness (1526) — a category error (ISS is w.r.t. the input deficit, not IC/parameter perturbations). *Action:* drop "in general" (1523); reframe 1526 as an untested hypothesis.

### C. Scope/focus — dSMC as the contribution — ⚪ DISMISSED BY AUTHOR (2026-07-17) (R2.1, R2.5, E1-scope)

*Reviewer point:* novelty is narrower than the four-controller framing suggests; R2.5 asked to move the LQR/PID/cSMC baseline descriptions into the simulation section and present the corrected dSMC as the clear center.

**Author ruling — disregard.** The current organization (baselines retained in the Controller Formulation body; cSMC kept as a reformulated contribution) is intentional and adequate for the AIAA venue. No action. *Recorded for trail; the recentering already done — dSMC-specific title (53), contributions leading with the corrected dSMC (96) — stands on its own.*

### D. Consistency, cross-refs — ✅ RESOLVED / MOOT (R2.4, R3.5)

- **3-tier/4-tier hierarchy:** removed with the swarm framing (only benign "hierarchy" at 939, re Faessler priority). Moot.
- **Mass 0.2/0.8:** consistently 0.8 kg (113); the 0.2 is arm length `l` (109), 0.1 is `k_MT` (108). No stray 0.2 kg mass.
- **Settling-time equation cross-ref:** the old KE/settling metrics (eqs 1–3) are gone; sole metric is `eq:ballistic-success`, `\ref`'d at 888 and 1476, both resolve.
- **Cross-ref/citation integrity:** 91 `\label`s, no duplicates; the 19-entry bib all resolve (consistent with the completed 2026-07-16 citation audit). No broken/wrong-numbered equation citations found.
- *Meta-note (not actionable):* these were resolved by deletion, so the paper never demonstrates cross-reference discipline was *fixed* — but the defects cannot recur.

### E. Metric definition — 🟡 PARTIAL (R3.4)

- `eq:ballistic-success` (882–888) defines `v_0`, `Θ̇_t=[θ̇ φ̇ ψ̇]`, the `π√3 ≈ 5.44 rad/s` reference, and the `t>1 s` grace window — all matching `ballistic_success.m` defaults (ReachTol=10, VelRatioMax=2, RotRatioMax=2, RotGraceT=1). **But** `x_t,y_t` (terminal position), `x_f,y_f` (target), `v_t` (terminal speed) are only inferable from prose, never bound as symbols.
  - *Action:* one clause binding those five symbols; optionally report threshold sensitivity (the reachability count is a coarse metric with no CI).

### F. Symmetric-quadrotor assumption — 🟡 PARTIAL (R2.2)

- Bare one-line assertion at 634 ("We assume … `J_xx=J_yy` … a standard simplifying assumption"), recurring only as labels (162, 903). **It does not say what the assumption enables.** Worse, the derived laws keep the inertias *distinct*: `u_2` carries `J_xx` / `u_3` carries `J_yy` (766, 774); `D_{2,2}=a₃l/J_xx`, `D_{3,3}=a₇l/J_yy` (1170–71); the `M_D` bound uses them separately (1401). Confirmed against `dsmc_constraints.m` (`D22=dt*a3*l/Jxx`, `D33=dt*a7*l/Jyy`, `u_Mx` uses `Jxx/(l*a3)`). The only genuine use is the equal roll/pitch rows of `T_M` (903, 916, "effective moment coefficient … numerically equal to `b`") — never connected to the 634 assertion.
  - *A reviewer could still say:* the equality is unnecessary and reads as a spurious restriction, inviting the exact generality question R2.2 raised.
  - *Action:* pin the assumption to its one real use (equal effective-moment coefficient on the `T_M` roll/pitch rows) and state the sliding laws, D-matrix, and ISS bound retain distinct `J_xx, J_yy` — or drop it and claim the generality.

### G. Literature / motivation / title — ✅ ADDRESSED (R4.1, R8.1)

- Recent cites in intro (86: Bouman2019, Denton2025, Harper2025, Starks2024, Liang2019, Michelena2025) and Related Work (89–93: Xiong2016, Xu2020, Kuang2024, Shao2022, Faessler2017, Mueller2013, Wang2005, JiangWang2001, Sarpturk1987). 2024–25 refs make it current. Motivation concrete (Launched Effects program). Non-inclusive title gone (53). Cooperative/swarm literature legitimately out of scope now.
  - *Optional:* one sentence deferring multi-vehicle/cooperative deployment to future work; note stronger nonlinear peers (MPC/adaptive/backstepping) are out of scope for this stabilization study.

### H. Comparison fairness / tuning-dependence — 🟡 PARTIAL (R8.2, R8.3, R4.4)

- The tuning-sensitive KE/settling metrics R8 attacked are gone, replaced by a binary envelope-recovery criterion (882–887); baseline failure argued **structural** — LQR/PID "operate too far from their hover linearization," cSMC "continuous-time stability … does not transfer to the sampled-data implementation" (888, 1523). **But** no best-effort-tuning statement: LQR `Q` is "manual tuning of the weights presented in [Tahir2020]" (261); cSMC laws "follow [Tahir2020]" (340, i.e. copied, not retuned for this test case); PID gains tabled with no fairness claim.
  - *A reviewer could still say:* the envelope separation is narrow (dSMC-sat 8/17 vs LQR 5/17), within plausible tuning variation; the structural claim is asserted, not demonstrated gain-independent, so "beats the baselines" remains vulnerable to the original tuning objection.
  - *Action:* state each baseline was tuned to its best achievable envelope-recovery (ideally a short gain-sensitivity note or a citation that the linearization/continuity breakdown is gain-independent); temper "beats" given the 8-vs-5-of-17 margin.

### I. Simulation vs. experiment — ⚪ DISMISSED BY AUTHOR (2026-07-17) (E4, R8.4)

*Reviewer point:* a comparative study with simulations only is unconvincing; justify sim-only or run real-UAV experiments.

**Author ruling — disregard.** Simulation-only is accepted for this venue and stage. The concern is further blunted because the editor tied E4 explicitly to the (now-resolved) absence of stability analysis, which is present throughout (406–574, 620–632, 1178–1403). Flight test remains future work (1526). No action.

### J/K. Model & data detail — ✅ (R4.2, R4.5)

- Full nonlinear (185–216) + linear (224–233) EOM, nomenclature table (98–155), mixing matrix. Landing results give a 4-stat table (`tab:sat_cmp_summary`), control-norm figures (TO_06), and the envelope figure (39). Could add confidence intervals / threshold sensitivity (see E), but the "too simple / too little data" complaints are largely met for the new scope.

### L. Equation font — ✅ RESOLVED (author, 2026-07-17) (R3.6)

The reviewed version used a smaller equation font; the current manuscript's display equations have already been enlarged and are now in line with a good, legible size. The remaining `\small`/`\footnotesize` usage is judged acceptable by the author. No action.

### M. Six-page limit — ⚪ MOOT (R3.3). Venue changed L-CSS → AIAA SciTech.

### N. iThenticate 17% — ⚪ NOTE. Report not downloadable; 17% is under typical (~20–25%) concern thresholds and expected for standard EOM/McCoy-ballistics/quadrotor-mixing content; the bib/citation audit is already complete (2026-07-16). No action unless the online report flags a concentrated passage.

---

## 5. Prioritized punch-list

### P1 — Do before any resubmission (cheap, language-only, highest leverage)
1. **Scope the stability claims to what's proven** (fixes A1, B1, B2 at once):
   - Abstract: "two nonlinear controllers **for which we derive Lyapunov-based stability conditions**"; note cSMC's certificate is **continuous-time only and does not survive discretization** (its own body says so at 888; it recovers 0/17).
   - State the certificates cover the **four actuated sliding variables**; **horizontal-position convergence is empirical only** (the 8/17 · 46-of-140 basin).
   - Surface the ISS **envelope-local** caveat in the abstract/conclusion, not just at 1381.
2. **Close or downgrade the frozen-D step (A2):** add a slew bound `|D_{k+1}−D_k| ≤ L·Δt`, or carry the remainder as a bounded exogenous input and mark the frozen-spectrum claim diagnostic-only.

### P2 — Strengthens methodology (a paragraph each)
3. **Kill the envelope circularity (A3):** report whether *failing* trials violate `eq:sat-envelope`.
4. **Best-effort-tuning statement (H)** for the baselines; temper "beats … in general" (1523).

### P3 — Polish (quick)
5. **Bind metric symbols** `x_t,y_t,x_f,y_f,v_t` (E).
6. **Clarify / justify or drop the symmetric assumption (F).**
7. **Separate the two denominators in the abstract** — 17-point stabilization envelope vs 140-trial landing sweep — so "recovers more of the envelope" and "fourfold, 7.9%→32.9%" aren't misread as one result.

*Dispositioned by author (2026-07-17), removed from the action list:* scope/focus relocation (§4.C), simulation-only justification (§4.I), and equation-font enlargement (§4.L, already done).

**Strategic note:** the reviews were **controls-venue** (CDC/L-CSS); the paper is now **aerospace** (AIAA SciTech), where the theory bar is a touch lower — consistent with the author dispositions of the scope, sim-only, and equation-font concerns. **P1 is venue-independent:** the "certified quantity ≠ reported quantity" mismatch is the first thing a reviewer who reads the abstract *and* the results will notice, and it's the cheapest to fix.

---

## 6. Appendix — reviewer-by-reviewer index

**Editor (`CommentsToAuthor.txt`)**
- E1 claims stronger than validated → §B (🟡); scope portion → §C (⚪ dismissed by author)
- E2 no stability analysis → §A (✅)
- E3 claimed-vs-observed unclear → §B (🟡)
- E4 subset of controllers / justify sim vs experiment → §C, §I (⚪ dismissed by author)

**Reviewer 2 (`Review22212.txt`)**
- R2.1 novelty narrower; dSMC is the contribution → §C (⚪ dismissed by author)
- R2.2 symmetric assumption unexplained → §F (🟡)
- R2.3 claims stronger than validated (energy/settling; theoretical vs empirical) → §B (🟡, energy/settling ✅)
- R2.4 consistency (3-tier/4-tier, mass 0.2/0.8, settling xref) → §D (✅/moot)
- R2.5 move baselines to sim section; dSMC as center → §C (⚪ dismissed by author)

**Reviewer 3 (`Review22213.txt`)**
- R3.1 condensed; problem formulation missing; Sec II ¶1 unclear → partially via §A/§E (setup clearer; no explicit "Problem Formulation" section)
- R3.2 gap between testing and theory → §A4 (🟡)
- R3.3 6 pages not enough → §M (moot)
- R3.4 metrics (eqs 1–3) not defined → §E (🟡)
- R3.5 typos citing equations → §D (✅)
- R3.6 equation font too small → §L (✅ resolved)

**Reviewer 4 (`Review22214.txt`)**
- R4.1 intro lacks latest results / cooperative control → §G (✅)
- R4.2 model not clear → §J (✅)
- R4.3 stability analysis missing → §A (✅)
- R4.4 simulation too simple; PID too traditional → §H, §K (🟡)
- R4.5 data should be more detailed → §K (✅ largely)

**Reviewer 8 (`Review22215.txt`)**
- R8.1 motivation weak; title non-inclusive → §G (✅)
- R8.2 comparison not justifiable (tuning-dependent) → §H (🟡)
- R8.3 Fig 3–5 not valid (tuning-dependent) → §H (🟡; those figures deleted)
- R8.4 sim only unconvincing; real experiments → §I (⚪ dismissed by author)

**`iThenticateScanResult.txt`** — 17% similarity → §N (informational).

---

## 7. Provenance & confidence

- **Numbers independently reproduced** from `constants.m` (not just cross-read): `m·l²/Jₓₓ = 0.8·0.2²/1.8e-3 = 17.78 ≈ 17.8` (1471); hover `mg/4b = 7.848/20 = 0.392` and thrust-to-weight `4b·Ω²ₘₐₓ/mg = 40/7.848 = 5.10` (916); `46/140 = 32.9%`, `11/140 = 7.9%` (82, 1478, 1513); `Ω²ₘₐₓ=2, b=5, d=2` match `dsmc_constraints.m`; `eq:ballistic-success` matches `ballistic_success.m` (ReachTol 10, ratios 2, grace 1 s); launch table matches `operational_launch.m` (Vo=94, el=64, az=15, p=−8.379). **No numeric mismatch found.**
- **Method confidence:** each lens was audited then adversarially re-verified by an independent agent that re-opened the files (did not trust the auditor's quotes); one "addressed" verdict (B1) was overturned to "partial" on re-check. Line numbers are current as of this date; re-verify after any edit pass.
- **Not covered here:** the online iThenticate passage-level report (not downloadable); a full semantic pass confirming every `\ref` points to the *intended* (not merely a valid) equation — spot-checked only.

---

## 8. Staged edits — drafted, NOT yet applied to `root.tex`

Concrete before/after text for the P1–P3 punch-list (§5). Nothing here has been written to `root.tex`; this is the review copy. Each is tagged:

- **MINOR** = camera-ready-safe for the submitted AIAA SciTech paper: pure language/framing, no new results, figures, or derivations.
- **MAJOR** = reserve for the journal: needs a new derivation, new analysis, or a re-run.

Three items (2, 3, 4) have a **minor form now** plus a **stronger major upgrade** for the journal; both are drafted so the choice is yours. All seven in their minor form together add ~90 words and change no number, figure, or proof.

The new prose was passed through `/humanizer` and `/karpathy-guidelines` — see §8.8 for what that changed and what it deliberately left alone.

### 8.1 — Item 1 (P1): Scope the stability claims to what's proven · **MINOR**

*Fixes A1 (x/y gap), B1 ("certified" oversell), B2 (flat ISS). Touches: abstract (82), a note after line 888, Future Work (1526).*

**(a) Abstract, line 82 — reword "certified", scope to actuated channels, caveat cSMC discretization + ISS envelope.**

Before:
```latex
We derive and test two nonlinear controllers certified through Lyapunov stability analysis: a continuous-time Sliding Mode controller (cSMC) and a discrete-time Sliding Mode Control (dSMC).
```
After:
```latex
We derive and test two nonlinear controllers for which we establish Lyapunov-based stability conditions: a continuous-time Sliding Mode controller (cSMC) and a discrete-time Sliding Mode Control (dSMC). These conditions govern the four actuated channels (altitude, roll, pitch, and yaw); the cSMC guarantee is continuous-time and does not survive discretization, whereas the dSMC is built directly in discrete time and holds under the sampled-data plant.
```

And, later in the same abstract:

Before:
```latex
Otherwise, we saturate the control inputs in order of priority (first yaw, then altitude/thrust, then pitch/roll), and obtain weaker input-to-state stability (ISS).
```
After:
```latex
Otherwise, we saturate the control inputs in order of priority (first yaw, then altitude/thrust, then pitch/roll), and obtain a weaker input-to-state stability (ISS) bound that holds while the trajectory stays within an explicit envelope.
```

**(b) Envelope section, insert after "…not the controller itself." (line 888), before the `Figure \ref{fig:ballistic_envelope}` sentence — the "certified quantity ≠ reported quantity" fix, stated once at the point of first exposure.**

Insert:
```latex
The criterion is scored on horizontal position, whereas our stability analysis certifies the four actuated sliding variables (altitude, roll, pitch, and yaw). Horizontal $(x,y)$ motion rides on the underactuated attitude channels; its convergence is shown empirically here, not proven.
```

**(c) Future Work, line 1526 — remove the category error (ISS is w.r.t. the input deficit, not IC/parameter perturbations) and the 8/17-vs-"demonstrated" overclaim.**

Before:
```latex
However, the stability certificates of both dSMC demonstrated on the ballistic envelope suggests that minor velocity and orientation perturbations won't destabilize the controller.
```
After:
```latex
The dSMC's stability certificates concern the sliding variables and the saturation deficit, not initial-condition or parameter perturbations. Whether the recovered subset (8 of 17 sampled states) also tolerates small velocity and orientation perturbations is an open question we plan to test.
```

### 8.2 — Item 2 (P1): Close or downgrade the frozen-D step · **MINOR** (with a **MAJOR** journal upgrade)

*Fixes A2. The `O(\Delta t)` remainder claim silently needs a bound on the step-to-step **rate** `|D_{k+1}-D_k|`, but `eq:sat-envelope` bounds only the **magnitude** `|D_k|`. Under fast thrust slew `d/dt(1/u_T)` is unbounded even with `|u_T| \ge \epsilon_u` — exactly the reaching transient on a tumbling deploy.*

**MINOR — make the slew bound an explicit hypothesis (the same move the paper already makes for `M_D` as assumption (iii)). Line 1250, replace the tail of the paragraph:**

Before:
```latex
plus the remainder $(D_k - D_{k+1})\, \xi_{k+1}$ from the $D_{k+1} \approx D_k$ step, which is likewise $O(|\xi_k|)$ and vanishes with $\xi$. The ISS-Lyapunov inequality of Section~\ref{sec:sat-iss} is the formal certificate of this claim, modulo the same slowly-varying-$D$ step: with an active deficit the remainder is $O(|\xi_k| + |\Delta u_k|)$ and can be carried as an additional input in Equation~\ref{eq:sat-ISS}, and under the envelope of Equation~\ref{eq:sat-envelope} its coefficient is $O(\Delta t)$ relative to $M_D$, perturbing $\nu$ and $\gamma$ only at higher order. The spectrum discussion above is diagnostic.
```
After:
```latex
plus the remainder $(D_k - D_{k+1})\, \xi_{k+1}$ from the $D_{k+1} \approx D_k$ step. Bounding this term needs the step-to-step \emph{variation} of $D_k$, not just its magnitude $M_D$, so we add to the envelope of Equation~\ref{eq:sat-envelope} a slew bound $|D_{k+1} - D_k| \leq L\, \Delta t$ along closed-loop trajectories. This holds whenever the thrust and attitude rates that drive $D_k$ are bounded, which the reaching transient satisfies once the fully-actuated variables enter the clip band. Under it the remainder is $O(L\,\Delta t\,|\xi_k|)$ and vanishes with $\xi$. The ISS-Lyapunov inequality of Section~\ref{sec:sat-iss} is the formal certificate: with an active deficit the remainder is $O(|\xi_k| + |\Delta u_k|)$, carried as an additional input in Equation~\ref{eq:sat-ISS}, and the slew bound makes its coefficient $O(L\,\Delta t)$, perturbing $\nu$ and $\gamma$ only at higher order. The frozen-time spectrum is therefore diagnostic; the ISS inequality, under Equation~\ref{eq:sat-envelope} and this slew bound, is the operative certificate.
```

**MAJOR (journal) — derive `L` from the closed-loop dynamics: bound `|\dot u_T|`, `|\dot\psi|` from the reaching law + envelope, and turn `L` into an explicit function of the gains and `\epsilon_u,\epsilon_a`, rather than a stated constant.** New analysis; hold for the journal.

### 8.3 — Item 3 (P2): Kill the envelope circularity · **MINOR** (acknowledge) or **MAJOR** (quantify)

*Fixes A3. Assumption (iii) is confirmed only on **successful** trials (1403), which satisfy the envelope by construction. Failing trials — which may fail **because** they leave the envelope — are never characterized.*

**MINOR — convert the hidden circularity into a stated limitation. Append to line 1403 (after "…never approaching zero thrust."):**

Insert:
```latex
This check is one-sided: successful trials satisfy Equation~\ref{eq:sat-envelope} by construction, so the figure confirms the envelope is non-vacuous on the recovered set but does not establish that failures are envelope violations. Characterizing where failing trials leave the envelope, and whether the envelope predicts the failure, is left to future work.
```

**MAJOR (journal) — actually compute it from the existing sweep log (no re-simulation): over all 140 trials, the fraction of failing trials that violate at least one condition of `eq:sat-envelope`, and which condition binds. Replaces the acknowledgment with a claim that the envelope separates recoveries from failures. Draft target text:**
```latex
Across all 140 sweep trials, every successful landing satisfies Equation~\ref{eq:sat-envelope}, and <X>\% of the failing trials violate at least one envelope condition (most often $|u_T| \geq \epsilon_u$ during the reaching transient), so the envelope separates recoveries from failures rather than labeling only the winners.
```
`<X>` comes from `logs/trajectory_optimization_log_sat_on.mat` — a quick post-hoc pass over each failing trial's `u_T`/attitude trace. If the number turns out *not* to cleanly separate, the minor acknowledgment above is the honest fallback.

### 8.4 — Item 4 (P2): Best-effort-tuning statement + temper "beats … in general" · **MINOR** (with a **MAJOR** journal upgrade)

*Fixes H / B3. The tuning-fairness objection (R8) is defused structurally in the text, but there is no statement that the baselines were tuned in good faith, and the Conclusion's "beats … in general" overclaims a narrow 8-vs-5/17 margin.*

**(a) MINOR — best-effort-tuning note. Append to line 888 (after "…does not transfer to the sampled-data implementation."):**

Insert:
```latex
Each baseline uses gains tuned for stable hover and step tracking (Tables~\ref{tab:pid-gains} and \ref{tab:smc-pid-gains}, and the LQR weights of Equation~\ref{eq:lqr}), not detuned to favor the dSMC; the failures here are structural — a linearization or continuity assumption broken by the tumbling deploy — rather than an artifact of gain choice. We did not exhaustively search each baseline's gain space, so we report the separation as observed on this envelope rather than as a gain-independent ranking.
```

**(b) MINOR — temper the Conclusion. Line 1523 (this sentence also carries Item 1's certificate-scope; combined rewrite):**

Before:
```latex
For launched-drone operations, or for off-nominal initial conditions and aggressive flight in general, the dSMC formulation here beats the COTS controllers and, unlike cSMC, keeps its stability certificates under our discretized control plant.
```
After:
```latex
For launched-drone operations, and for the off-nominal initial conditions and aggressive flight tested here, the dSMC formulation recovers more of the deployment envelope than the COTS controllers and, unlike cSMC, keeps its stability certificates under our discretized plant. These certificates cover the four actuated channels, and under saturation they hold while the trajectory stays inside the envelope of Equation~\ref{eq:sat-envelope}.
```

**MAJOR (journal) — a gain-sensitivity sweep: perturb the LQR `Q`, PID gains, and cSMC `\gamma` over a plausible range, re-run the 17-point envelope, and show the dSMC advantage persists. This is the real answer to R8's tuning objection and needs re-simulation; hold for the journal.**

### 8.5 — Item 5 (P3): Bind the metric symbols · **MINOR**

*Fixes E. `eq:ballistic-success` uses `x_t,y_t,x_f,y_f,v_t` but only `v_0` and `\dot\Theta_t` are defined; the rest are inferable from prose only.*

**Insert immediately after `eq:ballistic-success` (line 886), before "The first condition…":**

Insert:
```latex
Here $(x_f, y_f)$ is the target landing point and $v_0$ the deployment speed; $(x_t, y_t)$, $v_t$, and $\dot\Theta_t$ are the vehicle's horizontal position, speed, and Euler-angle rate at time $t$ within the stabilization window. The position bound is applied at the final time; the speed and rate bounds hold throughout the window, the rate bound only for $t > 1$\,s.
```
(Matches `ballistic_success.m`: terminal XY, whole-trace velocity/rotation ratios, 1 s rotation grace.)

### 8.6 — Item 6 (P3): Clarify the symmetric-quadrotor assumption · **MINOR**

*Fixes F. The bare assertion at 634 is never used where stated — the SMC/dSMC laws, `D_k`, and the `M_D` bound all keep `J_xx ≠ J_yy`. Pin it to its real use and state the derivation is inertia-general.*

**Line 634, replace:**

Before:
```latex
We assume that the quadrotor is symmetric, and so $J_{xx} = J_{yy}$. This is a standard simplifying assumption and holds for almost all commercial quadrotors.
```
After:
```latex
Our test vehicle is symmetric in the rotor plane, so $J_{xx} = J_{yy}$ numerically, as for almost all commercial quadrotors. The derivation itself does not require this: the sliding-mode laws, the deficit-coupling matrix $D_k$, and the ISS bound all keep $J_{xx}$ and $J_{yy}$ distinct. Symmetry enters only as the equal roll- and pitch-axis coefficients of the mixing matrix and, numerically, as the equal inertias of our test case; an asymmetric vehicle would change those values but not the structure of the analysis.
```

### 8.7 — Item 7 (P3): Separate the two denominators in the abstract · **MINOR**

*The "recovers more of the envelope" claim is the 17-point stabilization envelope (Fig. 39); the "7.9%→32.9%" claim is the 140-trial landing sweep (Table 7). An abstract-only reader can misread them as one result.*

**(a) Abstract sentence 4 — tag the envelope denominator.**

Before:
```latex
We find that the dSMC recovers more of the ballistic-deployment envelope than the LQR, PID, or continuous-time controllers, but that clipping the motor speeds to their physical limits reduces the recoverable set.
```
After:
```latex
Sampling 17 deployment states along one ballistic trajectory, we find the dSMC recovers from more of them than the LQR, PID, or continuous-time controllers, though clipping the motor speeds to their physical limits shrinks its recoverable set.
```

**(b) Abstract final sentence — tag the sweep denominator.**

Before:
```latex
However, despite this weaker stability, this priority-weighted saturation increases average landing-site reachability roughly fourfold, from $7.9\%$ (per-motor clipping) to $32.9\%$ (saturation algorithm).
```
After:
```latex
However, despite this weaker stability, on a separate 140-trial landing sweep the priority-weighted saturation raises average landing-site reachability roughly fourfold over per-motor clipping, from $7.9\%$ to $32.9\%$.
```

### 8.8 — Summary + writing-quality pass

| Item | §5 | Tier | `root.tex` touches | Needs |
|---|---|---|---|---|
| 1 | P1.1 | **MINOR** | abstract 82; after 888; 1526 | language |
| 2 | P1.2 | **MINOR** (MAJOR upgrade) | 1250 | stated slew bound · journal: derive `L` |
| 3 | P2.3 | **MINOR** now / **MAJOR** opt | after 1403 | acknowledge · journal: quantify from log |
| 4 | P2.4 | **MINOR** now / **MAJOR** opt | after 888; 1523 | temper + tuning caveat · journal: gain sweep |
| 5 | P3.5 | **MINOR** | after 886 | language |
| 6 | P3.6 | **MINOR** | 634 | language |
| 7 | P3.7 | **MINOR** | abstract 82 | language |

- **Conference (minor) set:** all seven in their minor form — camera-ready-safe, no new results/figures/derivations.
- **Journal (major) upgrades:** Item 2 (derive `L`), Item 3 (quantify envelope failures), Item 4 (gain-sensitivity study). Each needs new analysis or a re-run.
- **Note:** the abstract is edited by Items 1 + 7 and the Conclusion (1523) by Items 1 + 4 — the After text above is written to compose cleanly, but apply those two locations from the combined versions to avoid conflicting edits.

**`/humanizer` pass.** Kept the author's actual voice (first-person-plural "We", em-dashes where the paper already uses them, plain `is/are` copulas). Removed/avoided AI tells: no significance inflation, no "-ing" pseudo-depth, no hedge-stacking, no rule-of-three padding, straight quotes in inserts. One active de-AI edit: the Conclusion certificate clause was drafted as a participle stack ("certificates covering… holding…") and rewritten to finite verbs ("these certificates cover… they hold…"). The lists that remain (e.g. "altitude, roll, pitch, and yaw") are the four real channels, not decorative tricolons.

**`/karpathy-guidelines` pass.** Surgical: every insertion traces to one punch-list item; no adjacent prose "improved," no refactoring of untouched equations. Simplicity: each edit is the minimum that closes the gap (Item 2 states a hypothesis rather than proving `L`; Item 6 drops a false claim rather than adding a theorem). Tradeoffs surfaced, not hidden: Items 2/3/4 carry both a minor and a major path instead of silently picking one. Verified all `\ref`/`\label` targets exist in `root.tex`.
