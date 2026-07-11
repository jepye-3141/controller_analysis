# Citation Audit — AIAA SciTech Manuscript (`docs/scitech-paper/root.tex`)

**Date:** 2026-07-07
**Auditor:** Independent web-source verification pass
**Scope:** Every reference in `bibliography/aiaa_refs.bib` (30 entries), checked against its actual use in `root.tex`.
**Manuscript state audited:** `root.tex` last modified 2026-07-06; `root.aux`/`root.bbl` regenerated 2026-07-06. This audit reflects the *current* file, in which the earlier Sarpturk / Michelena / Shao / Claude fixes are already applied.

---

## 0. Addendum — 2026-07-08 (follow-up: PDFs verified + edits applied)

After the initial pass, the author supplied the three paywalled PDFs and authorized four edits. This section records the resulting changes; the per-reference sections below (§4.6, §4.8, §4.17) and §6 are annotated accordingly.

**Three residual items — now confirmed from the primary-source PDFs (all support the manuscript):**

- **Sarpturk1987** — ✅ Confirmed verbatim. The paper's **eq (7) `|sᵢ(k+1)| < |sᵢ(k)|`** is stated as "a necessary and sufficient condition … assuring both sliding motion and convergence"; eqs (11)/(13) give the two-sided control bound ("the input must lie on an open interval rather than a half-line"). Sarpturk states **no** `ηΔt<2` form, so the manuscript's scoped attribution (existence condition only, with `ηΔt<2` presented as a derived consequence) is exactly correct.
- **Xu2020** — ✅ Confirmed. Eqs (15)–(16): auxiliary states **`xᵢ, i ∈ {φ,θ,ψ}`** (`ẋᵢ = −ςᵢxᵢ − … − Δuᵢ`, controller `ūᵢ = Θᵢ − κᵢxᵢ`) — genuinely **per-channel across the three attitude moments**, and attitude-only. The manuscript's L89/L1024 description is precise; the earlier "abstract-limited" caveat is discharged.
- **Shao2022** — ✅ Confirmed. Definition 1 (`Λ_out`, bounded to `[xlb,xub]`, `|Λ_out(x)|≤|x|`) and Definition 2 (`Γsat(x) ∈ [−1,1]`, strictly bounded) are embedded directly in the ASMCSAT adaptive control law, alongside adaptive parameters (`γᵢ`, `ϑ`). The manuscript's L89 description is supported at the primary-source level.

**Net effect:** every one of the 19 cited references is now verified against a real source, with the two most load-bearing (Xiong2016, JiangWang2001) and these three all confirmed from primary full text. Nothing in the audit now rests on abstract-only evidence.

**Four edits applied to `root.tex` (2026-07-08):**

| Site | Before → After | Basis |
|------|----------------|-------|
| L340 (Tahir) | "a single **PID** layer" → "a single **PD** layer" | Tahir's outer loop is PD ("SMC PD," no integral). |
| L86 (Starks) | "a role **tested in some pilot programs**" → "a role **examined in feasibility modeling studies**" | Starks2024 is a registry-based modeling study, not a field pilot. |
| L86 (Denton) | "transition into powered flight **shortly after leaving the barrel**" → "transition into powered flight **at a predetermined point in their ballistic trajectory**" | Fits both SQUID (pre-apogee) and the TLMAV (near apogee/target); also sharpens the contrast with "any point along their trajectory." |
| L930 (Mueller) | "…above yaw, **a position supported by** Mueller and D'Andrea's work on trajectory interception." → "…above yaw; **the low priority of yaw is consistent with** Mueller and D'Andrea's treatment of yaw rotation as dynamically inessential for trajectory interception." | Mueller supports only the yaw-deprioritization (assumes ω₃≈0), not the full roll/pitch≻thrust≻yaw ranking (which Faessler carries). |

**Author decision:** **Harper2025** (§4.13) is retained as-is at the author's discretion.

**New item flagged (not edited — it concerns the author's own controller, not a citation):** with the L340 fix in place, the sentence reads "Tahir uses a single **PD** layer … we use two discrete **PID** layers," while the following sentence and Table 3 both say "**PD** control layer." Decide whether your own cascade uses integral action; if not, "two discrete **PD** layers" would make the passage internally consistent.

---

## 1. Method and access notes

For each cited key I (a) extracted the exact in-context claim from `root.tex` (line numbers given), (b) located the real source on the web, and (c) judged whether the source supports the claim. I did **not** pre-judge — a citation is only marked down when the source itself fails to support the stated use.

**Access levels achieved:**

- **Primary full text read:** Xiong2016 and JiangWang2001 (both Elsevier/ScienceDirect) were read in full through the connected browser session, which had full-text access. This is important: **the two most load-bearing references — including the exact equations and the exact numbered Definition/Example — were verified against the primary source, not a proxy.**
- **Open-access full text:** Kuang2024 (MDPI), Tahir2020 (IEEE Access), Wang2005 (IFAC-2005 proceedings PDF), Bouman2019 (arXiv), Denton2025 (Sage OA), Michelena2025 (army.mil), Harper2025 (DefenseScoop), Mueller2013 (author PDF), Faessler2017 (author thesis + slides + third-party corroboration).
- **Author-supplied PDFs, primary full text read (2026-07-08):** Xu2020, Shao2022, Sarpturk1987 (these were abstract-only / secondary-corroboration in the initial pass; now confirmed from primary text — see §0).
- **Abstract-only (body paywalled, not bypassed):** Starks2024, Liang2019 (abstracts sufficient for the verdicts).
- **Book (TOC + published errata):** mccoy1999modern.

**Citation inventory:** 30 bib entries. **19 are cited** — 48 `\cite` commands / 50 key-occurrences, confirmed by parsing `root.tex` and cross-checked against `root.aux`/`root.bbl`; every cited key resolves to a bib entry (no broken references). **11 are not cited anywhere** and therefore do not appear in the compiled bibliography (see §5).

---

## 2. Executive summary

**The bibliography is in good shape.** The paper's central technical reference (Xiong2016, 11 citations) is quoted **accurately at the equation level**, which is the single most important result of this audit. Of the 19 cited works, 14 are fully supported, 3 are supported with minor/nuance caveats, and **2 carry substantive issues** (one wording error, one source–claim mismatch). As of the 2026-07-08 follow-up (§0), every cited reference is verified against a real source and nothing rests on abstract-only evidence in a way that is contested; four edits have been applied and one (Harper2025) retained by author choice.

### Priority action list

> **Status (2026-07-08):** items 2–5 (Tahir, Starks, Denton, Mueller) have been **applied** to `root.tex`; item 1 (Harper2025) is **retained** at the author's discretion; item 6 (uncited entries) is still open. See §0 Addendum for the exact edits.

| # | Key | Issue | Severity | Recommended action |
|---|-----|-------|----------|--------------------|
| 1 | **Harper2025** | Source is about LASSO kamikaze/loitering munitions; "Launched Effects" appears once as a budget line. It does **not** describe the "flexible / deploy-anywhere / minimal-infrastructure / arbitrary-payload" platform claim hung on it (L86). | **High** | Recharacterize the sentence or cite an actual Launched Effects program description (PEO Aviation / DEVCOM). |
| 2 | **Tahir2020** | L340 says Tahir "uses a single **PID** layer to produce the desired Euler angles." Tahir's outer loop is **PD** (proportional-derivative, no integral); the paper calls its method "SMC **PD**" throughout. | **Medium** | Change "PID" → "PD" at L340. |
| 3 | **Starks2024** | Cited (L86) as a role "tested in some pilot programs." It is a **registry-based modeling/estimation study** (CARES 2013–2019), not a field pilot; its own conclusion says "Implementation studies are needed." | **Medium** | Soften to "feasibility/modeling study," or cite a real field pilot (e.g. Swedish drone-AED trials). |
| 4 | **Mueller2013** | Correctly supports "linearize input constraints inside an MPC loop" (L91), but at L930 it is cited as *support for the roll/pitch ≻ thrust ≻ yaw priority ranking*, which Mueller does not establish (it only assumes yaw ω₃≈0 as a trajectory-planning simplification). | **Low–Med** | Keep Mueller for the MPC/constraint point; drop or soften it as support for the priority ranking (Faessler2017 already carries that). |
| 5 | **Denton2025** | Supported, but the shared phrase "transition into powered flight **shortly after leaving the barrel**" (L86) fits SQUID better than the TLMAV, which powers up near apogee/target. | **Low** | Optional wording softening. |
| 6 | 11 uncited entries | Present in `.bib`, never `\cite`d. | Housekeeping | Decide: cite or delete (see §5). |

Everything else (Xiong2016, JiangWang2001, Wang2005, Sarpturk1987, Faessler2017, Kuang2024, Xu2020, Shao2022, Bouman2019, Liang2019, Michelena2025, mccoy1999modern, github, claude) is used appropriately.

---

## 3. Summary table — all 19 cited references

Strength = how well the source supports the specific claim. Appropriateness = whether it is used correctly in context.

| Key | Cites | Access | Strength | Appropriateness | Verdict |
|-----|:----:|--------|----------|-----------------|---------|
| Xiong2016 | 11 | **Primary full text** | Strong | Appropriate | ✅ Verified at equation level |
| Tahir2020 | 7 | OA full text | Strong | 1 wording error (PID→PD) | ⚠️ Fix wording |
| Wang2005 | 4 | Primary full text | Strong | Appropriate | ✅ Verified verbatim |
| JiangWang2001 | 4 | **Primary full text** | Strong | Appropriate | ✅ Verified incl. exact labels |
| Faessler2017 | 3 | Full (thesis/slides/3rd-party) | Strong | Appropriate | ✅ Supported |
| Xu2020 | 3 | **Primary full text** | Strong | Appropriate | ✅ Fully supported (per-channel confirmed) |
| Kuang2024 | 3 | OA full text | Strong | Appropriate | ✅ Verified verbatim |
| Sarpturk1987 | 2 | **Primary full text** | Strong | Appropriate (correctly scoped) | ✅ Confirmed verbatim (eq 7) |
| Mueller2013 | 2 | Author full text | Mixed | Over-extended at L930 | ⚠️ Decouple from priority ranking |
| mccoy1999modern | 2 | TOC + errata | Strong | Appropriate | ✅ Supported |
| Bouman2019 | 1 | arXiv full text | Strong | Appropriate | ✅ Supported |
| Denton2025 | 1 | OA full text | Strong | Minor phrasing nuance | ✅ Supported (soften shared phrase) |
| Harper2025 | 1 | Full text | Weak | **Mismatch** | ❌ Source does not support claim |
| Starks2024 | 1 | Abstract only | Weak | **Overstated** | ⚠️ "pilot program" overstates a modeling study |
| Liang2019 | 1 | Abstract only | Moderate | Appropriate | ✅ Supported (paraphrase) |
| Michelena2025 | 1 | Full text | Strong | Appropriate | ✅ Supported |
| Shao2022 | 1 | **Primary full text** | Strong | Appropriate | ✅ Confirmed (Defs 1–2) |
| github | 1 | Verified live | Strong | Appropriate | ✅ Resolves, matches project |
| claude | 1 | n/a | n/a | Appropriate | ✅ Tool acknowledgment |

---

## 4. Detailed findings

### 4.1 Xiong2016 — core reference — ✅ VERIFIED AT EQUATION LEVEL

**Source:** J.-J. Xiong, G. (Guobao) Zhang, "Discrete-time sliding mode control for a quadrotor UAV," *Optik* 127(8):3718–3722, 2016, doi 10.1016/j.ijleo.2016.01.010. Full text read via browser (ScienceDirect PII S003040261600022X).

This is the paper's foundational reference and the target of its "corrected derivation" contribution, so it received the deepest check. Every claim was verified against Xiong's actual text and equations.

| Manuscript claim | Line(s) | Xiong source | Verdict |
|---|---|---|---|
| "introduced the unconstrained formulation of the dSMC … discrete Lyapunov stability condition … for arbitrary discretization sampling times" | L89 | Abstract: "linear extrapolation … transform the continuous-time system into discrete-time system … new conditions are given ensuring the discrete-time system is asymptotically stable." Stability condition `2 − ηᵢΔt > 0` present in body. | ✅ Supported |
| discrete two-step model, "K₁…K₆ are per-axis drag coefficients" | L577, L1071 | Xiong eq (1) is the continuous model with drag terms −K₁ẋ/m … −K₆ψ̇/J_z (K₁–K₃ translational, K₄–K₆ rotational); the second-order "linear extrapolation" discretization matches the manuscript's `x_{k+2}=2x_{k+1}−x_k−…` form. | ✅ Supported |
| "problem setup … sliding surface of the fully actuated subsystem (z and ψ)" | L602 | Xiong eqs (4)–(5): `s_{z,k}=a_z(z_k^d−z_k)+(ż_k^d−ż_k)`, `s_{ψ,k}=a_ψ(ψ_k^d−ψ_k)+(ψ̇_k^d−ψ̇_k)` — identical structure. | ✅ Supported |
| "Lyapunov stability requirements presented in [Xiong]" (exponential reaching law) | L620 | Xiong eqs (8)–(9): `s_{i,k+1}−s_{i,k}=(−ηᵢs_{i,k})Δt`, plus `V=½s²`, `ΔV<0`. | ✅ Supported |
| **[Xiong] "proposed the following control laws"** — u₁,u₄ with η·s in the **denominator** | **L676–682** | **Xiong eq (10):** `u₁,k = [m/(cosφ_k cosθ_k)]·( [−a_z ż_k + g + K₃ ż_k] / [m + η₁ s_{z,k}] )`. **Xiong eq (11):** `u₄,k = I_z·( [−a_ψ ψ̇_k + K₆ ψ̇_k] / [I_z + η₂ s_{ψ,k}] )`. | ✅ **Faithful quotation** (α_z→a_z, η_z→η₁, η_ψ→η₂, J_zz→I_z) |
| "the control laws proposed in [Xiong] violate the exponential reaching condition" | L96, L701 | Manuscript's own derivation (L689–699), applied to the correctly-quoted equations. | ✅ Critique aimed at the real equations |

**Key result.** The manuscript reproduces Xiong's proposed altitude/yaw control laws *exactly*, including the unusual placement of the reaching gain `η·s` in the **denominator** `(m + η₁ s_z)`. The earlier internal worry (that a denominator form is atypical and might be a mis-transcription) is **resolved: Xiong really wrote it that way.**

**Corroborating detail worth noting in the paper.** Xiong is *internally inconsistent*: his **underactuated** laws put the reaching gain in the **numerator** — eq (9)/(10): `u₂,k = (I_x/(l a₃))·[ … − a₄ φ̇_k + η₃ s_{φ,k} ]`, `u₃,k = (I_y/(l a₇))·[ … − a₈ θ̇_k + η₄ s_{θ,k} ]` — while his **fully-actuated** laws put it in the **denominator** (eqs 10–11 above). This numerator-vs-denominator split within Xiong's own paper is independent evidence that the fully-actuated forms are erroneous, and it materially strengthens the manuscript's "corrected derivation" contribution. Consider citing it explicitly.

**One fairness note (not a citation defect).** Xiong's paper *claims* his laws are stable. The manuscript asserts the opposite. That is a legitimate correction claim, and since the quoted equations are verified accurate, the disagreement is substantive rather than a misreading. The paper may wish to state plainly that this contradicts Xiong's own stability claim — which is precisely the point of a corrected re-derivation.

---

### 4.2 Tahir2020 — ⚠️ ONE WORDING ERROR (PID → PD)

**Source:** A. Tahir, J. M. Böling, M.-H. Haghbayan, J. Plosila, "Comparison of Linear and Nonlinear Methods for Distributed Control of a Hierarchical Formation of UAVs," *IEEE Access* 8:95667–95680, 2020, doi 10.1109/ACCESS.2020.2988773. Open-access full text read (UTUPub mirror).

| Manuscript claim | Line(s) | Verdict |
|---|---|---|
| "standard nonlinear and linear system of equations presented in [Tahir]" | L163 | ✅ Tahir eqs (1)–(6) = 12-state nonlinear model; eqs (14)–(15) = linearization; identical state/control ordering. |
| LQR with tuned Q/R weights | L261 | ✅ Tahir §IV-B, eq (28): "standard LQR augmented with integral action," Q/R diagonal tuning; Appendix B lists Q, R=I₄. (Also a 15-state augmented LQR — matches the manuscript's LQR.) |
| reformulated cSMC "similar to the form derived in [Tahir]" | L96 | ✅ Tahir §IV-A: `s_α=c_α e_α+ė_α`, `u_α=û_α−K_α sat(s_α/β_α)`, `V=½s²`, `V̇=−η|s|`. |
| **"[Tahir] uses a single PID layer to produce the desired Euler angles"** | **L340** | ❌ **Tahir uses PD, not PID.** Abstract: "Sliding Mode Control connected with **Proportional-Derivative controller**"; method named "**SMC PD**"; the Euler-angle-reference layer (§IV-A-1, eq 18) is "three **PD** controllers," gains K_P, K_D only, **no integral term**. |
| "results of [Tahir] hold only for a limited range of initial conditions" | L96 | ✅ Fair. Tahir is linearized about hover, tested only near-hover (drones ~1 m apart; step/figure-8/spiral); explicit narrow SMC-PD stability caveats. Extreme ballistic-tumble ICs are outside its tested envelope. |
| "violates the continuous-dynamics assumption of [Tahir]" | L577 | ✅ Tahir's SMC and LQR are continuous-time designs (continuous sliding surfaces, `V̇`, continuous cost integral). |

**Action:** change "single **PID** layer" → "single **PD** layer" at L340. Everything else about Tahir is accurate. (The "single layer" abstraction is fine — Tahir's outer loop is one PD layer of three per-axis PD controllers.)

---

### 4.3 Wang2005 — ✅ VERIFIED VERBATIM

**Source:** Y. Wang, Y.-Y. Cao, Y.-X. Sun, "Stability Analysis and Anti-Windup Design for Discrete-Time Systems by a Saturation-Dependent Lyapunov Function Approach," *Proc. 16th IFAC World Congress* 38(1):759–764, 2005. Full text read (IFAC-2005 proceedings PDF).

- "discrete-time component-wise saturation model" (L93, L916) — ✅ §2: `x(k+1)=Ax(k)+Bσ(u(k))`, `σ(uᵢ)=sign(uᵢ)min{1,|uᵢ|}` applied element-wise.
- "saturation-dependent Lyapunov approach" (L93) — ✅ §3 title "A Saturation-Dependent Lyapunov Function": `V=xᵀP(η(k))x`.
- "anti-windup compensator … auxiliary state driven by the saturation deficit σ(u)−u" (L1016) — ✅ §5, verbatim: compensator `x_c(k+1)=A_c x_c+B_c y+E_c(σ(u(k))−u(k))`.

**Non-issue for the record:** the manuscript's own deficit convention is `Δu=u−ū = −(σ(u)−u)`; the sign differs from Wang's but the attribution is correct and the manuscript states its own convention explicitly.

---

### 4.4 JiangWang2001 — ✅ VERIFIED INCLUDING EXACT LABELS

**Source:** Z.-P. Jiang, Y. Wang, "Input-to-State Stability for Discrete-Time Nonlinear Systems," *Automatica* 37(6):857–869, 2001, doi 10.1016/S0005-1098(01)00028-0. Full text read via browser (ScienceDirect PII S0005109801000280).

- "discrete-time ISS-Lyapunov framework" (L93, L1262) — ✅ This is the foundational discrete-time ISS paper; §3.1 "ISS and ISS–Lyapunov functions."
- **"cf. Jiang and Wang, Definition 3.2 and Example 3.4 … ISS-Lyapunov inequality in the standard form" (L1370)** — ✅ **Exact labels confirmed against the journal text:**
  - **Definition 3.2**: "A continuous function V:Rⁿ→R≥0 is called an iss–Lyapunov function … α₁(|ξ|)≤V(ξ)≤α₂(|ξ|) … V(f(ξ,μ))−V(ξ)≤−α₃(|ξ|)+σ(|μ|)" — i.e., exactly the standard-form ISS-Lyapunov inequality the manuscript invokes.
  - **Example 3.4**: "specialize … to linear discrete-time systems x(k+1)=Ax(k)+Bu(k) where A is a Schur matrix …" — the quadratic ISS-Lyapunov construction for a Schur linear system.

The `cf.` citation is precise. (Note: the freely-available 1999 IFAC *conference* version numbers the same definition "2.2" and has no numbered Example 3.4; the manuscript correctly cites the *journal* numbering.)

---

### 4.5 Faessler2017 — ✅ SUPPORTED

**Source:** M. Faessler, D. Falanga, D. Scaramuzza, "Thrust Mixing, Saturation, and Body-Rate Control for Accurate Aggressive Quadrotor Flight," *IEEE RA-L* 2(2):476–482, 2017, doi 10.1109/LRA.2016.2640362. Verified via the authors' PhD thesis (reproduces this as "Paper G"), the official slide deck, the indexed abstract, and a third-party paper (Smeur et al., IMAV 2017) that reimplements the method.

- Priority order roll/pitch ≻ collective thrust ≻ yaw under saturation (L91, L930) — ✅ Abstract: "guarantees … the desired roll and pitch torques and the desired collective thrust, but not the desired yaw torque … Yaw torque is given least priority." Thesis §G.5 saturates yaw first, then thrust, preserving roll/pitch.
- Iterative mixer (L91, L939) — ✅ Thesis §G.4 "Iterative Thrust Mixing"; slide 9 "Iterate."
- No formal stability guarantee (L91, L939) — ✅ Authors hedge ("may prevent unstable behavior"); Smeur calls it "a heuristics based algorithm." No closed-loop stability proof.

Minor precision note (not a defect): in Faessler the iterative mixer (§G.4) and the prioritizing-saturation scheme (§G.5) are two stages of one module; "build the priority into an iterative mixer" lightly conflates them but is accurate as a summary.

---

### 4.6 Xu2020 — ✅ SUPPORTED (per-channel confirmed from PDF, 2026-07-08)

**Source:** G. Xu, Y. Xia, D.-H. Zhai, D. Ma, "Adaptive Prescribed Performance Terminal Sliding Mode Attitude Control for Quadrotor Under Input Saturation," *IET CTA* 14(17):2473–2480, 2020, doi 10.1049/iet-cta.2019.0488. Primary full text read (author-supplied PDF).

- "attitude-only … SMC under input saturation … proved closed-loop stability via Lyapunov" (L89, L1024) — ✅ Abstract: "attitude tracking problem of quadrotor … input saturation … an auxiliary system is designed … Lyapunov stability … closed-loop system." The attitude-only scope cleanly contrasts with Kuang's both-loops design, so the manuscript's framing is sound.
- "**per-channel** auxiliary states for the **three** attitude moments" (L89) — ✅ **Confirmed from the PDF (2026-07-08).** Eqs (15)–(16): auxiliary states `xᵢ, i ∈ {φ,θ,ψ}` with `ẋᵢ = −ςᵢxᵢ − … − Δuᵢ` and controller `ūᵢ = Θᵢ − κᵢxᵢ` — one scalar auxiliary state per attitude channel, attitude-only. The "per-channel across the three attitude moments" description is exact.

---

### 4.7 Kuang2024 — ✅ VERIFIED VERBATIM

**Source:** J. Kuang, M. Chen, "Adaptive Sliding Mode Control for Trajectory Tracking of Quadrotor UAVs Under Input Saturation and Disturbances," *Drones* 8(11):614, 2024, doi 10.3390/drones8110614. MDPI open-access full text read (with equations).

"vector-valued second-order auxiliary controller for the position and attitude loops … disturbance observer and adaptive laws for unknown mass and inertia" (L89, L1024) — ✅ every element confirmed:
- Position aux, eq (17): `β̇₁=−h₁β₁+β₂`, `β̇₂=−h₂β₂−β₁+ĝ_δΔu_δ`, `βⱼ∈R³` → vector-valued, second-order, position loop.
- Attitude aux, eq (35): same structure, `βⱼ=[βⱼφ βⱼθ βⱼψ]ᵀ` → attitude loop (so **both** loops).
- Disturbance observer (Lemma 1 / eqs 9,15,33); adaptive law for **mass** (eq 23, estimate of inverse mass) and **inertia** (eq 40, estimate of inverse inertia).

The manuscript's contrast (Kuang's vector-valued second-order subsystem vs. the manuscript's one scalar state per channel) is exact.

---

### 4.8 Sarpturk1987 — ✅ SUPPORTED (attribution correctly scoped)

**Source:** S. Z. Sarpturk, Y. Istefanopulos, O. Kaynak, "On the Stability of Discrete-Time Sliding Mode Control Systems," *IEEE TAC* 32(10):930–932, 1987, doi 10.1109/TAC.1987.1104468. **Primary full text read (author-supplied PDF, 2026-07-08).**

- "the discrete-time sliding-mode existence condition |s_{k+1}|<|s_k| of Sarpturk et al." (L93, L1464) — ✅ **Confirmed verbatim.** Sarpturk's **eq (7) `|sᵢ(k+1)| < |sᵢ(k)|`** is introduced as "A necessary and sufficient condition … assuring both sliding motion and convergence onto the ith hyperplane" (contrasted with the weaker eq (6) `[sᵢ(k+1)−sᵢ(k)]sᵢ(k)<0`, which is "necessary but not sufficient"). Eqs (11)/(13) impose the two-sided control bound: "in the discrete case the input must lie on an open interval rather than a half-line."
- The `ηΔt<2` / interval `(0,2)` boundary is presented as *following from* that existence condition (the manuscript's own derivation), **not** attributed to Sarpturk — ✅ correct: Sarpturk's paper states **no** `ηΔt<2` form (its only sampling-time remark is that as `T→0` the bounds tend to `±∞` and the discrete design approaches the continuous one). The `×Δt<2` form belongs to the later Gao–Hung reaching-law literature; the manuscript does not claim otherwise.

---

### 4.9 Mueller2013 — ⚠️ MIXED (supported for MPC/constraints; over-extended for the priority ranking)

**Source:** M. W. Mueller, R. D'Andrea, "A Model Predictive Controller for Quadrocopter State Interception," *ECC 2013*, pp. 1383–1389, doi 10.23919/ECC.2013.6669415. Author full text read.

- "linearize their input constraints inside an MPC loop" (L91) — ✅ The true thrust constraint is non-convex; the paper applies "conservative box constraints … to yield convex constraints" that are "affine functions of the state … and input," yielding "a special case of model predictive control." Fair paraphrase.
- state-interception MPC framing (L91, L930) — ✅ Title/abstract/body confirm a diminishing-horizon MPC for state interception.
- **"a position supported by Mueller and D'Andrea's work on trajectory interception"** for the roll/pitch ≻ thrust ≻ yaw ranking (L930) — ⚠️ **Not established by Mueller.** Mueller presents no saturation-priority ranking; its only yaw-related move is assuming `ω₃=0` because "rotation about ω₃ is not needed for the trajectories considered" — a trajectory-planning simplification, and motor-level mixing is explicitly delegated to another reference. It is weak, tangential support for "yaw lowest priority," not support for a priority *hierarchy*.

**Action:** keep Mueller for the MPC/constraint-linearization point (well supported); drop or soften it as support for the priority ranking, which Faessler2017 already carries.

---

### 4.10 mccoy1999modern — ✅ SUPPORTED

**Source:** R. L. McCoy, *Modern Exterior Ballistics: The Launch and Flight Dynamics of Symmetric Projectiles*, Schiffer, 1999 (2nd ed. 2012). TOC read via Internet Archive; Chapter-9 contents corroborated by the published dexadine/JBM errata.

- "6-DOF ballistic trajectory … equations outlined in Chapter 9" (L831) — ✅ TOC: "Chapter 9 **Six-Degrees-of-Freedom (6-DOF)** … Trajectories … 187"; §9.2 "Equations of Motion for Six-Degrees-of-Freedom Trajectories."
- "standard model of a 120 mm mortar shell given in [McCoy]" (L831) — ✅ Chapter 9's reference list includes Brown & McCoy's BRL report on 120 mm mortar aerodynamics (XM934-HE); the aero-coefficient tables sit at pp. 217–220 (end of Ch. 9), exactly where the project's ballistic code sources its 120 mm data.

(The bib year 1999 is correct — McCoy died in 1999 just after submitting the manuscript.)

---

### 4.11 Bouman2019 — ✅ SUPPORTED

**Source:** A. Bouman et al., "Design and Autonomous Stabilization of a Ballistically-Launched Multirotor," 2020 IEEE ICRA, pp. 8511–8517 (arXiv:1911.10269). Full text read.

"Caltech SQUID … transition into powered flight shortly after leaving the barrel" (L86) — ✅ "a ballistically-launched, autonomously-stabilizing multirotor … autonomous transition from passive to … active stabilization"; "Active stabilization begins once the arms are fully deployed and occurs before the trajectory's apogee"; transition "immediately after the multirotor leaves the launch tube."

---

### 4.12 Denton2025 — ✅ SUPPORTED (minor shared-phrase nuance)

**Source:** H. Denton, M. Benedict, H. Kang, "System identification of a thrust-vectoring, coaxial-rotor-based gun-launched micro air vehicle in hovering flight," *Int. J. Micro Air Vehicles* 17, 2025, doi 10.1177/17568293251361078. Sage open-access full text read.

"Texas A&M TLMAV … ballistically launched … transition into powered flight" (L86) — ✅ "the Tube-Launched MAV (TLMAV) developed at Texas A&M University … could potentially be launched from a 40-mm grenade launcher … transition from the projectile phase to a stable hover."

**Nuance:** the TLMAV's powered transition occurs *near apogee/target*, not immediately after launch ("After the vehicle reaches the apex … the rotor system is deployed"). The shared phrase "shortly after leaving the barrel" (L86) fits SQUID well but is a slight stretch for the TLMAV. Optional softening.

---

### 4.13 Harper2025 — ❌ SOURCE–CLAIM MISMATCH (highest-priority item)

**Source:** J. Harper, "Army's Fiscal 2026 Budget Proposal Aims to Equip Infantry Brigades with More Kamikaze Drones," *DefenseScoop*, 27 Jun 2025. Full text read.

**Manuscript claim (L86):** "Current military research efforts, such as the Launched Effects program, call for a flexible platform that can be deployed anywhere, requires minimal infrastructure, and can carry arbitrary payloads."

**What the article actually says:** it is about **LASSO** loitering (kamikaze) munitions ("Low Altitude Stalking and Strike Ordnance"). "Launched Effects" appears **once**, only as a budget realignment: "LASSO … is now part of the service's Launched Effects family of systems and has been realigned under that line item." Payloads are anti-armor warheads ("anti-tank weapon," "anti-armor warhead"), not "arbitrary." **No text** describes "deployed anywhere," "minimal infrastructure," or "arbitrary payloads."

**Verdict:** the load-bearing descriptive claim is **not supported** by this source.

**Action:** recharacterize the sentence, or cite an actual Launched Effects program description (e.g., Army PEO Aviation / DEVCOM Launched Effects materials). *Note:* this item was flagged in the project's earlier in-house audit and reportedly reviewed/retained by the author; it is re-flagged here on the merits, but the decision is yours.

---

### 4.14 Starks2024 — ⚠️ OVERSTATED ("pilot program")

**Source:** M. A. Starks et al., "Combinations of First Responder and Drone Delivery to Achieve 5-Minute AED Deployment in OHCA," *JACC: Advances* 3(7 Pt 2):101033, 2024, doi 10.1016/j.jacadv.2024.101033. Abstract read; body paywalled.

**Manuscript claim (L86):** medical-supply delivery by drones, "a role tested in some pilot programs."

**What the source is:** a **retrospective-registry modeling/estimation study** — "estimated the impact of a statewide program for drone-delivered AEDs in North Carolina" using CARES data (2013–2019), logistic-regression outcome models, optimized drone placement; conclusion: "Implementation studies are needed." It supports the *concept* of drone medical delivery but is **not** a field-tested pilot.

**Action:** soften to "a role studied in feasibility/modeling analyses," or cite an actual field pilot (e.g., Swedish drone-AED trials).

---

### 4.15 Liang2019 — ✅ SUPPORTED (paraphrase; abstract-level)

**Source:** M. Liang, D. Delahaye, "Drone Fleet Deployment Strategy for Large Scale Agriculture and Forestry Surveying," 2019 IEEE ITSC, pp. 4495–4500, doi 10.1109/ITSC.2019.8917235. Abstract read; body paywalled.

"mobile dispersed sensor network … wide-area surveying" (L86) — ✅ Abstract: "large forestry and agriculture mapping … a fleet of drones … strategy of drone fleet deployment for large scale area surveying." The exact phrase "mobile dispersed sensor network" is a fair paraphrase of a multi-drone wide-area survey (the paper frames it as coverage/mission-time optimization rather than a persistent sensor network).

---

### 4.16 Michelena2025 — ✅ SUPPORTED

**Source:** T. Michelena, "Swarm Technology in Sustainment Operations," army.mil, Jan 2025. Full text read.

"force-protection surveillance" (L86) — ✅ "drone swarm technology … continuous autonomous monitoring … A swarm of small sensor drones … monitor an area with visual or infrared detection … early warning … security." Distributed-sensor force protection is well supported (it is sustainment-security surveillance; it does not speak to *launched* drones, but that is not what this clause claims).

---

### 4.17 Shao2022 — ✅ SUPPORTED (confirmed from PDF, 2026-07-08)

**Source:** X. Shao, G. Sun, W. Yao, J. Liu, L. Wu, "Adaptive Sliding Mode Control for Quadrotor UAVs With Input Saturation," *IEEE/ASME Trans. Mechatronics* 27(3):1498–1509, 2022, doi 10.1109/TMECH.2021.3094575. **Primary full text read (author-supplied PDF, 2026-07-08).**

"embedded a bounded saturation function directly in the control law alongside adaptive parameters" (L89) — ✅ **Confirmed.** The paper defines **bounded saturation functions** explicitly — **Definition 1** `Λ_out(x)` (clipped to `[xlb, xub]`, with `|Λ_out(x)|≤|x|`) and **Definition 2** `Γsat(x)` = `sign(x)` for `|x|≥π/2`, `sin(x)` for `|x|<π/2`, with "`y = Γsat(x) ∈ [−1,1]` is strictly bounded" — and embeds them directly in the inner-loop adaptive SMC law (the "saturated adaptive sliding mode control law, ASMCSAT"), alongside adaptive parameters (`γᵢ`, `ϑ`, with an explicit adaptive law shown to be "stable and bounded"). This is the correct contrast with Xu/Kuang's external auxiliary systems. (The earlier "integral sliding surface" sub-claim was already dropped — correctly.)

---

### 4.18 github — ✅ VERIFIED

**Source:** J. E. Pye, "Controller Analysis Simulation Environment," https://github.com/jepye-3141/controller_analysis (L876). Resolves to a live public MATLAB repository whose files match this project (`analysis.m`, `discrete_smc_swarm.slx`, `constants.m`, `discrete_smc_swarm_single.slx`, etc.). Appropriate.

*(Trivial: the `.bib` URL has a trailing `#`; harmless, resolves fine.)*

---

### 4.19 claude — ✅ APPROPRIATE

**Source:** Anthropic, "Claude Opus 4," https://www.claude.ai (L1534). Used as a generative-AI tool acknowledgment ("used to improve the formatting and layout … reformat … equations … convert MATLAB to LaTeX … syntax and grammar. The author assumes full responsibility …"). Not a factual claim to verify. The bib title now follows the correct Anthropic naming convention ("Claude Opus 4"); set it to the exact version actually used if you want to be specific.

---

## 5. Uncited bibliography entries (housekeeping)

Eleven `.bib` entries are **never `\cite`d** in `root.tex` and therefore **do not appear in the compiled bibliography** (`root.bbl` contains exactly the 19 cited works). They are harmless as-is (BibTeX omits uncited entries) but should be resolved:

`Arnold2019` (robot-swarm definition), `Hicks2023` (DoD replicator/China strategy), `Barker2025` (USMC attack drone team), `Devcom2025` (DEVCOM ARL BAA topics), `Shahzad2023` (swarm-robotics review), `AWS2025` (leader election in distributed systems), `Yu2013` (quadrotor-swarm formation control), `Batra2021` (decentralized RL quadrotor swarms), `Hossein2018` (SMC formation control), `Choutri2018` (leader-follower quadrotor swarm), `copilot` (Microsoft Copilot tool).

**Recommendation:** decide per entry to either cite it (several — Arnold2019, Shahzad2023, Yu2013, Choutri2018, Hossein2018 — would fit naturally in the swarm/formation Introduction and Related Work) or delete it from the `.bib`. If `copilot` (Microsoft Copilot) was used as a tool, it should be acknowledged alongside `claude` at L1534; otherwise remove it. *(Its bib title "Copilot GPT-5.4 Think Deeper" also looks nonstandard — confirm the exact product/version if kept.)*

---

## 6. Residual items — RESOLVED 2026-07-08

The three items originally awaiting PDFs were supplied by the author and are now **confirmed from primary full text** (see §0 Addendum, and the updated §4.6 / §4.8 / §4.17): Xu2020 (per-channel auxiliary states `xᵢ, i∈{φ,θ,ψ}`), Shao2022 (bounded saturation functions `Λ_out`, `Γsat∈[−1,1]` in the ASMCSAT law), and Sarpturk1987 (eq (7) `|s(k+1)|<|s(k)|`, no `ηΔt<2` form). Nothing in the audit now rests on abstract-only evidence except two low-stakes items that were already sufficient at the abstract level and are not in dispute:

- **Starks2024 / Liang2019** — verdicts (modeling study; wide-area survey) stand on the abstracts; full texts remain paywalled but the abstracts fully support the (now edited/retained) usage.

No outstanding verification requests remain.

---

## 7. Bottom line

- **19/19 cited references were located and checked; 2 substantive issues** (Harper2025 mismatch; Tahir2020 PID→PD), **2 overstatements/over-extensions** (Starks2024; Mueller2013 at L930), **1 minor phrasing nuance** (Denton2025).
- **The paper's technical backbone is sound.** Xiong2016 is quoted correctly at the equation level (η·s genuinely in the denominator; Xiong is even self-inconsistent, reinforcing the correction), and the discrete-stability apparatus (Wang2005, JiangWang2001 incl. exact Definition 3.2 / Example 3.4, Sarpturk1987) checks out.
- **Fixes are small and local** — a one-word change (PID→PD), a re-characterized sentence (Harper2025), a softened verb (Starks2024), and a decoupled citation (Mueller2013 at L930) — plus a housekeeping decision on 11 uncited entries.
