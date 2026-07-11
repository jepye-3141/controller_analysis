# Mathematical Verification — "Input Saturation Handling for the Discrete Sliding Mode Controller"

**Scope.** `root.tex`, lines 890–1474 (§ Input Saturation Handling and all subsections through Saturated Control Results). Definitions taken from the rest of the paper: mixing Eq. 1 (`eq:thrusts`), dSMC surfaces and control laws (lines 574–821), gains Table `tab:gains-scalars` (line 782), vehicle constants from the nomenclature table ($m=0.8$ kg, $l=0.2$ m, $J_{xx}=J_{yy}=1.8\times10^{-3}$, $J_{zz}=1.5\times10^{-3}$ kg·m², $k_{MT}=0.1$ m, $\Delta t = 0.02$ s).

**Method.** Every displayed identity was re-derived independently. Matrix algebra, reaching-law identities, the $\Delta V$ expansion, the fixed point, and the residual ratios were additionally machine-checked symbolically (sympy 1.14.0); Appendix A lists the checked identities verbatim as they appear in the scripts' output. Logical/structural claims (cascade guarantees, proof ordering, boundedness assumptions) were checked by hand. Findings are collected in §3 (substantive, F1–F5) and §4 (minor, M1–M13), each with a remedial plan.

**Verdict in one line.** The algebra

 is essentially sound — every displayed equation from `eq:sat-mixing` through `eq:sat-residual-ratio` checks out symbolically — but the stage-4 fallback guarantee (line 1003) is false as written, and four smaller analytical gaps (dimensional claim at line 1424, ISS proof ordering, the $M_D<\infty$ assumption, the frozen-$D_k$ caveat) need patching before submission.

---

## 1. Status summary

| §  | Subsection (lines)                                         | Result                                                                                                                            |
| --- | ---------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| 2.1 | Motor Mixing and Saturation Limits (894–929)              | Verified; add$d = k_{MT}\,b$ link (M1)                                                                                          |
| 2.2 | Priority-Weighted Allocation (930–1005)                   | Stages 1–3 verified;**stage-4 guarantee false (F1)**; wording M2, M3                                                       |
| 2.3 | Auxiliary State and Modified Sliding Variable (1006–1030) | Verified; gain-mapping note M4                                                                                                    |
| 2.4 | Deficit-Coupling Matrix (1031–1168)                       | All four$D$ entries verified; substitution list inverted (M5); $D_{1,1}>0$ envelope (F4); implementation note M6              |
| 2.5 | Schur Stability of the Augmented System (1169–1239)       | Cancellation, closed loop, spectrum verified; frozen-$D_k$/LTV caveat (F5)                                                      |
| 2.6 | ISS-Lyapunov Stability (1240–1358)                        | $\Delta V$ expansion, Young steps, $\mu<0.35$ verified; **proof-ordering gap (F3)**; $M_D$ assumption nontrivial (F4) |
| 2.7 | Reduction to the Unconstrained Limit (1359–1368)          | Verified; "semi-definite" → definite (M7)                                                                                        |
| 2.8 | Steady-State Residuals (1369–1425)                        | Fixed point and all ratios verified;**dimensional claim wrong (F2)**; M8–M10                                               |
| 2.9 | Saturated Control Results (1426–1474)                     | Numbers internally consistent with abstract; reproducibility gap M11                                                              |

---

## 2. Equation-by-equation verification

### 2.1 Motor Mixing and Saturation Limits (lines 894–929)

**`eq:sat-mixing` (line 899).** $T_M = \begin{bmatrix} b & b & b & b \\ 0 & -bl & 0 & bl \\ -bl & 0 & bl & 0 \\ d & -d & d & -d \end{bmatrix}$. Consistency with Eq. 1 (`eq:thrusts`, rows $T, M_x, M_y, M_z$ acting on per-rotor thrusts) requires $T_i = b\,\Omega_i^2$ and $d = k_{MT}\, b$. Under exactly that identification, $T_M$ equals (Eq. 1 matrix)$\times b$ — machine-checked (A.6). The identity $d = k_{MT}b$ is never stated in the paper (nomenclature defines $b$, $d$, $k_{MT}$ independently); see M1. Sign conventions of the moment rows match Eq. 1.

**`eq:sat-deficit` (911).** $\Delta u_k = u_k - \bar u_k$ — definition, consistent with usage throughout ($\bar u_k = u_k - \Delta u_k$ at lines 1047, 1180).

**`eq:sat-BBT` (917).** $T_M T_M^T = \mathrm{diag}(4b^2,\, 2b^2l^2,\, 2b^2l^2,\, 4d^2)$ — machine-verified (A.1). Rows of $T_M$ are mutually orthogonal, which is what makes the pseudoinverse-style construction and the later decomposition clean.

**Feasible set.** $\mathcal{U} = T_M \cdot [\Omega^2_{\min}, \Omega^2_{\max}]^4$ is the image of a box under an invertible linear map — a parallelotope; convex; correct as the exact command-space feasible set.

**Opening claim, line 892** ("Inside $\mathcal{U}$, the closed loop is Lyapunov stable, as the controller is the same as the unconstrained dSMC") — only true when saturation has *never* occurred (so $\xi_k = 0$). After a saturation episode ends, $u_k \in \mathcal{U}$ but $\xi_k \neq 0$ and the controller differs from baseline dSMC until $\xi$ decays. §Reduction (2.7) states this correctly; the introduction overstates it. See M12.

### 2.2 Priority-Weighted Allocation (lines 930–1005)

**`eq:sat-TMinv` (937).** Both factored and evaluated forms machine-verified: the claimed $T_M^{-1}$ equals $T_M^T (T_M T_M^T)^{-1}$ and satisfies $T_M^{-1} T_M = I$ (A.2, A.3).

**`eq:sat-decomposition` (958).** $\Omega^2 = \frac{T}{4b}\mathbf{1} + \frac{M_x}{2bl}e_{M_x} + \frac{M_y}{2bl}e_{M_y} + \frac{M_z}{4d}e_{M_z}$ — machine-verified against $T_M^{-1}u$ (A.4); basis $\{\mathbf 1, e_{M_x}, e_{M_y}, e_{M_z}\}$ mutually orthogonal (A.5). The claim that scaling one command channel scales exactly one contribution and leaves the others untouched follows directly from the decomposition's additivity — correct. (Wording nit: "scales exactly one column of $T_M^{-1}\,u$" at line 923 — $T_M^{-1}u$ is a vector and has no columns; it is one column of $T_M^{-1}$ times the corresponding component of $u$, i.e. one *contribution* $v_\alpha$. See M3.)

**`eq:sat-maxscale` (969).** For $\Omega^2(\lambda) = \text{base} + \lambda\,\text{dir}$ with $\lambda \in [0,1]$: each component is affine in $\lambda$, so per-component feasibility is an interval in $\lambda$; when $\text{base}$ is feasible, the interval contains $\lambda = 0$ and its upper endpoint is $(\Omega^2_{\max} - \text{base}_i)/\text{dir}_i$ for $\text{dir}_i > 0$ and $(\Omega^2_{\min} - \text{base}_i)/\text{dir}_i$ for $\text{dir}_i < 0$. Taking the min over components and clamping to $[0,1]$ is then the largest feasible scale — correct **conditional on base feasibility**. When the base is infeasible the returned value is not guaranteed feasible (the formula can return a positive number whose corresponding point violates the bound on the base-infeasible component); the paper implicitly covers this by re-checking feasibility ("return ... if feasible") after each stage, which is sound. The zero-direction escape clause (line 978) is correct. Suggest one added sentence making the base-feasibility precondition explicit (M13).

**Cascade, items 1–3 (983–1001).**

- *Item 1 (pass-through):* trivially correct; $\Delta u_k = 0$.
- *Item 2, `eq:sat-stage1` (988):* base $= v_T + v_{M_x} + v_{M_y}$, dir $= v_{M_z}$, $\alpha_{M_z} = \lambda_{\max}$; return $(T, M_x, M_y, \alpha_{M_z}M_z)$ if feasible. Consistent with the decomposition and the priority order `eq:sat-priority` ($M_x, M_y \succ T \succ M_z$). Correct.
- *Item 3, `eq:sat-stage2` (997):* base $= v_{M_x} + v_{M_y}$, dir $= v_T$, $\beta_T = \lambda_{\max}$; return $(\beta_T T, M_x, M_y, 0)$ if feasible. Correct. Note $\beta_T = 0$ lies in the searched interval, so stage-3 failure implies in particular that $(0, M_x, M_y, 0)$ is infeasible — which is exactly the premise sentence opening item 4. That premise is therefore **correct**.
- *Item 4 (line 1003):* return $\bar u_k = (0, \gamma M_x, \gamma M_y, 0)$ with $\gamma \in [0,1]$, claimed "guaranteed to find $\gamma \in [0,1]$". **This guarantee is false as written — Finding F1.** The direction vector is

$$
v_{M_x} + v_{M_y} = \frac{1}{2bl}\,(-M_y,\; -M_x,\; M_y,\; M_x)^T,
$$

whose components come in $\pm$ pairs; for $(M_x, M_y) \neq (0,0)$ at least one component is strictly negative. With base $= 0$, feasibility of $\gamma\,(v_{M_x}+v_{M_y})$ requires $\gamma \cdot (\text{negative component}) \geq \Omega^2_{\min} \geq 0$, which fails for every $\gamma > 0$. Hence: if $\Omega^2_{\min} = 0$, the only feasible point is $\gamma = 0$, i.e. $\bar u_k = (0,0,0,0)$ — all four motors commanded to zero, and *no* roll/pitch authority retained (contradicting the item-4 narrative and the entire purpose of the priority order); `eq:sat-maxscale` indeed returns exactly $\gamma = 0$ in this case (the $\text{dir}_i<0$ branch yields $(0-0)/\text{dir}_i = 0$; machine-checked, A.14). If $\Omega^2_{\min} > 0$ (any idle floor), even $\gamma = 0$ is infeasible and stage 4 fails outright. Remedial plan under F1.

### 2.3 Auxiliary State and Modified Sliding Variable (lines 1006–1030)

**`eq:sat-xi-update` (1012).** $\xi_{k+1} = (I - K_\xi)\xi_k + \Delta u_k$, per-channel first-order filter, pole $1 - k_{\xi\alpha}$ — standard discrete anti-windup state; consistent with the cited Wang et al. construction. Correct.

**`eq:sat-tilde-s` (1019).** $\tilde s_k = s_k - D_k \xi_k$, with the substitution $s_\alpha \mapsto \tilde s_\alpha$ applied **only in the reaching-law terms** of the four control laws, equivalent-control terms unchanged. This is exactly the substitution under which the reaching-law identities below were verified; the scoping sentence is important and correct.

**`eq:sat-matched` (1026).** $k_{\xi\alpha} = \eta_\alpha \Delta t$. With Table `tab:gains-scalars` gains at 50 Hz this gives $k_\xi \in \{0.14, 0.28\}$, inside $(0,2)$ — consistent with the later Schur condition and with the claimed $\Delta t$-independence of the residual ratios (verified in 2.8).

### 2.4 Deficit-Coupling Matrix (lines 1031–1168)

**Definitions `eq:sat-A-def`, `eq:sat-C-def`, and `eq:sat-C-equals-minusA` (1035–1055).** With $\Delta s_i|_{\text{cmd}} = A_{ij}u_j + (\text{indep.})$ and $\bar u_j = u_j - \Delta u_j$, linearity gives $\Delta s_i|_{\text{act}} = \Delta s_i|_{\text{cmd}} - A_{ij}\Delta u_j$, hence $C = -A$, hence $D = C = -A$ for the cancellation to work (`eq:sat-D-equals-C`). Logic verified; and the cancellation is separately machine-verified in 2.5. Note the argument is exact because the discrete increments are affine in the inputs at fixed state — which they are, per the update equations at lines 585–596 and 736–748.

**Thrust entry `eq:sat-A-thrust` (1079).** From $\mathrm{d}z_{k+1} - \mathrm{d}z_k = \Delta t[\frac{c\phi\,c\theta}{m}u_1 - g - \frac{K_3}{m}\mathrm{d}z_k]$ (`eq:sat-ddz-evolution`, itself matching the dSMC $z$-dynamics at line 585ff): with the wrapped control law $u_1 = \frac{m}{c\phi\,c\theta}(-\alpha_z \dot z + \frac{K_3}{m}\dot z + g + \eta_z \tilde s_z)$,

$$
\Delta s_z\big|_{\text{cmd}} = -\eta_z \Delta t\, \tilde s_z \quad\text{(machine-verified, A.7)}, \qquad A_{1,1} = -\Delta t\,\frac{c\phi\,c\theta}{m},\ \ D_{1,1} = +\Delta t\,\frac{c\phi\,c\theta}{m} \quad\text{(A.8)}.
$$

**Yaw entry (analogous).** $\Delta s_\psi|_{\text{cmd}} = -\eta_\psi \Delta t\,\tilde s_\psi$ and $D_{4,4} = \Delta t / J_{zz}$ — machine-verified (A.9, A.10) against the $\psi$-dynamics and the wrapped $u_4$ law.

**Roll entries `eq:sat-A-roll` (1140).** Using the underactuated $y,\phi$ update equations (736–748), the wrapped $u_2$ law with $\delta_y = g_1 u_1 - K_2 \dot y/m$, $g_1 = T_y/m$:

$$
\Delta s_\phi\big|_{\text{cmd}} = -\eta_\phi \Delta t\,\tilde s_\phi \ \text{(A.11)}, \qquad A_{2,1} = -\Delta t\, a_1 g_{1,k},\quad A_{2,2} = -\frac{\Delta t\, a_3 l}{J_{xx}} \ \text{(A.12, A.13)}.
$$

The row-2 zeros $A_{2,3} = A_{2,4} = 0$ are structural — $u_3, u_4$ do not appear in the $y,\phi$ updates (verified by inspection of lines 736–748). **Important implementation subtlety:** the identities A.11–A.13 hold only when the $\delta_y$ inside the wrapped $u_2$ law is evaluated with the *commanded* $u_1$ (as the derivation at `eq:sat-Dsphi-udep` implicitly does), not the saturated $\bar u_1$. If the flight code computes $\delta_y$ from $\bar u_1$, the effective $D_{2,1}$ changes and the cancellation in 2.5 acquires an unmodeled residual. Worth one explicit sentence in the paper and a check of the Simulink implementation (M6).

**Pitch entries `eq:sat-A-pitch` (1147).** $A_{3,1} = -\Delta t\, a_5 g_{2,k}$, $A_{3,3} = -\Delta t\, a_7 l / J_{yy}$ — correct by the roll derivation under the symmetry substitution. However, the prose at line 1145 states the substitution **backwards**: it says the pitch entries are obtained by "replacing $\{a_5,a_6,a_7,a_8\}$ with $\{a_1,a_2,a_3,a_4\}$ ... $g_{2,k}$ with $g_{1,k}$, $\mathrm{d}x_k$ with $\mathrm{d}y_k$, $u_{3,k}$ with $u_{2,k}$, and $J_{yy}$ with $J_{xx}$" — all five replacements are inverted (that substitution maps the *pitch* formulas back to the *roll* ones). The displayed formulas are nonetheless correct. See M5.

**`eq:sat-D-matrix` (1157).**

$$
D_k = \Delta t \begin{bmatrix} \frac{c\phi\,c\theta}{m} & 0 & 0 & 0 \\ a_1 g_{1,k} & \frac{a_3 l}{J_{xx}} & 0 & 0 \\ a_5 g_{2,k} & 0 & \frac{a_7 l}{J_{yy}} & 0 \\ 0 & 0 & 0 & \frac{1}{J_{zz}} \end{bmatrix}
$$

consistent with the four verified entries and the structural zeros. Hover limit $g_{1}, g_{2} \to 0$ (since $T_y = c\phi\,s\theta\,s\psi + s\phi\,c\psi \to 0$ and $T_x = c\phi\,s\theta\,c\psi - s\phi\,s\psi \to 0$ at $\phi = \theta = 0$): confirmed, $D_k$ diagonal at hover as stated.

**Positivity / anti-windup direction (line 1167).** "Diagonal elements are positive (since $a_3 = a_7 > 0$)" — true for rows 2–4, but $D_{1,1} = \Delta t\, c\phi\, c\theta / m > 0$ additionally requires $|\phi|, |\theta| < 90°$, which is not guaranteed in the ballistic-tumble regime this paper targets; the sentence should carry that envelope qualifier (folded into F4). The stated ordering for computing $D_k$ ($D_{1,1}, D_{4,4}$ from state; then $u_T$ from $\tilde s_z$; then $a_1, a_2, a_5, a_6$; then the remaining entries) is internally consistent. Separately — and worth *adding* to the paper — the intended anti-windup direction on the thrust channel is in fact attitude-independent:

$$
\frac{\partial u_T}{\partial \xi_T} = \frac{m}{c\phi\,c\theta}\,\eta_z\,(-D_{1,1}) = -\eta_z \Delta t \qquad \text{(machine-verified, A.16)},
$$

i.e. the $c\phi\,c\theta$ factors cancel, so a positive thrust deficit always reduces commanded thrust by the same factor regardless of attitude — a stronger and cleaner statement than the sign argument given.

### 2.5 Schur Stability of the Augmented System (lines 1169–1239)

**`eq:sat-commanded-Ds` (1173) and `eq:sat-saturated-plant` (1181).** The reaching-law substitution carries $\Delta s_k|_{\text{cmd}} = -E\tilde s_k$ forward channel-wise — this is exactly what A.7, A.9, A.11 verify. Adding the deficit response via $C = -A = D$ gives $\Delta s_k = -E\tilde s_k + D_k \Delta u_k$. Correct.

**`eq:sat-cancellation` (1215).** Machine-verified (A.17): substituting `eq:sat-tilde-s`, `eq:sat-xi-update`, `eq:sat-saturated-plant` into $\tilde s_{k+1} = s_{k+1} - D\xi_{k+1}$, the two $D\Delta u_k$ contributions cancel and

$$
\tilde s_{k+1} = (I - E)\,\tilde s_k + D_k K_\xi\, \xi_k.
$$

The paper's derivation explicitly takes $D_{k+1} \approx D_k$ (line 1201) — good that it is stated. The exact relation carries an extra term $(D_k - D_{k+1})\,\xi_{k+1}$, which is $O(|D_{k+1}-D_k|\cdot|\xi|)$ and vanishes with $\xi$; see F5 for the one-sentence patch that makes the downstream ISS claim airtight.

**`eq:sat-closed-loop` (1187).** $X_{k+1} = \mathcal{A}_k X_k + \mathcal{B}\Delta u_k$ with $\mathcal{A}_k = \begin{bmatrix} I-E & D_k K_\xi \\ 0 & I - K_\xi\end{bmatrix}$, $\mathcal{B} = \begin{bmatrix}0\\ I\end{bmatrix}$ — machine-verified assembly (A.18).

**`eq:sat-spectrum` (1226) and `eq:sat-schur` (1232).** Block upper-triangular with diagonal blocks $\Rightarrow$ $\sigma(\mathcal{A}_k) = \{1-\eta_\alpha\Delta t\} \cup \{1-k_{\xi\alpha}\}$ (A.19); Schur iff $0 < \eta_\alpha\Delta t < 2$ and $0 < k_{\xi\alpha} < 2$. Correct. This is also equivalent to the Sarpturk discrete sliding-mode existence condition $|s_{k+1}| < |s_k|$ under the pure reaching law, as the paper later notes — checked: $|1 - \eta\Delta t| < 1 \iff 0 < \eta\Delta t < 2$.

**"Parameter-uniform" claim (line 1238).** True for the *Schur condition* — inertias enter only through $D_k$, which sits in the off-diagonal block and cannot move eigenvalues of a block-triangular matrix. But the claim should not be read as parameter-uniform *transient behavior*: the decay constants of the coupled system (via $\nu$ in 2.6) depend on $M_D = \sup_k|D_k|$, which is strongly parameter-dependent ($D_{2,2} = \Delta t\, a_3 l/J_{xx} \approx 4.44$ for this vehicle). One qualifying clause recommended (part of F5).

**LTV caveat.** $\mathcal{A}_k$ is time-varying through $D_k$; pointwise (frozen-time) Schur eigenvalues do not in general imply exponential stability of an LTV system. Here the structure rescues the argument — the diagonal blocks are *constant* Schur matrices and $D_k$ appears only in the nilpotent-position block, so with $M_D < \infty$ the state transition matrix is exponentially bounded (equivalently: $\xi$ decays autonomously; $\tilde s$ is a stable filter driven by $D_k K_\xi \xi_k$, bounded by $M_D|K_\xi||\xi_k|$). The ISS section is the actual certificate. The paper never says this; a two-sentence bridge is recommended so a reviewer does not raise the classic frozen-time counterexample (F5).

### 2.6 ISS-Lyapunov Stability (lines 1240–1358)

**`eq:sat-lyap` (1246).** $V_k = \tilde s_k^T W_s \tilde s_k + \rho\, \xi_k^T W_\xi \xi_k$, $W_s, W_\xi \succ 0$ diagonal, $\rho > 0$ — valid ISS-Lyapunov candidate; positive definite in $X = (\tilde s, \xi)$.

**Steps 1–3, `eq:sat-DV` (1285).** The six-term expansion (a)–(f) is machine-verified to be *exactly* $V_{k+1} - V_k$ under the closed-loop updates (A.20) — no dropped terms:
(a) $\tilde s$-decay, (b) $\tilde s$–$\xi$ cross, (c) cross-square residual, (d) $\xi$-decay, (e) $\xi$–$\Delta u$ cross, (f) deficit penalty.

**Step 4 (decay identities).** $(I-E)^TW_s(I-E) - W_s = -\mathrm{diag}(\eta_\alpha\Delta t(2-\eta_\alpha\Delta t)(W_s)_{\alpha\alpha})$ — the scalar identity $(1-x)^2 w - w = -x(2-x)w$ is machine-verified (A.21); negative definite exactly under the Schur condition; max decay at $\eta\Delta t = 1$, vanishing at $\{0, 2\}$ — all correct. Same for the $\xi$ block.

**Step 5 (first Young, `eq:sat-young-template`, `eq:sat-young`, `eq:sat-mu-bound`).** Template $2u^TPv \le \mu u^TPu + \frac{1}{\mu}v^TPv$ is the standard weighted Young inequality (valid: $(\sqrt\mu u - v/\sqrt\mu)^TP(\cdot) \ge 0$). Applied with $u = (I-E)\tilde s$, $P = W_s$, $v = D_kK_\xi\xi$. The paper's own warning that the absorbed form carries $(1-\eta\Delta t)^2$ — *not* the decay form — is correct and important; the net per-channel $\tilde s$ coefficient is

$$
-\bigl[\eta_\alpha\Delta t(2-\eta_\alpha\Delta t) - \mu(1-\eta_\alpha\Delta t)^2\bigr](W_s)_{\alpha\alpha},
$$

machine-verified via the factorization $(1-x)^2\bigl(\mu - \frac{x(2-x)}{(1-x)^2}\bigr)$ (A.22), giving `eq:sat-mu-bound` $0 < \mu < \min_\alpha \frac{\eta_\alpha\Delta t(2-\eta_\alpha\Delta t)}{(1-\eta_\alpha\Delta t)^2}$. Numerics: $f(x) = \frac{x(2-x)}{(1-x)^2}$ gives $f(0.14) = 0.3521$, $f(0.28) = 0.9290$; the binding channels are indeed $(z, \psi)$ and the paper's "$\mu < 0.35$" is correct (A.22). *(This presumes the gain mapping $\eta_T \equiv \eta_z$, $\eta_{M_x} \equiv \eta_\phi$, $\eta_{M_y} \equiv \eta_\theta$, $\eta_{M_z} \equiv \eta_\psi$, which the paper implies at lines 1030 and 1323 but never writes down — M4.)*

**Step 6 (ρ-domination, lines 1325–1327).** The bound "(c) + Young leftover $\le (1+\frac{1}{\mu})M_D^2|K_\xi|^2|W_s|\,|\xi_k|^2$" is correct: (c) $= \xi^TK_\xi^TD^TW_sDK_\xi\xi \le M_D^2|K_\xi|^2|W_s||\xi|^2$ and the leftover is $\frac{1}{\mu}\times$ the same form. The domination condition $\rho\,\min_\alpha k_{\xi\alpha}(2-k_{\xi\alpha})(W_\xi)_{\alpha\alpha} > (1+\frac{1}{\mu})M_D^2|K_\xi|^2|W_s|$ is achievable for any finite $M_D$. **However, it is stated one step too early — Finding F3:** step 7 then erodes the very $\xi$-decay margin this condition spends, and the condition is never restated with the eroded margin.

**Step 7 (second Young, `eq:sat-ISS`, lines 1331–1347).** Application with $u = (I-K_\xi)\xi$, $v = \Delta u$, $P = \rho W_\xi$, parameter $\nu_2$: correct, and the paper again correctly notes the absorbed form erodes the margin, requiring $\nu_2 < \min_\alpha \frac{k_{\xi\alpha}(2-k_{\xi\alpha})}{(1-k_{\xi\alpha})^2}$ (same numeric bound 0.3521 under matched gains, A.22). The collected form $\Delta V_k \le -\nu V_k + \gamma|\Delta u_k|^2$ is the standard discrete ISS-Lyapunov inequality; $\gamma \sim \rho(1+\frac{1}{\nu_2})|W_\xi|$ matches the two $|\Delta u|^2$ sources exactly (term (f) plus the second Young leftover). The $\nu$ formula (line 1345) is flagged "~" and is fine as an informal scale, with two caveats folded into F3: (i) its $\xi$-branch omits the subtraction of the absorbed cross-square bound from step 6, overstating the margin; (ii) the $(W)_{\alpha\alpha}$ weights are dropped. Exact forms in F3's remedial plan.

**Conclusions 1–3 (lines 1352–1357).** Given $\Delta V \le -\nu V + \gamma|\Delta u|^2$ with $\nu \in (0,1)$: (1) $\Delta u \equiv 0 \Rightarrow V_k \le (1-\nu)^kV_0$ — immediate; (2) $\Delta u_k \to 0 \Rightarrow V_k \to 0$ — correct (discrete comparison/geometric convolution: $V_k \le (1-\nu)^kV_0 + \gamma\sum_j(1-\nu)^{k-1-j}|\Delta u_j|^2 \to 0$ when $|\Delta u_j|^2 \to 0$); (3) $\sup|\Delta u|^2 \le D_{\sup} \Rightarrow \limsup V_k \le \gamma D_{\sup}/\nu$ — geometric series bound $\gamma D_{\sup}\sum(1-\nu)^i = \gamma D_{\sup}/\nu$. All three verified. Assumption (ii) (hover-trim feasibility) is used implicitly to argue deficits are transient in practice; assumption (iii) $M_D < \infty$ is doing real work and is **not automatic** for this vehicle class — Finding F4.

### 2.7 Reduction to the Unconstrained Limit (lines 1359–1368)

$u_k \in \mathcal{U}\ \forall k \Rightarrow \Delta u_k = 0 \Rightarrow \xi_{k+1} = (I-K_\xi)\xi_k \to 0$ exponentially $\Rightarrow \tilde s \to s$; with $W_s = \frac{1}{2}I$, $V \to \frac{1}{2}\sum_\alpha s_\alpha^2$, matching the per-channel unconstrained Lyapunov functions of `eq:discrete-lyapunov`. All correct. One wording fix: line 1367 calls $\frac{1}{2}\sum s_\alpha^2$ "positive semi-definite" — as a function of $s$ it is positive *definite* (it is only semi-definite in the augmented $(s,\xi)$ state, which is not the object under discussion there). M7.

### 2.8 Steady-State Residuals Under Sustained Saturation (lines 1369–1425)

**Fixed point `eq:sat-xi-infty` (1385).** $(I - \mathcal{A})X^\infty = \mathcal{B}c$: the second block row gives $K_\xi\xi^\infty = c \Rightarrow \xi^\infty = K_\xi^{-1}c$; the first gives $E\tilde s^\infty = DK_\xi\xi^\infty = Dc \Rightarrow \tilde s^\infty = E^{-1}Dc$. Machine-verified (A.23). $E, K_\xi$ invertible under the Schur condition; $I - \mathcal{A}$ block-triangular so the fixed point is unique. Freezing $D_k = D$ over the steady-state window is stated — fine.

**Same-channel ratios `eq:sat-residual-T/theta` (1399–1403).** $|\tilde s_i^\infty|/|c_\alpha| = |D_{i\alpha}|/(\eta_i\Delta t)$; substituting `eq:sat-D-matrix`: $\frac{c\phi c\theta}{m\eta_T}$, $\frac{a_3 l}{J_{xx}\eta_{M_x}}$, $\frac{a_7 l}{J_{yy}\eta_{M_y}}$, $\frac{1}{J_{zz}\eta_{M_z}}$ — all four machine-verified (A.24), and $\Delta t$-independent as claimed (the $\Delta t$ in $D$ cancels the one in $E$; this is the payoff of matched gains). Numerics for this vehicle: $1/(m\eta_T) = 0.179$, $a_3l/(J_{xx}\eta_{M_x}) = 15.87$.

**Cross ratios `eq:sat-residual-cross` (1412).** Column-1 coupling: $\frac{|\tilde s_\phi^\infty|}{|c_T|} = \frac{|a_1||g_{1,k}|}{\eta_{M_x}}$, $\frac{|\tilde s_\theta^\infty|}{|c_T|} = \frac{|a_5||g_{2,k}|}{\eta_{M_y}}$. Structure verified; but the paper writes the roll one as $a_1|g_{1,k}|$ — missing absolute value on $a_1$, which can be negative ($a_1 = 6m/(u_T c\psi)$, sign flips with $c\psi$), while the pitch one correctly has $|a_5|$. M8.

**Gain-limit sentence (line 1419).** As $\eta_\alpha\Delta t \to 2$: thrust ratio $\to \Delta t/(2m) = 0.0125$; at $\eta_\alpha\Delta t = 1$: $\Delta t/m = 0.025$ for thrust, $a_3l\Delta t/J_{xx} = 4.44$ for roll — all three values verified (A.25), and the stated monotone-decrease in $\eta_\alpha$ and the decay-vs-residual trade are correct. The parenthetical "(identically for pitch and yaw)" is only half true: pitch is identical ($a_7 = a_3$, $J_{yy} = J_{xx}$), but yaw is $\Delta t/J_{zz} = 13.3$, a different value (same construction, different number). M9.

**`eq:sat-residual-ratio` (1421) and the line-1424 claim.** The ratio $\frac{|\tilde s_\phi^\infty|/|c_{M_x}|}{|\tilde s_z^\infty|/|c_T|} = \frac{m\,a_3\,l}{J_{xx}}\cdot\frac{\eta_T}{\eta_{M_x}}$ is algebraically correct (with the $c\phi c\theta \approx 1$ approximation carried from `eq:sat-residual-T`). But the accompanying claim — "large for any quadrotor with $J_{\alpha\alpha} \ll m\,l$" — is **dimensionally ill-posed — Finding F2**: $J_{\alpha\alpha}$ is kg·m², $m\,l$ is kg·m, and the ratio itself has units 1/m (value $\approx 88.9\ \mathrm{m^{-1}}$ for this vehicle at $\eta_T/\eta_{M_x} = 0.5$). A comparison between quantities of different dimension cannot ground "large for any quadrotor". Remedial plan under F2.

The closing qualitative statement — recovery to the sliding surface requires the deficit itself to vanish; the controller cannot null the residual while saturated — follows correctly from the ISS bound and the fixed point.

### 2.9 Saturated Control Results (lines 1426–1474)

Internal consistency checks only (simulation outputs cannot be re-derived): Table `tab:sat_cmp_summary` reachability $0.629$ vs $0.464$ matches the abstract's $62.9\%$ vs $46.4\%$; the $|u_k| \approx 40$ priority-allocation ceiling vs $>800$ clipped-spike narrative is consistent with the saturation mechanism (the projection bounds the *commanded* mixer input, clipping does not bound $|u_k|$ at all). One reproducibility gap: $b$, $d$, $\Omega^2_{\min}$, $\Omega^2_{\max}$ are never given numeric values anywhere (nomenclature lists $b$, $d$ with "--"; `tab:gains-scalars` omits them), yet every saturated-run statistic depends on them. M11.

---

## 3. Substantive findings and remedial plans

TODO for future work, do not ignore this if you are reading through the file for things to do: make sure to verify F1 against code if possible. I think that this fallback is accurate behavior, but it is worth checking again.

> **[RESOLVED 2026-07-03]** F1 verified against `dsmc_constraints.m` (stage 3, `max_feasible_scale_local(zeros(4,1), c_Mx+c_My, ...)`): the code implements the paper's item-4 formula literally and, with `Omega2_min = 0`, returns exactly **γ = 0** whenever `(Mx, My) ≠ 0` — i.e. `ū = (0,0,0,0)`, all motors commanded off for that tick, matching this report's prediction (A.14). The paper's formula matches the code; the "guaranteed to find γ" narrative does not match the resulting behavior. Full paper-vs-code audit (incl. M4/M6/M11 code checks and three new critical findings): `docs/paper_code_agreement_saturation.md`.

### F1 — Stage-4 fallback guarantee is false as written (line 1003) — **must fix**

**Claim:** "we scale the joint roll/pitch contribution by $\gamma \in [0,1]$ and return $\bar u_k = (0, \gamma M_x, \gamma M_y, 0)^T$. While this fallback is guaranteed to find $\gamma \in [0,1]$ ..."

**Why it fails.** $v_{M_x} + v_{M_y} = \frac{1}{2bl}(-M_y, -M_x, M_y, M_x)^T$ has a strictly negative component whenever $(M_x, M_y) \neq 0$. With zero base (T and $M_z$ both zeroed), any $\gamma > 0$ drives that component below $0 \le \Omega^2_{\min}$. So: $\Omega^2_{\min} = 0$ ⟹ only $\gamma = 0$ is feasible, and `eq:sat-maxscale` returns exactly $0$, i.e. $\bar u_k = 0$ — all motors off, *no* roll/pitch authority retained, contradicting both the sentence and the priority order's purpose; $\Omega^2_{\min} > 0$ ⟹ no feasible $\gamma$ at all, the cascade exits without a feasible command, and the "guarantee" is void. Machine demonstration in A.14–A.15. Because the reported simulations apparently recover through deep saturation, the Simulink implementation likely does *not* do what item 4 says — **check the code against the paper**; the fix below is probably what any working implementation effectively does.

**Remedial plan (preferred).** Re-center the motor band in stage 4 instead of zeroing thrust. Take

$$
\text{base} = \frac{\Omega^2_{\min} + \Omega^2_{\max}}{2}\,\mathbf{1} \quad\Longleftrightarrow\quad T_c = 2b\,(\Omega^2_{\min} + \Omega^2_{\max}), \qquad \text{dir} = v_{M_x} + v_{M_y},
$$

$$
\gamma = \lambda_{\max}(\text{base}, \text{dir}), \qquad \bar u_k = (T_c,\ \gamma M_x,\ \gamma M_y,\ 0)^T.
$$

Now base is strictly interior (each component sits mid-band, margin $(\Omega^2_{\max} - \Omega^2_{\min})/2 > 0$ to both bounds), so $\gamma > 0$ *strictly* for any $(M_x, M_y) \neq 0$ — a true guarantee (A.15). Physically this is also the right fallback: differential moments require a nonzero common mode to differentiate about, so maximizing retained roll/pitch authority forces mid-band collective, not zero collective. The narrative sentence then changes from "discarding all thrust control" to "pinning collective to the band midpoint (a fixed, non-commanded thrust $T_c$), sacrificing altitude *tracking* but preserving maximal attitude authority" — arguably a stronger selling point, and it removes the false claim. Note $\Delta u_k$ then has a thrust component $T - T_c$, which the auxiliary state already handles.

**Remedial plan (minimal).** If the flight code truly returns $(0, \gamma M_x, \gamma M_y, 0)$: state $\Omega^2_{\min} = 0$ explicitly as an assumption, replace "guaranteed to find $\gamma \in [0,1]$" with the honest statement that the closed form returns $\gamma = 0$ and the fallback is total authority loss (motor cutoff) — and reconcile the following sentence, which currently implies roll/pitch is retained. This is strictly worse than the preferred fix but at least true.

### F2 — Dimensionally ill-posed claim at line 1424 — **must fix**

"large for any quadrotor with $J_{\alpha\alpha} \ll m\,l$" compares kg·m² against kg·m; and `eq:sat-residual-ratio` itself carries units of 1/m, so "large" is unit-dependent (large in $\mathrm{m}^{-1}$, not large in $\mathrm{km}^{-1}$).

**Remedial plan.** Two clean options: (a) report the number — "$\frac{m\,a_3\,l}{J_{xx}}\frac{\eta_T}{\eta_{M_x}} \approx 89\ \mathrm{m^{-1}}$ for this vehicle; equivalently the moment channels hold residuals two orders of magnitude larger than thrust per unit deficit *in SI units of the respective sliding variables*" — and drop the "any quadrotor" universality; or (b) nondimensionalize properly: the dimensionless driver is $\frac{m\,l^2}{J_{xx}}$ (this vehicle: $\frac{0.8 \times 0.04}{0.0018} = 17.8$), so write the ratio as $\frac{a_3}{l}\cdot\frac{m\,l^2}{J_{xx}}\cdot\frac{\eta_T}{\eta_{M_x}}$ and claim largeness for $m\,l^2 \gg J_{\alpha\alpha}$, which *is* a legitimate small-quadrotor trait (thin rotor-plane mass distribution). Option (b) preserves the intended physical point rigorously.

### F3 — ISS proof-ordering gap: ρ fixed before the ν₂ erosion it must survive (steps 6–7, lines 1325–1347)

Step 6 chooses $\rho$ against the *un-eroded* $\xi$-decay margin $k_{\xi\alpha}(2-k_{\xi\alpha})$; step 7 then spends $\nu_2(1-k_{\xi\alpha})^2$ of that margin. Nothing false results (any $\rho$ can be enlarged), but as written the constant chosen in step 6 need not satisfy the condition actually required at the end, and the final $\nu$'s $\xi$-branch (line 1345) also omits the subtraction of the absorbed cross-square term, overstating the margin.

**Remedial plan.** Swap the quantifier order and restate once, exactly:

1. Fix $\mu$ per `eq:sat-mu-bound` and $\nu_2 < \min_\alpha \frac{k_{\xi\alpha}(2-k_{\xi\alpha})}{(1-k_{\xi\alpha})^2}$ *first* (both bounds equal $0.3521$ under matched gains — worth saying).
2. Then choose $\rho$ against the eroded margin:

$$
\rho\,\min_\alpha\bigl[k_{\xi\alpha}(2-k_{\xi\alpha}) - \nu_2(1-k_{\xi\alpha})^2\bigr](W_\xi)_{\alpha\alpha} \;>\; \Bigl(1+\tfrac{1}{\mu}\Bigr)M_D^2\,|K_\xi|^2\,|W_s|.
$$

3. Exact decay constant (replacing the "~" version, or as a footnote to it), for the record:

$$
\nu = \frac{\min\Bigl(\min_\alpha\bigl[\eta_\alpha\Delta t(2-\eta_\alpha\Delta t) - \mu(1-\eta_\alpha\Delta t)^2\bigr](W_s)_{\alpha\alpha},\ \ \min_\alpha\rho\bigl[k_{\xi\alpha}(2-k_{\xi\alpha}) - \nu_2(1-k_{\xi\alpha})^2\bigr](W_\xi)_{\alpha\alpha} - \bigl(1+\tfrac{1}{\mu}\bigr)M_D^2|K_\xi|^2|W_s|\Bigr)}{\max(|W_s|,\ \rho\,|W_\xi|)},
$$

$$
\gamma = \rho\Bigl(1 + \tfrac{1}{\nu_2}\Bigr)|W_\xi| \quad (\text{exact as stated}).
$$

Presentation-only change; no conclusion is invalidated.

### F4 — Assumption (iii) $M_D = \sup_k|D_k| < \infty$ is nontrivial precisely in the regime this paper targets (line 1352; also line 1167)

$D_{2,1} = \Delta t\,a_1 g_{1,k} = \frac{6\,\Delta t\,T_y}{u_T\,c\psi}$ and $D_{3,1} = -\frac{6\,\Delta t\,T_x}{u_T\,c\phi\,c\psi}$ blow up as commanded $u_T \to 0$ or as $c\psi \to 0$ (resp. $c\phi c\psi \to 0$). A ballistically launched, tumbling vehicle can pass near both. Stating "$M_D < \infty$" as an assumption without an envelope makes the ISS certificate conditional on something unverified in the very scenario of interest. Relatedly, the $D_{1,1} > 0$ direction argument at line 1167 silently assumes $|\phi|, |\theta| < 90°$.

**Remedial plan.** Add one explicit envelope condition where (iii) is introduced, e.g.: *there exist $\epsilon_u, \epsilon_a > 0$ such that along closed-loop trajectories $|u_T| \ge \epsilon_u$, $|c\psi| \ge \epsilon_a$, $|c\phi\,c\psi| \ge \epsilon_a$ (and $|\phi|,|\theta| < 90°$ for $D_{1,1} > 0$); then $M_D \le \Delta t\,\max(\ldots)$ explicitly.* Tie it to the simulation evidence: the successful-landing trials of Fig. `fig:sat_control_norm_on` empirically satisfy this (thrust stays near the allocation bound, not near zero). Optionally note the anti-windup thrust direction is exact and envelope-free ($\partial u_T/\partial\xi_T = -\eta_z\Delta t$, §2.4), so only the *cross-coupling magnitude*, not the correction direction, needs the envelope.

### F5 — Frozen-time / LTV gap: state why pointwise Schur suffices here (lines 1201, 1226, 1238)

Two related, currently-implicit steps: (i) the cancellation uses $D_{k+1} \approx D_k$; (ii) the spectrum of the time-varying $\mathcal{A}_k$ is evaluated pointwise. For general LTV systems pointwise Schur eigenvalues do not imply stability, and a reviewer may say so.

**Remedial plan.** Two sentences suffice: *"Because the diagonal blocks of $\mathcal{A}_k$ are constant and $D_k$ enters only the off-diagonal block, the frozen-time spectrum is $k$-independent and the LTV system is exponentially stable whenever $M_D < \infty$: $\xi$ decays autonomously and $\tilde s$ obeys a constant-coefficient stable recursion driven by the bounded input $D_kK_\xi\xi_k$ (plus the $(D_k - D_{k+1})\xi_{k+1}$ remainder from the $D_{k+1}\approx D_k$ step, likewise $O(|\xi|)$). The ISS-Lyapunov inequality of Section [sat-iss] is the formal certificate; the spectrum discussion is diagnostic."* Also append to line 1238 that parameter-uniformity concerns the Schur *condition*, while decay constants depend on $M_D$ through $\nu$.

---

## 4. Minor findings (wording, notation, reproducibility)

**M1 (line 899/909).** State $d = k_{MT}\,b$ (or define $k_{MT} \triangleq d/b$) to tie `eq:sat-mixing` to Eq. 1; currently the two mixing descriptions are formally disconnected, and A.6 shows they agree only under that identity.

**M2 (line 1003).** Even after F1's fix, soften "guaranteed" language to reference the explicit constructive property (strict interiority of the base) that provides the guarantee.

**M3 (line 923).** "scales exactly one column of $T_M^{-1}\,u$" → "scales exactly one column-contribution $v_\alpha$ of the decomposition (one column of $T_M^{-1}$ times the corresponding component of $u$)".

**M4 (lines 928/1177).** Write the gain mapping once: $(\eta_T, \eta_{M_x}, \eta_{M_y}, \eta_{M_z}) \equiv (\eta_z, \eta_\phi, \eta_\theta, \eta_\psi)$. The $\mu < 0.35$ numeric and all residual numerics depend on it; it is currently only implied (lines 1030, 1323).

**M5 (line 1145).** Substitution list is inverted; should read: replacing $\{a_1, a_2, a_3, a_4\}$ with $\{a_5, a_6, a_7, a_8\}$, $g_{1,k}$ with $g_{2,k}$, $\mathrm{d}y_k$ with $\mathrm{d}x_k$, $u_{2,k}$ with $u_{3,k}$, and $J_{xx}$ with $J_{yy}$. Displayed formulas `eq:sat-A-pitch` are already correct.

**M6 (lines 1095–1140).** Add an implementation sentence: the $\delta_y$ (and $\delta_x$) terms inside the wrapped roll/pitch laws must be evaluated with the commanded $u_1$, not the saturated $\bar u_1$, for `eq:sat-cancellation` to hold with the stated $D_{2,1}$, $D_{3,1}$. Verify the Simulink code does this.

**M7 (line 1367).** "positive semi-definite" → "positive definite" (in $s$; it is the reduction of $V$ after $\xi \to 0$).

**M8 (`eq:sat-residual-cross`, line 1412).** $a_1|g_{1,k}| \to |a_1|\,|g_{1,k}|$ ($a_1$ changes sign with $c\psi$).

**M9 (line 1419).** "(identically for pitch and yaw)": true for pitch; yaw follows the same construction but the value is $\Delta t/J_{zz} = 13.3$, not $a_3l\Delta t/J_{xx} = 4.44$. Suggest "(identically for pitch; $\Delta t/J_{zz}$ for yaw)".

**M10 (§2.8/§dSMC notation drift).** $a_z$ (line 1070) vs $\alpha_z$ (line 609); $\Delta_{k,k+1}$ (line 1073) vs $\Delta_{k+1,k}$ (lines 618–766); $\mathrm{d}z_k$ (line 1067) vs $\dot z_k$ (line 609). Harmonize to the dSMC section's conventions.

**M11 (results reproducibility).** $b$, $d$, $\Omega^2_{\min}$, $\Omega^2_{\max}$ have no numeric values anywhere ($b$, $d$ are "--" in the nomenclature; absent from `tab:gains-scalars`), yet the reachability table, the $|u_k| \approx 40$ ceiling, and the entire saturated comparison depend on them. Add them to `tab:gains-scalars` (and note $d = k_{MT}b$ per M1 fixes only the ratio, not the scale).

**M12 (line 892).** "Inside $\mathcal{U}$, the closed loop ... is the same as the unconstrained dSMC" → qualify: identical when no prior saturation has occurred ($\xi_k = 0$); after saturation ends, the controller *converges* to baseline as $\xi$ decays (cross-reference §Reduction).

**M13 (`eq:sat-maxscale`, line 969).** Add: "when the base point is itself feasible, the returned point is feasible; each cascade stage's base is re-checked by the trailing feasibility test." Makes the stage-wise "if feasible" re-checks visibly load-bearing rather than redundant.

---

## Appendix A — Machine-checked identities (sympy 1.14.0)

Scripts: `verify_part1.py`, `verify_part2.py` (included alongside this report; each prints PASS/FAIL per identity; all 26 checks below PASS). Symbolic, exact — no floating-point tolerance except where a numeric value is itself the claim.

1. `eq:sat-BBT`: $T_MT_M^T = \mathrm{diag}(4b^2, 2b^2l^2, 2b^2l^2, 4d^2)$.
2. `eq:sat-TMinv`: claimed matrix $= T_M^T(T_MT_M^T)^{-1}$.
3. `eq:sat-TMinv`: claimed matrix satisfies $T_M^{-1}T_M = I$.
4. `eq:sat-decomposition`: $T_M^{-1}u = v_T + v_{M_x} + v_{M_y} + v_{M_z}$.
5. Basis $\{\mathbf 1, e_{M_x}, e_{M_y}, e_{M_z}\}$ mutually orthogonal.
6. `eq:sat-mixing` $=$ (Eq. 1 matrix)$\times b$ under $T_i = b\Omega_i^2$, $d = k_{MT}b$.
7. $z$-channel: $\Delta s_z|_{\text{cmd}} = -\eta_z\Delta t\,\tilde s_z$ under the wrapped $u_1$ law.
8. $A_{1,1} = -\Delta t\,c\phi c\theta/m$ (hence $D_{1,1} = +\Delta t\,c\phi c\theta/m$).
9. $\psi$-channel: $\Delta s_\psi|_{\text{cmd}} = -\eta_\psi\Delta t\,\tilde s_\psi$.
10. $A_{4,4} = -\Delta t/J_{zz}$.
11. $\phi$-channel: $\Delta s_\phi|_{\text{cmd}} = -\eta_\phi\Delta t\,\tilde s_\phi$ (with $\delta_y$ from commanded $u_1$).
12. $A_{2,1} = -\Delta t\,a_1g_{1}$.
13. $A_{2,2} = -\Delta t\,a_3l/J_{xx}$; row-2 zeros structural by inspection of lines 736–748.
14. Stage-4 as written: $\lambda_{\max}(0, v_{M_x}{+}v_{M_y}) = 0$ for $\Omega^2_{\min} = 0$ ⟹ $\bar u = 0$; infeasible for $\Omega^2_{\min} > 0$.
15. Stage-4 remedial: base $= \frac{\Omega^2_{\min}+\Omega^2_{\max}}{2}\mathbf 1$ gives $\gamma = 1 > 0$ in the numeric demo; strictly positive in general by interiority.
16. Anti-windup sensitivity: $\partial u_T/\partial\xi_T = -\eta_z\Delta t$ exactly (attitude terms cancel).
17. `eq:sat-cancellation`: $\tilde s_{k+1} = (I-E)\tilde s_k + DK_\xi\xi_k$; both $D\Delta u_k$ terms cancel (under $D_{k+1} = D_k$).
18. `eq:sat-closed-loop` assembly.
19. `eq:sat-spectrum`: $\sigma(\mathcal{A}) = \{1-\eta_\alpha\Delta t\}\cup\{1-k_{\xi\alpha}\}$.
20. `eq:sat-DV`: six-term expansion equals $V_{k+1} - V_k$ exactly.
21. Step-4 identity: $(1-x)^2w - w = -x(2-x)w$.
22. Step-5 net coefficient factorization $(1-x)^2(\mu - f(x))$, $f(x) = \frac{x(2-x)}{(1-x)^2}$; $f(0.14) = 0.3521$, $f(0.28) = 0.9290$ ⟹ binding $\mu < 0.352$ on $(z,\psi)$ (paper: $0.35$ ✓); same bound for $\nu_2$ under matched gains.
23. `eq:sat-xi-infty`: $\xi^\infty = K_\xi^{-1}c$, $\tilde s^\infty = E^{-1}Dc$.
24. Residual ratios `eq:sat-residual-T/theta` from $E^{-1}D$; numerics $1/(m\eta_T) = 0.1786$, $a_3l/(J_{xx}\eta_{M_x}) = 15.873$; `eq:sat-residual-ratio` value $= 88.9\ \mathrm{m^{-1}}$.
25. Limit values: $\Delta t/(2m) = 0.0125$; $\Delta t/m = 0.025$; $a_3l\Delta t/J_{xx} = 4.444$.
26. Sarpturk equivalence: $|1-\eta\Delta t| < 1 \iff 0 < \eta\Delta t < 2$.

**Checked by hand (not symbolically):** parallelotope structure of $\mathcal{U}$; interval logic of `eq:sat-maxscale` incl. base-feasibility precondition and dir$_i = 0$ clause; cascade stage ordering and the stage-3→4 premise; $C = -A$ affinity argument; hover limits $g_1, g_2 \to 0$; the $D_k$ computation ordering; LTV rescue argument (F5); ISS conclusions 1–3 via geometric series; the step-6 operator-norm bound; abstract/table number consistency.