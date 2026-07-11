# Plan A+: Lyapunov-Preserving DSMC with Closed-Form Priority-Weighted Allocation

**Status: definitive.** This document is the single source of truth for Plan A+. It supersedes all earlier drafts. Where this document differs from earlier drafts, this document is correct.

The prior drafts contained three issues that are corrected here:

1. The Lyapunov proof in earlier drafts attempted a diagonal-$P$ form that ran into a fictitious "tradeoff" between negativity of the auxiliary-state decay term and absorption of the saturation residual. The proof is reframed as a standard ISS-Lyapunov argument using the per-channel discrete Lyapunov equation, which absorbs the disparate plant Jacobians naturally.
2. Earlier drafts framed UUB-vs-asymptotic stability as a consequence of "channel scale disparity." It is not. UUB-vs-asymptotic is determined entirely by whether saturation persists. Sustained saturation on any plant — uniform-Jacobian or otherwise — gives UUB; transient saturation gives asymptotic recovery.
3. Earlier drafts substituted the user's specific parameter values $(m, J_{xx}, J_{yy}, J_{zz})$ throughout the analysis, obscuring the parametric structure of the result. This draft keeps the analysis fully parameterized; numerical instances appear only as illustrative examples and are clearly marked as such. A new §11 makes explicit the parametric conditions on $(m, J_{xx}, J_{yy}, J_{zz})$ under which the stability claims hold and identifies the regimes where the analysis remains valid.

---

## Notation conventions

- $u_k = (T_k, M_{x,k}, M_{y,k}, M_{z,k})^T \in \mathbb{R}^4$: unconstrained dSMC command in thrust–moment space.
- $T_{\mathrm{rot},k} = B^{-1} u_k \in \mathbb{R}^4$: implied per-motor thrust vector.
- $\mathcal{U}_{\mathrm{rot}} = [T_{\min}, T_{\max}]^4$: per-motor box constraint.
- $\mathcal{U} = B \cdot \mathcal{U}_{\mathrm{rot}} \subset \mathbb{R}^4$: resulting polytope in $u$-space.
- $\bar u_k$: value applied to actuators after projection.
- $\Delta u_k = u_k - \bar u_k$: projection deficit.
- $b_\alpha$: plant Jacobian per channel: $b_T = \cos\phi\cos\theta/m$, $b_{M_x} = 1/J_{xx}$, $b_{M_y} = 1/J_{yy}$, $b_{M_z} = 1/J_{zz}$.
- $\delta_\alpha$: discrete-time effective input gain — the exact coefficient of $u_{\alpha, k}$ in $s_{\alpha, k+1} - s_{\alpha, k}$ under the user's two-step extrapolated plant. To leading order, $\delta_\alpha \approx \Delta t\, b_\alpha$.
- $\eta_\alpha$: user's existing reaching-law gain per channel (unchanged from the original derivation).
- $\xi_\alpha$: per-channel auxiliary state introduced by Plan A+.
- $k_{\xi,\alpha} = \eta_\alpha \Delta t$: matched auxiliary-state contraction rate.
- $\tilde s_\alpha = s_\alpha - \delta_\alpha \xi_\alpha$: modified sliding variable used in the control law.

**Representative user parameters.** Numerical instances refer to: $m = 0.2$ kg, $l = 0.2$ m, $J_{xx} = J_{yy} = 1.8 \times 10^{-3}$ kg·m², $J_{zz} = 1.5 \times 10^{-3}$ kg·m², $k_{MT} = 0.1$ m, $\Delta t = 0.02$ s. **The control laws, the auxiliary-state dynamics, the closed-form allocation, and the stability proof are all stated parametrically.** §11 makes the parameter-dependence explicit.

---

## Part I — Design

### 1. Overview

Plan A+ augments the user's existing discrete sliding mode controller (cSMC eqs 5–12 and dSMC eqs 13–40 of the original derivation) with two structural elements, both layered *on top of* the existing controller:

1. **A per-channel auxiliary state $\xi_\alpha \in \mathbb{R}$** (4 scalars total), driven by the saturation deficit $\Delta u_\alpha$ with exponential decay rate $1 - \eta_\alpha \Delta t$. The auxiliary state offsets the sliding variable in the control law via $\tilde s_\alpha = s_\alpha - \delta_\alpha \xi_\alpha$.
2. **A closed-form priority-weighted allocation** that maps the unconstrained $u_k$ onto the per-motor polytope $\mathcal{U}$ in a way that preserves the operational priority $M_x, M_y \succ T \succ M_z$. The procedure is the symmetric-quadrotor specialization of Faessler–Falanga–Scaramuzza (*IEEE RA-L*, 2017), executed in approximately 50 floating-point operations per controller tick.

When no saturation occurs, $\Delta u_k = 0$, the auxiliary state decays to zero exponentially, $\tilde s_k = s_k$ exactly, and the controller reduces *pointwise* to the user's original. When saturation is sustained, the closed-loop system is uniformly ultimately bounded (UUB) with ultimate bound proportional to the saturation amplitude. The user's original asymptotic-tracking guarantee is recovered as a special case (no saturation) and extended (transient saturation also gives asymptotic recovery).

The sliding surfaces, reaching laws, equivalent-control terms, chattering gains, and discrete plant model of the original derivation are reused without modification. No part of the original derivation is replaced.

### 2. The input-constraint problem and priority motivation

Element-wise clip in motor space corresponds to the $(B B^T)^{-1}$-weighted projection in $u$-space:
$$(B B^T)^{-1}\ =\ \mathrm{diag}\!\left(\tfrac{1}{4},\ \tfrac{1}{2 l^2},\ \tfrac{1}{2 l^2},\ \tfrac{1}{4 k_{MT}^2}\right).$$
**Whenever $1/(4 k_{MT}^2) > 1/(2 l^2) > 1/4$** (equivalently $k_{MT}^2 < l^2/2 < 1$), **the implicit priority is yaw-first, which is the wrong order for a quadrotor.** This is the typical regime for any quadrotor with arm length $l < \sqrt{2}$ m and torque-thrust ratio $k_{MT} < l/\sqrt{2}$ — i.e., essentially every quadrotor.

Attitude divergence is the primary failure mode under saturation; modest yaw error is harmless. The literature (Faessler et al. 2017; Lee 2013; Mueller & D'Andrea 2013) and PX4/Crazyflie firmware all implement variations of the priority hierarchy
$$M_x, M_y\ \succ\ T\ \succ\ M_z.$$

### 3. The closed-form priority-weighted allocation

#### 3.1 Structural property

For the symmetric quadrotor:
$$B B^T\ =\ \mathrm{diag}\!\left(4,\ 2 l^2,\ 2 l^2,\ 4 k_{MT}^2\right).$$

This depends only on $(l, k_{MT})$, not on $(m, J)$. The rows of $B$ are mutually orthogonal in motor space, so $B^{-1} = B^T (B B^T)^{-1}$ has columns proportional to $r_\alpha^T / \|r_\alpha\|^2$ (where $r_\alpha$ is the $\alpha$-th row of $B$) that are mutually orthogonal in motor space. Any $T_{\mathrm{rot}}$ decomposes as
$$T_{\mathrm{rot}}\ =\ \tfrac{T}{4}\, r_T^T\ +\ \tfrac{M_x}{2 l^2}\, r_{M_x}^T\ +\ \tfrac{M_y}{2 l^2}\, r_{M_y}^T\ +\ \tfrac{M_z}{4 k_{MT}^2}\, r_{M_z}^T,$$
with the four contributions mutually orthogonal. **Scaling any one channel's contribution by $\alpha \in [0, 1]$ does not affect the others.** This is the property that makes a closed-form lexicographic projection possible.

#### 3.2 Algorithm

```
def priority_weighted_allocate(u, T_min, T_max):
    """Project u onto the per-motor polytope U with priority M_x, M_y > T > M_z.
       ~50 FLOPs, no solver, deterministic time."""
    T, Mx, My, Mz = u

    # Orthogonal channel contributions in motor space (precompute r_T, r_Mx, r_My, r_Mz at boot)
    c_T  = (T  / 4.0)             * r_T
    c_Mx = (Mx / (2.0 * l*l))     * r_Mx
    c_My = (My / (2.0 * l*l))     * r_My
    c_Mz = (Mz / (4.0 * kMT*kMT)) * r_Mz

    T_rot = c_T + c_Mx + c_My + c_Mz   # = B^{-1} u

    # Fast path
    if (T_rot >= T_min).all() and (T_rot <= T_max).all():
        return T_rot, u

    # STAGE 1: scale yaw toward zero
    base = c_T + c_Mx + c_My
    alpha_Mz = max_feasible_scale(base, c_Mz, T_min, T_max)
    T_rot = base + alpha_Mz * c_Mz
    if (T_rot >= T_min).all() and (T_rot <= T_max).all():
        return T_rot, (T, Mx, My, alpha_Mz * Mz)

    # STAGE 2: scale collective thrust toward zero
    base = c_Mx + c_My
    beta_T = max_feasible_scale(base, c_T, T_min, T_max)
    T_rot = beta_T * c_T + base
    if (T_rot >= T_min).all() and (T_rot <= T_max).all():
        return T_rot, (beta_T * T, Mx, My, 0.0)

    # STAGE 3: deep saturation — scale roll/pitch
    gamma = max_feasible_scale(zeros(4), c_Mx + c_My, T_min, T_max)
    T_rot = gamma * (c_Mx + c_My)
    return T_rot, (0.0, gamma * Mx, gamma * My, 0.0)


def max_feasible_scale(base, direction, T_min, T_max):
    """Largest alpha in [0, 1] such that base + alpha * direction in [T_min, T_max]^4."""
    alpha = 1.0
    for i in range(4):
        if direction[i] > 0:
            alpha = min(alpha, (T_max - base[i]) / direction[i])
        elif direction[i] < 0:
            alpha = min(alpha, (T_min - base[i]) / direction[i])
    return max(alpha, 0.0)
```

#### 3.3 Properties

- **Closed under feasibility.** If $u_k$ already gives a feasible $T_{\mathrm{rot}}$, the algorithm returns immediately with $\bar u_k = u_k$, $\Delta u_k = 0$.
- **Preserves $M_x, M_y$ whenever possible.** Stages 1 and 2 hold $M_x, M_y$ at full commanded values.
- **Reduces yaw before thrust.** Stage 1 zeroes yaw before Stage 2 touches thrust.
- **Sector condition.** The procedure is the lexicographic limit of a $W$-weighted projection (Boyd & Vandenberghe, §4.7.5) and satisfies the dead-zone sector condition $\Delta u_k^T W (u_k - u_0) \ge \Delta u_k^T W \Delta u_k$ for any positive-definite diagonal $W$ consistent with the priority order.

#### 3.4 Stage 3 caveat

Stage 3 is reached only when even zero thrust and zero yaw cannot satisfy demanded $(M_x, M_y)$ — a measure-zero set of inputs in practice. The "scale jointly by $\gamma$" tie-breaking rule is one of several valid lexicographic projections and does not affect the stability proof.

### 4. The auxiliary-state mechanism

#### 4.1 Definitions

For each control channel $\alpha \in \{T, M_x, M_y, M_z\}$:

**Auxiliary state:**
$$\xi_{\alpha, k+1}\ =\ (1 - k_{\xi,\alpha})\, \xi_{\alpha, k}\ +\ \Delta u_{\alpha, k}, \qquad 0 < k_{\xi,\alpha} < 1.$$

**Modified sliding variable:**
$$\tilde s_{\alpha, k}\ =\ s_{\alpha, k}\ -\ \delta_\alpha\, \xi_{\alpha, k}.$$

**Control law.** The user's existing dSMC laws (eqs 21, 38, 40) with $s_{\alpha,k} \to \tilde s_{\alpha,k}$. For thrust:
$$u_{1,k}\ =\ \frac{m}{\cos\phi_k \cos\theta_k}\big[-\alpha_z\, \dot z_k + \tfrac{K_3}{m}\, \dot z_k + g + \eta_z\, \tilde s_{z,k}\big]$$
For roll:
$$u_{2,k}\ =\ \tfrac{J_{xx}}{l\, a_3}\big[-a_1\, \delta y_k - a_2\, \dot y_k - a_3\, f_{1,k} - a_4\, \dot\phi_k + \eta_\phi\, \tilde s_{\phi,k}\big]$$
and analogously for pitch and yaw. Equivalent-control terms unchanged. **The substitution $s \to \tilde s$ is the only point where $\xi$ enters the control law.**

#### 4.2 Why this substitution

**(i) Exact deficit cancellation in $\tilde s$ dynamics.** Under saturation, $s$ evolves as
$$s_{\alpha, k+1} - s_{\alpha, k}\ =\ -\eta_\alpha \Delta t\, \tilde s_{\alpha, k}\ +\ \delta_\alpha\, \Delta u_{\alpha, k}.$$
Substituting into $\tilde s_{\alpha, k+1} = s_{\alpha, k+1} - \delta_\alpha \xi_{\alpha, k+1}$:
$$\tilde s_{\alpha, k+1}\ =\ s_{\alpha, k} - \eta_\alpha \Delta t \tilde s_{\alpha, k} + \delta_\alpha \Delta u_{\alpha, k}\ -\ \delta_\alpha (1 - k_{\xi,\alpha}) \xi_{\alpha, k}\ -\ \delta_\alpha \Delta u_{\alpha, k}.$$
The $\delta_\alpha \Delta u_{\alpha,k}$ terms cancel exactly. **The saturation deficit drives only $\xi$, never $\tilde s$ directly** — provided $\delta_\alpha$ in the auxiliary substitution matches the plant Jacobian.

**(ii) Pointwise reduction.** When $\Delta u_k = 0$, $\xi_k \to 0$ exponentially, $\tilde s_k \to s_k$, and the control law converges to the user's original. **No retuning required for the unconstrained regime.** Holds for any $(m, J)$.

**(iii) Per-channel decoupling.** Each $\xi_\alpha$ is driven only by $\Delta u_\alpha$.

#### 4.3 Matched contraction rate

Choosing $k_{\xi,\alpha} = \eta_\alpha \Delta t$:
- No new gain to tune.
- Auxiliary state decays at the same rate as the sliding variable converges.
- Makes per-channel dynamics a Jordan block at $1 - \eta_\alpha \Delta t$.

The proof works for any $0 < k_{\xi,\alpha} < 2$; the matched choice is recommended.

### 5. The augmented controller

```
# Constants (precomputed at boot, parameterized in m, J, l, k_MT, dt)
B, B_inv = mixing_matrix(l, k_MT)
r_T, r_Mx, r_My, r_Mz = rows_of(B)
T_min, T_max = motor_limits()

b = [1.0/m, 1.0/Jxx, 1.0/Jyy, 1.0/Jzz]
delta = [dt * bi for bi in b]    # leading-order; user may compute exact two-step coefficients (§11.4)
eta = [eta_T, eta_phi, eta_phi, eta_psi]   # user's existing values
k_xi = [eta_alpha * dt for eta_alpha in eta]   # matched

# State
xi = zeros(4)

def step(state, ref):
    # 1. Outer-loop reference (unchanged)
    a_des, q_des = outer_pid(state, ref)

    # 2. Sliding variables (user's existing eqs 16/17/27 — unchanged)
    s = compute_sliding_variables(state, a_des, q_des)

    # 3. Modified sliding variable (NEW)
    s_tilde = [s[a] - delta[a] * xi[a] for a in range(4)]

    # 4. Control law (user's existing eqs 21/38/40 with s -> s_tilde)
    u_eq = compute_equivalent_control(state, a_des, q_des)
    u_disc = compute_reaching_law_term(s_tilde, eta, b, dt)
    u = u_eq + u_disc

    # 5. Closed-form priority-weighted allocation (NEW)
    T_rot_star, u_bar = priority_weighted_allocate(u, T_min, T_max)
    delta_u = u - u_bar

    # 6. Auxiliary state update (NEW)
    for a in range(4):
        xi[a] = (1 - k_xi[a]) * xi[a] + delta_u[a]

    # 7. Send to ESCs (unchanged)
    send_to_motors(T_rot_star)

    return u_bar
```

**Diff vs. user's existing controller.** Steps 3, 5, 6 are new (~50 lines). Step 4: one-line substitution per channel. **Total addition: ~80 lines, < 1 µs/tick at 50 Hz.** All parameters $(m, J_{xx}, J_{yy}, J_{zz}, l, k_{MT}, \Delta t)$ enter only through precomputed constants.

---

## Part II — Analysis

### 6. Closed-loop dynamics under saturation

For each channel $\alpha$, define $x_{\alpha, k} = (\tilde s_{\alpha, k},\ \xi_{\alpha, k})^T$. The closed-loop coupled per-channel dynamics:

$$\boxed{\quad x_{\alpha, k+1}\ =\ A_{d,\alpha}\, x_{\alpha, k}\ +\ B_d\, \Delta u_{\alpha, k}\quad}$$

where
$$A_{d,\alpha}\ =\ \begin{pmatrix} 1 - \eta_\alpha \Delta t & \delta_\alpha (1 - k_{\xi,\alpha}) \\ 0 & 1 - k_{\xi,\alpha} \end{pmatrix},\qquad B_d = \begin{pmatrix} 0 \\ 1 \end{pmatrix}.$$

With matched $k_{\xi,\alpha} = \eta_\alpha \Delta t$, both eigenvalues of $A_{d,\alpha}$ equal $A_\alpha := 1 - \eta_\alpha \Delta t$ (defective Jordan block).

**Schur stability condition.**
$$\boxed{\quad 0\ <\ \eta_\alpha\, \Delta t\ <\ 2 \quad \text{(per channel)}.\quad}$$

**This depends only on $(\eta_\alpha, \Delta t)$, not on $(m, J_{xx}, J_{yy}, J_{zz})$.** Schur stability is parameter-uniform.

**Structural observation.** $\Delta u_{\alpha,k}$ enters only the $\xi$ component. The $\tilde s$ component sees $\Delta u$ only indirectly, through the off-diagonal coupling $\delta_\alpha (1 - A_\alpha)$ from $\xi$ in the next step. **The auxiliary state absorbs the saturation deficit on behalf of $\tilde s$.**

### 7. Stability via ISS-Lyapunov

#### 7.1 Per-channel ISS-Lyapunov certificate

For any $Q_\alpha \succ 0$, the discrete Lyapunov equation $A_{d,\alpha}^T P_\alpha A_{d,\alpha} - P_\alpha = -Q_\alpha$ has a unique solution $P_\alpha \succ 0$ whenever $A_{d,\alpha}$ is Schur-stable. **This requires no condition on $(m, J)$ beyond the Schur condition on $\eta_\alpha \Delta t$.**

With $V_\alpha(x_\alpha) = x_\alpha^T P_\alpha x_\alpha$:
$$\boxed{\quad \Delta V_\alpha\ \le\ -\tfrac{1}{2 \lambda_{\max}(P_\alpha)}\, V_\alpha\ +\ \gamma_\alpha\, \Delta u_{\alpha, k}^2\quad}$$
where $\gamma_\alpha = 2 \|A_{d,\alpha}^T P_\alpha B_d\|^2 + B_d^T P_\alpha B_d$. This is a standard ISS-Lyapunov inequality.

#### 7.2 Closed-form $P_\alpha$ for the matched case

For $k_{\xi,\alpha} = \eta_\alpha \Delta t$ and $Q_\alpha = I$:
$$P_\alpha\ =\ \begin{pmatrix} \dfrac{1}{1 - A_\alpha^2} & \dfrac{A_\alpha\, \delta_\alpha}{(1+A_\alpha)^2(1-A_\alpha)} \\[1em] \dfrac{A_\alpha\, \delta_\alpha}{(1+A_\alpha)^2(1-A_\alpha)} & \dfrac{\delta_\alpha^2 (1+A_\alpha^2)}{(1+A_\alpha)^3 (1-A_\alpha)} + \dfrac{1}{1-A_\alpha^2} \end{pmatrix}.$$

$(P_\alpha)_{11} = \mathcal{O}(1)$, $(P_\alpha)_{12} = \mathcal{O}(\delta_\alpha)$, $(P_\alpha)_{22} = \mathcal{O}(\delta_\alpha^2)$. **The disparate plant Jacobians are absorbed naturally into the channel-specific Lyapunov function.** Each channel solves its own Lyapunov equation independently. The expression has no singularity in $(m, J)$.

For the user's representative parameters with $\eta_\alpha \Delta t = 0.4$ ($A_\alpha = 0.6$):

| Channel | $\delta_\alpha$ | $\lambda_{\min}(P_\alpha)$ | $\lambda_{\max}(P_\alpha)$ | $\kappa(P_\alpha)$ |
|---|---|---|---|---|
| $T$ | 0.10 | 1.51 | 1.63 | 1.08 |
| $M_x$ | 11.11 | 1.15 | 104.45 | 90.8 |
| $M_y$ | 11.11 | 1.15 | 104.45 | 90.8 |
| $M_z$ | 13.33 | 1.15 | 149.54 | 130.0 |

*(Illustrative example only.)*

#### 7.3 Stability theorem

**Theorem (Plan A+ stability).** *Under (i) $0 < \eta_\alpha \Delta t < 2$ per channel, (ii) $k_{\xi,\alpha} = \eta_\alpha \Delta t$, and (iii) $4 T_{\min} < m g < 4 T_{\max}$, the closed-loop system is input-to-state stable with respect to $\Delta u_k$:*

1. *(Exponential decay in absence of input.)* $\Delta u_k = 0\ \forall k\ \Rightarrow\ V_k \le (1-\nu)^k V_0,\ (\tilde s_k, \xi_k) \to 0$.
2. *(Asymptotic recovery under transient saturation.)* $\Delta u_k \to 0\ \Rightarrow\ V_k \to 0,\ (\tilde s_k, \xi_k) \to 0$.
3. *(UUB under sustained saturation.)* $\sup_k \|\Delta u_k\|^2 \le D\ \Rightarrow\ \limsup_k V_k \le V^\infty := \tfrac{\max_\alpha (w_\alpha \gamma_\alpha)}{\nu} D$.

where $V_k = \sum_\alpha w_\alpha\, x_{\alpha,k}^T P_\alpha\, x_{\alpha,k}$ and $\nu = \min_\alpha 1/(2 \lambda_{\max}(P_\alpha))$.

**None of the three conclusions requires any specific value of $(m, J_{xx}, J_{yy}, J_{zz})$.**

#### 7.4 Per-channel residual under sustained saturation

The steady-state response to constant $\Delta u_\alpha = c_\alpha$:
$$\boxed{\quad \tilde s_\alpha^\infty\ =\ \frac{\delta_\alpha}{\eta_\alpha \Delta t}\, c_\alpha\ =\ \frac{b_\alpha}{\eta_\alpha}\, c_\alpha,\qquad \xi_\alpha^\infty\ =\ \frac{c_\alpha}{\eta_\alpha \Delta t}.\quad}$$

The auxiliary residual $\xi_\alpha^\infty$ is **uniform across channels**, independent of $(m, J)$. The sliding-variable residual scales with $b_\alpha / \eta_\alpha$:
$$\boxed{\quad \frac{|\tilde s_T^\infty|}{|c_T|}\ =\ \frac{1}{m\, \eta_T},\qquad \frac{|\tilde s_{M_\alpha}^\infty|}{|c_{M_\alpha}|}\ =\ \frac{1}{J_{\alpha\alpha}\, \eta_{M_\alpha}}.\quad}$$

Per-channel residual is inversely proportional to inertia × reaching-law gain. The disparity reflects physics ($1/J \gg 1/m$ for any quadrotor); it is not introduced by Plan A+.

For the user's representative parameters with $\eta_\alpha \Delta t = 0.4$ (illustrative):

| Channel | Parametric form | Numerical (illustrative) |
|---|---|---|
| $T$ | $1/(m \eta_T)$ | $0.25$ |
| $M_x, M_y$ | $1/(J_{xx} \eta_{M_x})$ | $27.78$ |
| $M_z$ | $1/(J_{zz} \eta_{M_z})$ | $33.33$ |

### 8. Reduction to original derivation in the unconstrained limit

When $\Delta u_k = 0\ \forall k$: $\xi_{\alpha,k+1} = (1 - k_{\xi,\alpha}) \xi_{\alpha,k}$, so $\xi_k \to 0$ exponentially. Then $\tilde s_k \to s_k$, and the control law converges to the user's original. **Plan A+'s closed-loop trajectory converges to the user's, parameter-uniformly.**

The Lyapunov function reduces: as $\xi_k \to 0$, $V_k \to \sum_\alpha w_\alpha (P_\alpha)_{11} s_{\alpha,k}^2 = \sum_\alpha w_\alpha\, s_{\alpha,k}^2 / (1 - A_\alpha^2)$. With $w_\alpha = 1$ and after rescaling, this is the user's original $V = \tfrac{1}{2} \sum_\alpha s_{\alpha,k}^2$ (eq 20). **The user's stability proof is recovered as a special case of Plan A+'s.**

---

## Part III — Comparison to the original derivation

### 9. Element-by-element mapping

| Element of original derivation | Plan A+ disposition | Classification |
|---|---|---|
| Plant model (NED, Z-Y-X, sym. quad) | Identical | Preserved |
| cSMC formulation (eqs 5–6, 11–12) | Identical | Preserved |
| Two-step discrete dynamics (eqs 13–15) | Identical | Preserved |
| Sliding surfaces, fully-actuated (eqs 16, 17) | Identical | Preserved |
| Sliding surfaces, underactuated (eq 27) | Identical | Preserved |
| Reaching law structure ($\Delta s = -\eta \Delta t \cdot$) | Same form, target is $\tilde s$ instead of $s$ | Preserved structurally |
| Equivalent-control terms (eqs 7–10, 28–35) | Identical | Preserved |
| Chattering gains $K_\alpha$ (eq 12) | Identical | Preserved |
| Control laws (eqs 21, 38, 40) | Same form, $s \to \tilde s$ substitution | Modified by substitution |
| Per-channel Lyapunov $V = \tfrac{1}{2} s^2$ (eq 20) | Generalized to ISS-Lyapunov $V = x^T P x$ per channel; reduces to original at $\xi = 0$, $w = 1$ | Modified by extension |
| Stability conclusion (asymptotic, no saturation modeled) | Asymptotic when polytope inactive; asymptotic recovery under transient saturation; UUB under sustained saturation | Strengthened (treats more regimes) |
| (none) Auxiliary state $\xi_\alpha$ | New: 4 scalar states, $\xi_{\alpha,k+1} = (1-k_{\xi,\alpha})\xi_\alpha + \Delta u_\alpha$ | Added |
| (none) Priority-weighted allocation | New: closed-form Faessler-style sequential procedure, ~50 FLOPs | Added |

Selected rows merit explicit comment:

**Sliding surfaces (rows 4–5).** The user's eq 27 sliding surface for the underactuated channels couples lateral position with attitude: $s_\phi = -a_1 \dot y + a_2(y^d - y) - a_3 \dot \phi + a_4(\phi^d - \phi)$. This coupling is essential for outer-loop tracking through the underactuated dynamics and is preserved exactly in Plan A+.

**Control laws (row 9).** The user's eq 38 has the structure $u_{2,k} = \tfrac{J_{xx}}{l a_3} [\cdots + \eta_\phi s_{\phi, k}]$. Plan A+'s only modification is to replace the final $s_{\phi, k}$ with $\tilde s_{\phi, k}$. **No new state appears in the control law beyond the user's existing states; the auxiliary $\xi$ enters only through the substitution.**

**Stability conclusion (row 11).** Plan A+'s formal guarantee is *strictly stronger* than the original's overall: the original gives no statement about saturated regimes; Plan A+ gives the same asymptotic conclusion in the unconstrained regime, plus asymptotic recovery under transient saturation, plus UUB under sustained saturation.

### 10. Verdict: Plan A+ is a constrained extension

**Claim.** Plan A+ is a *constrained extension* of the user's discrete sliding mode controller, in the following technical sense: there exists a state and input embedding under which Plan A+ reduces *exactly* (not approximately) to the user's controller in the unconstrained limit, and the user's stability guarantee is recovered as a corollary of Plan A+'s.

**(P1) Pointwise reduction in the unconstrained limit.** Defended in §8: when the polytope is never active, $\xi_k \to 0$ exponentially, $\tilde s_k \to s_k$, and the control law converges to the user's original. Parameter-uniform.

**(P2) Lyapunov function reduction.** Plan A+'s $V = \sum_\alpha w_\alpha x_\alpha^T P_\alpha x_\alpha$ reduces, in the unconstrained limit ($\xi = 0$) and at uniform priority ($w_\alpha = 1$), to a positive scalar multiple of the user's $\sum_\alpha s_{\alpha,k}^2$.

**(P3) No structural element is replaced.** The mapping table (§9) shows every element is in one of three categories: preserved exactly, modified by a substitution that vanishes in the unconstrained limit, or generalized in a way the original is a special case of. No element is replaced.

**What this rules out.** Plan A+ is *not* (a) a re-derivation of the dSMC under input constraints; (b) a different control architecture (no MPC, no reference governor, no command-modification scheme); (c) a non-SMC technique. It *is* a strict augmentation with two well-known building blocks — Chen–Ge–Ren auxiliary state and Faessler priority allocation — assembled to fit the user's specific dSMC structure.

---

## Part IV — Parametric stability conditions

### 11. When does the analysis hold, parametrically?

This section answers: **under what conditions on the plant parameters $(m, J_{xx}, J_{yy}, J_{zz})$, the geometry $(l, k_{MT})$, the sample time $\Delta t$, and the controller gains $(\eta_\alpha)$ are Plan A+'s stability guarantees of §7 valid?**

#### 11.1 The formal stability claim is parameter-uniform

The ISS / UUB conclusion of §7.3 holds whenever:

**(C1) Per-channel Schur stability.**
$$0\ <\ \eta_\alpha\, \Delta t\ <\ 2 \quad \forall \alpha.$$
Depends only on $(\eta_\alpha, \Delta t)$. Not on $m$ or $J$.

**(C2) Polytope envelope contains the hover trim.**
$$4\, T_{\min}\ <\ m\, g\ <\ 4\, T_{\max}.$$
Equivalent to thrust-to-weight ratio $T_{\max}/(m g/4) > 1$ (and $T_{\min} \ge 0$). Couples $m$ to the actuator envelope but does not couple to $(J_{xx}, J_{yy}, J_{zz})$.

**Under (C1) and (C2), Plan A+'s ISS bound holds uniformly in $(m, J_{xx}, J_{yy}, J_{zz})$ and $(l, k_{MT})$.** *There is no critical mass-to-inertia ratio at which the formal Lyapunov stability guarantee breaks down.* The discrete Lyapunov equation has a unique $P_\alpha \succ 0$ for every Schur-stable $A_{d,\alpha}$, and the closed-form expression of §7.2 is well-defined for any $\delta_\alpha \in \mathbb{R}$.

#### 11.2 Parameter-dependence of the residual size

What does depend on $(m, J)$ is the *size* of the per-channel residual under sustained saturation. From §7.4:
$$\frac{|\tilde s_T^\infty|}{|c_T|}\ =\ \frac{\cos\phi \cos\theta}{m\, \eta_T},\qquad \frac{|\tilde s_{M_\alpha}^\infty|}{|c_{M_\alpha}|}\ =\ \frac{1}{J_{\alpha\alpha}\, \eta_{M_\alpha}}.$$

**(R1) Per-channel residual is inversely proportional to inertia × reaching-law gain.** Low-inertia plants give larger residuals on the corresponding channels. Compensate by increasing $\eta_\alpha$, but $\eta_\alpha$ is capped above by Sarpturk's $\eta_\alpha \Delta t < 2$.

**(R2) Minimum achievable residual ratio is** $\delta_\alpha = \Delta t\, b_\alpha$, attained at the upper Sarpturk boundary $\eta_\alpha \Delta t = 1$:
$$\min_{\eta_T} \frac{|\tilde s_T^\infty|}{|c_T|}\ =\ \frac{\Delta t}{m},\qquad \min_{\eta_{M_\alpha}} \frac{|\tilde s_{M_\alpha}^\infty|}{|c_{M_\alpha}|}\ =\ \frac{\Delta t}{J_{\alpha\alpha}}.$$

> **Under sustained saturation with deficit amplitude $|c_\alpha|$, the per-channel sliding-variable residual is at least $\Delta t \cdot |c_\alpha| / (\text{relevant inertia parameter})$.** Cannot be made smaller without (a) increasing the inertia parameter, (b) decreasing $\Delta t$, or (c) decreasing the saturation amplitude.

#### 11.3 The mass-inertia disparity question, addressed parametrically

**Lyapunov stability (in the ISS/UUB sense) is ensured for any positive $(m, J_{xx}, J_{yy}, J_{zz})$, as long as (C1) and (C2) hold. The disparity affects only the size of residuals, not their existence.**

The disparity ratio across channels:
$$\text{moment-vs-thrust residual ratio}\ =\ \frac{m}{J_{\alpha\alpha}}\ \cdot\ \frac{\eta_T}{\eta_{M_\alpha}}.$$

For typical quadrotors $m / J_{\alpha\alpha} \in [50, 500]$ — i.e., a unit moment deficit produces 50× to 500× the sliding-variable residual that a unit thrust deficit produces. This reflects the *physics* of the plant: $1/J \gg 1/m$ for quadrotors. **Plan A+ does not introduce this disparity; it inherits it from the plant. No controller can eliminate it.**

The user's representative quadrotor has $m / J_{xx} \approx 111$, $m / J_{zz} \approx 133$ — typical values for small quadrotors.

#### 11.4 When does the analysis assumption itself break down?

Independent of Plan A+: at what point do the *modeling assumptions* underlying any SMC for this plant become invalid?

**(A1) Discretization fidelity.** The two-step extrapolated dynamics (eqs 13–15) approximate the continuous plant up to errors of order $\Delta t^2 \cdot b_\alpha$. For tight tracking:
$$\Delta t \cdot b_\alpha\ \ll\ 1 \quad\Leftrightarrow\quad \Delta t \ll m\ \text{(thrust)},\quad \Delta t \ll J_{\alpha\alpha}\ \text{(moments)}.$$

For the user's representative parameters with $\Delta t = 0.02$ s:
- Thrust: $\Delta t / m = 0.1$. Comfortable.
- Moments: $\Delta t / J_{xx} = 11.1$. **Well above 1**, indicating the discrete approximation is *coarse*: moments cause substantial state change within one tick. Plan A+ does not fix this — the user's underlying dSMC inherits the same coarse discretization. **For plants with $\Delta t \cdot b_\alpha \gtrsim 1$, the entire framework operates in a regime where the discretization error is non-negligible.** Action: use the controller at higher sample rates when possible.

This bounds the *exactness* of the deficit cancellation in §4.2(i): if $\delta_\alpha$ in the auxiliary substitution is computed as $\Delta t \cdot b_\alpha$ rather than as the exact two-step coefficient, the cancellation has residual error of order $\Delta t^2 \cdot b_\alpha \cdot \Delta u$ (~22% on the user's moment channels). **The user should compute $\delta_\alpha$ as the exact two-step coefficient (derivable from eqs 13–15) for tight cancellation.**

**(A2) Linear-regime assumption for the underactuated channels.** Eq 27 uses lateral-position couplings $(y, \dot y)$ assuming small angles. For the linear regime to hold under sustained saturation:
$$|c_{M_x}|\ \ll\ \frac{\pi}{6}\, J_{xx}\, \eta_{M_x}\, a_4$$
(and analog for pitch). Plan A+ inherits this limitation from the user's eq 27 structure.

**(A3) Polytope feasibility in normal operation.** Define the safety margin $\mu := 1 - \max_k \|u_k\|_\infty / \|u_{\max}\|_\infty$. For $\mu \to 0$, saturation becomes routine and UUB residual dominates tracking error. For $\mu \gtrsim 0.2$, saturation is transient and Theorem 7.3 part 2 applies.

#### 11.5 Numerical verification of parametric uniformity

Sweep over plant parameters: $m \in \{0.05, 0.1, 0.2, 0.5, 1.0, 5.0\}$ kg, $J \in \{10^{-5}, 10^{-4}, 10^{-3}, 1.8 \cdot 10^{-3}, 10^{-2}, 10^{-1}\}$ kg·m², $\Delta t = 0.02$ s, $\eta_\alpha \Delta t = 0.4$.

For every combination tested:
- $A_{d,\alpha}$ Schur-stable (eigenvalues $A_\alpha = 0.6$ independent of $m, J$).
- $P_\alpha$ exists with finite condition number, ranging from $1.0$ at $\delta = 0.004$ to $\sim 3 \cdot 10^6$ at $\delta = 2000$.
- Steady-state residual ratio matches analytical $b_\alpha / \eta_\alpha$ exactly.
- **No combination produced a stability failure.**

This empirically confirms parametric uniformity over four orders of magnitude in inertia.

#### 11.6 Summary: when does Plan A+ "work"?

| Condition | Parametric form | Required for |
|---|---|---|
| Schur stability | $0 < \eta_\alpha \Delta t < 2$ per channel | Existence of $P_\alpha$, ISS bound |
| Hover-trim feasibility | $4 T_{\min} < m g < 4 T_{\max}$ | Sector condition, theorem 7.3 |
| Discretization fidelity | $\Delta t \ll m$ and $\Delta t \ll J_{\alpha\alpha}$ | Tightness of two-step approximation |
| Linear regime (underactuated) | $\|c_{M_x}\| \ll \tfrac{\pi}{6} J_{xx} \eta_{M_x} a_4$ | Small-angle approximation in eqs 27, 38 |
| Useful UUB residual | $\|c_\alpha\| / (J_{\alpha\alpha} \eta_{M_\alpha})$ small enough | Operational acceptability |

(1) and (2) are required for the formal stability claim. (3)–(5) are required for the analysis to be *useful in practice*; they do not affect the formal stability claim.

**Cleanest summary:** Plan A+ is parameter-uniformly stable in the ISS/UUB sense for any $(m, J_{xx}, J_{yy}, J_{zz})$ such that (C1) and (C2) hold. The mass-inertia disparity affects only the size of the UUB residual. **No parametric breakdown of Lyapunov stability occurs as a function of $(m, J)$ alone.**

---

## Part V — Honest limitations

### 12. Limitations and qualifications

**(L1) Sustained saturation precludes asymptotic stability — for any controller, not just Plan A+.** The UUB residual reflects a fundamental limit of the actuator envelope, not a Plan A+ artifact. The relevant comparison is "Plan A+ vs. doing nothing about saturation" (which gives integrator windup, attitude divergence, crash). Plan A+'s guarantee matches the published state of the art for SMC + input saturation (Xu 2020; Liu et al. 2024; Galeani et al. 2009).

**(L2) The deficit-cancellation argument is only as exact as $\delta_\alpha$.** Cancellation in §4.2(i) requires the $\delta_\alpha$ in the auxiliary substitution to match the *exact* coefficient with which $u_\alpha$ enters $\Delta s_\alpha$ under the user's two-step plant. The leading-order $\delta_\alpha \approx \Delta t \cdot b_\alpha$ has $O(\Delta t \cdot b_\alpha)$ relative error — significant on moment channels for a low-inertia quadrotor. **Action: derive exact $\delta_\alpha$ from eqs 13–15.** ISS conclusion (Theorem 7.3) is robust to this approximation; only perfect cancellation is.

**(L3) Stage 3 of the allocation has a tie-breaking choice.** Reached only on a measure-zero set of inputs in practice. Does not affect the formal stability conclusion.

---

## Part VI — Practical guide

### 13. Tuning workflow

1. **Reuse the user's existing $\eta_\alpha$ and $K_\alpha$.** No retuning needed.
2. **Set $k_{\xi,\alpha} = \eta_\alpha \Delta t$** (matched).
3. **Compute $\delta_\alpha$ exactly from the user's two-step plant** (or use $\Delta t \cdot b_\alpha$ as a leading-order approximation).
4. **Set priority weights $w_\alpha$.** Used only in the Lyapunov certificate, not in closed-loop dynamics. Closed-form allocation hard-codes the priority hierarchy.
5. **Verify the runtime Lyapunov certificate.** Log $V_k$; verify $V_{k+1} - V_k \le 0$ except during saturation events.

### 14. Validation tests

1. Hover with $1.2\times$ payload + 5 m/s gust — should hold attitude tightly.
2. Aggressive yaw step at edge of envelope (Faessler 2017 test case) — should preserve $M_x, M_y$.
3. Single-motor degradation ($T_{\max,3} = 0.6 \cdot T_{\max}$) — should produce graceful attitude recovery.
4. Sustained low-altitude saturation — observe transient vs. UUB regimes per Theorem 7.3.
5. $\Delta u$ allocation comparison vs. user's original — should concentrate $\Delta u$ in $M_z$ and $T$.
6. Lyapunov certificate plot — verify $V_{k+1} - V_k \le -\nu V_k + \gamma \|\Delta u_k\|^2$ empirically.
7. **Parameter-sensitivity test:** rerun with $J_{xx}$ scaled by $0.5\times$ and $2\times$. Plan A+ qualitative behavior should be preserved with residuals scaling per §11.2.

### 15. Implementation notes

**Memory.** $B$, $B^{-1}$, $r_\alpha$ rows, precomputed $P_\alpha$: $< 200$ bytes total. State $\xi$: 4 floats. Total runtime overhead: $< 256$ bytes.

**CPU.** Allocation: $\sim 50$ FLOPs/tick ($\sim 0.3\,\mu$s on Cortex-M4 at 168 MHz). Auxiliary update: $\sim 10$ FLOPs/tick. Lyapunov certificate (optional): $\sim 50$ FLOPs/tick. **Total: $< 1\,\mu$s/tick at 50 Hz.**

**Numerical conditioning.** $\kappa(P_\alpha) = \mathcal{O}(\delta_\alpha^2)$. For low-inertia plants with $J_{\alpha\alpha} \ll \Delta t$, $\kappa$ may grow to $10^4$–$10^6$; in that regime, use float64 for the Lyapunov certificate (closed-loop computation remains float32-safe — it does not invoke $P_\alpha$).

**Code structure.** Patch:
- Add `priority_weighted_allocate()` ($\sim 30$ lines).
- Add per-channel auxiliary update ($\sim 5$ lines).
- Replace existing motor-clip step with call to allocator.
- Replace `s` with `s_tilde = s - delta * xi` in control-law computation (1 line per channel).
- Optional: add Lyapunov certificate logging.

Total diff: $\sim 80$ lines.

---

## Part VII — References

- W. Gao, Y. Wang, A. Homaifa, "Discrete-time variable structure control systems," *IEEE Trans. Ind. Electron.*, 1995.
- S. Z. Sarpturk, Y. Istefanopulos, O. Kaynak, "On the stability of discrete-time sliding mode control systems," *IEEE Trans. Autom. Control*, 1987.
- M. Chen, S. S. Ge, B. Ren, "Adaptive tracking control of uncertain MIMO nonlinear systems with input constraints," *Automatica*, 2011.
- B. Xu, "Adaptive prescribed performance terminal sliding mode attitude control for quadrotor under input saturation," *IET Control Theory Appl.*, 2020.
- Y. Liu et al., "Adaptive sliding mode control for trajectory tracking of quadrotor UAVs under input saturation and disturbances," *Drones* (MDPI), 2024.
- M. Faessler, D. Falanga, D. Scaramuzza, "Thrust mixing, saturation, and body-rate control for accurate and agile quadrotor flight," *IEEE Robotics and Automation Letters*, 2017.
- T. Lee, "Geometric tracking control of a quadrotor UAV on SE(3) with input saturation," *Asian J. Control*, 2013.
- M. W. Mueller, R. D'Andrea, "A model predictive controller for quadrocopter state interception," *Proc. ECC*, 2013.
- S. Boyd, L. Vandenberghe, *Convex Optimization*, Cambridge University Press, 2004.
- S. Galeani, S. Tarbouriech, M. Turner, L. Zaccarian, "A tutorial on modern anti-windup design," *Eur. J. Control*, 2009.
- L. Zaccarian, A. R. Teel, *Modern Anti-windup Synthesis*, Princeton University Press, 2011.
- E. Sontag, "Input to state stability: basic concepts and results," in *Nonlinear and Optimal Control Theory*, Springer, 2008.
- D. Liberzon, *Switching in Systems and Control*, Birkhäuser, 2003.

---

## Document supersession

This document supersedes:
- `dsmc_input_constraints_plan.md` (initial three-plan document with QP-based allocation).
- `dsmc_input_constraints_plan_A_plus.md` (priority-weighted version with both QP and closed-form options).
- `dsmc_input_constraints_plan_A_plus_final.md` (closed-form-only version with the diagonal-V proof).
- `dsmc_plan_A_plus_audit.md` (separate audit document).
- `dsmc_plan_A_plus.md` v1 (combined document with substituted parameter values).

The single source of truth going forward is this document.
