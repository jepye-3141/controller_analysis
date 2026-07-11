# Literature Review: Determining Launch Parameters and the Powered-Flight Transition Point of a Ballistically Launched Quadrotor to Maximize Landing Accuracy

*Compiled 2026-07-09 for the ATLIS Sims project. Companion document: `2026-07-09_implementation_strategies_launch_parameter_targeting.md`.*

## Overview

The ATLIS problem is to fire a folded quadrotor swarm from a 120 mm mortar, let it coast ballistically, deploy and spin up the rotors near apogee, and have a discrete sliding-mode controller (dSMC) arrest the tumble and hold station so the swarm lands as close as possible to a commanded ground position. Two decisions govern accuracy: the six launch parameters that set the ballistic arc (`Vo, el, az, w_z0, w_y0, p` in the code), and the transition point where passive ballistic flight hands off to powered control. This review surveys how the current literature makes both decisions, drawing on four communities that each own part of the problem — ballistically launched rotorcraft hardware, aggressive-attitude flight recovery, precision-airdrop release-point planning, and exterior-ballistic dispersion analysis — plus the powered-descent-guidance and surrogate-optimization literatures that supply the reachability object and the outer optimizer.

The review covers 31 works spanning 2005 to 2026, weighted toward the last decade. No single published pipeline solves the exact ATLIS problem (ballistic shell deploys a *powered* swarm, then targets a landing point over all six launch degrees of freedom), so the value here is in the mapping: which published method owns which sub-problem, and how the pieces compose. That mapping is worked out in the companion implementation-strategies document.

## Search Methodology

Searches ran across Google Scholar, arXiv, IEEE Xplore, AIAA ARC, Semantic Scholar, Springer, and ScienceDirect, using term families around four axes: ballistically/gun/tube-launched multirotors and their transition phase; recovery of quadrotors from arbitrary or aggressive initial attitudes; precision-airdrop computed-air-release-point and transition-altitude optimization; and exterior-ballistic trajectory prediction, dispersion, and impact-point control. A second pass covered powered-descent guidance, reachable-set and landing-footprint computation, and surrogate/Bayesian/cross-entropy optimization of expensive simulators. Bibliographic metadata and citation counts were verified against the Semantic Scholar Graph API where a DOI or arXiv ID resolved; counts are marked approximate or unavailable where the index did not return a record. Inclusion favored peer-reviewed venues (JGCD, IEEE T-RO/RA-L/T-Mech, IROS, ICRA), recognized preprints, and the standardization/reference literature (STANAG 4355, McCoy). This complements two earlier in-repo research notes (`docs/archived/ballistic_targeting_optimization.md` and `comparative_survey_targeting_optimization.md`); it re-verifies their load-bearing citations and adds the ballistic-launch-hardware and aggressive-recovery threads those notes under-covered.

## Theme 1: Ballistically and Tube-Launched Rotorcraft — the Physical System and Its Transition

These papers define the system class ATLIS belongs to: a rotorcraft folded into a projectile, launched by cannon or mortar, that unfolds and stabilizes in mid-air. They establish where the transition happens (near apogee, after passive aerodynamic stabilization) and why the deployment state matters — the drone inherits whatever attitude and body rates the ballistic phase leaves it with.

### Design and Autonomous Stabilization of a Ballistically Launched Multirotor (Bouman et al., 2019)
**Source**: IEEE ICRA 2020 | [arXiv:1911.10269](https://arxiv.org/abs/1911.10269) / [DOI:10.1109/ICRA40945.2020.9197542](https://doi.org/10.1109/ICRA40945.2020.9197542) | ~24 citations
**Method**: Hardware design plus a two-phase (passive then active) stabilization pipeline; the 6-inch SQUID carries an onboard IMU/visual-inertial suite and recovers to stable hover after a 15 m/s pneumatic launch.
**Summary**: SQUID passively orients into the airstream during the ballistic phase using deployable fins, then triggers active stabilization once the arms lock — before apogee. The paper reports the full launch-to-hover transition with onboard state estimation.
**Relevance**: The canonical academic reference for the ATLIS system class and the direct precedent for "passive ballistic phase → active stabilization near apogee." The design intent — make the attitude at the start of active control *predictable* — is exactly the deployment-state conditioning that a launch-parameter optimizer needs.

### Design of a Ballistically-Launched Foldable Multirotor (Pastor et al., 2019)
**Source**: IEEE/RSJ IROS 2019 | [DOI:10.1109/IROS40897.2019.8968549](https://doi.org/10.1109/IROS40897.2019.8968549) | ~14 citations
**Method**: Mechanical design and field demonstration of a 3D-printed multirotor launched from a three-inch barrel, with a nichrome burn-wire arm-release mechanism; tested from a vehicle moving up to 50 mph.
**Summary**: The proof-of-concept establishing feasibility of a passive ballistic-to-controllable transition, validating aerodynamic stability and deployment reliability of the folding scheme.
**Relevance**: Documents the launch-from-a-moving-platform case and the mechanical deployment trigger, both of which set the initial-condition dispersion that ATLIS's ballistic phase must model.

### Conception and Manufacturing of a Projectile-Drone Hybrid System (Gnemmi et al., 2017)
**Source**: IEEE/ASME Transactions on Mechatronics 22(2), 940–951 | [DOI:10.1109/TMECH.2017.2654018](https://doi.org/10.1109/TMECH.2017.2654018) | ~25 citations
**Method**: Full system engineering of the GLMAV (Gun-Launched Micro Air Vehicle) — a ~1 kg tube-launched coaxial rotorcraft — covering ballistic launch, coaxial-rotor deployment, and transition to powered hover.
**Summary**: The French-German ISL program's hybrid projectile-drone, launched from a sub-10 kg portable tube, deploys counter-rotating rotors after a ballistic arc and transitions to hovering/maneuvering flight over the target.
**Relevance**: The closest fielded analog to ATLIS in launch energy and mission profile (tube launch, ballistic coast, deploy over target). Its transition-phase control challenge motivates the whole "recover from ballistic hand-off" problem.

### The Transition Phase of a Gun-Launched Micro Air Vehicle (Chauffaut, Escareño & Lozano, 2012)
**Source**: Journal of Intelligent & Robotic Systems | [DOI:10.1007/s10846-012-9732-3](https://doi.org/10.1007/s10846-012-9732-3) | citations not available (sparsely indexed)
**Method**: Theoretical study and high-fidelity simulation of the GLMAV's ballistic, transient, and operational phases, proposing a control policy to stabilize the vehicle through rotor deployment.
**Summary**: Isolates the transition phase as the critical control problem — the rotors deploy from the rear, start turning, and must arrest the projectile's ballistic state — and validates a stabilizing policy in simulation.
**Relevance**: The earliest explicit treatment of "control authority begins mid-ballistic-arc." Its framing of the transient as the make-or-break window is precisely ATLIS's apogee-deployment tumble-arrest problem.

### Design, Development, and Flight Testing of a Tube-Launched Coaxial-Rotor Based Micro Air Vehicle (Denton, Benedict & Kang, 2022)
**Source**: International Journal of Micro Air Vehicles 14 | [DOI:10.1177/17568293221117189](https://doi.org/10.1177/17568293221117189) | ~9 citations
**Method**: Design and flight test of a 366 g, 52 mm-folded coaxial-rotor TLMAV with a thrust-vectoring gimbal for pitch/roll and differential rpm for yaw; launched vertically from a pneumatic cannon.
**Summary**: Demonstrates a stable projectile phase, passive rotor unfolding, and transition to hover *from arbitrarily large attitude angles*, with wind-tunnel-verified gust tolerance to 5 m/s.
**Relevance**: The Texas A&M TLMAV is the grenade-launcher-scale sibling of ATLIS. "Transition to hover from arbitrarily large attitude angles" is the same tumble-recovery specification ATLIS's dSMC must meet, and the coaxial thrust-vectoring actuation is a design point of comparison.

### Development of a Tube-Launched Tail-Sitter Unmanned Aerial Vehicle (Cai, Denton, Benedict & Kang, 2024)
**Source**: International Journal of Micro Air Vehicles 16 | [DOI:10.1177/17568293241254045](https://doi.org/10.1177/17568293241254045) | ~6 citations
**Method**: Design and testing of a tube-launched tail-sitter variant, extending the TLMAV line toward fixed-wing-assisted range with a rotor-borne transition.
**Summary**: Broadens the tube-launched-MAV design space to a hybrid tail-sitter, characterizing the launch-to-controlled-flight transition for a different airframe.
**Relevance**: Shows the design-space breadth of the launched-MAV community and that the transition-management problem recurs across airframes — useful context for where ATLIS's rotor-only approach sits.

### System Identification of a Thrust-Vectoring, Coaxial-Rotor-Based Gun-Launched Micro Air Vehicle in Hovering Flight (Denton, Benedict & Kang, 2025)
**Source**: International Journal of Micro Air Vehicles 17 | [DOI:10.1177/17568293251361078](https://doi.org/10.1177/17568293251361078) | citations not available (2025)
**Method**: Frequency-domain system identification of the hovering TLMAV to extract a control-oriented dynamics model.
**Summary**: Produces an identified model of the coaxial gun-launched MAV in hover, closing the loop between hardware and a model usable for controller synthesis.
**Relevance**: The kind of identified plant model that a high-fidelity ATLIS reachability study would need if it moved beyond the current linearized 12-state model; a template for validating the dynamics that feed the deployment-to-landing reachability map.

### Mid-Air Helicopter Delivery at Mars Using a Jetpack (Delaune et al., 2022)
**Source**: IEEE Aerospace Conference 2022 | [arXiv:2203.03704](https://arxiv.org/abs/2203.03704) / [DOI:10.1109/AERO53065.2022.9843825](https://doi.org/10.1109/AERO53065.2022.9843825) | ~7 citations
**Method**: EDL architecture design in which a jetpack decelerates a Mars helicopter after backshell separation to reach aerodynamic conditions suitable for mid-air rotor take-off.
**Summary**: A lander-free entry-descent-landing concept that deliberately chooses the mid-air state at which the rotorcraft takes over, trading backshell mass for payload.
**Relevance**: A planetary instance of "pick the mid-air hand-off state to make the powered phase succeed." It frames transition-state selection as a design variable with a downstream feasibility constraint — the same coupling ATLIS optimizes between launch parameters and deployment state.

## Theme 2: Recovery and Stabilization from Aggressive or Arbitrary Initial Conditions

The powered phase of ATLIS begins with the drone tumbling at whatever rates the ballistic separation imparts. This theme is the control literature on recovering a multirotor from large-attitude, high-rate initial states — the physics that the ATLIS reachability map summarizes, and the benchmark its dSMC controller is implicitly competing against.

### Automatic Re-Initialization and Failure Recovery for Aggressive Flight with a Monocular Vision-Based Quadrotor (Faessler et al., 2015)
**Source**: IEEE ICRA 2015 | [DOI:10.1109/ICRA.2015.7139420](https://doi.org/10.1109/ICRA.2015.7139420) | ~110 citations
**Method**: Staged recovery controller — stabilize attitude, then altitude, then re-initialize visual state estimation — demonstrated by throwing the quadrotor into the air.
**Summary**: Recovers from unknown initial attitude with significant velocity at >85% success over hundreds of throws, with accelerations above 40 m/s² outdoors, using only onboard monocular vision.
**Relevance**: Establishes the staged "arrest attitude first" recovery philosophy that ATLIS's tumble-arrest phase mirrors, and provides a success-rate benchmark for recovery-from-throw that the ballistic-deployment envelope can be measured against.

### Control of a Quadrotor with Reinforcement Learning (Hwangbo et al., 2017)
**Source**: IEEE Robotics and Automation Letters 2(4) | [DOI:10.1109/LRA.2017.2720851](https://doi.org/10.1109/LRA.2017.2720851) | ~567 citations
**Method**: A neural-network policy mapping state directly to rotor commands, trained with a stability-oriented RL algorithm; validated in simulation and on hardware.
**Summary**: The learned policy stabilizes a quadrotor thrown upside-down with 5 m/s initial velocity, demonstrating recovery from very harsh initialization without a predefined control structure.
**Relevance**: A direct data point on the achievable recovery envelope from inverted, high-rate states — the operating regime of an apogee-deployed tumbling swarm — and an alternative controller class (learned policy) to benchmark the dSMC reachability against.

### A Computationally Efficient Motion Primitive for Quadrocopter Trajectory Generation (Mueller, Hehn & D'Andrea, 2015)
**Source**: IEEE Transactions on Robotics 31(6) | [DOI:10.1109/TRO.2015.2479878](https://doi.org/10.1109/TRO.2015.2479878) | ~365 citations
**Method**: Closed-form state-to-state motion primitives with feasibility (thrust/body-rate) verification, generating and checking on the order of a million trajectories per second.
**Summary**: Any initial position/velocity/acceleration is connected to a target state by a jerk-optimal primitive, with fast input-feasibility tests; demonstrated in ball-catching interception.
**Relevance**: The enabling technology for exhaustively evaluating "can the drone reach the target from this deployment state?" at scale. Its million-trajectories-per-second feasibility check is exactly the primitive an ATLIS reachability-heatmap or divert-capability computation would build on.

## Theme 3: Release-Point and Transition-Altitude Optimization for Precision Airdrop

This is the closest published analog to the ATLIS launch-parameter problem. The precision-airdrop community has, for a decade, chosen release point, aircraft heading, and parachute transition altitude to hit a desired impact distribution under wind and dispersion uncertainty. Swap "parachute recourse" for "powered dSMC recourse" and the mathematical structure — maximize expected overlap between a deployment-conditioned reachability object and the target distribution — is identical.

### A Probabilistic Algorithm for Ballistic Parachute Transition Altitude Optimization (Leonard et al., 2017)
**Source**: Journal of Guidance, Control, and Dynamics 40(12), 3037–3049 | [DOI:10.2514/1.G002243](https://doi.org/10.2514/1.G002243) | ~4 citations
**Method**: Propagates a desired impact-point distribution *backward* in time via the Stochastic Liouville Equation (a Fokker–Planck–Kolmogorov form) to the airdrop altitude, then optimizes release point, heading, and transition altitude.
**Summary**: The desired impact distribution is an input; the algorithm shapes dispersion, improves accuracy, and steers around obstacles or across multiple drop zones by choosing when the main parachute deploys.
**Relevance**: The single most transferable template for ATLIS. "Transition altitude" is ATLIS's deployment point; "release point/heading" are its launch parameters; the backward-density-propagation-to-a-decision structure is directly portable to selecting `(Vo, el, az, p)` for a commanded landing distribution.

### Koopman Operator Approach to Airdrop Mission Planning Under Uncertainty (Leonard, Rogers & Gerlach, 2019)
**Source**: Journal of Guidance, Control, and Dynamics | [DOI:10.2514/1.G004277](https://doi.org/10.2514/1.G004277) | ~10 citations
**Method**: Uses the Koopman operator to pull a mission score function back through the family of stochastic airdrop dynamics, enabling release-point optimization without per-candidate Monte Carlo re-simulation.
**Summary**: Recasts uncertainty propagation as the linear evolution of observables, giving an efficient route from launch decision to expected mission score.
**Relevance**: An efficiency play for the ATLIS outer loop. If the ballistic-plus-reachability map is expensive, a Koopman/operator surrogate of the launch→score map is an alternative to the current GP surrogate — worth knowing as a fallback if the response surface proves non-stationary.

### Probabilistic Release Point Optimization for Airdrop with Variable Transition Altitude (Leonard, Rogers & Gerlach, 2020)
**Source**: Journal of Guidance, Control, and Dynamics | [DOI:10.2514/1.G004959](https://doi.org/10.2514/1.G004959) | ~5 citations
**Method**: Joint optimization of release point and a *variable* transition altitude under wind/dispersion uncertainty, extending the 2017 algorithm to co-optimize the hand-off timing.
**Summary**: Shows that letting the transition altitude vary as a decision variable — rather than fixing it — measurably improves accuracy and dispersion shaping.
**Relevance**: Direct evidence that co-optimizing the transition point *with* the launch parameters beats fixing it. ATLIS currently deploys at apogee by construction; this paper is the argument for promoting deployment timing/state to a decision variable.

### Robust Parafoil Terminal Guidance Using Massively Parallel Processing (Rogers & Slegers, 2013)
**Source**: Journal of Guidance, Control, and Dynamics 36(5), 1336–1345 | [DOI:10.2514/1.59782](https://doi.org/10.2514/1.59782) | ~63 citations
**Method**: Ranks candidate terminal trajectories by GPU-parallel Monte Carlo simulation of impact-point variance under wind uncertainty, selecting the most robust option online.
**Summary**: Demonstrates onboard massively parallel Monte Carlo for real-time robust guidance — the first fielded system of its kind — showing thousands of stochastic rollouts per decision are tractable.
**Relevance**: The existence proof that the expensive inner object (Monte-Carlo-scored trajectory ranking) can be evaluated fast enough to sit inside a decision loop — the compute-budget argument behind treating ATLIS's `sweep_landing_centroid` as an affordable inner evaluation.

### Experimental Investigation of Stochastic Parafoil Guidance Using a Graphics Processing Unit (Slegers, Brown & Rogers, 2015)
**Source**: Control Engineering Practice 36, 27–38 | [ScienceDirect S0967066114002755](https://www.sciencedirect.com/science/article/abs/pii/S0967066114002755) | ~40 citations (approximate)
**Method**: Flight-test validation of the GPU-Monte-Carlo stochastic guidance strategy on a physical parafoil system.
**Summary**: Confirms in hardware that online stochastic ranking of trajectories improves terminal accuracy under real wind, moving the 2013 method from simulation to fielded practice.
**Relevance**: The validation half of the Rogers–Slegers line, showing the stochastic-ranking approach survives contact with real dispersion — reassurance for any ATLIS plan that scores launch candidates by simulated landing statistics.

## Theme 4: Exterior-Ballistic Trajectory Prediction and Dispersion

The forward map from launch parameters to deployment state is exterior ballistics. ATLIS already implements a 6-DOF mortar model (`eom2`, McCoy Ch. 9); this theme situates that choice, supplies the standardized reduced-order alternatives used in fire-control, and covers how launch and aerodynamic uncertainty disperse the deployment state — the noise the eventual stochastic ATLIS phase must propagate.

### Modified Projectile Linear Theory for Rapid Trajectory Prediction (Hainz & Costello, 2005)
**Source**: Journal of Guidance, Control, and Dynamics 28(5), 1006–1014 | [DOI:10.2514/1.8027](https://doi.org/10.2514/1.8027) | ~89 citations
**Method**: Linearizes projectile dynamics while retaining the dominant nonlinear effects, using the state-transition matrix for fast impact-point prediction at high gun elevations.
**Summary**: Delivers rapid yet accurate impact-point estimates for indirect-fire munitions where naive linear theory fails, at low enough cost for embedded onboard use.
**Relevance**: The reduced-order ballistic predictor that could serve as a cheap screening fidelity in a multi-fidelity ATLIS optimizer — fast enough to explore the launch box before spending 6-DOF `eom2` calls on promising regions.

### Projectile Monte-Carlo Trajectory Analysis Using a Graphics Processing Unit (Ilg, Rogers & Costello, 2011)
**Source**: AIAA Atmospheric Flight Mechanics Conference 2011 | [DOI:10.2514/6.2011-6266](https://doi.org/10.2514/6.2011-6266) | ~24 citations
**Method**: GPU-parallel 6-DOF Monte Carlo dispersion analysis of a projectile, with dispersed initial state, aerodynamic coefficients, and atmosphere.
**Summary**: Brings 6-DOF Monte Carlo dispersion into the real-time regime, establishing that thousands of high-fidelity ballistic rollouts per launch condition are computationally feasible.
**Relevance**: The compute existence proof for the ballistic half of a stochastic ATLIS pipeline: dispersing the six launch parameters through `eom2` in bulk to characterize the deployment-state distribution is tractable on commodity hardware.

### Parametric Study of Guidance of a 160-mm Projectile Steered with Lateral Thrusters (Głębocki & Jacewicz, 2020)
**Source**: Aerospace 7(5), 61 | [DOI:10.3390/aerospace7050061](https://doi.org/10.3390/aerospace7050061) | ~17 citations
**Method**: 6-DOF MATLAB/Simulink Monte Carlo of a spin-stabilized, Magnus-affected projectile with pulsed lateral thrusters, sweeping inertial, aerodynamic, wind, and initial-condition uncertainties.
**Summary**: Quantifies how each uncertainty source and control-timing choice moves the mean impact point and dispersion for a pulsed-control artillery projectile.
**Relevance**: A near-exact methodological match to ATLIS's own MATLAB/Simulink 6-DOF-plus-control setup, and a catalog of which launch and aero uncertainties dominate dispersion — the shortlist for what to promote to noise variables in a stochastic ATLIS phase.

### Prediction and Control of Projectile Impact Point Using Approximate Statistical Moments (Demir & Singh, 2017)
**Source**: arXiv preprint (also American Control Conference 2018) | [arXiv:1710.00289](https://arxiv.org/abs/1710.00289) | citations not available
**Method**: Transforms projectile dynamics so nonlinearities become monomials, then derives approximate first- and second-moment dynamics via mean-field approximation to predict impact-point mean and standard deviation.
**Summary**: Recovers reliable analytic estimates of impact-point mean and spread under wind and measurement noise without full Monte Carlo, and uses them for control.
**Relevance**: An analytic-moment alternative to Monte Carlo for propagating launch uncertainty to the deployment (or impact) distribution — cheaper than sampling if ATLIS needs fast in-the-loop dispersion estimates, and a precedent for moment-based rather than sample-based objectives.

### Validation of the NATO Armaments Ballistic Kernel for Use in Small-Arms Fire Control Systems (Corriveau, 2017)
**Source**: Defence Technology 13(3), 188–199 | [ScienceDirect S2214914717300569](https://www.sciencedirect.com/science/article/pii/S2214914717300569) | citations not available
**Method**: Compares the 4-DOF Modified Point Mass model of the NATO Armaments Ballistic Kernel (NABK, per STANAG 4355) against high-fidelity 6-DOF trajectories.
**Summary**: Establishes where the standardized reduced-order fire-control model agrees with and departs from full 6-DOF, validating NABK for fire-control use within quantified bounds.
**Relevance**: Grounds ATLIS's ballistic modeling in the fire-control standardization literature (STANAG 4355 / Modified Point Mass) and quantifies the 4-DOF-vs-6-DOF trade — relevant if a standardized, auditable ballistic kernel is ever wanted alongside the McCoy 6-DOF `eom2` implementation.

## Theme 5: Powered-Descent Guidance, Reachable Sets, and Landing Footprints

Once the drone is under power, "where can it land from this deployment state?" is a reachable-set / landing-footprint question. The planetary powered-descent-guidance community has the sharpest tools here — convex trajectory optimization with global-optimality guarantees, and data-driven reachable-set estimation — which formalize the object ATLIS currently approximates with its dSMC landing-centroid sweep.

### Convex Programming Approach to Powered Descent Guidance for Mars Landing (Açıkmeşe & Ploen, 2007)
**Source**: Journal of Guidance, Control, and Dynamics 30(5) | [DOI:10.2514/1.27553](https://doi.org/10.2514/1.27553) | ~707 citations
**Method**: Lossless convexification recasts the nonconvex minimum-fuel powered-descent problem (with a lower thrust bound) as a convex program solvable to global optimality by interior-point methods.
**Summary**: The foundational result that fuel-optimal pinpoint-landing guidance can be solved reliably and in real time, later flight-demonstrated on rocket testbeds.
**Relevance**: Defines the reference formulation for optimally guiding a powered vehicle to a target landing point. A convexified ATLIS terminal-guidance layer that replaced or audited the dSMC would start here, and its global-optimality guarantee is a benchmark for the dSMC's achievable footprint.

### Minimum-Landing-Error Powered-Descent Guidance for Mars Landing Using Convex Optimization (Blackmore, Açıkmeşe & Scharf, 2010)
**Source**: Journal of Guidance, Control, and Dynamics 33(4), 1161–1171 | [DOI:10.2514/1.47202](https://doi.org/10.2514/1.47202) | ~369 citations
**Method**: Extends lossless convexification to minimize distance-to-target when the target is unreachable, with a deterministic bound on solver iterations.
**Summary**: When no feasible trajectory reaches the target, it returns the reachable point closest to it — the precise semantics of a landing footprint's edge.
**Relevance**: This *is* the landing-footprint boundary computation. For ATLIS it formalizes the reachability half-radius the sweep estimates empirically: the set of deployment states from which the target is reachable, and how far short the miss is when it is not.

### Survey of Trajectory Optimization Methods for Mars Entry and Powered Descent (Liu, Li & Xin, 2025)
**Source**: Journal of Guidance, Control, and Dynamics | [DOI:10.2514/1.G009183](https://doi.org/10.2514/1.G009183) | ~6 citations
**Method**: Review of entry-and-powered-descent trajectory optimization, covering convex, reachable-set/controllable-set, and advanced guidance methods.
**Summary**: A current survey organizing the descent-guidance field, including reachable and controllable set analysis and their role in divert-capability assessment.
**Relevance**: The best single entry point to the powered-descent-guidance literature for ATLIS, mapping the menu of methods (and where reachable-set analysis fits) so the project can locate its dSMC-plus-sweep approach against the state of the art.

### Data-Driven Reachable Set Computation Using Adaptive Gaussian Process Classification and Monte Carlo Methods (Devonport & Arcak, 2019)
**Source**: American Control Conference 2020 | [arXiv:1910.02500](https://arxiv.org/abs/1910.02500) / [DOI:10.23919/ACC45564.2020.9147918](https://doi.org/10.23919/ACC45564.2020.9147918) | ~55 citations
**Method**: Recasts reachable-set estimation as binary classification with a Gaussian-process classifier, using the GP's uncertainty to adaptively pick new samples, with probabilistic correctness guarantees.
**Summary**: Learns a reachable set from simulator samples and refines its boundary where the classifier is least certain, bounding the sample count for a target accuracy/confidence.
**Relevance**: The method for turning ATLIS's expensive dSMC sweep into a data-efficient reachable-set/feasibility model. It directly addresses the sharp stability boundary noted in the ATLIS docs: classify feasible versus infeasible deployment states and sample adaptively near the edge, rather than gridding uniformly.

## Theme 6: Surrogate, Stochastic, and Risk-Aware Outer-Loop Optimization

The outer problem — choose launch parameters that maximize landing accuracy when each evaluation is an expensive simulation — is black-box optimization of a costly simulator. This theme covers the surrogate and sampling methods ATLIS already uses or has banked (GP/Kriging, cross-entropy), plus the risk-aware objective formulations that matter once dispersion is introduced.

### Recent Advances in Surrogate-Based Optimization (Forrester & Keane, 2009)
**Source**: Progress in Aerospace Sciences 45(1–3), 50–79 | [DOI:10.1016/j.paerosci.2008.11.001](https://doi.org/10.1016/j.paerosci.2008.11.001) | ~2416 citations
**Method**: Review of surrogate construction (Kriging/GP, RBF) and surrogate-based optimization strategies (expected improvement, infill criteria) for expensive aerospace simulations.
**Summary**: The standard reference for fitting a cheap surrogate to costly simulator output and optimizing on it, with guidance on each method's strengths and failure modes.
**Relevance**: The methodological backbone of ATLIS's existing `surrogate_optimize.m` (GP surrogate over the launch box, optimize on the mean). This survey is the citation that justifies and situates that choice and its infill/verification steps.

### A Tutorial on Bayesian Optimization (Frazier, 2018)
**Source**: arXiv preprint | [arXiv:1807.02811](https://arxiv.org/abs/1807.02811) | ~2386 citations
**Method**: Tutorial on Gaussian-process Bayesian optimization, covering expected-improvement and knowledge-gradient acquisition and the noisy-expectation setting.
**Summary**: Lays out BO for low-dimensional expensive black boxes, including acquisition functions suited to objectives that are themselves expectations over disturbances.
**Relevance**: The reference for upgrading ATLIS's deterministic on-surrogate optimization to a proper acquisition-driven loop once the inner sweep becomes stochastic — the knowledge-gradient discussion maps directly onto the banked Plan A. Note: in the current deterministic sweep the acquisition layer collapses to argmax-of-mean, so this matters mainly for the stochastic phase.

### Cross-Entropy Motion Planning (Kobilarov, 2012)
**Source**: International Journal of Robotics Research 31(7) (RSS 2011) | [author PDF](https://asco.lcsr.jhu.edu/papers/Ko2012.pdf) | ~150 citations (approximate)
**Method**: Adaptive importance sampling via the cross-entropy method over the space of trajectories, fitting a proposal distribution that concentrates on low-cost regions.
**Summary**: A gradient-free, distribution-fitting search that handles non-differentiable, multi-modal objectives by iteratively refining a sampling distribution toward the optimum.
**Relevance**: The basis for ATLIS's banked Plan B (CEM outer loop over the launch box). It is the recommended MVP outer optimizer in the in-repo comparative survey — robust to the non-smooth, possibly multi-modal launch response surface a stationary GP can struggle with.

### Sample-Efficient Cross-Entropy Method for Real-Time Planning (Pinneri et al., 2020)
**Source**: Conference on Robot Learning (CoRL) 2020 | [arXiv:2008.06389](https://arxiv.org/abs/2008.06389) | ~156 citations
**Method**: iCEM — adds temporally correlated (colored-noise) sampling, memory across iterations, and elite reuse to CEM, cutting samples 2.7–22× on high-dimensional control.
**Summary**: Makes CEM sample-efficient enough for real-time planning while preserving its gradient-free robustness.
**Relevance**: The modern upgrade to a CEM-based ATLIS outer loop. Its sample-efficiency tricks matter directly because each ATLIS launch evaluation is a ~6–16 min sweep — fewer elite samples per iteration is fewer sweeps.

### Minimum-Fuel Powered Descent in the Presence of Random Disturbances (Ridderhof & Tsiotras, 2019)
**Source**: AIAA Scitech 2019 Forum | [DOI:10.2514/6.2019-0646](https://doi.org/10.2514/6.2019-0646) | ~28 citations
**Method**: Covariance steering — designs feedback that drives the terminal state *covariance* to a prescribed Gaussian while minimizing fuel under random disturbances.
**Summary**: Provides a rigorous link between disturbance statistics and the terminal landing-state distribution, bounding dispersion by design rather than by post-hoc Monte Carlo.
**Relevance**: The tool for the risk-aware ATLIS phase (banked Plan D). Convolving a covariance-steered terminal distribution with the target gives expected hit probability analytically — an alternative to sampling the dSMC landing spread when worst-case guarantees are wanted.

### Explicit Trajectory Dispersion Control for Precision Landing Guidance of Reusable Rockets (Chen, Zhang & Li, 2025)
**Source**: Journal of Guidance, Control, and Dynamics | [DOI:10.2514/1.G009289](https://doi.org/10.2514/1.G009289) | ~1 citation
**Method**: Explicit in-loop control of trajectory dispersion for precision landing, propagating and shaping the landing-error distribution during guidance.
**Summary**: A recent method that treats landing dispersion as a controlled quantity in the guidance law itself, tightening the landing ellipse for reusable-rocket recovery.
**Relevance**: State-of-the-art evidence that landing dispersion can be a first-class controlled objective, not just a measured outcome — a forward pointer for how ATLIS might eventually shape the swarm's landing spread rather than only reporting its half-radius.

### Kill-Probability-Maximization Guidance: Breaking from the Miss-Distance-Minimization Paradigm (Mudrik & Oshman, 2026)
**Source**: arXiv preprint | [arXiv:2604.17811](https://arxiv.org/abs/2604.17811) | citations not available (2026 preprint)
**Method**: Recasts guidance to maximize single-shot kill probability under a probabilistic lethality model, modifying differential-game guidance laws via Bayesian decision theory.
**Summary**: Argues that minimizing expected miss distance is the wrong objective when a probabilistic hit/lethality function is available, and shows Monte-Carlo SSKP gains from optimizing that function directly.
**Relevance**: The conceptual argument for ATLIS's objective. The landing-reachability ratio is already a probabilistic hit function; this paper is the case for optimizing launch parameters against expected reachability-target overlap rather than centroid miss distance alone — a possible refinement of the current `‖p_centroid − p_target‖²` objective.

## Research Gaps and Future Directions

Reading across the themes, several gaps stand out for the ATLIS problem specifically.

No published pipeline co-optimizes all six launch degrees of freedom against a *powered-controller* reachability object. The airdrop work (Leonard et al.) optimizes two to three degrees of freedom with parachute recourse; the ballistic-launch rotorcraft work (SQUID, GLMAV, TLMAV) demonstrates the hardware transition but does not optimize launch parameters for landing accuracy; the powered-descent work computes footprints but from a given deployment state, not back through a ballistic phase. ATLIS sits in the intersection, and the composition is the contribution.

The transition point is treated as fixed almost everywhere the powered phase is powered rotors. ATLIS deploys at apogee by construction, yet the one paper that lets transition timing vary as a decision variable (Leonard et al., 2020) reports accuracy gains from doing so. Whether apogee is optimal for a tumbling dSMC-stabilized swarm — versus deploying earlier or later to trade tumble severity against altitude margin — is open and testable in the existing simulation.

The deterministic-versus-stochastic boundary is the central methodological fork, and the in-repo notes are candid that the current sweep is deterministic in the drone initial condition. The airdrop and projectile-dispersion literatures (Rogers–Slegers, Ilg–Rogers–Costello, Głębocki–Jacewicz) all live on the stochastic side, and their central object — the deployment-state *distribution* propagated from launch uncertainty — does not yet exist in ATLIS. Until wind, aero, and initial-condition dispersions are injected, the sophisticated stochastic machinery (knowledge-gradient BO, CEM-with-noise, covariance steering, CVaR) is solving a problem the pipeline does not have; once they are, that machinery is exactly right.

Reachability is estimated by brute grid sweep, where the reachable-set literature offers data-efficient alternatives. Devonport & Arcak's adaptive GP classification and Blackmore's minimum-landing-error convex program both compute the feasible/infeasible boundary more directly than uniform sampling — relevant precisely because the ATLIS docs flag a sharp dSMC stability boundary that a uniform grid samples wastefully.

The objective is centroid miss distance, where the field is shifting to probability-of-hit. Both the missile-guidance (Mudrik–Oshman) and airdrop (Leonard) communities optimize the overlap of a probabilistic hit function with a target distribution, not a point-to-point miss. ATLIS's reachability ratio is already such a function; the objective could follow.

## Key Takeaways

- The precision-airdrop transition-altitude / release-point line (Leonard, Rogers, Gerlach, 2017–2020) is the closest and most directly portable template for ATLIS: same decision structure (choose launch/release plus hand-off state to hit a desired impact distribution), differing only in parachute-versus-powered recourse.
- The ballistically launched rotorcraft community (SQUID, GLMAV, TLMAV) has solved the *hardware* transition ATLIS assumes but has not optimized launch parameters for landing accuracy — that optimization is where ATLIS adds to the field.
- Reachability from an arbitrary tumbling deployment state is a mature, quantifiable object: convex minimum-landing-error guidance (Blackmore et al.) and data-driven GP-classification reachable sets (Devonport & Arcak) both sharpen what the ATLIS sweep currently approximates by grid.
- The right outer optimizer depends on whether the inner sweep is deterministic (current: on-surrogate GP optimization à la Forrester–Keane suffices) or stochastic (future: knowledge-gradient BO per Frazier, or CEM/iCEM per Kobilarov and Pinneri, with covariance steering or CVaR for risk-awareness).
- Two under-explored levers with published support: promote the deployment/transition point to a decision variable (Leonard et al., 2020), and switch the objective from centroid miss to expected reachability-target overlap (Mudrik–Oshman; Leonard et al., 2017).

## References

1. Bouman, A., Nadan, P., Anderson, M., Pastor, D., Izraelevitz, J., Burdick, J., & Kennedy, B. (2019/2020). "Design and Autonomous Stabilization of a Ballistically Launched Multirotor." *IEEE ICRA*. [arXiv:1911.10269](https://arxiv.org/abs/1911.10269) · [DOI:10.1109/ICRA40945.2020.9197542](https://doi.org/10.1109/ICRA40945.2020.9197542)
2. Pastor, D., Izraelevitz, J., Nadan, P., Bouman, A., Burdick, J., & Kennedy, B. (2019). "Design of a Ballistically-Launched Foldable Multirotor." *IEEE/RSJ IROS*. [DOI:10.1109/IROS40897.2019.8968549](https://doi.org/10.1109/IROS40897.2019.8968549)
3. Gnemmi, P., Changey, S., Meder, K., Roussel, E., Rey, C., Steinbach, C., & Berner, C. (2017). "Conception and Manufacturing of a Projectile-Drone Hybrid System." *IEEE/ASME Transactions on Mechatronics* 22(2), 940–951. [DOI:10.1109/TMECH.2017.2654018](https://doi.org/10.1109/TMECH.2017.2654018)
4. Chauffaut, C., Escareño, J., & Lozano, R. (2012). "The Transition Phase of a Gun-Launched Micro Air Vehicle." *Journal of Intelligent & Robotic Systems*. [DOI:10.1007/s10846-012-9732-3](https://doi.org/10.1007/s10846-012-9732-3)
5. Denton, H., Benedict, M., & Kang, H. (2022). "Design, Development, and Flight Testing of a Tube-Launched Coaxial-Rotor Based Micro Air Vehicle." *International Journal of Micro Air Vehicles* 14. [DOI:10.1177/17568293221117189](https://doi.org/10.1177/17568293221117189)
6. Cai, J., Denton, H., Benedict, M., & Kang, H. (2024). "Development of a Tube-Launched Tail-Sitter Unmanned Aerial Vehicle." *International Journal of Micro Air Vehicles* 16. [DOI:10.1177/17568293241254045](https://doi.org/10.1177/17568293241254045)
7. Denton, H., Benedict, M., & Kang, H. (2025). "System Identification of a Thrust-Vectoring, Coaxial-Rotor-Based Gun-Launched Micro Air Vehicle in Hovering Flight." *International Journal of Micro Air Vehicles* 17. [DOI:10.1177/17568293251361078](https://doi.org/10.1177/17568293251361078)
8. Delaune, J., Izraelevitz, J., Sirlin, S., et al. (2022). "Mid-Air Helicopter Delivery at Mars Using a Jetpack." *IEEE Aerospace Conference*. [arXiv:2203.03704](https://arxiv.org/abs/2203.03704) · [DOI:10.1109/AERO53065.2022.9843825](https://doi.org/10.1109/AERO53065.2022.9843825)
9. Faessler, M., Fontana, F., Forster, C., & Scaramuzza, D. (2015). "Automatic Re-Initialization and Failure Recovery for Aggressive Flight with a Monocular Vision-Based Quadrotor." *IEEE ICRA*. [DOI:10.1109/ICRA.2015.7139420](https://doi.org/10.1109/ICRA.2015.7139420)
10. Hwangbo, J., Sa, I., Siegwart, R., & Hutter, M. (2017). "Control of a Quadrotor with Reinforcement Learning." *IEEE Robotics and Automation Letters* 2(4). [DOI:10.1109/LRA.2017.2720851](https://doi.org/10.1109/LRA.2017.2720851)
11. Mueller, M. W., Hehn, M., & D'Andrea, R. (2015). "A Computationally Efficient Motion Primitive for Quadrocopter Trajectory Generation." *IEEE Transactions on Robotics* 31(6). [DOI:10.1109/TRO.2015.2479878](https://doi.org/10.1109/TRO.2015.2479878)
12. Leonard, A., Klein, B., Jumonville, C., Rogers, J., Gerlach, A., & Doman, D. (2017). "A Probabilistic Algorithm for Ballistic Parachute Transition Altitude Optimization." *Journal of Guidance, Control, and Dynamics* 40(12), 3037–3049. [DOI:10.2514/1.G002243](https://doi.org/10.2514/1.G002243)
13. Leonard, A., Rogers, J., & Gerlach, A. (2019). "Koopman Operator Approach to Airdrop Mission Planning Under Uncertainty." *Journal of Guidance, Control, and Dynamics*. [DOI:10.2514/1.G004277](https://doi.org/10.2514/1.G004277)
14. Leonard, A., Rogers, J., & Gerlach, A. (2020). "Probabilistic Release Point Optimization for Airdrop with Variable Transition Altitude." *Journal of Guidance, Control, and Dynamics*. [DOI:10.2514/1.G004959](https://doi.org/10.2514/1.G004959)
15. Rogers, J., & Slegers, N. (2013). "Robust Parafoil Terminal Guidance Using Massively Parallel Processing." *Journal of Guidance, Control, and Dynamics* 36(5), 1336–1345. [DOI:10.2514/1.59782](https://doi.org/10.2514/1.59782)
16. Slegers, N., Brown, A., & Rogers, J. (2015). "Experimental Investigation of Stochastic Parafoil Guidance Using a Graphics Processing Unit." *Control Engineering Practice* 36, 27–38. [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0967066114002755)
17. Hainz, L. C., & Costello, M. (2005). "Modified Projectile Linear Theory for Rapid Trajectory Prediction." *Journal of Guidance, Control, and Dynamics* 28(5), 1006–1014. [DOI:10.2514/1.8027](https://doi.org/10.2514/1.8027)
18. Ilg, M., Rogers, J., & Costello, M. (2011). "Projectile Monte-Carlo Trajectory Analysis Using a Graphics Processing Unit." *AIAA Atmospheric Flight Mechanics Conference*. [DOI:10.2514/6.2011-6266](https://doi.org/10.2514/6.2011-6266)
19. Głębocki, R., & Jacewicz, M. (2020). "Parametric Study of Guidance of a 160-mm Projectile Steered with Lateral Thrusters." *Aerospace* 7(5), 61. [DOI:10.3390/aerospace7050061](https://doi.org/10.3390/aerospace7050061)
20. Demir, C., & Singh, A. (2017). "Prediction and Control of Projectile Impact Point Using Approximate Statistical Moments." *arXiv* (also ACC 2018). [arXiv:1710.00289](https://arxiv.org/abs/1710.00289)
21. Corriveau, D. (2017). "Validation of the NATO Armaments Ballistic Kernel for Use in Small-Arms Fire Control Systems." *Defence Technology* 13(3), 188–199. [ScienceDirect](https://www.sciencedirect.com/science/article/pii/S2214914717300569)
22. Açıkmeşe, B., & Ploen, S. (2007). "Convex Programming Approach to Powered Descent Guidance for Mars Landing." *Journal of Guidance, Control, and Dynamics* 30(5), 1353–1366. [DOI:10.2514/1.27553](https://doi.org/10.2514/1.27553)
23. Blackmore, L., Açıkmeşe, B., & Scharf, D. (2010). "Minimum-Landing-Error Powered-Descent Guidance for Mars Landing Using Convex Optimization." *Journal of Guidance, Control, and Dynamics* 33(4), 1161–1171. [DOI:10.2514/1.47202](https://doi.org/10.2514/1.47202)
24. Liu, X., Li, S., & Xin, M. (2025). "Survey of Trajectory Optimization Methods for Mars Entry and Powered Descent." *Journal of Guidance, Control, and Dynamics*. [DOI:10.2514/1.G009183](https://doi.org/10.2514/1.G009183)
25. Devonport, A., & Arcak, M. (2019/2020). "Data-Driven Reachable Set Computation Using Adaptive Gaussian Process Classification and Monte Carlo Methods." *American Control Conference*. [arXiv:1910.02500](https://arxiv.org/abs/1910.02500) · [DOI:10.23919/ACC45564.2020.9147918](https://doi.org/10.23919/ACC45564.2020.9147918)
26. Forrester, A. I. J., & Keane, A. J. (2009). "Recent Advances in Surrogate-Based Optimization." *Progress in Aerospace Sciences* 45(1–3), 50–79. [DOI:10.1016/j.paerosci.2008.11.001](https://doi.org/10.1016/j.paerosci.2008.11.001)
27. Frazier, P. I. (2018). "A Tutorial on Bayesian Optimization." *arXiv*. [arXiv:1807.02811](https://arxiv.org/abs/1807.02811)
28. Kobilarov, M. (2012). "Cross-Entropy Motion Planning." *International Journal of Robotics Research* 31(7), 855–871 (RSS 2011). [PDF](https://asco.lcsr.jhu.edu/papers/Ko2012.pdf)
29. Pinneri, C., Sawant, S., Blaes, S., Achterhold, J., Stueckler, J., Rolinek, M., & Martius, G. (2020). "Sample-Efficient Cross-Entropy Method for Real-Time Planning." *Conference on Robot Learning (CoRL)*. [arXiv:2008.06389](https://arxiv.org/abs/2008.06389)
30. Ridderhof, J., & Tsiotras, P. (2019). "Minimum-Fuel Powered Descent in the Presence of Random Disturbances." *AIAA Scitech 2019 Forum*. [DOI:10.2514/6.2019-0646](https://doi.org/10.2514/6.2019-0646)
31. Chen, X., Zhang, R., & Li, H. (2025). "Explicit Trajectory Dispersion Control for Precision Landing Guidance of Reusable Rockets." *Journal of Guidance, Control, and Dynamics*. [DOI:10.2514/1.G009289](https://doi.org/10.2514/1.G009289)
32. Mudrik, L., & Oshman, Y. (2026). "Kill-Probability-Maximization Guidance: Breaking from the Miss-Distance-Minimization Paradigm." *arXiv*. [arXiv:2604.17811](https://arxiv.org/abs/2604.17811)

*Foundational surrogate/design-of-experiments references already cited in the ATLIS codebase docs (McKay et al. 1979, LHS; Sacks et al. 1989, DACE; Jones et al. 1998, EGO; Rasmussen & Williams 2006, GPs; McCoy, *Modern Exterior Ballistics*, 2nd ed.) underpin Themes 4–6 and are not re-listed here.*
