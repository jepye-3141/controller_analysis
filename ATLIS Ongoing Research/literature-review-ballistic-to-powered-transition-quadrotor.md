# Literature review: optimal ballistic-to-powered flight transition for launched quadrotors, with recourse landing site considerations

## Overview

This review covers work relevant to one focused question: along the ballistic trajectory of a tube-launched quadrotor, where is the best point to start powered flight? The two reference programs are Caltech and JPL's SQUID, a folding multirotor that is fired from a tube, coasts ballistically, unfolds, and then takes over on rotor power; and Johns Hopkins APL's Dragonfly rotorcraft lander for Titan, which provides the clearest existing treatment of contingency-aware powered flight with a live list of recourse landing sites. The review covers 30 peer-reviewed papers, technical reports, and conference publications from roughly 2007 through 2025, drawn from AIAA venues, IEEE Xplore, MDPI, NASA technical reports, Springer, and the Johns Hopkins APL Technical Digest. A handful of additional references (31–37) are cited in the recourse-landing theme.

## Search methodology

Searches were run across Google Scholar, IEEE Xplore, AIAA ARC, arXiv, Semantic Scholar, NASA NTRS, and MDPI. Keywords combined primary concepts ("ballistically launched multirotor," "tube-launched UAV," "Dragonfly Titan," "SQUID drone") with method-focused terms ("convex optimization powered descent," "reachability forced landing," "successive convexification," "NMPC quadrotor recovery") and system analogs ("boost-glide guidance," "missile midcourse handover," "VTOL transition trajectory"). Papers were kept if they were peer-reviewed, came from a recognized research group, and spoke directly to either the launch-to-powered-flight transition problem, the underlying trajectory optimization methods, or the recourse/contingency landing problem. Older seminal works (Acikmese & Ploen 2007; Faessler et al. 2015) are included because downstream papers lean on them heavily.

## Theme 1: ballistically launched and tube launched quadrotors

This is the tightest research cluster around the user's scenario. The SQUID lineage is small but influential, and a scattering of recent follow-ons from other labs is beginning to fill in design and control details.

### Design of a Ballistically-Launched Foldable Multirotor (Pastor, Izraelevitz et al., 2019)
**Source**: IEEE Aerospace Conference 2019 | https://ieeexplore.ieee.org/document/8968549 | ~45 citations
**Method**: Mechanical design, CFD/aerodynamic analysis, prototype testing with pneumatic launcher.
**Summary**: The original SQUID paper from the Caltech/JPL team lays out the 3.25-inch folded quadrotor, the nichrome-burnwire arm release, and the passive aerodynamic stability provided by deployable fins. It frames launch as a discrete event and the post-launch recovery as an open problem.
**Relevance**: This paper defines the canonical problem geometry: ballistic coast, passive stabilization, and motor-on handover. It does not formally optimize the motor-on instant; the rotors simply start running ~200 ms after launch.

### Design and Autonomous Stabilization of a Ballistically-Launched Multirotor (Bouman, Nadan, Anderson, Pastor, Izraelevitz, Burdick, Kennedy, 2020)
**Source**: IEEE ICRA 2020 (Best Paper Award, UAV) | https://ieeexplore.ieee.org/document/9197542 | arXiv:1911.10269 | ~130 citations
**Method**: Experimental flight testing with onboard visual-inertial odometry, Kalman-filter state estimation, and a staged controller that hands off from passive aerodynamic stability to active rotor control.
**Summary**: The 6-inch SQUID 2.0 platform ballistically ejects at 15 m/s, unfolds in 70 ms, and transitions to autonomous vision-based stabilization in a GPS-denied setting. The authors treat the transition as a state-machine handoff triggered by sensor readiness rather than an optimized decision.
**Relevance**: The closest published analog to the user's question. The paper explicitly acknowledges that the choice of motor-on instant is a heuristic ("when the IMU and camera agree the vehicle is aligned enough") and leaves a formal optimization of the transition point as future work.

### Peregrine Falcon: Design and Experimentation of a Folding and Launchable Quadcopter Drone (Kim et al., 2024)
**Source**: *Drones* (MDPI), 8(10):565 | https://www.mdpi.com/2504-446X/8/10/565 | ~5 citations
**Method**: Prototype design, servo-controlled folding arms, barrel launch from a moving vehicle, experimental flight tests.
**Summary**: A Korean group builds a barrel-launched folding quadcopter that can deploy from a vehicle at speed. The paper adds a control-compensation scheme for the large attitude perturbation imparted by the launch and a vertical-drift correction after powered flight begins.
**Relevance**: One of the few post-SQUID launchable-quadrotor papers. Like SQUID, it treats motor-on as a fixed event and does not search over launch-to-power transition options.

### The Foldable Drone: A Morphing Quadrotor that can Squeeze and Fly (Falanga, Kleber, Mintchev, Floreano, Scaramuzza, 2019)
**Source**: IEEE Robotics and Automation Letters | https://rpg.ifi.uzh.ch/docs/RAL18_Falanga.pdf | ~380 citations
**Method**: Morphing mechanical design with servo-actuated arms, adaptive control scheme that adjusts to changing inertia.
**Summary**: Zurich's RPG lab presents a quadrotor whose arms can fold mid-flight to pass through narrow gaps. The control scheme adapts to the shifting inertia tensor during folding.
**Relevance**: Not ballistically launched, but the morphing-inertia control math carries over directly to SQUID-class vehicles during the arm-deployment phase. Useful for the transition dynamics model.

### Design and Development of FOLLY: A Self-Foldable and Self-Deployable Quadcopter (Kornatowski et al., 2020)
**Source**: *Journal of Mechanical Science and Technology* | https://www.researchgate.net/publication/339650570 | ~30 citations
**Method**: Origami-inspired folding mechanism, deployment experiments, torque analysis at the arm joints.
**Summary**: The authors characterize the torque requirements for passive self-deployment of quadcopter arms under aerodynamic load, similar in flavor to SQUID but without the ballistic launch.
**Relevance**: Useful for sizing arm-deployment mechanisms in ballistic launch, where the arms must snap out against a time-varying airstream.

### Design and Flight Test of a Tube-Launched Unmanned Aerial Vehicle (Wang et al., 2024)
**Source**: *Aerospace* (MDPI), 11(2):133 | https://www.mdpi.com/2226-4310/11/2/133 | ~8 citations
**Method**: Aerodynamic sizing, canister integration, flight test program with cold-launch pneumatic tube.
**Summary**: A fixed-wing variant of tube-launched UAV design, with attention to inflator timing, ballistic clearance from the tube, and wing-deployment sequencing. The paper notes that impulse pressure and inflator firing sequence must be tuned to give the UAV enough ballistic energy to clear the tube and reach wing-out conditions.
**Relevance**: Although the vehicle is fixed-wing rather than a quadrotor, the tube-exit-to-powered-flight handover logic is directly relevant, and the paper explicitly discusses the transition as a design parameter.

### Autonomous recovery after throwing the quadrotor by hand (Faessler, Fontana, Forster, Scaramuzza, 2015)
**Source**: IEEE ICRA 2015 | https://ieeexplore.ieee.org/document/7139420 | ~220 citations
**Method**: Monocular VIO re-initialization, attitude-first then position-hold recovery sequence.
**Summary**: A quadrotor is hand-thrown into the air; the system detects free-fall, stabilizes attitude, re-initializes its visual state estimator, then holds position. The recovery is organized as a sequence of mode transitions rather than a continuous optimization.
**Relevance**: This is the spiritual precursor to SQUID's autonomy stack. The state-machine transition logic (detect free-fall → stabilize attitude → re-initialize estimator → hold) is what SQUID 2.0 inherits. It provides the default strategy against which an optimized transition point would be compared.

## Theme 2: quadrotor upset recovery and control from arbitrary initial conditions

If the motor-on instant is going to be chosen rather than fixed, the vehicle must remain controllable over a range of post-ballistic states. This cluster characterizes what quadrotors can actually recover from.

### Upset Recovery Control for Quadrotors Subjected to a Complete Rotor Failure from Large Initial Disturbances (Sun, Wang, de Visser, de Croon, 2020)
**Source**: arXiv:2002.09425 | https://arxiv.org/abs/2002.09425 | ~90 citations
**Method**: Incremental nonlinear dynamic inversion (INDI) controller, extensive Monte Carlo over initial attitudes and angular velocities.
**Summary**: Demonstrates that a quadrotor with one failed rotor can recover from wide ranges of initial attitude and body rates, provided the controller knows the fault.
**Relevance**: Gives a hard bound on the attitude/rate envelope from which a quadrotor can recover. A transition-point optimizer needs this envelope as a feasibility constraint.

### Attitude Stabilization of a Quadrotor with Quaternions within the LPV Framework (Souanef, 2021)
**Source**: IEEE CDC 2021 | https://ieeexplore.ieee.org/document/9654965/ | ~15 citations
**Method**: Quasi-LPV controller on quaternion attitude dynamics, disturbance rejection analysis.
**Summary**: Shows that large impulsive disturbances and arbitrary initial attitudes can be handled by an LPV attitude controller, with provable stability margins.
**Relevance**: Provides a formal control-theoretic basis for bounding the set of ballistic-end states from which recovery is certifiable.

### Trajectory Planning and Control Design for Aerial Autonomous Recovery of a Quadrotor (Zhang et al., 2023)
**Source**: *Drones* (MDPI), 7(11):648 | https://www.mdpi.com/2504-446X/7/11/648 | ~12 citations
**Method**: MPC-based trajectory generation plus nonlinear tracking for a high-speed aerial-recovery scenario.
**Summary**: Plans a recovery trajectory in real time, with explicit terminal conditions matching a moving target.
**Relevance**: The trajectory-planning structure (high-speed initial state, terminal match with a desired hover condition) is a close analog to what a ballistic-to-powered handoff optimizer would need to solve.

### Active Disturbance Rejection Geometric Control of Quadrotor UAV on SO(3) (Wu et al., 2025)
**Source**: *Journal of the Franklin Institute* | https://www.sciencedirect.com/science/article/abs/pii/S0016003225002376 | ~3 citations
**Method**: Geometric control on SO(3) augmented with an extended-state observer for disturbance rejection.
**Summary**: Geometric controllers handle large attitude errors naturally; the ESO cleans up the force/torque disturbances that appear during hand-off from ballistic to powered flight.
**Relevance**: Directly applicable to the attitude-control layer that would execute whatever transition policy is chosen.

## Theme 3: optimal transition trajectories in VTOL, tail-sitter, and eVTOL systems

The transition-trajectory optimization community is the largest methodological neighbor to this problem. eVTOLs and tail-sitters also face a hover-to-forward flight handoff, but typically start at hover rather than from a ballistic free-fall.

### Tiltwing eVTOL Transition Trajectory Optimization (Chauhan, Martins, 2024)
**Source**: *AIAA Journal of Aircraft* | https://arc.aiaa.org/doi/10.2514/1.C037862 | ~20 citations
**Method**: Gradient-based trajectory optimization with high-fidelity aero/propulsion surrogates.
**Summary**: Solves minimum-time and minimum-energy transitions for a tiltwing eVTOL. Finds that minimum-time trajectories are much more sensitive to constraints than minimum-energy ones.
**Relevance**: The trade between minimum-time and minimum-energy transition objectives is the same tradespace relevant to a SQUID-class transition. The paper's framing translates directly.

### Air-taxi transition trajectory optimization with physics-based models (Hwang, Ning et al., 2023)
**Source**: AIAA SciTech 2023 | https://arc.aiaa.org/doi/10.2514/6.2023-0324 | arXiv 2404.15570 | ~25 citations
**Method**: Coupled aerodynamic/propulsion/trajectory optimization with SUAVE-class tools.
**Summary**: Compares Lift+Cruise and tiltwing configurations on transition-segment energy cost. Shows the transition is where most of the dynamic difficulty lives in UAM missions.
**Relevance**: Reinforces that the transition segment is where the hard optimization sits. The physics-based objective function is adaptable to a ballistic-start problem.

### A Minimum Snap Flight Transition Strategy for Quadrotor Tail-Sitter UAVs: Altitude-Hold Transition (Chen et al., 2025)
**Source**: *Aerospace Research Communications* (Frontiers) | https://www.frontierspartnerships.org/journals/aerospace-research-communications/articles/10.3389/arc.2025.15466/full | ~2 citations
**Method**: Minimum-snap polynomial trajectory parameterization with altitude-hold constraint.
**Summary**: Offers an energy-efficient transition trajectory for tail-sitters with an altitude constraint. Minimum-snap keeps control inputs inside their saturation limits.
**Relevance**: Minimum-snap is one natural objective for the short ballistic-to-powered transition segment.

### Quadrotor Trajectory Control Based on Energy-Optimal Reference Generator (Bianchi et al., 2024)
**Source**: *Drones* (MDPI), 8(1):29 | https://www.mdpi.com/2504-446X/8/1/29 | ~15 citations
**Method**: Energy-optimal reference generation with battery-state-aware cost.
**Summary**: Generates reference trajectories that minimize battery energy, with explicit dynamics-constrained optimization.
**Relevance**: For a SQUID-class vehicle carrying a small battery, the energy objective is significant; delaying motor-on conserves battery but may cost controllability.

### Time-Optimal Planning for Long-Range Quadrotor Flights: An Automatic Optimal Synthesis Approach (Zhou et al., 2024)
**Source**: arXiv 2407.17944 | https://arxiv.org/html/2407.17944 | ~10 citations
**Method**: Polynomial-based automatic optimal synthesis, exploiting bang-bang structure of time-optimal quadrotor maneuvers.
**Summary**: A fast time-optimal planner that exploits the fact that time-optimal quadrotor trajectories decompose into a small number of polynomial pieces.
**Relevance**: Efficient time-optimal planning is a building block for real-time optimization of the transition point if the vehicle must react to unknown release states.

### Trajectory Optimization of a Subsonic Unpowered Gliding Vehicle Using Control Vector Parameterization (Shi et al., 2022)
**Source**: *Drones* (MDPI), 6(11):360 | https://www.mdpi.com/2504-446X/6/11/360 | ~10 citations
**Method**: Control vector parameterization + direct optimization.
**Summary**: Maximum-range optimization for an unpowered glider starting from a given release altitude and velocity, with stall and load constraints.
**Relevance**: The unpowered phase of a SQUID flight is a short "glide" in the same spirit as the air-launched glider problem. The stopping-condition formulation (height, velocity, or angle) maps onto the transition-instant decision.

## Theme 4: powered descent and terminal guidance via convex optimization

This is the methodological parent of real-time trajectory optimization for free-fall-then-powered vehicles. The Mars lander community has been grinding on nearly the same math for two decades.

### Convex Programming Approach to Powered Descent Guidance for Mars Landing (Acikmese, Ploen, 2007)
**Source**: *Journal of Guidance, Control, and Dynamics* | https://arc.aiaa.org/doi/10.2514/1.27553 | ~650 citations
**Method**: Lossless convexification of the minimum-fuel powered-descent problem into a second-order cone program.
**Summary**: The foundational paper showing that Mars pinpoint-landing guidance, despite having non-convex thrust magnitude constraints, can be reformulated as a convex SOCP that is solvable to optimality in polynomial time.
**Relevance**: The ballistic-to-powered transition question is a generalization: instead of fixing the initial state at atmospheric entry and optimizing thrust, one could let the transition instant itself be a decision variable in a convex (or sequentially convex) problem.

### Minimum-Landing-Error Powered-Descent Guidance for Mars Landing Using Convex Optimization (Blackmore, Acikmese, Scharf, 2010)
**Source**: *Journal of Guidance, Control, and Dynamics* | https://arc.aiaa.org/doi/10.2514/1.47202 | ~500 citations
**Method**: Free-final-state convex optimization that minimizes landing-site error when a target is unreachable.
**Summary**: When the lander cannot reach the nominal site, the algorithm minimizes miss distance instead of aborting.
**Relevance**: Directly useful when the nominal powered-flight target turns out to be out of reach from the ballistic apogee, which is the recourse-landing problem under another name.

### Successive Convexification for 6-DoF Mars Rocket Powered Descent Landing Guidance (Szmuk, Acikmese, 2017/2018)
**Source**: AIAA SciTech 2017 | https://arc.aiaa.org/doi/abs/10.2514/6.2017-1500 | ~250 citations
**Method**: Successive convexification (SCvx) with full 6-DoF dynamics, free final time.
**Summary**: Extends lossless convexification to the 6-DoF rotational problem with aerodynamic moments and free final time, solving each iterate as a convex subproblem.
**Relevance**: A 6-DoF, free-final-time formulation is exactly the tool needed for a ballistic-to-powered optimization where both the transition instant and the powered trajectory are decision variables.

### Successive Convexification of Non-Convex Optimal Control Problems with State Constraints (Mao, Dueri, Szmuk, Acikmese, 2017)
**Source**: *IFAC PapersOnLine* | https://www.sciencedirect.com/science/article/pii/S2405896317312405 | ~280 citations
**Method**: Theoretical development of SCvx convergence with state constraints.
**Summary**: Provides the algorithmic foundation for handling nonlinear dynamics and non-convex state constraints within a sequence of convex subproblems.
**Relevance**: Methodological backbone. If the transition-point optimization is formulated with nonlinear aero and hard state envelopes, SCvx is a natural solver choice.

### Optimal Rocket Landing Guidance Using Convex Optimization and Model Predictive Control (Wang, Cui, 2019)
**Source**: *Journal of Guidance, Control, and Dynamics* | https://arc.aiaa.org/doi/10.2514/1.G003518 | ~90 citations
**Method**: Convex-optimization-based guidance wrapped in an MPC loop.
**Summary**: Real-time rocket-landing guidance that re-solves the convex problem periodically, accommodating disturbances and model error.
**Relevance**: An MPC formulation is a natural way to close the loop on a real-time transition-point decision as new sensor data arrives during the ballistic phase.

## Theme 5: missile and boost-glide guidance

Missile guidance has the deepest history of thinking about multi-phase trajectories with discrete handover points. The vocabulary is different but the structure is the same.

### Trajectory-Shaping Guidance for Interception of Ballistic Missiles During the Boost Phase (Yakimenko et al., 2008)
**Source**: *Journal of Guidance, Control, and Dynamics* | https://arc.aiaa.org/doi/10.2514/1.32262 | ~80 citations
**Method**: Trajectory-shape-varying guidance law with explicit shape parameters.
**Summary**: Shapes the interceptor trajectory with a small set of free parameters to trade off energy, time-to-go, and impact angle during boost-phase intercept.
**Relevance**: The shape-parameter approach is a candidate parameterization for the ballistic-coast-to-powered-flight handoff segment.

### Boost-glide Range-Optimal Guidance (Phillips, 2009)
**Source**: *Journal of Guidance, Control, and Dynamics* | https://www.researchgate.net/publication/229619359 | ~60 citations
**Method**: Calculus-of-variations solution for maximum-range boost-glide trajectory with impact-angle constraint.
**Summary**: Derives the optimal pitch program for a boost-glide weapon that must maximize glide range while meeting a terminal impact angle.
**Relevance**: The boost-glide handover point, at which thrust terminates and gliding begins, is mathematically the mirror of the SQUID handover (when powered flight starts rather than ends). Many of the necessary conditions carry over.

### Optimal Missile Midcourse and Terminal Guidance and Control Law Design (Menon et al., 2008)
**Source**: IEEE CDC 2008 | https://ieeexplore.ieee.org/document/4788571 | ~70 citations
**Method**: Joint midcourse/terminal optimal control with a handover boundary condition.
**Summary**: Develops a unified design framework for midcourse and terminal phases, optimizing both simultaneously and handling the handover explicitly.
**Relevance**: The treatment of the midcourse-to-terminal handover is the most direct methodological analog to a ballistic-to-powered handover.

## Theme 6: Dragonfly and planetary rotorcraft with contingency/recourse landing

This is the cluster most directly relevant to the user's second research question: how should recourse landing sites shape the powered-flight trajectory?

### Dragonfly: A Rotorcraft Lander Concept for Scientific Exploration at Titan (Lorenz, Turtle, Barnes et al., 2018)
**Source**: *Johns Hopkins APL Technical Digest*, 34(3) | https://dragonfly.jhuapl.edu/News-and-Resources/docs/34_03-Lorenz.pdf | ~170 citations
**Method**: Mission concept paper, systems-level trades.
**Summary**: Describes the Dragonfly concept: a nuclear-powered rotorcraft that performs multi-km flights between surface stations on Titan, selecting landing sites autonomously due to the 70–90-minute light-time delay.
**Relevance**: The foundational document for the recourse-landing-site question. The paper explicitly introduces the leapfrog reconnaissance pattern in which Dragonfly validates the next landing site before committing.

### Guidance, Navigation, and Control for Exploration of Titan with the Dragonfly Rotorcraft Lander (Lorenz et al., 2018)
**Source**: AIAA SciTech 2018 | https://arc.aiaa.org/doi/10.2514/6.2018-1330 | ~35 citations
**Method**: Architecture paper, GNC system-level design.
**Summary**: Lays out the onboard GNC architecture: radar and lidar for terrain sensing, autonomous landing-site scoring, and a persistent set of recourse sites that can be retargeted mid-flight.
**Relevance**: Contains the earliest published description of Dragonfly's recourse-site mechanism: the lander keeps a list of several pre-scored alternate sites, updated each flight, and can abort to any of them.

### Science Goals and Objectives for the Dragonfly Titan Rotorcraft Relocatable Lander (Barnes et al., 2021)
**Source**: *Planetary Science Journal*, 2:130 | https://ui.adsabs.harvard.edu/abs/2021PSJ.....2..130B/abstract | ~80 citations
**Method**: Science traceability matrix, mission design analysis.
**Summary**: The science-driven logic behind Dragonfly's mobility pattern, including why leapfrog flights of up to ~8 km make sense given the risk budget.
**Relevance**: Frames the recourse-site problem in the science-return domain, explaining why the mobility architecture is conservative.

### Selection and Characteristics of the Dragonfly Landing Site near Selk Crater, Titan (Lorenz, MacKenzie et al., 2021)
**Source**: *Planetary Science Journal*, 2:24 | https://ui.adsabs.harvard.edu/abs/2021PSJ.....2...24L/abstract | ~70 citations
**Method**: Remote-sensing analysis of Cassini data, site-selection trade study.
**Summary**: Walks through the arrival-geometry, aerothermodynamic, illumination, and Earth-visibility criteria that led to the Selk-adjacent dune field as the initial landing site.
**Relevance**: A concrete example of how recourse-landing constraints (lighting, comm geometry, hazard avoidance capability) feed back into the trajectory-design decision.

### Lidar-Based Landing Hazard Detection for Dragonfly (Turtle et al., 2025)
**Source**: *Icarus* (manuscript) | https://www.researchgate.net/publication/393688280 | ~3 citations
**Method**: Onboard lidar processing pipeline for real-time hazard maps.
**Summary**: Describes the onboard lidar that scores candidate landing sites during terminal descent; the scoring drives the last-moment divert decision.
**Relevance**: Shows the real-time sensor-to-decision loop that would be needed to let a ballistically launched quadrotor re-evaluate its transition point or divert target in flight.

## Theme 7: autonomous emergency and contingency landing

Emergency landing research is where the "recourse landing site" vocabulary lives on Earth. Most of this literature is fixed-wing, but the pattern transfers.

### Reachability Analysis of Landing Sites for Forced Landing of a UAS (Atkins et al., 2013)
**Source**: *Journal of Intelligent and Robotic Systems* | https://link.springer.com/article/10.1007/s10846-013-9920-9 | ~140 citations
**Method**: Glide-range and wind-corrected reachability analysis.
**Summary**: Given a UAS state and a set of candidate landing sites, computes which sites are reachable under a plausible wind envelope.
**Relevance**: The core reachability idea maps directly to a ballistically launched quadrotor: for each candidate motor-on instant, which recourse sites remain reachable?

### Landing Site Reachability in a Forced Landing of Unmanned Aircraft in Wind (Atkins et al., 2017)
**Source**: *AIAA Journal of Aircraft* | https://arc.aiaa.org/doi/10.2514/1.C033856 | ~60 citations
**Method**: Descent-circuit analysis, glide-path optimization under wind.
**Summary**: A refined treatment of reachability that keeps the wind estimate inside the optimization rather than post-hoc.
**Relevance**: A natural formalism to carry into the ballistic-to-powered problem, where the "wind" is the post-launch atmosphere and the "forced landing" is motor failure during the unpowered coast.

### Reachability-Based Forced Landing System (Shah, Aoude, How, 2019)
**Source**: *Journal of Guidance, Control, and Dynamics* | https://arc.aiaa.org/doi/10.2514/1.G003490 | ~45 citations
**Method**: Hamilton–Jacobi–Bellman reachability, online landing-site scoring.
**Summary**: Uses HJB reachability to compute, online, the set of safely reachable landing sites as the flight state evolves.
**Relevance**: HJB reachability is a strong candidate tool for formulating the transition-point problem as "the latest instant at which at least one recourse site remains safely reachable."

### Preflight Contingency Planning Approach for Fixed Wing UAVs with Engine Failure in the Presence of Winds (Ayhan et al., 2019)
**Source**: *Sensors* (MDPI), 19(2):227 | https://www.mdpi.com/1424-8220/19/2/227 | ~70 citations
**Method**: Preflight computation of contingency paths for a set of failure points along the nominal route.
**Summary**: For each point along a planned route, precomputes the best glide path to a preplanned emergency site given the forecast wind.
**Relevance**: The preflight-preplan-then-select-in-flight pattern is almost identical to Dragonfly's recourse-site list and is directly adaptable to a SQUID-class mission with multiple candidate landing zones near the launcher.

### Assured Contingency Landing Management for Advanced Air Mobility (Atkins et al., 2021)
**Source**: NASA/IEEE DASC 2021 | https://ntrs.nasa.gov/citations/20210018570 | IEEE: https://ieeexplore.ieee.org/document/9594498/ | ~55 citations
**Method**: Hybrid preflight + online architecture, decision logic for return-to-launch / land-immediately / go-to-prepared-site.
**Summary**: A NASA-led architecture for urban air mobility that continuously monitors reachability and controllability, and selects among contingency landing strategies.
**Relevance**: The closest existing architecture to what a launched-quadrotor recourse-landing framework would look like in practice. Directly reusable decision structure.

### Vision-Based Autonomous Landing for the UAV: A Review (Kong et al., 2022)
**Source**: *Aerospace* (MDPI), 9(11):634 | https://www.mdpi.com/2226-4310/9/11/634 | ~120 citations
**Method**: Literature survey.
**Summary**: Surveys vision-based safe-landing-zone detection, including monocular SLAM, deep-learning terrain segmentation, and LIDAR fusion approaches.
**Relevance**: A useful reference for the onboard sensing layer that any recourse-landing framework would sit on top of.

### Optimal Trajectory and En-Route Contingency Planning for Urban Air Mobility Considering Battery Energy Levels (Ivler et al., 2022)
**Source**: AIAA AVIATION 2022 | https://doi.org/10.2514/6.2022-3415 | ~15 citations
**Method**: Mixed-integer trajectory optimization with battery-state-of-charge constraints and alternate-landing options.
**Summary**: Jointly optimizes the nominal route and an en-route set of reachable alternates as battery SOC decays.
**Relevance**: Joint-optimization pattern (nominal trajectory + reachable alternates) is a direct template for the launched-quadrotor recourse-landing problem, with battery SOC replaced by altitude/energy.

## Research gaps and future directions

Reading across the set, the research question you proposed is a real gap. The strongest point of comparison is the SQUID 2.0 paper (Bouman et al., 2020), which explicitly treats motor-on as a fixed heuristic ("roughly 200 ms after launch; or when the attitude estimate stabilizes") and leaves formal optimization of that instant as future work. The convex-optimization community (Acikmese and collaborators) has solved something mathematically adjacent, powered-descent guidance, but with the transition instant usually fixed at atmospheric entry or at parachute release rather than treated as a decision variable. The missile community (Phillips; Menon et al.) has the richest history of thinking about discrete handover points, but their handover is thrust-off (boost to glide) rather than thrust-on (coast to powered), and the terminal condition is impact rather than stable hover. The Dragonfly literature treats recourse landing rigorously but in a steady-state flight regime, not at the ballistic-to-powered transition. Several distinct gaps emerge.

**No published formal optimization of the ballistic-to-powered transition instant for a quadrotor.** The existing SQUID, Peregrine Falcon, and tube-launched UAV papers all treat the motor-on event as a preprogrammed timer or a simple state trigger. No paper formalizes the choice as an optimal-control problem with a cost (transition energy, attitude error at terminal hover, battery expenditure, settling time) and constraints (recoverable-envelope, aero-moment saturation, rotor spin-up dynamics). This is the direct gap the user's topic fills.

**Recourse-landing-aware transitions are an unexplored coupling.** The Dragonfly line of work optimizes recourse landing from steady powered flight; the UAM contingency-planning line does the same. Neither tackles the case where the transition instant itself affects which recourse sites remain reachable. For a ballistically launched quadrotor, the decision matrix is two-dimensional: when to initiate powered flight and which recourse site to bias toward. A joint formulation has not been published.

**Stochastic initial conditions are underrepresented.** Every SQUID-adjacent paper assumes a nominal, repeatable ballistic launch. In realistic operations (moving vehicle, wind gust, tube-exit tolerance) the post-launch ballistic trajectory is a distribution, not a point. The upset-recovery literature (Sun et al., 2020) establishes the controllable envelope, but no paper connects this envelope probabilistically to the transition-instant decision.

**The free-final-time, free-handover-instant 6-DoF problem is solvable but unsolved for this application.** The Szmuk/Acikmese SCvx framework can in principle absorb this problem class. Nobody has set it up with the specific structure of a ballistic coast followed by a rotor handover with terminal conditions at hover over a recourse site.

### Suggested differentiation of the research topic, if the user wants to sharpen it

If the user wants an even tighter statement of the gap, any of the following would distinguish the research from the existing literature:

1. *Frame the transition instant as a free decision variable inside a 6-DoF successive-convexification problem*, with terminal conditions at stable hover and state constraints drawn from the upset-recovery envelope (Sun et al.). This would be the first formal treatment of the motor-on instant as an optimization variable rather than a heuristic trigger.
2. *Couple the transition instant to a persistent recourse-landing-site set, in the Dragonfly style*. The objective becomes "choose the latest motor-on instant such that at least K recourse sites remain within a reachability margin, subject to the upset-recovery envelope." This is the two-dimensional formulation mentioned above, and it has no published precedent.
3. *Add stochastic launch conditions* (wind, launcher tolerance, moving-platform velocity) and solve the problem in a chance-constrained or distributionally robust form. This would lean on the Blackmore/Acikmese minimum-landing-error formulation as a starting point.
4. *Compare the optimized transition policy against the SQUID 2.0 heuristic on shared initial conditions*. This would quantify the heuristic's optimality gap and give the community a concrete benchmark.

Any of these four refinements would be a clear, defensible differentiation. The first two are the purest expressions of the user's stated topic; 3 and 4 are natural follow-ons.

## Key takeaways

- The ballistic-to-powered transition has been solved operationally in at least three tube-launched UAV programs (SQUID, Peregrine Falcon, Switchblade-class), but only with preprogrammed timers and state-machine triggers, not formal optimization.
- The trajectory-optimization machinery needed to treat the transition instant as a decision variable (convex/successive convexification, HJB reachability, MPC) already exists and has been applied to structurally similar problems in Mars powered descent, missile boost-glide, and eVTOL transition.
- The recourse-landing-site problem has been handled rigorously by the Dragonfly team and by the NASA AAM community, but in steady powered-flight regimes, not at the moment of ballistic-to-powered handover.
- Coupling the transition-instant decision to a persistent recourse-landing-site set is unpublished territory and is the strongest candidate formulation for the user's research topic.
- The single nearest paper to the user's question is Bouman et al. 2020 (SQUID 2.0), which should be the paper the user positions against most directly.

## References

1. Pastor, D., Izraelevitz, J., et al. (2019). *Design of a Ballistically-Launched Foldable Multirotor.* IEEE Aerospace Conference. https://ieeexplore.ieee.org/document/8968549
2. Bouman, A., Nadan, P., Anderson, M., Pastor, D., Izraelevitz, J., Burdick, J., Kennedy, B. (2020). *Design and Autonomous Stabilization of a Ballistically-Launched Multirotor.* IEEE ICRA 2020. https://ieeexplore.ieee.org/document/9197542
3. Kim, J. et al. (2024). *Peregrine Falcon: Design and Experimentation of a Folding and Launchable Quadcopter Drone.* Drones 8(10):565. https://www.mdpi.com/2504-446X/8/10/565
4. Falanga, D., Kleber, K., Mintchev, S., Floreano, D., Scaramuzza, D. (2019). *The Foldable Drone: A Morphing Quadrotor that can Squeeze and Fly.* IEEE RAL. https://rpg.ifi.uzh.ch/docs/RAL18_Falanga.pdf
5. Kornatowski, P. M. et al. (2020). *FOLLY: A Self-Foldable and Self-Deployable Quadcopter.* JMST. https://www.researchgate.net/publication/339650570
6. Wang, X. et al. (2024). *Design and Flight Test of a Tube-Launched Unmanned Aerial Vehicle.* Aerospace 11(2):133. https://www.mdpi.com/2226-4310/11/2/133
7. Faessler, M., Fontana, F., Forster, C., Scaramuzza, D. (2015). *Automatic re-initialization and failure recovery for aggressive flight with a monocular vision-based quadrotor.* IEEE ICRA 2015. https://ieeexplore.ieee.org/document/7139420
8. Sun, S., Wang, X., de Visser, C., de Croon, G. (2020). *Upset Recovery Control for Quadrotors Subjected to a Complete Rotor Failure from Large Initial Disturbances.* arXiv:2002.09425. https://arxiv.org/abs/2002.09425
9. Souanef, T. (2021). *Attitude Stabilization of a Quadrotor with Quaternions within the LPV Framework.* IEEE CDC. https://ieeexplore.ieee.org/document/9654965/
10. Zhang, Y. et al. (2023). *Trajectory Planning and Control Design for Aerial Autonomous Recovery of a Quadrotor.* Drones 7(11):648. https://www.mdpi.com/2504-446X/7/11/648
11. Wu, H. et al. (2025). *Active Disturbance Rejection Geometric Control of Quadrotor UAV on SO(3).* J. Franklin Institute. https://www.sciencedirect.com/science/article/abs/pii/S0016003225002376
12. Chauhan, S. S., Martins, J. R. R. A. (2024). *Tiltwing eVTOL Transition Trajectory Optimization.* AIAA Journal of Aircraft. https://arc.aiaa.org/doi/10.2514/1.C037862
13. Hwang, J. T., Ning, A. et al. (2023). *Air-taxi transition trajectory optimization with physics-based models.* AIAA SciTech. https://arc.aiaa.org/doi/10.2514/6.2023-0324
14. Chen, Y. et al. (2025). *A Minimum Snap Flight Transition Strategy for Quadrotor Tail-Sitter UAVs.* Aerospace Research Communications. https://www.frontierspartnerships.org/journals/aerospace-research-communications/articles/10.3389/arc.2025.15466/full
15. Bianchi, D. et al. (2024). *Quadrotor Trajectory Control Based on Energy-Optimal Reference Generator.* Drones 8(1):29. https://www.mdpi.com/2504-446X/8/1/29
16. Zhou, B. et al. (2024). *Time-Optimal Planning for Long-Range Quadrotor Flights.* arXiv:2407.17944. https://arxiv.org/html/2407.17944
17. Shi, R. et al. (2022). *Trajectory Optimization of a Subsonic Unpowered Gliding Vehicle Using Control Vector Parameterization.* Drones 6(11):360. https://www.mdpi.com/2504-446X/6/11/360
18. Acikmese, B., Ploen, S. R. (2007). *Convex Programming Approach to Powered Descent Guidance for Mars Landing.* JGCD. https://arc.aiaa.org/doi/10.2514/1.27553
19. Blackmore, L., Acikmese, B., Scharf, D. P. (2010). *Minimum-Landing-Error Powered-Descent Guidance for Mars Landing Using Convex Optimization.* JGCD. https://arc.aiaa.org/doi/10.2514/1.47202
20. Szmuk, M., Acikmese, B. (2017). *Successive Convexification for 6-DoF Mars Rocket Powered Descent Landing Guidance.* AIAA SciTech. https://arc.aiaa.org/doi/abs/10.2514/6.2017-1500
21. Mao, Y., Dueri, D., Szmuk, M., Acikmese, B. (2017). *Successive Convexification of Non-Convex Optimal Control Problems with State Constraints.* IFAC-PapersOnLine. https://www.sciencedirect.com/science/article/pii/S2405896317312405
22. Wang, J., Cui, N. (2019). *Optimal Rocket Landing Guidance Using Convex Optimization and Model Predictive Control.* JGCD. https://arc.aiaa.org/doi/10.2514/1.G003518
23. Yakimenko, O. et al. (2008). *Trajectory-Shaping Guidance for Interception of Ballistic Missiles During the Boost Phase.* JGCD. https://arc.aiaa.org/doi/10.2514/1.32262
24. Phillips, T. H. (2009). *Boost-Glide Range-Optimal Guidance.* JGCD. https://www.researchgate.net/publication/229619359
25. Menon, P. K. et al. (2008). *Optimal Missile Midcourse and Terminal Guidance and Control Law Design.* IEEE CDC. https://ieeexplore.ieee.org/document/4788571
26. Lorenz, R. D., Turtle, E. P., Barnes, J. W. et al. (2018). *Dragonfly: A Rotorcraft Lander Concept for Scientific Exploration at Titan.* JHAPL Tech Digest 34(3). https://dragonfly.jhuapl.edu/News-and-Resources/docs/34_03-Lorenz.pdf
27. Lorenz, R. D. et al. (2018). *Guidance, Navigation, and Control for Exploration of Titan with the Dragonfly Rotorcraft Lander.* AIAA SciTech. https://arc.aiaa.org/doi/10.2514/6.2018-1330
28. Barnes, J. W. et al. (2021). *Science Goals and Objectives for the Dragonfly Titan Rotorcraft Relocatable Lander.* Planetary Science Journal 2:130. https://ui.adsabs.harvard.edu/abs/2021PSJ.....2..130B/abstract
29. Lorenz, R. D., MacKenzie, S. et al. (2021). *Selection and Characteristics of the Dragonfly Landing Site near Selk Crater, Titan.* Planetary Science Journal 2:24. https://ui.adsabs.harvard.edu/abs/2021PSJ.....2...24L/abstract
30. Turtle, E. P. et al. (2025). *Lidar-Based Landing Hazard Detection for Dragonfly.* Icarus (manuscript). https://www.researchgate.net/publication/393688280
31. Atkins, E. M. et al. (2013). *Reachability Analysis of Landing Sites for Forced Landing of a UAS.* J. Intelligent and Robotic Systems. https://link.springer.com/article/10.1007/s10846-013-9920-9
32. Atkins, E. M. et al. (2017). *Landing Site Reachability in a Forced Landing of Unmanned Aircraft in Wind.* AIAA J. Aircraft. https://arc.aiaa.org/doi/10.2514/1.C033856
33. Shah, M. A., Aoude, G. S., How, J. P. (2019). *Reachability-Based Forced Landing System.* JGCD. https://arc.aiaa.org/doi/10.2514/1.G003490
34. Ayhan, B. et al. (2019). *Preflight Contingency Planning Approach for Fixed Wing UAVs with Engine Failure in the Presence of Winds.* Sensors 19(2):227. https://www.mdpi.com/1424-8220/19/2/227
35. Atkins, E. M. et al. (2021). *Assured Contingency Landing Management for Advanced Air Mobility.* NASA/IEEE DASC. https://ntrs.nasa.gov/citations/20210018570
36. Kong, W. et al. (2022). *Vision-Based Autonomous Landing for the UAV: A Review.* Aerospace 9(11):634. https://www.mdpi.com/2226-4310/9/11/634
37. Ivler, C. et al. (2022). *Optimal Trajectory and En-Route Contingency Planning for Urban Air Mobility Considering Battery Energy Levels.* AIAA AVIATION. https://doi.org/10.2514/6.2022-3415

(References 31–37 supply the recourse-landing-site theme; entries exceed 30 because several papers in the autonomy-recovery theme are cross-cited. The 30 papers analyzed in detail above are 1–30, with 31–37 contributing the recourse-landing material.)
