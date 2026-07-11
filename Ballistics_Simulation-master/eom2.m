% File eom2.m defines mortar_propagate; callers invoke it as eom2().
function ballistic_sol = mortar_propagate(launch, env, output)
    % launch struct fields:
    %   Vo, el, az, w_z0, w_y0, alpha_0, beta_0, p, x_0, y_0, z_0, t_max
    t_max   = launch.t_max;
    Vo      = launch.Vo;
    el_0    = launch.el;
    az_0    = launch.az;
    w_z0    = launch.w_z0;
    w_y0    = launch.w_y0;
    alpha_0 = launch.alpha_0;
    beta_0  = launch.beta_0;
    p       = launch.p;
    x_0     = launch.x_0;
    y_0     = launch.y_0;
    z_0     = launch.z_0;

    %% Initial conditions
    Q = ((sind(az_0 + beta_0))^2)+((cosd(az_0 + beta_0))^2)*((cosd(el_0 + alpha_0))^2);
    dx0 = (1/sqrt(Q))*[(-w_z0*((cosd(az_0 + beta_0))^2)*sind(el_0 + alpha_0)*cosd(el_0 ...
                            + alpha_0) + w_y0*sind(az_0 + beta_0));
                       (w_z0*((cosd(az_0 + beta_0))^2)*((cosd(el_0 + alpha_0))^2) ...
                            + w_z0*((sind(az_0 + beta_0))^2));
                       ((-w_z0*sind(az_0 + beta_0)*cosd(az_0 + beta_0)*sind(el_0 + alpha_0)) ...
                            - (w_y0*cosd(az_0 + beta_0)*cosd(el_0 + alpha_0)))];
    
    % ODE state x (12 states), inertial frame (x = range/north, y = altitude/up, z = cross-range/east):
    %  x(1:3)   = velocity (m/s)
    %  x(4:6)   = angular rate vector h (rad/s); spin rate p = (I_y/I_x)*(h.r)
    %  x(7:9)   = projectile pointing unit vector
    %  x(10:12) = CG position (m): range, altitude, cross-range
    % Output .trajectory keeps 15 columns: 13:15 are backfilled post-hoc
    % with the derived attitude [theta phi psi] (rad, NWU; body x-axis along
    % the pointing vector, phi = 0 -- the ballistic_deploy_state.m packaging
    % convention). Interpolate attitude by interpolating r (cols 7:9) and
    % deriving, never by interpolating the angle columns (psi wraps at +-pi).
    
    % Set initial velocities
    v0 = [Vo*cosd(el_0)*cosd(az_0);
          Vo*sind(el_0)*cosd(az_0);
          Vo*sind(az_0)];
    
    % Set initial orientation pointing vector
    r0 = [cosd(el_0 + alpha_0)*cosd(az_0 + beta_0);
          sind(el_0 + alpha_0)*cosd(az_0 + beta_0);
          sind(az_0 + beta_0)]; % az is negative yaw, and sin(neg) = -sin(pos);

    % Initial h = H/I_y = (I_x/I_y)*p*r0 + r0 x dx0  (McCoy eq. 9.4)
    h0 = (env.I_x*p/env.I_y)*r0 + cross(r0, dx0);   % cross() avoids hand-expansion sign errors
    
    % Set initial position
    e0 = [x_0; % initial position in range direction (m)
          y_0; % initial position in altitude (m)
          z_0]; % initial position in cross-range direction (m)
    
    % The legacy integrated Euler-angle channel (o: degree IC mixed with
    % rad/s do = h integration, frame-mangled componentwise by the output
    % rotation) was removed from the ODE state 2026-07-10: it was never a
    % valid attitude, nothing consumed it, and its scaling perturbed ode45
    % step selection (error control weighs all state components). Cols
    % 13:15 of .trajectory are now backfilled with the derived attitude --
    % see the backfill block below.
    
    x0 = [v0; h0; r0; e0];

    % Run the ODE solver to propagate the submunitions through the air.
    % Evaluate system of differential equations.
    % Fixed 0.1 s output grid (dense output, ~ the old natural-step density),
    % decoupled from the internal solver steps so tightening tolerances does
    % not inflate .trajectory row count or downstream row-indexed sampling
    % (analysis.m envelope: test_points = 1:10:rows). Integration still stops
    % at the impact event; the event time is appended as the final row.
    tspan = 0:0.1:t_max;
    % Tolerances tightened 2026-07-10 (author decision): the defaults
    % (RelTol 1e-3 / AbsTol 1e-6) carried ~0.4 m of true position error
    % (0.25-0.42 m at impact, +4.6 cm apogee); 1e-8/1e-10 is converged to
    % ~2e-6 m in position (verified against 1e-9/1e-11 on the same output
    % grid). Overridable per-call for convergence studies via optional
    % launch.rel_tol / launch.abs_tol fields.
    if isfield(launch, 'rel_tol'), rel_tol = launch.rel_tol; else, rel_tol = 1e-8;  end
    if isfield(launch, 'abs_tol'), abs_tol = launch.abs_tol; else, abs_tol = 1e-10; end
    Opt = odeset('Events', @impactEvent, 'RelTol', rel_tol, 'AbsTol', abs_tol);
    [t,x] = ode45(@(t,x) sixdof_ballistics(t, x, env, az_0),tspan,x0,Opt);
    
    % Orientation angles (aoa and sideslip)
    alpha = acosd(x(:,2)./((x(:,1).^2+x(:,2).^2+x(:,3).^2).^0.5));
    beta = acosd(x(:,3)./((x(:,1).^2+x(:,2).^2+x(:,3).^2).^0.5));
    
    % Find the apogee of the munition's flight.
    [max_ht,I] = max(x(:,11));
    ap_vel = x(I, 1:3);
    ap_vel(2) = 0;
    ap_view = cross(ap_vel, [0 1 0]) / norm(cross(ap_vel, [0 1 0]));
    ap_view = [ap_view(1) -ap_view(3) ap_view(2)]; % corr. to idx 10 12 11
    ap_view_min = -1*[ap_vel(1) -ap_vel(3) ap_vel(2)];
    
    % Interpolate post-apogee time, position (range/cross-range), orientation
    %  (alpha/beta), and velocity at altitude = 0. Alpha is the angle in the
    %  x-y (vertical) plane; beta is the angle in the x-z (ground) plane.
    impact_inputs = [t(I:end), ...
                     x(I:end,10), x(I:end,12), ...
                     alpha(I:end), beta(I:end), ...
                     x(I:end,1), x(I:end,2), x(I:end,3)];
    impacts = interp1(x(I:end,11), impact_inputs, 0);
    impact_time  = impacts(1);
    range        = impacts(2);
    cross_range  = impacts(3);
    impact_alpha = impacts(4);
    impact_beta  = impacts(5);
    vel_x_imp    = impacts(6);
    vel_y_imp    = impacts(7);
    vel_z_imp    = impacts(8);

    % Create 3D vector of impact direction
    impactVect_x_coord = cosd(impact_beta)*cosd(impact_alpha);
    impactVect_y_coord = sind(impact_beta)*cosd(impact_alpha);
    impactVect_z_coord = sind(impact_alpha);
    impactVect = [impactVect_x_coord, impactVect_y_coord, impactVect_z_coord];

    % Find total 3D impact angle in degrees using dot product
    vert = [0,1,0];
    impact_angle = acosd(dot(vert,impactVect)/(norm(vert)*...
        norm(impactVect))) - 90;

    impact_vel = sqrt(vel_x_imp^2 + vel_y_imp^2 + vel_z_imp^2);
    
    % Calculate total distance travelled at each timestep
    total_dis = sqrt(x(:,10).^2 + x(:,12).^2);
    
    % Distance at impact, interpolated to altitude 0 (avoids negative-alt overshoot)
    total_dis_imact = interp1(x(I:end,11), total_dis(I:end),0);
    ballistic_sol.time = t;
    % Output .trajectory is NUE->NWU rotated; raw solver state x stays NUE
    NUEtoNWU = [1 0 0;
                0 0 -1;
                0 1 0];
    supertrans = blkdiag(NUEtoNWU, NUEtoNWU, NUEtoNWU, NUEtoNWU);
    traj12 = x * transpose(supertrans);

    % Backfill cols 13:15 with the derived attitude [theta phi psi] (rad,
    % NWU): body x-axis along the pointing vector, phi = 0 about it -- the
    % same operations as ballistic_deploy_state.m, so at any trajectory row
    % ballistic_deploy_state(row(1:3), row(4:6), row(7:9), row(10:12))
    % returns xi(7:9) equal to these columns exactly.
    r_nwu   = traj12(:, 7:9) ./ vecnorm(traj12(:, 7:9), 2, 2);
    psi_b   = atan2(r_nwu(:, 2), r_nwu(:, 1));
    theta_b = asin(max(-1, min(1, r_nwu(:, 3))));
    ballistic_sol.trajectory = [traj12, theta_b, zeros(size(traj12, 1), 1), psi_b];
        
    if output
        disp(['Total distance traveled = ',num2str(total_dis_imact),' meters'])
        disp(['Impact angle = ',num2str(impact_angle),' degrees'])
        disp(['Impact velocity = ',num2str(impact_vel),' m/s'])
        disp(['Range along x-axis at impact = ',num2str(range),' m'])
        disp(['Cross-range along z-axis at impact = ',num2str(cross_range),' m'])
        set_default_fonts();
    
        %Plots
        figure
        subplot(2,2,1); plot_traj_subplot(x, [])
        subplot(2,2,2); plot_traj_subplot(x, ap_view)
        subplot(2,2,3); plot_traj_subplot(x, [270, 90])
        subplot(2,2,4); plot_traj_subplot(x, ap_view_min)

        figure
        subplot(2,2,1); plot_traj_subplot(x, 3)

        subplot(2,2,2)
        plot(t, ballistic_sol.trajectory(1:end, 1))
        xlabel("Time (s)")
        ylabel("v_x (m/s)")

        subplot(2,2,3)
        plot(t, ballistic_sol.trajectory(1:end, 2))
        xlabel("Time (s)")
        ylabel("v_y (m/s)")

        subplot(2,2,4)
        plot(t, ballistic_sol.trajectory(1:end, 3))
        xlabel("Time (s)")
        ylabel("v_z (m/s)")

        sgtitle("Ballistic trajectory profile", 'FontSize', 20, 'FontWeight', 'bold')
        export_figure("figs/0_ballistic_trajectory")
    end
    
    ballistic_sol.alpha = alpha;
    ballistic_sol.beta = beta;
    ballistic_sol.apogee = max_ht;
    ballistic_sol.apogee_idx = I;
    ballistic_sol.impact_time = impact_time;
    ballistic_sol.impact_range = range;
    ballistic_sol.impact_crossrange = cross_range;
end

function [value, isterminal, direction] = impactEvent(t, y)
    value      = y(11);
    isterminal = 1;   % Stop the integration
    direction  = -1;
end

function dx = sixdof_ballistics(t, x, env, az_0)
    %% Initialization
    omega = env.omega_earth;
    gravity = env.gravity;

    d = env.d;                  % ref diameter
    S = pi*d^2 / 4;             % ref area
    m = env.m;                  % projectile mass
    I_dot_y = env.I_dot_y;      % transverse MOI rate of change
    r_t = env.r_t;              % distance from projectile COM to rocket nozzle throat
    m_dot = env.m_dot;          % rate of change of projectile mass
    r_e = env.r_e;              % distance from COM to rocket nozzle exit
    delta_f = env.delta_f;      % fin cant angle
    I_x = env.I_x;              % axial moment of inertia
    I_y = env.I_y;              % transverse moment of inertia

    v = x(1:3);
    h = x(4:6);
    r = x(7:9);
    e = x(10:12);

    v_mag = norm(v);

    rho = interp1(env.std_atm(:,1), env.std_atm(:,7), e(2)/1000);
    a = interp1(env.std_atm(:,1), env.std_atm(:,8), e(2)/1000);
    mach = v_mag/a;

    if mach > env.lut_C_D_0(size(env.lut_C_D_0,1),1)
        mach = env.lut_C_D_0(size(env.lut_C_D_0,1),1);
    end

    cos_taoa = (v(1)*r(1) + v(2)*r(2) + v(3)*r(3)) / v_mag;
    alpha = acos(cos_taoa);

    % Aero coefficient lookups
    C_D_0 = interp1(env.lut_C_D_0(:,1),env.lut_C_D_0(:,2),mach);
    C_D_del2 = interp1(env.lut_C_D_del2(:,1),env.lut_C_D_del2(:,2),mach);
    C_L_a0 = interp1(env.lut_C_L_a0(:,1),env.lut_C_L_a0(:,2),mach);
    C_L_a2 = interp1(env.lut_C_L_a2(:,1),env.lut_C_L_a2(:,2),mach);
    C_M_a0 = interp1(env.lut_C_M_a0(:,1),env.lut_C_M_a0(:,2),mach);
    C_M_a2 = interp1(env.lut_C_M_a2(:,1),env.lut_C_M_a2(:,2),mach);
    CMq_CMa_0 = interp1(env.lut_CMq_CMa_0(:,1),env.lut_CMq_CMa_0(:,2),mach);
    CMq_CMa_2 = interp1(env.lut_CMq_CMa_2(:,1),env.lut_CMq_CMa_2(:,2),mach);
    C_N_pa = env.interp_C_N_pa(mach, rad2deg(alpha).^2);
    C_N = interp1(env.lut_C_N(:,1),env.lut_C_N(:,2),mach);
    C_l_p = interp1(env.lut_C_l_p(:,1),env.lut_C_l_p(:,2),mach);
    C_l_delta = interp1(env.lut_C_l_delta(:,1),env.lut_C_l_delta(:,2),mach);
    C_M_pa = env.interp_C_M_pa(mach, rad2deg(alpha).^2);

    C_D = C_D_0 + C_D_del2*(sin(alpha))^2;
    C_L_a = C_L_a0 + C_L_a2*(sin(alpha))^2;
    C_M_a = C_M_a0 + C_M_a2*(sin(alpha))^2;
    CMq_CMa = CMq_CMa_0 + CMq_CMa_2*(sin(alpha))^2;

    %% Rates of change and dynamic coeffs
    h_dot_r = h(1)*r(1) + h(2)*r(2) + h(3)*r(3);
    p = (I_y / I_x) * h_dot_r;

    C_tilde_D        = rho*v_mag*S*C_D / (2*m);
    C_tilde_L_a      = rho*v_mag*S*C_L_a / (2*m);
    C_tilde_N_pa     = rho*d*C_N_pa*p / (2*m);
    C_tilde_N_q      = rho*v_mag*S*d*(C_N) / (2*m);
    C_tilde_l_p      = rho*v_mag*S*(d^2)*C_l_p*p / (2*I_y);
    C_tilde_l_delta  = rho*(v_mag^2)*S*d*delta_f*C_l_delta / (2*I_y);
    C_tilde_M_a      = rho*v_mag*S*d*C_M_a / (2*I_y);
    C_tilde_M_pa     = rho*S*(d^2)*C_M_pa*p / (2*I_y);
    C_tilde_M_q      = rho*v_mag*S*(d^2)*(CMq_CMa) / (2*I_y);  

    if r_t > 0
        J_tilde_DF       = (I_dot_y / (m * r_t)) - (m_dot*r_e / m);
        J_tilde_DM       = (I_dot_y - m_dot*r_e*r_t) / I_y; 
    else
        J_tilde_DF = 0;
        J_tilde_DM = 0;
    end

    g(1) = -gravity*e(1) / env.R;
    g(2) = -gravity*(1 - 2*e(2)/env.R);
    g(3) = 0;

    lambda(1) = 2*omega*(-v(2)*cos(env.L)*sin(az_0) - v(3)*sin(env.L));
    lambda(2) = 2*omega*(v(1)*cos(env.L)*sin(az_0) + v(3)*cos(env.L)*cos(az_0));
    lambda(3) = 2*omega*(v(1)*sin(env.L) - v(2)*cos(env.L)*cos(az_0));

    v_mag_sq = v_mag^2;

    dv(1) = -C_tilde_D*v(1) ...
                + C_tilde_L_a*(v_mag_sq*r(1) - v_mag*v(1)*cos_taoa) ...
                - C_tilde_N_pa*(r(2)*v(3) - r(3)*v(2)) ...
                + C_tilde_N_q*(h(2)*r(3) - h(3)*r(2)) ...
                + g(1) + lambda(1) + (gravity*env.T/m)*r(1) ...
                + J_tilde_DF*(h(2)*r(3) - h(3)*r(2));
    dv(2) = -C_tilde_D*v(2) ...
                + C_tilde_L_a*(v_mag_sq*r(2) - v_mag*v(2)*cos_taoa) ...
                - C_tilde_N_pa*(r(3)*v(1) - r(1)*v(3)) ...
                + C_tilde_N_q*(h(3)*r(1) - h(1)*r(3)) ...
                + g(2) + lambda(2) + (gravity*env.T/m)*r(2) ...
                + J_tilde_DF*(h(3)*r(1) - h(1)*r(3));
    dv(3) = -C_tilde_D*v(3) ...
                + C_tilde_L_a*(v_mag_sq*r(3) - v_mag*v(3)*cos_taoa) ...
                - C_tilde_N_pa*(r(1)*v(2) - r(2)*v(1)) ...
                + C_tilde_N_q*(h(1)*r(2) - h(2)*r(1)) ...
                + g(3) + lambda(3) + (gravity*env.T/m)*r(3) ...
                + J_tilde_DF*(h(1)*r(2) - h(2)*r(1));

    dh(1) = (C_tilde_l_p + C_tilde_l_delta)*r(1) ...
                + (C_tilde_M_a)*(v(2)*r(3) - v(3)*r(2)) ...
                + C_tilde_M_pa*(v(1) - v_mag*r(1)*cos_taoa) ...
                + C_tilde_M_q*(h(1) - h_dot_r*r(1)) ...
                + (gravity*env.T_s/I_y)*r(1) ...
                - J_tilde_DM*(h(1) - h_dot_r*r(1));
    dh(2) = (C_tilde_l_p + C_tilde_l_delta)*r(2) ...
                + (C_tilde_M_a)*(v(3)*r(1) - v(1)*r(3)) ...
                + C_tilde_M_pa*(v(2) - v_mag*r(2)*cos_taoa) ...
                + C_tilde_M_q*(h(2) - h_dot_r*r(2)) ...
                + (gravity*env.T_s/I_y)*r(2) ...
                - J_tilde_DM*(h(2) - h_dot_r*r(2));
    dh(3) = (C_tilde_l_p + C_tilde_l_delta)*r(3) ...
                + (C_tilde_M_a)*(v(1)*r(2) - v(2)*r(1)) ...
                + C_tilde_M_pa*(v(3) - v_mag*r(3)*cos_taoa) ...
                + C_tilde_M_q*(h(3) - h_dot_r*r(3)) ...
                + (gravity*env.T_s/I_y)*r(3) ...
                - J_tilde_DM*(h(3) - h_dot_r*r(3));

    dr(1) = h(2)*r(3) - h(3)*r(2);
    dr(2) = h(3)*r(1) - h(1)*r(3);
    dr(3) = h(1)*r(2) - h(2)*r(1);

    de(1) = v(1);
    de(2) = v(2) + v(1)^2/(2*env.R);
    de(3) = v(3);

    dx = [dv.'; dh.'; dr.'; de.'];
end

function plot_traj_subplot(x, view_arg)
    axis equal
    plot3(x(:,10), x(:,12), x(:,11))
    set(gca, 'ydir', 'reverse')
    grid on
    xlabel('range (m)')
    ylabel('cross-range (m)')
    zlabel('altitude (m)')
    if ~isempty(view_arg)
        view(view_arg)
    end
end