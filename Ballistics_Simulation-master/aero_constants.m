% env = aero_constants(atm_file, projectile_file) -- both file args required
%   (e.g. 'std_atm.csv', 'Aerodynamic_Char_120mm_Mortar.xlsx'); a bare call errors.
function env = aero_constants(atm_file, projectile_file)
    
    % Environmental characteristics
    env.L = 0;              % latitude of firing site, degrees; positive for north
    env.alt = 0;            % altitude of firing site, m
    env.gravity = gravitywgs84(env.alt, env.L);
    env.R = 6371220;        %Radius of earth in meters
    env.omega_earth = 0.00007292; % rad/s, angular velocity of earth
    env.std_atm = readmatrix(atm_file,'Range','A2:k43');
    
    % Projectile dimensions
    env.d = 119.56/1000;    % diameter in m
    env.I_x = 0.02335;      % Axial moment of inertia kg*m^2
    env.I_y = 0.23187;      % Transverse moment of inertia kg*m^2
    env.m = 13.585;         % mass in kg
    
    % (For rockets / finned projectiles)
    env.I_dot_y = 0;        % RoC of transverse MoI
    env.r_t = 0;            % Displacement to nozzle throat from CoG
    env.m_dot = 0;          % RoC of projectile mass
    env.r_e = 0;            % Displacement to nozzle exit from CoG
    env.delta_f = 0;        % Fin cant
    env.T = 0;              % Thrust (kgf: eom2 forms gravity*T, a kgf->N
                            % conversion -- supply kgf, not N, if ever enabled; audit C9)
    env.T_s = 0;            % Rocket spin torque (kgf*m, same convention)
    
    % Aerodynamic coefficient tables from McCoy, Modern Exterior Ballistics,
    % 2nd ed., Ch. 9 (120mm mortar; data extracted from p. 220)
    env.lut_C_D_0       = readmatrix(projectile_file,'Range','A5:B11');
    env.lut_C_D_del2    = readmatrix(projectile_file,'Range','A15:B22');
    env.lut_C_L_a0      = readmatrix(projectile_file,'Range','A26:B30');
    env.lut_C_L_a2      = readmatrix(projectile_file,'Range','A34:B41');
    env.lut_C_M_a0      = readmatrix(projectile_file,'Range','A45:B51');
    env.lut_C_M_a2      = readmatrix(projectile_file,'Range','A55:B63');
    env.lut_CMq_CMa_0   = readmatrix(projectile_file,'Range','A67:B72');
    env.lut_CMq_CMa_2   = readmatrix(projectile_file,'Range','A76:B83');
    env.lut_C_N_pa      = readmatrix(projectile_file,'Range','A87:C109');
    % lut_C_N is the pitch-damping FORCE coefficient (C_Nq + C_Nalphadot),
    % consumed by eom2's (h x r) term -- NOT the normal-force slope C_Nalpha.
    % It is deliberately all-zero (McCoy neglects this force); populating it
    % with a normal-force value would inject a large spurious force. (audit C2)
    env.lut_C_N         = readmatrix(projectile_file,'Range','A113:B114');
    env.lut_C_l_p       = readmatrix(projectile_file,'Range','A118:B125');
    env.lut_C_l_delta   = readmatrix(projectile_file,'Range','A129:B130');
    env.lut_C_M_pa      = readmatrix(projectile_file,'Range','A134:C180');

    % Highest Mach every 1-D table covers; the RHS clamps mach to this so no
    % interp1 can fall off the top of the shortest table (-> NaN, poisoning the
    % state). Today all force/moment tables end at 0.95 (C_l_p reaches 2.5), so
    % this = 0.95 and changes nothing -- it just stops keying the clamp off
    % C_D_0 alone, which would NaN the others if C_D_0 were ever extended. (audit C8)
    env.mach_max = min([env.lut_C_D_0(end,1),    env.lut_C_D_del2(end,1), ...
        env.lut_C_L_a0(end,1),   env.lut_C_L_a2(end,1),   env.lut_C_M_a0(end,1), ...
        env.lut_C_M_a2(end,1),   env.lut_CMq_CMa_0(end,1), env.lut_CMq_CMa_2(end,1), ...
        env.lut_C_N(end,1),      env.lut_C_l_p(end,1),    env.lut_C_l_delta(end,1)]);

    % Pre-build scattered interpolants (avoid reconstructing per ODE call).
    % ExtrapolationMethod 'nearest' clamps queries outside the table hull
    % (Mach 0-1.55, alpha^2 0-1316 deg^2, i.e. total AoA <= ~36 deg): the
    % default unbounded linear extrapolation fabricated sign-changing Magnus
    % coefficients for high-tumble launches (bugsweep 2026-07-10).
    env.interp_C_N_pa = scatteredInterpolant(env.lut_C_N_pa(:,1), env.lut_C_N_pa(:,2), env.lut_C_N_pa(:,3), 'linear', 'nearest');
    env.interp_C_M_pa = scatteredInterpolant(env.lut_C_M_pa(:,1), env.lut_C_M_pa(:,2), env.lut_C_M_pa(:,3), 'linear', 'nearest');
end