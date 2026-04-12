%% Discretization test
clc

function state_dot = temporary_dynamics(t, state, u, params)
    dt      = params.dt   ;
    g       = params.g    ;
    m       = params.m    ;
    Jxx     = params.Jxx  ;
    Jyy     = params.Jyy  ;
    Jzz     = params.Jzz  ;
    l       = params.l    ;
    Jmp     = params.Jmp  ;
    K1      = params.K1   ;
    K2      = params.K2   ;
    K3      = params.K3   ;
    K4      = params.K4   ;
    K5      = params.K5   ;
    K6      = params.K6   ;
    nuz     = params.nuz  ;
    nupsi   = params.nupsi;
    nu3     = params.nu3  ;
    nu4     = params.nu4  ;
    az      = params.az   ;
    apsi    = params.apsi ;
    b       = params.b    ;
    d       = params.d    ;
    TM      = params.TM   ;
    Omegas = inv(TM)*u;
    Omegas = sqrt(Omegas);
    Omegar = Omegas(1) - Omegas(2) + Omegas(3) - Omegas(4);

    x = state(1);
    y = state(2);
    z = state(3);
    phi = state(4);
    theta = state(5);
    psi = state(6);
    
    x_dot = state(7);
    y_dot = state(8);
    z_dot = state(9);
    phi_dot = state(10);
    theta_dot = state(11);
    psi_dot = state(12);
    
    x_ddot = (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi))*u(1)/m - K1*x_dot/m;
    y_ddot = (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi))*u(1)/m - K2*y_dot/m;
    z_ddot = (cos(phi)*cos(theta))*u(1)/m - g - K3*z_dot/m;
    phi_ddot = theta_dot*psi_dot*((Jyy - Jzz)/Jxx) + (Jmp/Jxx)*theta_dot*Omegar + (l/Jxx)*u(2) - (K4*l/Jxx)*phi_dot;
    theta_ddot = psi_dot*phi_dot*((Jzz - Jxx) / Jyy) - (Jmp/Jxx)*phi_dot*Omegar + (l/Jyy)*u(3) - (K5*l/Jyy)*theta_dot;
    psi_ddot = phi_dot*theta_dot*((Jxx - Jyy)/Jzz) + (1/Jzz)*u(4) - (K6/Jzz)*psi_dot;

    state_dot = [x_dot; y_dot; z_dot; phi_dot; theta_dot; psi_dot; ...
                  x_ddot; y_ddot; z_ddot; phi_ddot; theta_ddot; psi_ddot];
end

function uk = SMC_discrete(xkp1, xkp1_d, xk, xk_d, ukm1, params)
    dt      = params.dt   ;
    g       = params.g    ;
    m       = params.m    ;
    Jxx     = params.Jxx  ;
    Jyy     = params.Jyy  ;
    Jzz     = params.Jzz  ;
    l       = params.l    ;
    Jmp     = params.Jmp  ;
    K1      = params.K1   ;
    K2      = params.K2   ;
    K3      = params.K3   ;
    K4      = params.K4   ;
    K5      = params.K5   ;
    K6      = params.K6   ;
    nuz     = params.nuz  ;
    nupsi   = params.nupsi;
    nu3     = params.nu3  ;
    nu4     = params.nu4  ;
    az      = params.az   ;
    apsi    = params.apsi ;
    b       = params.b    ;
    d       = params.d    ;
    TM      = params.TM   ;
              
    Omegas = inv(TM)*ukm1;
    Omegas = sqrt(Omegas);
    Omegar = real(Omegas(1) - Omegas(2) + Omegas(3) - Omegas(4));

    % x form:
    % x
    % y
    % z
    % phi
    % theta
    % psi

    % xkp2(1)  = 2*xkp1(1) - xkp(1) + (dt^2)*(cos(xk(4))*sin(xk(5))*cos(x(6)) + sin(x(4))*sin(x(6)))*uk(1)/m - dt*K1(xkp1(1) - xk(1))/m;
    % xkp2(2)  = 2*xkp1(2) - xkp(2) + (dt^2)*(cos(xk(4))*sin(xk(5))*sin(x(6)) - sin(x(4))*cos(x(6)))*uk(1)/m - dt*K2(xkp1(2) - xk(2))/m;
    % xkp2(3)  = 2*xkp1(3) - xkp(3) + (dt^2)*(cos(xk(4))*cos(xk(5))*uk(1)/m - g) - dt*K3*(xkp1(3) - xk(3))/m;
    % xkp2(4)  = 2*xkp1(4) - xkp(4) + (xkp1(5) - xk(5))*(xkp1(6) - xk(6))*(Jyy - Jzz)/Jxx + dt^2*uk(2)/Jxx + dt*Jmp*(xkp1(5)-xk(5))*Omegar/Jxx - dt*K4*l*(xkp1(4) - xk(4))/Jxx;
    % xkp2(5)  = 2*xkp1(5) - xkp(5) + (xkp1(6) - xk(6))*(xkp1(4) - xk(4))*(Jzz - Jxx)/Jyy + dt^2*uk(3)/Jyy - dt*Jmp*(xkp1(4)-xk(4))*Omegar/Jyy - dt*K5*l*(xkp1(5) - xk(5))/Jyy;
    % xkp2(6)  = 2*xkp1(6) - xkp(6) + (xkp1(4) - xk(4))*(xkp1(5) - xk(5))*(Jxx - Jyy)/Jzz + dt^2*uk(4)/Jzz - dt*K6*(xkp1(6) - xk(6))/Jzz;

    dxk   = (xkp1 - xk)/dt;
    dxk_d = (xkp1_d - xk_d)/dt; % should be zero

    % Actuated surfaces
    szk = az*(xk_d(3) - xk(3)) + (dxk_d(3) - dxk(3));
    spsik = apsi*(xk_d(6) - xk(6)) + (dxk_d(6) - dxk(6));
    % u1_k = (m/(cos(xk(4))*cos(xk(5))))*(((-az*dxk(3) + K3*dxk(3))/(m + nuz*szk)) + g + nuz*szk)
    u1_k = (m/(cos(xk(4))*cos(xk(5))))*(((-az*dxk(3) + (K3/m)*dxk(3))) + g + nuz*szk);
    % u4_k = Jzz*(( -apsi*dxk(6) + K6*dxk(6) )/(Jzz + nupsi*spsik) + nupsi*spsik)
    u4_k = Jzz*(( -apsi*dxk(6) + (K6/Jzz)*dxk(6) ) + nupsi*spsik);

    % Underactuated surfaces
    a1 = -12*m/(u1_k*cos(xk(6)));
    a2 = -8*m/(u1_k*cos(xk(6)));
    a3 = 1;
    a4 = 6;
    a5 = 12*m/(u1_k*cos(xk(4))*cos(xk(6)));
    a6 = 8*m/(u1_k*cos(xk(4)*cos(xk(6))));
    a7 = 1;
    a8 = 6;

    sphik = a1*(dxk_d(2) - dxk(2)) + a2*(xk_d(2) - xk(2)) + a3*(dxk_d(4) - dxk(4)) + a4*(xk_d(4) - xk(4));
    sthetak = a5*(dxk_d(1) - dxk(1)) + a6*(xk_d(1) - xk(1)) + a7*(dxk_d(5) - dxk(5)) + a8*(xk_d(5) - xk(5));

    g1_k = (cos(xk(4))*sin(xk(5))*sin(xk(6)) - sin(xk(4))*cos(xk(6)))/m;
    g2_k = (cos(xk(4))*sin(xk(5))*cos(xk(6)) + sin(xk(4))*sin(xk(6)))/m;
    f1_k = ( dxk(5)*dxk(6)*(Jyy - Jzz) + Jmp*dxk(5)*Omegar - K4*l*dxk(4) )/Jxx;
    f2_k = ( dxk(6)*dxk(4)*(Jzz - Jxx) - Jmp*dxk(4)*Omegar - K5*l*dxk(5) )/Jyy;

    u2_k = (Jxx/(l*a3))*(-a1*((g1_k*u1_k - K2*dxk(2))/m) - a2*dxk(2) - a3*f1_k - a4*dxk(4) + nu3*sphik);
    u3_k = (Jyy/(l*a7))*(-a5*((g2_k*u1_k - K1*dxk(1))/m) - a6*dxk(1) - a7*f2_k - a8*dxk(5) + nu4*sthetak);
    
    uk = [u1_k; u2_k; u3_k; u4_k];
end

xk = zeros(12,1); % x y z roll pitch yaw
xkp1 = zeros(12,1);
xd = [2; 1; -5; 0; 0; pi/6.218];
xkp1_d = xd;
xk_d = xd;
ukm1 = zeros(4,1);
b=5;
d=2;

% xk(3) = 10;
% xkp1(3) = 10;

params.dt = 0.01;
params.g = 9.81;
params.m = 2;
params.Jxx = 1.25;
params.Jyy = 1.25;
params.Jzz = 2.5;
params.l = 0.2; % m
params.Jmp = 0.2;
params.K1 = 0.01;
params.K2 = 0.01;
params.K3 = 0.01;
params.K4 = 0.012;
params.K5 = 0.012;
params.K6 = 0.012;
params.nuz = 2;
params.nupsi = 2; 
params.nu3 = 5;
params.nu4 = 5;
params.az = 1;
params.apsi = 1;
params.b = 5;
params.d = 2;
params.TM = [b b b b;
      0 -b 0 b;
      -b 0 b 0;
      d -d d -d];
tspan = 0:params.dt:(4000*params.dt);

for i = 1:4000
    xkp2 = zeros(12,1);
    uk = SMC_discrete(xkp1(1:6), xkp1_d, xk(1:6), xk_d, ukm1, params)
    
    [tout, xkp2] = ode45(@(t, x) temporary_dynamics(t, x, uk, params), tspan(i:i+1), xkp1);
    xk = xkp1
    xkp1 = xkp2(end,:).';
    ukm1 = uk;
end

% Scratch
% xkp2 = sym("xkp2", 6);
% xkp1 = sym("xkp1", 6);
% xk = sym("xk", 6);
% xk_d = sym("xk_d", 6);
% xkp1_d = sym("xkp1_d", 6);
% uk = sym("uk", 4);
% syms dt g m Jxx Jyy Jzz l Jmp K1 K2 K3 K4 K5 K6 nuz nupsi nu3 nu4 az apsi b d TM Omegar
% 
% xkp2(1)  = 2*xkp1(1) - xk(1) + (dt^2)*(cos(xk(4))*sin(xk(5))*cos(xk(6)) + sin(xk(4))*sin(xk(6)))*uk(1)/m - dt*K1*(xkp1(1) - xk(1))/m;
% xkp2(2)  = 2*xkp1(2) - xk(2) + (dt^2)*(cos(xk(4))*sin(xk(5))*sin(xk(6)) - sin(xk(4))*cos(xk(6)))*uk(1)/m - dt*K2*(xkp1(2) - xk(2))/m;
% xkp2(3)  = 2*xkp1(3) - xk(3) + (dt^2)*(cos(xk(4))*cos(xk(5))*uk(1)/m - g) - dt*K3*(xkp1(3) - xk(3))/m;
% xkp2(4)  = 2*xkp1(4) - xk(4) + (xkp1(5) - xk(5))*(xkp1(6) - xk(6))*(Jyy - Jzz)/Jxx + dt^2*uk(2)/Jxx + dt*Jmp*(xkp1(5)-xk(5))*Omegar/Jxx - dt*K4*l*(xkp1(4) - xk(4))/Jxx;
% xkp2(5)  = 2*xkp1(5) - xk(5) + (xkp1(6) - xk(6))*(xkp1(4) - xk(4))*(Jzz - Jxx)/Jyy + dt^2*uk(3)/Jyy - dt*Jmp*(xkp1(4)-xk(4))*Omegar/Jyy - dt*K5*l*(xkp1(5) - xk(5))/Jyy;
% xkp2(6)  = 2*xkp1(6) - xk(6) + (xkp1(4) - xk(4))*(xkp1(5) - xk(5))*(Jxx - Jyy)/Jzz + dt^2*uk(4)/Jzz - dt*K6*(xkp1(6) - xk(6))/Jzz;
% 
% dxk_d = (xkp1_d - xk_d)/dt;
% dxk = (xkp1 - xk)/dt;
% szk = az*(xk_d(3) - xk(3)) + (dxk_d(3) - dxk(3));
% szkp1 = (az*(xk_d(3) - xkp1(3))) - ((xkp2(3) - xkp1(3)))/dt;
% 
% eq1 = szkp1 - szk == (-nuz*szk)*dt;
% uk1 = solve(eq1, uk(1));
% test = subs(eq1, uk(1), uk1)