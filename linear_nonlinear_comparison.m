% Sanity check: linearized vs nonlinear quadrotor dynamics from the same IC
% (ode45, 1 s; compare x1 = nonlinear vs x2 = linear in the workspace).
clc; clear all

[t1, x1] = ode45(@nl_dynamics, [0:0.1:1], [0; 0; 0; 0; 0; 0; 0.05; 0.05; 0.05; 0; 0; 0]);
[t2, x2] = ode45(@lin_dynamics, [0:0.1:1], [0; 0; 0; 0; 0; 0; 0.05; 0.05; 0.05; 0; 0; 0]);

function xdot = lin_dynamics(t, state)
    g = 9.81; % m/s^2
    l = 0.2; % m
    m = 0.8; % kg
    Jmp = 0;
    Jxx = 1.8e-3; % kgm^2
    Jyy = 1.8e-3; % kgm^2
    Jzz = 1.5e-3; % kgm^2
    kmt = 0.1; % m

    % constant per-rotor thrust command (N)
    Thrust = [1; 1; 1; 1];

    % hover-linearized A, B, C, D
    A = [0 0 0 0 0 0 -g 0 0 0 0 0;
         0 0 0 0 0 0 0 g 0 0 0 0;
         0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 1 0 0 0 0 0 0 0;
         0 0 0 1 0 0 0 0 0 0 0 0;
         0 0 0 0 0 1 0 0 0 0 0 0;
         1 0 0 0 0 0 0 0 0 0 0 0;
         0 1 0 0 0 0 0 0 0 0 0 0;
         0 0 -1 0 0 0 0 0 0 0 0 0];
    B = [0 0 0 0;
         0 0 0 0;
         -1/m 0 0 0;
         0 1/Jxx 0 0;
         0 0 1/Jyy 0;
         0 0 0 1/Jzz;
         0 0 0 0;
         0 0 0 0;
         0 0 0 0;
         0 0 0 0;
         0 0 0 0;
         0 0 0 0];
    C = eye(12);
    D = zeros(12, 4);
    u = [1 1 1 1;
       0 -l 0 l;
       l 0 -l 0;
       kmt -kmt kmt -kmt] * Thrust; % plus config
    u(1) = u(1) - g*m; % linear model's T is delta from hover weight

    xdot = A*state + B*u;
end

function xdot = nl_dynamics(t, state)
    g = 9.81; % m/s^2
    l = 0.2; % m
    m = 0.8; % kg
    Jmp = 0;
    Jxx = 1.8e-3; % kgm^2
    Jyy = 1.8e-3; % kgm^2
    Jzz = 1.5e-3; % kgm^2
    kmt = 0.1; % m

    % constant per-rotor thrust command (N)
    Thrust = [1; 1; 1; 1];

    xdot = zeros(12, 1);

    F = [1 1 1 1;
       0 -l 0 l;
       l 0 -l 0;
       kmt -kmt kmt -kmt] * Thrust; % plus config
    vx = state(1);
    vy = state(2);
    vz = state(3);
    wx = state(4);
    wy = state(5);
    wz = state(6);
    theta = state(7);
    phi = state(8);
    psi = state(9);
    x = state(10);
    y = state(11);
    z = state(12);
    T = F(1); % absolute thrust (cf. delta-from-hover in lin_dynamics)
    Mx = F(2);
    My = F(3);
    Mz = F(4);

    xdot(1)         = -vz*wy + vy*wz - g*sin(theta);
    xdot(2)         = -vx*wz + vz*wx + g*cos(theta)*sin(phi);
    xdot(3)         = -vy*wx + vx*wy + g*cos(theta)*cos(phi) - T/m;
    xdot(4)         = (1/Jxx)*(-wy*wz*(Jzz - Jyy) + Mx);
    xdot(5)         = (1/Jyy)*(-wx*wz*(Jxx - Jzz) + My);
    xdot(6)         = Mz / Jzz;
    xdot(7)         = wy*cos(phi) - wz*sin(phi);
    xdot(8)         = wx + wy*sin(phi)*tan(theta) + wz*cos(phi)*tan(theta);
    xdot(9)         = wy*(sin(phi)/cos(theta)) + wz*(cos(phi)/cos(theta));
    xdot(10)        = vx*cos(psi)*cos(theta) + ...
                        vy*(-sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi)) + ...
                        vz*(sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi));
    xdot(11)        = vx*sin(psi)*cos(theta) + ...
                        vy*(cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi)) + ...
                        vz*(-cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi));
    xdot(12)        = vx*sin(theta) - vy*cos(theta)*sin(phi) - vz*cos(theta)*cos(phi);
end