g = 9.81; % m/s^2
l = 0.2; % m
m = 0.8; % kg
Jmp = 0;
Jxx = 1.8e-3; % kgm^2
Jyy = 1.8e-3; % kgm^2
Jzz = 1.5e-3; % kgm^2
kmt = 0.1; % m
kwt = 0.1;
dt   = 1/50;
K = 1;
m_uncertain = m;

constants_struct.g = g;
constants_struct.l = l;
constants_struct.Jmp = Jmp;
constants_struct.Jxx = Jxx;
constants_struct.Jyy = Jyy;
constants_struct.Jzz = Jzz;
constants_struct.kmt = kmt;
constants_struct.kwt = kwt;
constants_struct.dt = dt;
constants_struct.K = K;
constants_struct.m = m;
constants_struct.m_uncertain = m_uncertain;
constants_struct.saturation_on = true;
constants_struct_info = Simulink.Bus.createObject(constants_struct);
constants_struct_bus = evalin("base", constants_struct_info.busName);

% Step 1: Generate control signal AS DELTA FROM GRAVITY
Thrust = [1 - g*m/4; 1 - g*m/4; 1 - g*m/4; 1 - g*m/4];

% Step 2: Apply current state to A, B, C, D matrices
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
C = [0 0 0 0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 0 0 0 0 1;
     0 0 0 0 0 0 1 0 0 0 0 0;
     0 0 0 0 0 0 0 1 0 0 0 0;
     0 0 0 0 0 0 0 0 1 0 0 0];
D = zeros(6, 4);

Aa = [A zeros(12, 3); C(1:3, :), zeros(3, 3)];
Ba = [B; zeros(3, 4)];
Ca = [C zeros(6, 3)];
ssmodel = ss(Aa, Ba, Ca, []);

Q = diag([3 3 6000 1080 1080 1080 180 180 180 0.5 0.5 1000 15 15 300]);
Q = Q / norm([3 3 6000 1080 1080 1080 180 180 180 0.5 0.5 1000 15 15 300]);
R = 1*eye(4);
N = zeros(15,4);

[Kd Sd Pd] = lqrd(Aa, Ba, Q, R, N, 1/50);
Ki_d = Kd(:, 13:15);
Kp_d = Kd(:, 1:12);

Co = ctrb(A, B);
unco = length(A) - rank(Co)

Ob = obsv(A, C);
unobsv = length(A) - rank(Ob)

return
