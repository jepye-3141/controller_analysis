g = 9.81; % m/s^2
l = 0.2; % m
m = 0.8; % kg
Jmp = 0;
Jxx = 1.8e-3; % kgm^2
Jyy = 1.8e-3; % kgm^2
Jzz = 1.5e-3; % kgm^2
kmt = 0.1; % m

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
% C = [0 0 0 0 0 0 0 0 1 0 0 0;
%      0 0 0 0 0 0 0 0 0 1 0 0;
%      0 0 0 0 0 0 0 0 0 0 1 0;
%      0 0 0 0 0 0 0 0 0 0 0 1];
% D = zeros(6, 4);

% u = [1 1 1 1;
%    0 -l 0 l;
%    l 0 -l 0;
%    kmt -kmt kmt -kmt] * Thrust; % plus config

Aa = [A zeros(12, 3); C(1:3, :), zeros(3, 3)];
Ba = [B; zeros(3, 4)];
Ca = [C zeros(6, 3)];
ssmodel = ss(Aa, Ba, Ca, []);
% ssmodel = ss(A, B, C, [], 1/50);
% ssmodel = ss(A, B, C, []);

Q = diag([3 3 6000 1080 1080 1080 180 180 180 0.5 0.5 1000 15 15 300]);
Q = Q / norm([3 3 6000 1080 1080 1080 180 180 180 0.5 0.5 1000 15 15 300]);
% R = [1 0 0 0; 0 10 0 0; 0 0 10 0; 0 0 0 10];
% Q = diag([4.5 4.5 6000 1080 1080 1080 180 180 180 0.75 0.75 1000 22.5 22.5 30000]);
% Q = diag([4.5 4.5 6000 1080 1080 1080 180 180 180 0.75 0.75 1000 1000 22.5 22.5 30000]);
R = 1*eye(4);
N = zeros(15,4);

% [Kd Sd Pd] = lqr(Aa, Ba, Q, R, N);
% Ki = Kd(:, 13:15);
% Kp = Kd(:, 1:12);
[Kd Sd Pd] = lqrd(Aa, Ba, Q, R, N, 1/50);
Ki_d = Kd(:, 13:15);
Kp_d = Kd(:, 1:12);
% [K_d S_d P_d] = lqi(ssmodel, Q, R, N)

Co = ctrb(A, B);
unco = length(A) - rank(Co)

Ob = obsv(A, C);
unobsv = length(A) - rank(Ob)

return
