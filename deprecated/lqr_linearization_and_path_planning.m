%% two-point BVP, infeasible, not working
clc
% Step 1: Calculate a reference trajectory
function [f, df] = fun(x0, ns, ts, ws)
    optODE = odeset('RelTol', 1e-3, 'AbsTol', 1e-3);
    z0 = x0;
    
    if nargout > 1
        zhist = [];
        for ks = 1:ns
            [tspan, zs] = ode45(@(t, x)state(t, x, ws, ks), [ts(ks), ts(ks+1)], z0, optODE);
            z0 = zs(end, :)';
            zhist = [zhist; zs(1,:)];
        end
        f = zs(end, :);
        syms T Mx My Mz
        w = [T Mx My Mz];
        eq1 = w*w.';

        df = [];
        for i = 1:size(ws, 1)
            df = [df; double(vpa(subs(gradient(eq1, [T, Mx, My, Mz]), [T, Mx, My, Mz], ws(i, :)))).'];
        end
    else
        for ks = 1:ns
            [tspan, zs] = ode45(@(t, x)state(t, x, ws, ks), [ts(ks), ts(ks+1)], z0, optODE);
            z0 = zs(end, :)';
        end
        f = zs(end, :);
    end
end

function dx  = state(t, x, w, ks)
    dx = zeros(13, 1);
    g = 9.81;
    m = 0.8;
    Jxx = 0.1;
    Jyy = 0.1;
    Jzz = 0.1;

    vx = x(1);
    vy = x(2);
    vz = x(3);
    wx = x(4);
    wy = x(5);
    wz = x(6);
    theta = x(7);
    phi = x(8);
    psi = x(9);
    xpos = x(10);
    ypos = x(11);
    zpos = x(12);
    T = w(ks, 1);
    Mx = w(ks, 2);
    My = w(ks, 3);
    Mz = w(ks, 4);

    
    % Step 3: Calculate derivatives
    dx(1)         = -vz*wy + vy*wz - g*sin(theta);
    dx(2)         = -vx*wz + vz*wx + g*cos(theta)*sin(phi);
    dx(3)         = -vy*wx + vx*wy + g*cos(theta)*cos(phi) - T/m;
    dx(4)         = (1/Jxx)*(-wy*wz*(Jzz - Jyy) + Mx);
    dx(5)         = (1/Jyy)*(-wx*wz*(Jxx - Jzz) + My);
    dx(6)         = Mz / Jzz;
    dx(7)         = wy*cos(phi) - wz*sin(phi);
    dx(8)         = wx + wy*sin(phi)*tan(theta) + wz*cos(phi)*tan(theta);
    dx(9)         = wy*(sin(phi)/cos(theta)) + wz*(cos(phi)/cos(theta));
    dx(10)        = vx*cos(psi)*cos(theta) + ...
                        vy*(-sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi)) + ...
                        vz*(sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi));
    dx(11)        = vx*sin(psi)*cos(theta) + ...
                        vy*(cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi)) + ...
                        vz*(-cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi));
    dx(12)        = vx*sin(theta) - vy*cos(theta)*sin(phi) - vz*cos(theta)*cos(phi);
    dx(13)        = w(ks,:)*w(ks,:).';

end

function [J, dJ] = obj(x0, ns, ts, ws)
    if nargout > 1
        [f, df] = fun(x0, ns, ts, ws);
        J = f(13);
        dJ = df;
    else
        f = fun(x0, ns, ts, ws);
        J = f(13);
    end
end

function [c, ceq] = ctr(x0, xf, ns, ts, ws)
    f = fun(x0, ns, ts, ws);
    ceq = [f(1:3)'; f(7:9)'; f(10:12)' - xf];
    c = [];
end

optNLP = optimoptions('fmincon', 'SpecifyObjectiveGradient', true, ...
    'GradConstr', 'off', 'DerivativeCheck', 'off', ...
    'Display', 'iter', 'TolX', 1e-3, 'TolFun', 1e-3, 'TolCon', 1e-2, ...
    'DiffMinChange', 1e-6, 'MaxFunctionEvaluations', 1e5);

t0 = 0;
tf = 10;
x0 = zeros(13,1);
xf = [2; 4; 10];

ns = 50;
ts = t0:(tf-t0)/ns:tf;

w0 = zeros(ns, 4);
[wopt, fval] = fmincon(@(ws)obj(x0, ns, ts, ws), w0, [], [], [], [], [], [], ...
               @(ws)ctr(x0, xf, ns, ts, ws), optNLP);

z0 = x0;
state_eval = [];
t = [];
optODE = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
for ks = 1:ns
    [tspan, zs] = ode45(@(t, x) state(t, x, wopt, ks), [ts(ks), ts(ks+1)], z0, optODE);
    z0 = zs(end, :);
    state_eval = [state_eval; zs(:,1:12)];
    t = [t; tspan];
end

%% Multiple Shooting - Working
clc
function dx = state_dot(x, u)
    dx = zeros(12, 1);
    g = 9.81;
    m = 0.8;
    Jxx = 0.1;
    Jyy = 0.1;
    Jzz = 0.1;

    vx = x(1);
    vy = x(2);
    vz = x(3);
    wx = x(4);
    wy = x(5);
    wz = x(6);
    theta = x(7);
    phi = x(8);
    psi = x(9);
    xpos = x(10);
    ypos = x(11);
    zpos = x(12);
    T = u(1);
    Mx = u(2);
    My = u(3);
    Mz = u(4);

    
    % Step 3: Calculate derivatives
    dx(1)         = -vz*wy + vy*wz - g*sin(theta);
    dx(2)         = -vx*wz + vz*wx + g*cos(theta)*sin(phi);
    dx(3)         = -vy*wx + vx*wy + g*cos(theta)*cos(phi) - T/m;
    dx(4)         = (1/Jxx)*(-wy*wz*(Jzz - Jyy) + Mx);
    dx(5)         = (1/Jyy)*(-wx*wz*(Jxx - Jzz) + My);
    dx(6)         = Mz / Jzz;
    dx(7)         = wy*cos(phi) - wz*sin(phi);
    dx(8)         = wx + wy*sin(phi)*tan(theta) + wz*cos(phi)*tan(theta);
    dx(9)         = wy*(sin(phi)/cos(theta)) + wz*(cos(phi)/cos(theta));
    dx(10)        = vx*cos(psi)*cos(theta) + ...
                        vy*(-sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi)) + ...
                        vz*(sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi));
    dx(11)        = vx*sin(psi)*cos(theta) + ...
                        vy*(cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi)) + ...
                        vz*(-cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi));
    dx(12)        = vx*sin(theta) - vy*cos(theta)*sin(phi) - vz*cos(theta)*cos(phi);
end

function [c, ceq] = cf(y, opts)
    N = opts.N;
    T = opts.T;
    M = opts.M;
    x_init = opts.x0;
    x_final = opts.xf;
    Nx = opts.Nx;
    Nu = opts.Nu;

    x = reshape(y(1:Nx*N), [], Nx);
    u = reshape(y(Nx*N+1:end), [], Nu);

    h = T / (N-1) / (M-1);

    node_states = zeros(N, Nx);
    for i = 1:N-1
        x0 = x(i, :);
        u0 = u(i, :);
        states = zeros(M, Nx);
        states(1,:) = x0;

        for j=1:M-1
           k1 = state_dot(states(j, :), u0);
           k2 = state_dot(states(j, :) + h/2 * k1, u0);
           k3 = state_dot(states(j, :) + h/2 * k2, u0);
           k4 = state_dot(states(j, :) + h * k3, u0);
           states(j+1, :) = states(j, :) + h/6*(k1 +2*k2 +2*k3 +k4).';
        end
        node_states(i+1,:) = states(end,:);
    end

    ceq_temp = x(2:end,:) - node_states(2:end,:);
    ceq_temp = [ceq_temp; x(1,:) - x_init];
    ceq_temp = [ceq_temp; x(end,:) - x_final];

    ceq = reshape(ceq_temp, [], 1);
    c = [];
end

function [L] = objfun(y, opts)
    N = opts.N;
    Nx = opts.Nx;
    u = y(Nx*N + 1: end);

    L = sum(y.^2);
end

N           = 100;           % number of nodes => (N-1) subintervals
M           = 5;            % number of points per subinterval
T           = 10;            % final time
Nx     = 12;            % number of states
Nu   = 4;            % number of controls
x0          = zeros(1,12);       % initial states
xf          = [zeros(1,9) 2 4 10];      % final states
% Store parameters for the use of constraint and objective function
opts.N         = N;         
opts.M           = M;          
opts.T           = T;          
opts.Nx     = Nx;    
opts.Nu   = Nu;  
opts.x0          = x0;    
opts.xf          = xf;

xL1 = -Inf*ones(N*3, 1);                  
xU1 = Inf*ones(N*3, 1);
xL2 = -Inf*ones(N*3, 1);                  
xU2 = Inf*ones(N*3, 1);
xL3 = -(pi/2)*ones(N*3, 1);                  
xU3 = (pi/2)*ones(N*3, 1);
xL4 = zeros(N*3, 1);                  
xU4 = Inf*ones(N*3, 1);
xL = [xL1; xL2; xL3; xL4];
xU = [xU1; xU2; xU3; xU4];

u1L = zeros(N-1, 1);
u2L = -1.5*ones(N-1, 1);
u3L = -1.5*ones(N-1, 1);
u4L = -1.5*ones(N-1, 1);

u1U = 12*ones(N-1, 1);
u2U = 1.5*ones(N-1, 1);
u3U = 1.5*ones(N-1, 1);
u4U = 1.5*ones(N-1, 1);

uL = [u1L; u2L; u3L; u4L];
uU = [u1U; u2U; u3U; u4U];

yL = [xL; uL];
yU = [xU; uU];
y0 = 0.1*ones((Nx + Nu)*N - Nu, 1);

cf(y0, opts)
objfun(y0, opts)

options = optimoptions('fmincon','Display','Iter','Algorithm','interior-point',...
    'MaxFunEvals',Inf,'ConstraintTolerance',1e-3,'OptimalityTolerance',1e-7); % interior-point

tic
[yopt,f] = fmincon(@(y) objfun(y, opts),y0,[],[],[],[],yL,yU,...
        @(y) cf(y, opts),options);
toc 

x = reshape(yopt(1:Nx*N), [], Nx);
u = reshape(yopt(Nx*N+1: end), [], Nu);

[c, ceq] = cf(yopt, opts);
ceq = reshape(ceq(1:Nx*N), [], Nx)

figure()
stairs(linspace(0, T, N), [u; nan nan nan nan])
hold on
title('Control values')
xlabel('time (s)');
ylabel('u');

% plot states
figure()
plot(linspace(0, T, N), x(:,10))
hold on
plot(linspace(0, T, N), x(:,11))
plot(linspace(0, T, N), x(:,12))
title('States')
xlabel('time (s)');
ylabel('states')

%% RRT* with MATLAB - Sort of working
clc
startPose = [0; 0; 0; 0; 0; 0; 0];  % [x y z qw qx qy qz]
goalPose = [2; 4; 10; 0; 0; 0; 0];
wpts = [startPose goalPose];
tpts = [0 10];
numsamples = 500;

[q,qd,qdd,qddd,qdddd,pp,timepoints,tsamples] = minsnappolytraj(wpts,tpts,numsamples);

syms x(t) y(t) z(t)
g = 9.81;
theta = atan(diff(x,t,t) / (g + diff(z,t,t)));
phi = asin(-diff(y,t,t) / sqrt(diff(x,t,t)^2 + diff(y,t,t)^2 + (g + diff(z,t,t))^2));
% phi = atan( (diff(x,t,t)/(g + diff(z,t,t))) * (cos(atan( (diff(x,t,t)/(g + diff(z,t,t))) ))) );

% thetadot = (x*(g + diff(z,t,t)) + diff(x,t,t)*z)/( (g + diff(z,t,t))^2 + diff(x,t,t)^2);
% phidot = ((diff(x,t,t)*diff(x,t,t,t) - (g+diff(z,t,t))*diff(z,t,t,t))*diff(y,t,t) - (diff(x,t,t)^2 + (g+diff(z,t,t))^2)*diff(y,t,t,t)) /...
%     (diff(x,t,t)^2 + diff(y,t,t)^2 + (g+diff(z,t,t))^2)*sqrt(diff(x,t,t)^2 + (g+diff(z,t,t))^2);

thetadot = diff(theta, t);
phidot = diff(phi, t);

thetaddot = diff(thetadot, t);
phiddot = diff(phidot, t);
us = [];
for i = 1:size(q,2)
    fprintf("[]")
end
fprintf("\n")

for i = 1:size(q,2)
    fprintf("[]")
    s = q(1:6,i).';
    sd = qd(1:6,i).';
    sdd = qdd(1:6,i).';
    sddd = qddd(1:6,i).';
    
    theta_dd_sub = subs(thetaddot,    diff(x, t, t, t, t), 0);
    theta_dd_sub = subs(theta_dd_sub, diff(y, t, t, t, t), 0);
    theta_dd_sub = subs(theta_dd_sub, diff(z, t, t, t, t), 0);
    theta_dd_sub = subs(theta_dd_sub, diff(x, t, t, t), sddd(1));
    theta_dd_sub = subs(theta_dd_sub, diff(y, t, t, t), sddd(2));
    theta_dd_sub = subs(theta_dd_sub, diff(z, t, t, t), sddd(3));
    theta_dd_sub = subs(theta_dd_sub, diff(x, t, t), sdd(1));
    theta_dd_sub = subs(theta_dd_sub, diff(y, t, t), sdd(2));
    theta_dd_sub = subs(theta_dd_sub, diff(z, t, t), sdd(3));
    theta_dd_sub = subs(theta_dd_sub, diff(x, t), sd(1));
    theta_dd_sub = subs(theta_dd_sub, diff(y, t), sd(2));
    theta_dd_sub = subs(theta_dd_sub, diff(z, t), sd(3));
    theta_dd_sub = subs(theta_dd_sub, x(t), s(1));
    theta_dd_sub = subs(theta_dd_sub, y(t), s(2));
    theta_dd_sub = double(subs(theta_dd_sub, z(t), s(3)));
    
    phi_dd_sub = subs(phiddot,    diff(x, t, t, t, t), 0);
    phi_dd_sub = subs(phi_dd_sub, diff(y, t, t, t, t), 0);
    phi_dd_sub = subs(phi_dd_sub, diff(z, t, t, t, t), 0);
    phi_dd_sub = subs(phi_dd_sub, diff(x, t, t, t), sddd(1));
    phi_dd_sub = subs(phi_dd_sub, diff(y, t, t, t), sddd(2));
    phi_dd_sub = subs(phi_dd_sub, diff(z, t, t, t), sddd(3));
    phi_dd_sub = subs(phi_dd_sub, diff(x, t, t), sdd(1));
    phi_dd_sub = subs(phi_dd_sub, diff(y, t, t), sdd(2));
    phi_dd_sub = subs(phi_dd_sub, diff(z, t, t), sdd(3));
    phi_dd_sub = subs(phi_dd_sub, diff(x, t), sd(1));
    phi_dd_sub = subs(phi_dd_sub, diff(y, t), sd(2));
    phi_dd_sub = subs(phi_dd_sub, diff(z, t), sd(3));
    phi_dd_sub = subs(phi_dd_sub, x(t), s(1));
    phi_dd_sub = subs(phi_dd_sub, y(t), s(2));
    phi_dd_sub = double(subs(phi_dd_sub, z(t), s(3)));
    
    theta_d_sub = subs(thetadot,    diff(x, t, t, t), sddd(1));
    theta_d_sub = subs(theta_d_sub, diff(y, t, t, t), sddd(2));
    theta_d_sub = subs(theta_d_sub, diff(z, t, t, t), sddd(3));
    theta_d_sub = subs(theta_d_sub, diff(x, t, t), sdd(1));
    theta_d_sub = subs(theta_d_sub, diff(y, t, t), sdd(2));
    theta_d_sub = subs(theta_d_sub, diff(z, t, t), sdd(3));
    theta_d_sub = subs(theta_d_sub, diff(x, t), sd(1));
    theta_d_sub = subs(theta_d_sub, diff(y, t), sd(2));
    theta_d_sub = subs(theta_d_sub, diff(z, t), sd(3));
    theta_d_sub = subs(theta_d_sub, x(t), s(1));
    theta_d_sub = subs(theta_d_sub, y(t), s(2));
    theta_d_sub = double(subs(theta_d_sub, z(t), s(3)));
    
    phi_d_sub = subs(phidot,    diff(x, t, t, t), sddd(1));
    phi_d_sub = subs(phi_d_sub, diff(y, t, t, t), sddd(2));
    phi_d_sub = subs(phi_d_sub, diff(z, t, t, t), sddd(3));
    phi_d_sub = subs(phi_d_sub, diff(x, t, t), sdd(1));
    phi_d_sub = subs(phi_d_sub, diff(y, t, t), sdd(2));
    phi_d_sub = subs(phi_d_sub, diff(z, t, t), sdd(3));
    phi_d_sub = subs(phi_d_sub, diff(x, t), sd(1));
    phi_d_sub = subs(phi_d_sub, diff(y, t), sd(2));
    phi_d_sub = subs(phi_d_sub, diff(z, t), sd(3));
    phi_d_sub = subs(phi_d_sub, x(t), s(1));
    phi_d_sub = subs(phi_d_sub, y(t), s(2));
    phi_d_sub = double(subs(phi_d_sub, z(t), s(3)));
    
    u = zeros(4, 1);
    u(1) = 0.8*sqrt(sdd(1)^2 + sdd(2)^2 + (g + sdd(3))^2);
    u(2) = 0.1*phi_dd_sub;
    u(3) = 0.1*theta_dd_sub;
    u(4) = 0.1*sdd(6);

    us = [us u];
end
fprintf("\n")

% Check whether this is a valid control schema
z0 = zeros(12, 1);
state_eval = [];
t = [];
optODE = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
for i = 1:(size(us, 2)-1)
    [tspan, zs] = ode45(@(t, x) state_dot(x, us(:,i)), tsamples(i:i+1), z0, optODE);
    z0 = zs(end, :);
    state_eval = [state_eval; zs(:,1:12)];
    t = [t; tspan];
end
state_eval

%% Sampling-based controls planning
% 1: Define a discrete set of possible control inputs by first looking at
%       the range of moments commanded previously as a starting point.
clc
g = 9.81;
m = 0.8;

u_min = min(us, [], 2);
u_max = max(us, [], 2);
min_moment = min(u_min(2:end));
max_moment = max(u_max(2:end));

dT = 0.1*(u_max(1) - u_min(1));
dM = 0.1*(max_moment - min_moment);
steps = 3;

T_disc = linspace(u_min(1)-dT/2, u_max(1)+dT/2, steps);
Mx_disc = linspace(min_moment - dM/2, max_moment + dM/2, steps);
My_disc = linspace(min_moment - dM/2, max_moment + dM/2, steps);
Mz_disc = linspace(min_moment - dM/2, max_moment + dM/2, steps);

% Append steady state efforts
T_disc = [T_disc g*m];
Mx_disc = [Mx_disc 0];
My_disc = [My_disc 0];
Mz_disc = [Mz_disc 0];

% And then create the combinations of control actions
U = [];
for a=1:steps+1
    for b=1:steps+1
        for c=1:steps+1
            for d=1:steps+1
                u_combo = [T_disc(a); Mx_disc(b); My_disc(c); Mz_disc(d)];
                U = [U u_combo];
            end
        end
    end
end

% 2: Define initial and goal states
% 
x0 = [0; 0; 0; 0; 0;  0];
xf = [0; 0; 0; 2; 4; 10];

% 3: Loop through a random tree between x0 and xf. 
% i. Pick a random state in the operational area
% ii. Find the closest node as defined by total state difference
% iii. Propagate forward from the near node to the new node, using diff.
%        flatness
% iv. Check the final and intermediate nodes for goal reaching
% v. If goal not reached, pick another random node and repeat.

%%
% Step 2: Linearize about each point
syms vx vy vz wx wy wz theta phi psi x y z T Mx My Mz
xdot = [-vz*wy + vy*wz - g*sin(theta);
        -vx*wz + vz*wx + g*cos(theta)*sin(phi);
        -vy*wx + vx*wy + g*cos(theta)*cos(phi) - T/m;
        (1/Jxx)*(-wy*wz*(Jzz - Jyy) + Mx);
        (1/Jyy)*(-wx*wz*(Jxx - Jzz) + My);
        Mz / Jzz;
        wy*cos(phi) - wz*sin(phi);
        wx + wy*sin(phi)*tan(theta) + wz*cos(phi)*tan(theta);
        wy*(sin(phi)/cos(theta)) + wz*(cos(phi)/cos(theta));
        vx*cos(psi)*cos(theta) + ...
          vy*(-sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi)) + ...
          vz*(sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi));
        vx*sin(psi)*cos(theta) + ...
          vy*(cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi)) + ...
          vz*(-cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi));
        vx*sin(theta) - vy*cos(theta)*sin(phi) - vz*cos(theta)*cos(phi)];
        
Ajac = jacobian(xdot, [vx vy vz wx wy wz theta phi psi x y z]);
Bjac = jacobian(xdot, [T Mx My Mz]);

Ajac_mat = [];
Bjac_mat = [];

for i = 1:length(state_eval)
    Ajac_eval = double(vpa(subs(Ajac, [vx vy vz wx wy wz theta phi psi x y z], state_eval(i, :))));
    Bjac_eval = double(vpa(subs(Bjac, [vx vy vz wx wy wz theta phi psi x y z], state_eval(i, :))));

    Ajac_mat = cat(3, Ajac_mat, Ajac_eval);
    Bjac_mat = cat(3, Bjac_mat, Bjac_eval);
end

% Step 3: LQR DARE at each step
Ki_d_mat = [];
Kp_d_mat = [];
for i = 1:length(state_eval)
    Aa = [Ajac_mat(:,:,i) zeros(12, 3); C(1:3, :), zeros(3, 3)];
    Ba = [Bjac_mat(:,:,i); zeros(3, 4)];
    Ca = [C zeros(6, 3)];
    ssmodel = ss(Aa, Ba, Ca, []);
    
    % Q = diag([3 3 6000 1080 1080 1080 180 180 180 0.5 0.5 1000 15 15 30000]);
    % R = [1 0 0 0; 0 10 0 0; 0 0 10 0; 0 0 0 10];
    Q = diag([4.5 4.5 6000 1080 1080 1080 180 180 180 0.75 0.75 1000 22.5 22.5 30000]);
    R = 1*eye(4);
    N = zeros(15,4);
    
    [Kd Sd Pd] = lqrd(Aa, Ba, Q, R, N, 1/50);
    Ki_d_eval = Kd(:, 13:15);
    Kp_d_eval = Kd(:, 1:12);

    Ki_d_mat = cat(3, Ki_d_mat, Ki_d_eval);
    Kp_d_mat = cat(3, Kp_d_mat, Kp_d_eval);
end
sample_points = state_eval;
