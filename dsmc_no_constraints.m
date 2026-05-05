function u = dsmc_no_constraints(A, B, state, xd, x0) %#ok<INUSL>
persistent xk;
persistent xkp1;
persistent xk_d;
persistent xkp1_d;
persistent ukm1;

if isempty(xk)
    xk = x0;
    xkp1 = xk;
    xk_d = xd;
    xkp1_d = xd;
    ukm1 = zeros(4,1);
end

if abs(norm(xkp1 - state)) > 1e-9
    xk = xkp1;
    xkp1 = state;
end
if abs(norm(xkp1_d - xk_d)) > 1e-9
    xk_d = xkp1_d;
    xkp1_d = xd;
end
if abs(norm(xkp1_d - xd)) > 1e-9
    xk_d = xkp1_d;
    xkp1_d = xd;
end

dt = 1/50;
g = 9.81;
m = 0.8;
Jxx = 1.8e-3; % kgm^2
Jyy = 1.8e-3; % kgm^2
Jzz = 1.5e-3; % kgm^2
l = 0.2; % m
Jmp = 1e-5; % kgm^2
K1 = 0.0001;
K2 = 0.0001;
K3 = 0.0001;
K4 = 0.0012;
K5 = 0.0012;
K6 = 0.0012;
nuz = 7;
nupsi = 7; 
nu3 = 14;
nu4 = 14;
az = 6;
apsi = 1;
b = 5;
d = 2;
TM = [b b b b;
      0 -b 0 b;
      -b 0 b 0;
      d -d d -d];
Omegas = inv(TM)*ukm1;
Omegas = real(sqrt(complex(Omegas)));
Omegar = real(Omegas(1) - Omegas(2) + Omegas(3) - Omegas(4));

dxk   = (xkp1 - xk)/dt;
dxk_d = (xkp1_d - xk_d)/dt; % should be zero

szk = az*(xk_d(3) - xk(3)) + (dxk_d(3) - dxk(3));
szk = min(szk, 2);
szk = max(szk, -2);
spsik = apsi*(xk_d(6) - xk(6)) + (dxk_d(6) - dxk(6));
spsik = min(spsik, 2);
spsik = max(spsik, -2);

u_T = (m/(cos(xk(4))*cos(xk(5))))*(-az*dxk(3) + (K3/m)*dxk(3) + g + nuz*szk);
u_Mz = Jzz*(( -apsi*dxk(6) + (K6/Jzz)*dxk(6) ) + nupsi*spsik);

a1 = 6*m/(u_T*cos(xk(6))); % 6
a2 = 2*m/(u_T*cos(xk(6))); % 2
a3 = 2; 
a4 = 8; 
a5 = -6*m/(u_T*cos(xk(4))*cos(xk(6))); % 6
a6 = -2*m/(u_T*cos(xk(4))*cos(xk(6))); % 2
a7 = 2; 
a8 = 8; %

sphik = a1*(dxk_d(2) - dxk(2)) + a2*(xk_d(2) - xk(2)) + a3*(dxk_d(4) - dxk(4)) + a4*(xk_d(4) - xk(4));
sthetak = a5*(dxk_d(1) - dxk(1)) + a6*(xk_d(1) - xk(1)) + a7*(dxk_d(5) - dxk(5)) + a8*(xk_d(5) - xk(5));

g1_k = (cos(xk(4))*sin(xk(5))*sin(xk(6)) - sin(xk(4))*cos(xk(6)))/m;
g2_k = (cos(xk(4))*sin(xk(5))*cos(xk(6)) + sin(xk(4))*sin(xk(6)))/m;
f1_k = ( dxk(5)*dxk(6)*(Jyy - Jzz) + Jmp*dxk(5)*Omegar - K4*l*dxk(4) )/Jxx;
f2_k = ( dxk(6)*dxk(4)*(Jzz - Jxx) - Jmp*dxk(4)*Omegar - K5*l*dxk(5) )/Jyy;

u_Mx = (Jxx/(l*a3))*(-a1*(g1_k*u_T - (K2*dxk(2)/m)) - a2*dxk(2) - a3*f1_k - a4*dxk(4) + nu3*sphik);
u_My = (Jyy/(l*a7))*(-a5*(g2_k*u_T - (K1*dxk(1)/m)) - a6*dxk(1) - a7*f2_k - a8*dxk(5) + nu4*sthetak);

u = [u_T; u_Mx; u_My; u_Mz];

ukm1 = u;
end