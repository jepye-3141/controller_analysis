function xdot = system_dynamics(A, B, u, x, constants)
    g = constants.g          ;
    l = constants.l          ;
    Jmp = constants.Jmp        ;
    Jxx = constants.Jxx        ;
    Jyy = constants.Jyy        ;
    Jzz = constants.Jzz        ;
    kmt = constants.kmt        ;
    kwt = constants.kwt        ;
    dt = constants.dt         ;
    K = constants.K          ;
    m = constants.m          ;
    m_uncertain = constants.m_uncertain;
    u = reshape(u, [4, 1]);
    x = reshape(x, [12, 1]);
    % xdot = A*x + B*u;
    xdot=zeros(12,1);
    
    vx = x(1);
    vy = x(2);
    vz = x(3);
    wx = x(4);
    wy = x(5);
    wz = x(6);
    theta = x(7);
    phi = x(8);
    psi = x(9);
    % T = K*u(1);
    % Mx = K*u(2);
    % My = K*u(3);
    % Mz = K*u(4);
    T = u(1);
    Mx = u(2);
    My = u(3);
    Mz = u(4);
    
    % Step 3: Calculate derivatives
    xdot(1)         = -vz*wy + vy*wz - g*sin(theta);
    xdot(2)         = -vx*wz + vz*wx + g*cos(theta)*sin(phi);
    xdot(3)         = -vy*wx + vx*wy + g*cos(theta)*cos(phi) - T/m;
    xdot(4)         = (1/Jxx)*(-wy*wz*(Jzz - Jyy) + Mx - (kwt/kmt)*Jmp*Mz*wy);
    xdot(5)         = (1/Jyy)*(-wx*wz*(Jxx - Jzz) + My - (kwt/kmt)*Jmp*Mz*wx);
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