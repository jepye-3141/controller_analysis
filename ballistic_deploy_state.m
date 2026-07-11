function [xi, att] = ballistic_deploy_state(v_nwu, h_nwu, r_nwu, pos_nwu)
% Drone 12-state deploy seed from ballistic quantities (NWU trajectory frame).
%
%   xi = ballistic_deploy_state(v, h, r, pos)
%
% Inputs are slices of ballistic_sol.trajectory (or arc/time interpolations
% of it): v = cols 1:3 velocity, h = cols 4:6 angular rate H/I_y (~ body
% omega for the fin-stabilized round; axial I_x/I_y understatement is the
% accepted approximation), r = cols 7:9 pointing unit vector, pos = cols
% 10:12 position. Since later 2026-07-10, trajectory cols 13:15 carry the
% SAME derived attitude [theta phi psi] backfilled post-hoc by eom2 (the
% legacy integrated-o channel -- degree IC mixed with rad/s do = h, never a
% valid attitude -- was removed from the ODE state). At exact trajectory
% rows this helper's xi(7:9) equals cols 13:15; for interpolated stations
% always interpolate r (cols 7:9) and call this helper -- never interpolate
% the angle columns themselves (psi wraps at +-pi).
%
% Packaging convention: the drone's body x-axis is aligned with the shell
% pointing vector r, with zero roll about that axis -- at apogee (nose
% ~horizontal) the drone deploys near-level. Angles and frames follow
% system_dynamics.m exactly (theta pitch, phi roll, psi yaw; world z up,
% body z down): body->world
%   M = [cp*ct, cp*st*sf - sp*cf, cp*st*cf + sp*sf;
%        sp*ct, sp*st*sf + cp*cf, sp*st*cf - cp*sf;
%        st,   -ct*sf,           -ct*cf]
% (p=psi, t=theta, f=phi; rows 1:2 match the standard ZYX body->world, row
% 3 is negated for the z-up world -- these are the xdot(10:12) coefficient
% rows in system_dynamics.m). M is orthogonal (M' = inv(M), det = -1), and
% with psi = atan2(r2, r1), theta = asin(r3), phi = 0 its first column is
% exactly r-hat.
%
% Returns xi = [vel_body(3); rotvel_body(3); theta; phi; psi; pos_world(3)]
% (the *_single model seed order) and att with the angles, M, and the
% nose-alignment residual norm(M(:,1) - r_hat) for verification.

    v_nwu   = v_nwu(:);
    h_nwu   = h_nwu(:);
    pos_nwu = pos_nwu(:);
    r_hat   = r_nwu(:) / norm(r_nwu);   % re-unit (linear interpolation shrinks unit vectors)

    psi0   = atan2(r_hat(2), r_hat(1));
    theta0 = asin(max(-1, min(1, r_hat(3))));
    phi0   = 0;   % roll about the nose axis is not indexed by separation

    cp = cos(psi0);   sp = sin(psi0);
    ct = cos(theta0); st = sin(theta0);
    cf = cos(phi0);   sf = sin(phi0);
    M = [cp*ct, cp*st*sf - sp*cf, cp*st*cf + sp*sf;
         sp*ct, sp*st*sf + cp*cf, sp*st*cf - cp*sf;
         st,   -ct*sf,           -ct*cf];

    vel0    = M.' * v_nwu;   % world -> body
    rotvel0 = M.' * h_nwu;

    xi = [vel0; rotvel0; theta0; phi0; psi0; pos_nwu];

    att = struct('psi', psi0, 'theta', theta0, 'phi', phi0, 'M', M, ...
                 'nose_err', norm(M(:, 1) - r_hat));
end
