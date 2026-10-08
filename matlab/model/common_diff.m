function [cd, T] = common_diff(sys)
%COMMON_DIFF  Common/differential (Sigma/Delta) coordinates for the two slides, Section 8.6.
%   x_cd = T.x*x = [m z zd  s_S sd_S i_S  s_D sd_D i_D]'
%   u_cd = T.u*u = [q_b F V_S V_D]',  y_cd = T.y*y = [p e_S e_D]'
%   with (.)_S = mean of the two drones and (.)_D = drone 1 minus drone 2.
I3 = eye(3);
T.x = blkdiag(eye(3), [0.5*I3, 0.5*I3; I3, -I3]);
T.u = blkdiag(eye(2), [0.5 0.5; 1 -1]);
T.y = blkdiag(1, [0.5 0.5; 1 -1]);
cd.A  = T.x*sys.A/T.x;
cd.B  = T.x*sys.B/T.u;
cd.C  = T.y*sys.C/T.x;
cd.D  = T.y*sys.D/T.u;
cd.E  = T.x*sys.E;
cd.Dd = T.y*sys.Dd;
end
