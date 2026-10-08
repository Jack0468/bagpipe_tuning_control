function [sc, S] = scale_model(sys, P, eq)
%SCALE_MODEL  Scale the LTI model by expected maximum deviations, handover Section 8.7.
%   x = S.x*x_s etc., so every scaled variable is O(1). The tuning errors are
%   scaled by the target itself, so |e_s| <= 1 means "within target".
sx = [eq.qbar*P.gap.max, 0.04, 0.05, 0.005, 0.01, 0.5, 0.005, 0.01, 0.5];
su = [eq.qbar, 50, P.V_max, P.V_max];
sy = [50, P.e_target, P.e_target];
sd = [1, 1, 1, 1, 0.1];
S.x = diag(sx);  S.u = diag(su);  S.y = diag(sy);  S.d = diag(sd);
sc.A  = S.x\sys.A*S.x;
sc.B  = S.x\sys.B*S.u;
sc.E  = S.x\sys.E*S.d;
sc.C  = S.y\sys.C*S.x;
sc.D  = S.y\sys.D*S.u;
sc.Dd = S.y\sys.Dd*S.d;
end
