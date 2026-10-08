function F = feasibility_checks(P, eq, verbose)
%FEASIBILITY_CHECKS  Equilibrium and breath-gap feasibility checks, handover Section 7.
if nargin < 3, verbose = true; end
% Gap statistics from a long sample of the (truncated) distributions
g = breath_gaps(P, 2e4, 'random', 12345);
t_gap = diff(g, 1, 2);
t_blow = g(2:end, 1) - g(1:end-1, 2);
F.phi_bar = mean(t_gap)/(mean(t_gap) + mean(t_blow));
F.t_gap_max = P.gap.max;
F.t_blow_min = P.blow.min;

F.qb_mean_needed = eq.qbar/(1 - F.phi_bar);
F.ok_mean_flow = P.qb_max >= F.qb_mean_needed;
F.t_refill = eq.qbar*F.t_gap_max/(P.qb_max - eq.qbar);
F.ok_refill = F.t_refill <= F.t_blow_min;
F.dz_gap = eq.Qv_bar*F.t_gap_max/P.A_c;
F.ok_travel = P.zbar + F.dz_gap <= P.z_max;
F.F_gap_end = P.k*(P.zbar + F.dz_gap) + P.A_c*P.pbar;
F.ok_force = F.F_gap_end <= P.F_max && eq.Fbar > 0;
F.all_ok = F.ok_mean_flow && F.ok_refill && F.ok_travel && F.ok_force;

if verbose
    yn = {'FAIL', 'ok'};
    fprintf('\n=== Feasibility checks (Section 7) ===\n');
    fprintf('mean gap fraction phi = %.3f\n', F.phi_bar);
    fprintf('1 mean flow : need q_b,max >= %.3g, have %.3g kg/s  [%s]\n', ...
        F.qb_mean_needed, P.qb_max, yn{F.ok_mean_flow + 1});
    fprintf('2 refill    : %.2f s after the longest gap <= shortest interval %.2f s  [%s]\n', ...
        F.t_refill, F.t_blow_min, yn{F.ok_refill + 1});
    fprintf('3 travel    : zbar + dz_gap = %.3f + %.3f m <= z_max %.3f m  [%s]\n', ...
        P.zbar, F.dz_gap, P.z_max, yn{F.ok_travel + 1});
    fprintf('4 force     : F at end of longest gap %.1f N <= F_max %.0f N, Fbar = %.1f N  [%s]\n', ...
        F.F_gap_end, P.F_max, eq.Fbar, yn{F.ok_force + 1});
end
end
