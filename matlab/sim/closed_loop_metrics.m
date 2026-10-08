function m = closed_loop_metrics(lg, P, scn)
%CLOSED_LOOP_METRICS  Performance summary of a closed-loop run (handover Section 14.2).
if isempty(lg.t)
    m = struct();
    return
end
w = lg.t >= scn.metric_start;
e = [lg.e1(w), lg.e2(w)];
m.frac_in_target = mean(abs(e) <= P.e_target);          % per tenor
m.max_abs_e = max(abs(e));
m.rms_e = sqrt(mean(e.^2));
m.max_abs_eD = max(abs(lg.eD(w)));
m.min_beat_period = 1./(2*m.max_abs_e);                 % [s], worst case
dp = lg.p(w) - P.pbar;
m.max_abs_dp = max(abs(dp));
m.rms_dp = sqrt(mean(dp.^2));
m.p_min = min(lg.p(w));
m.p_max = max(lg.p(w));
m.frac_p_in_window = mean(lg.p(w) >= P.p_min & lg.p(w) <= P.p_max);
m.z_max = max(lg.z(w));
m.F_max = max(lg.F(w));
m.qb_max_used = max(lg.qb(w));
m.V_max_used = max(abs([lg.V1(w); lg.V2(w)]));
m.s_range = [min([lg.s1(w); lg.s2(w)]), max([lg.s1(w); lg.s2(w)])];
end
