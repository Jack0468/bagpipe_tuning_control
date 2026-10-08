function R = run_closed_loop_studies(P, eq, sys, ctrl, quick)
%RUN_CLOSED_LOOP_STUDIES  Closed-loop simulations of handover Sections 14.2, 11 and 13.3.
%   1. 'performance' scenario (random gaps, drift, temperatures), detailed plots
%   2. worst-case gap sequence
%   3. Monte Carlo over random gap sequences (statistics across seeds)
%   4. robustness: plant/sensor parameters perturbed, controller unchanged
%   5. microphone window study: controller redesigned for each window Tw
%   quick = true shortens the runs and the number of seeds (smoke test).
if nargin < 5, quick = false; end
if quick
    T_perf = 150;  n_mc = 3;  T_rob = 100;  Tw_list = [2, 5, 10];
else
    T_perf = 600;  n_mc = 10;  T_rob = 300;  Tw_list = [2, 5, 10, 20];
end

%% 1-2. Performance and worst-case runs
scn = default_scenario(P, 'performance', 1, T_perf);
out = simulate_closed_loop(P, eq, ctrl, scn);
plot_run(out, P, 'cl_1_performance', 'Performance scenario (random gaps, seed 1)');
R.performance = out.metrics;
scn_w = default_scenario(P, 'worst', 1, T_perf);
out_w = simulate_closed_loop(P, eq, ctrl, scn_w);
plot_run(out_w, P, 'cl_2_worst_case', 'Worst-case gap sequence');
R.worst = out_w.metrics;
fprintf('\n=== Closed loop (Section 14.2) ===\n');
print_metrics('performance (seed 1)', out);
print_metrics('worst-case gaps', out_w);

%% 3. Monte Carlo over random breath-gap sequences
mc = cell(n_mc, 1);
for s = 1:n_mc
    o = simulate_closed_loop(P, eq, ctrl, default_scenario(P, 'performance', s, T_perf));
    mc{s} = o.metrics;  mc{s}.failed = o.failed;
end
M = [cellfun(@(m) min(m.frac_in_target), mc), cellfun(@(m) max(m.max_abs_e), mc), ...
     cellfun(@(m) m.max_abs_dp, mc), cellfun(@(m) m.p_min, mc), cellfun(@(m) m.z_max, mc)];
R.monte_carlo = array2table(M, 'VariableNames', ...
    {'frac_in_target_min', 'max_abs_e', 'max_abs_dp', 'p_min', 'z_max'});
R.monte_carlo_failed = sum(cellfun(@(m) m.failed, mc));
fprintf('\nMonte Carlo, %d seeds x %g s (failed runs: %d)\n', n_mc, T_perf, R.monte_carlo_failed);
fprintf('  %-20s %10s %10s %10s\n', '', 'worst', 'median', 'best');
fprintf('  %-20s %10.3f %10.3f %10.3f\n', 'fraction in target', min(M(:, 1)), median(M(:, 1)), max(M(:, 1)));
fprintf('  %-20s %10.3f %10.3f %10.3f\n', 'max |e| [Hz]', max(M(:, 2)), median(M(:, 2)), min(M(:, 2)));
fprintf('  %-20s %10.1f %10.1f %10.1f\n', 'max |dp| [Pa]', max(M(:, 3)), median(M(:, 3)), min(M(:, 3)));
fprintf('  %-20s %10.0f %10.0f %10.0f\n', 'min p [Pa]', min(M(:, 4)), median(M(:, 4)), max(M(:, 4)));
fprintf('  %-20s %10.3f %10.3f %10.3f\n', 'max z [m]', max(M(:, 5)), median(M(:, 5)), min(M(:, 5)));

%% 4. Robustness: perturbed plant / sensor, nominal controller (Section 11)
cases = robustness_cases(P);
scn_r = default_scenario(P, 'performance', 1, T_rob);
rows = cell(numel(cases), 7);
for c = 1:numel(cases)
    o = simulate_closed_loop(P, eq, ctrl, scn_r, cases(c).P);
    m = o.metrics;
    if o.failed || isempty(fieldnames(m))
        rows(c, :) = {cases(c).name, true, NaN, NaN, NaN, NaN, NaN};
    else
        rows(c, :) = {cases(c).name, false, min(m.frac_in_target), max(m.max_abs_e), ...
            m.max_abs_dp, m.p_min, m.frac_p_in_window};
    end
end
R.robustness = cell2table(rows, 'VariableNames', {'case', 'failed', 'frac_in_target_min', ...
    'max_abs_e', 'max_abs_dp', 'p_min', 'frac_p_in_window'});
fprintf('\nRobustness (Section 11), %g s performance scenario, nominal controller:\n', T_rob);
disp(R.robustness);

%% 5. Microphone window study (controller redesigned for each window)
rows = zeros(numel(Tw_list), 5);
for k = 1:numel(Tw_list)
    Pw = P;  Pw.sens.Tw_sigma = Tw_list(k);  Pw.sens.Tw_delta = Tw_list(k);
    cw = design_controllers(Pw, eq, sys, ctrl.opts);
    o = simulate_closed_loop(Pw, eq, cw, scn_r);
    rows(k, :) = [Tw_list(k), cw.slideS.kI, 1/cw.slideS.kI, min(o.metrics.frac_in_target), ...
        max(o.metrics.max_abs_e)];
end
R.window_study = array2table(rows, 'VariableNames', ...
    {'Tw_s', 'kI', 'ramp_error_per_Hz_per_s', 'frac_in_target_min', 'max_abs_e'});
fprintf('Microphone window study (controller redesigned for each Tw):\n');
disp(R.window_study);
fig = figure('Position', [50 50 600 350]);
yyaxis left;  plot(rows(:, 1), rows(:, 4), 'o-');  ylabel('min fraction of time in target');
yyaxis right; plot(rows(:, 1), rows(:, 5), 's-');  ylabel('max |e| [Hz]');
xlabel('estimator window T_w [s]');  grid on;  title('Effect of the microphone window');
save_fig(fig, 'cl_3_window_study');

save(fullfile(results_dir(), 'closed_loop_summary.mat'), 'R');
end

function cases = robustness_cases(P)
c = {};
add = @(name, Pm) struct('name', name, 'P', Pm);
Pm = P; Pm.beta_c = 1.5*P.beta_c;                         c{end+1} = add('beta_c x1.5', Pm);
Pm = P; Pm.beta_c = 0.5*P.beta_c;                         c{end+1} = add('beta_c x0.5', Pm);
Pm = P; Pm.beta_d1 = 1.5*P.beta_d1;                       c{end+1} = add('beta_d1 x1.5 (reed mismatch)', Pm);
Pm = P; Pm.k = 0.6*P.k;                                   c{end+1} = add('k x0.6', Pm);
Pm = P; Pm.k = 1.5*P.k;                                   c{end+1} = add('k x1.5', Pm);
Pm = P; Pm.V0 = 0.8*P.V0;                                 c{end+1} = add('V0 x0.8', Pm);
Pm = P; f = 1.3; Pm.K_c = f*P.K_c; Pm.K_d1 = f*P.K_d1; Pm.K_d2 = f*P.K_d2; Pm.K_d = f*P.K_d;
                                                          c{end+1} = add('reed flow x1.3', Pm);
Pm = P; Pm.slide(2).b_s = 1.3*P.slide(2).b_s;             c{end+1} = add('slide 2 friction x1.3', Pm);
Pm = P; Pm.sens.Tw_sigma = 2*P.sens.Tw_sigma; Pm.sens.Tw_delta = 2*P.sens.Tw_delta;
                                                          c{end+1} = add('mic window x2 (not redesigned)', Pm);
Pm = P; Pm.sens.sigma_e = 3*P.sens.sigma_e;               c{end+1} = add('mic noise x3', Pm);
cases = [c{:}];
end

function plot_run(out, P, name, ttl)
L = out.log;
fig = figure('Position', [50 50 1100 1000]);
tl = tiledlayout(fig, 5, 1, 'TileSpacing', 'compact');
title(tl, ttl);
ax = nexttile(tl);  plot(ax, L.t, L.e1, L.t, L.e2);  yline(ax, P.e_target*[-1 1], 'r:');
ylabel(ax, 'e [Hz]');  legend(ax, 'e_1', 'e_2', 'Location', 'best');  grid(ax, 'on');
ax = nexttile(tl);  plot(ax, L.t, L.eS, L.t, L.eSm, L.t, L.eD, L.t, L.eDm);
ylabel(ax, '[Hz]');  legend(ax, 'e_\Sigma', 'e_\Sigma mic', 'e_\Delta', 'e_\Delta mic', 'Location', 'best');
grid(ax, 'on');
ax = nexttile(tl);  plot(ax, L.t, L.p);  yline(ax, [P.p_min, P.p_max], 'r:');
ylabel(ax, 'p [Pa]');  grid(ax, 'on');  shade_gaps(ax, out.gaps);
ax = nexttile(tl);  yyaxis(ax, 'left');  plot(ax, L.t, L.qb*1e3);  ylabel(ax, 'q_b [g/s]');
yyaxis(ax, 'right');  plot(ax, L.t, L.F);  ylabel(ax, 'F [N]');  grid(ax, 'on');
ax = nexttile(tl);  yyaxis(ax, 'left');  plot(ax, L.t, L.z*1e3, L.t, L.z_ref*1e3, '--');
ylabel(ax, 'z, z_{ref} [mm]');
yyaxis(ax, 'right');  plot(ax, L.t, L.s1*1e3, L.t, L.s2*1e3);  ylabel(ax, 's_1, s_2 [mm]');
grid(ax, 'on');  xlabel(ax, 't [s]');
save_fig(fig, name);
end

function print_metrics(name, out)
m = out.metrics;
if out.failed
    fprintf('%-22s FAILED: %s\n', name, out.fail_msg);
    return
end
fprintf(['%-22s in target %.1f%% / %.1f%%, max|e| %.3f / %.3f Hz, max|dp| %.0f Pa, ', ...
    'p in [%.0f, %.0f] Pa, z max %.3f m, F max %.0f N\n'], name, 100*m.frac_in_target, ...
    m.max_abs_e, m.max_abs_dp, m.p_min, m.p_max, m.z_max, m.F_max);
end
