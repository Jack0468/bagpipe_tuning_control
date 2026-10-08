function run_all(varargin)
%RUN_ALL  Run the whole project pipeline. Figures and summaries go to <repo>/results/.
%   run_all            full run (about 10-15 minutes)
%   run_all('quick')   shorter scenarios and fewer Monte Carlo runs (smoke test)
%   Steps follow the handover task list (Section 17): parameters -> equilibrium ->
%   LTI model (checked against finite differences) -> structural properties ->
%   feasibility -> open-loop simulations -> controller design and loop analysis ->
%   closed-loop studies -> one-microphone estimator prototype.
quick = any(strcmpi(varargin, 'quick'));
startup_paths();
if ~usejava('desktop'), set(0, 'DefaultFigureVisible', 'off'); end
logfile = fullfile(results_dir(), 'run_all_log.txt');
if exist(logfile, 'file'), delete(logfile); end
diary(logfile);
cleanup = onCleanup(@() diary('off'));
t0 = tic;

P = bagpipe_params();
fprintf('Bagpipe tuning control - run_all (%s mode), %s\n', ternary(quick, 'quick', 'full'), datestr(now));
fprintf('WARNING: these parameters are still PLACEHOLDERS - replace them with measurements:\n  %s\n', ...
    strjoin(P.placeholders, ', '));

eq = compute_equilibrium(P);
fprintf('\n=== Operating point (Section 7) ===\n');
fprintf('p = %.0f Pa, m = %.4g kg, q_b = %.4g kg/s, F = %.1f N, z = %.3f m, s = [%.2g %.2g] m\n', ...
    P.pbar, eq.mbar, eq.qbar, eq.Fbar, P.zbar, eq.sbar);

sys = lti_analytic(P, eq);
num = lti_numeric(P, eq);
fprintf('\n=== LTI model check (Section 8.8): analytic vs finite-difference Jacobians ===\n');
for f = {'A', 'B', 'C', 'E', 'Dd'}
    X = sys.(f{1});  Y = num.(f{1});
    fprintf('  %-2s max relative difference %.1e\n', f{1}, max(abs(X(:) - Y(:)))/max(abs(X(:))));
end
save(fullfile(results_dir(), 'lti_model.mat'), 'sys', 'P', 'eq');

structural_analysis(sys, P, eq, true);
feasibility_checks(P, eq, true);

fprintf('\n=== Open-loop simulations (Section 14.1) ===\n');
run_open_loop_tests(P, eq, sys);

ctrl = design_controllers(P, eq, sys);
fprintf('\n=== Controller design (Section 15) ===\n');
fprintf('bag closed-loop poles: %s 1/s\n', num2str(ctrl.bag.cl_poles.', '%.3g '));
fprintf('bag observer poles:    %s 1/s\n', num2str(ctrl.bag.obs_poles.', '%.3g '));
fprintf('slide servo poles:     %s 1/s; tuning-loop gain kI = %.3g 1/s\n', ...
    num2str(ctrl.slideS.servo_poles.', '%.3g '), ctrl.slideS.kI);
loop_analysis(P, eq, sys, ctrl, true);

run_closed_loop_studies(P, eq, sys, ctrl, quick);
run_estimator_demo(quick);

fprintf('\nDone in %.0f s. Figures: %s\n', toc(t0), results_dir('figures'));
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
