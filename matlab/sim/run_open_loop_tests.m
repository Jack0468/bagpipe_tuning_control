function R = run_open_loop_tests(P, eq, sys)
%RUN_OPEN_LOOP_TESTS  Open-loop simulations of handover Section 14.1 (A2 of the Week 9 sheet).
%   Nonlinear model (solid) against the LTI model (dashed) for steps, a ramp, sinusoids,
%   breath gaps, disturbances, small vs large deviations, parameter uncertainty and the
%   microphone sensor model. Figures go to results/figures/ol_*.png.
ub = eq.ubar;  db = eq.dbar;
dfix = @(t) db;
R = struct();

%% 1. Steps in each input (applied at t = 1 s)
T = 20;
steps = {[0.05*eq.qbar; 0; 0; 0], '+5% q_b';
         [0; 2; 0; 0],             '+2 N F';
         [0; 0; 0.005; 0.005],     '+5 mV V_\Sigma';
         [0; 0; 0.005; -0.005],    '+10 mV V_\Delta'};
ylabels = {'p [Pa]', 'e_1 [Hz]', 'e_2 [Hz]'};
fig = figure('Position', [50 50 1200 700]);
tl = tiledlayout(fig, 3, 4, 'TileSpacing', 'compact');
for c = 1:4
    o = simulate_open_loop(P, eq, @(t) ub + steps{c, 1}*(t >= 1), dfix, T, sys);
    for r = 1:3
        ax = nexttile(tl, (r - 1)*4 + c);
        plot(ax, o.nl.t, o.nl.y(:, r), '-', o.lin.t, o.lin.y(:, r), '--');  grid(ax, 'on');
        if max(o.nl.y(:, r)) - min(o.nl.y(:, r)) < 1e-3, ylim(ax, mean(o.nl.y(:, r)) + [-1 1]); end
        if r == 1, title(ax, ['step ', steps{c, 2}]); end
        if c == 1, ylabel(ax, ylabels{r}); end
    end
end
xlabel(tl, 't [s]');  legend(nexttile(tl, 1), 'nonlinear', 'LTI', 'Location', 'best');
save_fig(fig, 'ol_1_steps');

%% 2. Ramp in q_b (+20% over 20 s) and 3. sinusoids in F
o = simulate_open_loop(P, eq, @(t) ub + [0.2*eq.qbar*min(t/20, 1); 0; 0; 0], dfix, 30, sys);
fig = figure('Position', [50 50 1100 380]);
tl = tiledlayout(fig, 1, 2);
ax = nexttile(tl);  plot(ax, o.nl.t, o.nl.y(:, 1), o.lin.t, o.lin.y(:, 1), '--');  grid(ax, 'on');
title(ax, 'ramp: q_b +20% over 20 s');  xlabel(ax, 't [s]');  ylabel(ax, 'p [Pa]');
legend(ax, 'nonlinear', 'LTI', 'Location', 'best');
wlist = [0.3, 3, 30];  amp = 2;  Gpf = ss(sys.A, sys.B(:, 2), sys.C(1, :), 0);
R.sine = zeros(numel(wlist), 3);
ax = nexttile(tl);  hold(ax, 'on');
for k = 1:numel(wlist)
    w0 = wlist(k);  Tn = max(10*2*pi/w0, 20);
    o = simulate_open_loop(P, eq, @(t) ub + [0; amp*sin(w0*t); 0; 0], dfix, Tn, sys);
    sel = o.nl.t > Tn - 2*2*pi/w0;                       % last two periods
    a_nl = (max(o.nl.y(sel, 1)) - min(o.nl.y(sel, 1)))/2;
    a_bode = amp*abs(freqresp(Gpf, w0));
    R.sine(k, :) = [w0, a_nl, a_bode];
    plot(ax, o.nl.t*w0/(2*pi), o.nl.y(:, 1) - P.pbar, 'DisplayName', sprintf('\\omega = %g rad/s', w0));
end
grid(ax, 'on');  xlabel(ax, 't [periods]');  ylabel(ax, '\deltap [Pa]');  legend(ax);
title(ax, 'sinusoidal F (2 N): amplitude vs Bode in console');
save_fig(fig, 'ol_2_ramp_sine');
fprintf('\nsinusoids in F (2 N): w [rad/s], p amplitude nonlinear, |G_pF|*2 from Bode\n');
fprintf('  %6.2f  %8.2f  %8.2f\n', R.sine.');

%% 4. Breath gaps with constant squeeze force (no control)
F = feasibility_checks(P, eq, false);
q_blow = eq.qbar/(1 - F.phi_bar);                 % mean flow matched to the gaps
fig = figure('Position', [50 50 1100 380]);
tl = tiledlayout(fig, 1, 2);
modes = {'random', 'worst'};
for k = 1:2
    g = breath_gaps(P, 60, modes{k}, 1);
    ugap = @(t) [q_blow*~any(t >= g(:, 1) & t < g(:, 2)); eq.Fbar; 0; 0];
    o = simulate_open_loop(P, eq, ugap, dfix, 60, sys);
    ax = nexttile(tl);
    plot(ax, o.nl.t, o.nl.y(:, 1), o.lin.t, o.lin.y(:, 1), '--');  grid(ax, 'on');
    yline(ax, [P.p_min, P.p_max], 'r:');  shade_gaps(ax, g);
    title(ax, sprintf('%s gaps, constant F (open loop)', modes{k}));
    xlabel(ax, 't [s]');  ylabel(ax, 'p [Pa]');
    R.gaps(k).p_min = min(o.nl.y(:, 1));  R.gaps(k).events = o.events;  R.gaps(k).failed = o.failed;
    fprintf('open-loop %s gaps: min p = %.0f Pa (p_min %.0f), stopped early: %d\n', ...
        modes{k}, R.gaps(k).p_min, P.p_min, o.failed);
end
legend(nexttile(tl, 1), 'nonlinear', 'LTI', 'Location', 'best');
save_fig(fig, 'ol_3_breath_gaps');

%% 5. Disturbances: drone 1 warmer, chanter warmer, bag warmer, reed drift
dfun = @(t) db + [0.5*(t >= 10); 1.0*(t >= 2); 0; 0.2*(t >= 6); 0.5*min(max((t - 14)/6, 0), 1)];
o = simulate_open_loop(P, eq, @(t) ub, dfun, 25, sys);
eS = mean(o.nl.y(:, 2:3), 2);  eD = o.nl.y(:, 2) - o.nl.y(:, 3);
fig = figure('Position', [50 50 1100 420]);
tl = tiledlayout(fig, 1, 2);
ax = nexttile(tl);  plot(ax, o.nl.t, o.nl.y(:, 2:3), o.lin.t, o.lin.y(:, 2:3), '--');  grid(ax, 'on');
legend(ax, 'e_1', 'e_2', 'e_1 LTI', 'e_2 LTI', 'Location', 'best');  ylabel(ax, 'e [Hz]');
title(ax, 'T_{d1} +1 K @2 s, T_c +0.2 K @6 s, T_{bag} +0.5 K @10 s, drift @14-20 s');
ax = nexttile(tl);  plot(ax, o.nl.t, eS, o.nl.t, eD);  grid(ax, 'on');
legend(ax, 'e_\Sigma', 'e_\Delta', 'Location', 'best');  ylabel(ax, 'e [Hz]');
title(ax, 'common / differential: drift and chanter move e_\Sigma only');
xlabel(tl, 't [s]');
save_fig(fig, 'ol_4_disturbances');

%% 6. Linear vs nonlinear: small and large q_b steps
fig = figure('Position', [50 50 1100 380]);
tl = tiledlayout(fig, 1, 2);
sizes = [0.05, 0.5];
for k = 1:2
    s = sizes(k);
    o = simulate_open_loop(P, eq, @(t) ub + [s*eq.qbar*(t >= 1); 0; 0; 0], dfix, 40, sys);
    ax = nexttile(tl);  plot(ax, o.nl.t, o.nl.y(:, 1), o.lin.t, o.lin.y(:, 1), '--');  grid(ax, 'on');
    yline(ax, P.p_max, 'r:');
    ttl = sprintf('q_b step +%g%%', 100*s);
    stop = o.events(strcmp({o.events.what}, 'z hit 0') | strcmp({o.events.what}, 'p reached 0'));
    if ~isempty(stop)
        ttl = sprintf('%s: nonlinear run stops at %.1f s (%s)', ttl, stop(1).t, stop(1).what);
    end
    title(ax, ttl);  xlabel(ax, 't [s]');  ylabel(ax, 'p [Pa]');
    t_end = o.nl.t(end);                         % the nonlinear run may stop at an event
    R.linearity(k) = abs(o.nl.y(end, 1) - interp1(o.lin.t, o.lin.y(:, 1), t_end));
    R.linearity_t(k) = t_end;
end
legend(nexttile(tl, 1), 'nonlinear', 'LTI', 'Location', 'best');
save_fig(fig, 'ol_5_linearity');
fprintf('p difference nonlinear - LTI at end of run: +5%% step %.1f Pa (t = %.1f s), +50%% step %.1f Pa (t = %.1f s)\n', ...
    R.linearity(1), R.linearity_t(1), R.linearity(2), R.linearity_t(2));

%% 7. Parameter uncertainty (k and beta) and the microphone model
fig = figure('Position', [50 50 1200 380]);
tl = tiledlayout(fig, 1, 3);
ax = nexttile(tl);  hold(ax, 'on');
for f = [0.5, 1, 1.5]
    Pk = P;  Pk.k = f*P.k;  eqk = compute_equilibrium(Pk);
    o = simulate_open_loop(Pk, eqk, @(t) eqk.ubar + [0; 2*(t >= 1); 0; 0], @(t) eqk.dbar, 20);
    plot(ax, o.nl.t, o.nl.y(:, 1), 'DisplayName', sprintf('k x %.1f', f));
end
grid(ax, 'on');  legend(ax);  title(ax, 'F step +2 N, bag stiffness \pm50%');  ylabel(ax, 'p [Pa]');
ax = nexttile(tl);  hold(ax, 'on');
for f = [0.5, 1, 1.5]
    Pb = P;  Pb.beta_d1 = f*P.beta_d1;
    o = simulate_open_loop(Pb, eq, @(t) ub + [0.05*eq.qbar*(t >= 1); 0; 0; 0], dfix, 20);
    plot(ax, o.nl.t, o.nl.y(:, 2), 'DisplayName', sprintf('\\beta_{d1} x %.1f', f));
end
grid(ax, 'on');  legend(ax);  title(ax, 'q_b step +5%: tuning error e_1');  ylabel(ax, 'e_1 [Hz]');
% Microphone estimator model applied to a slide pulse
o = simulate_open_loop(P, eq, @(t) ub + [0; 0; 0.01; 0.01]*(t >= 1 & t < 3), dfix, 30, sys);
Ts = 0.005;  tg = (0:Ts:30).';
e1 = interp1(o.nl.t, o.nl.y(:, 2), tg);  e2 = interp1(o.nl.t, o.nl.y(:, 3), tg);
S = mic_sensor_init(P.sens, Ts);  rs = RandStream('mt19937ar', 'Seed', 7);  em = zeros(size(tg));
for k = 1:numel(tg)
    [S, em(k)] = mic_sensor_step(S, (e1(k) + e2(k))/2, e1(k) - e2(k), rs);
end
ax = nexttile(tl);  plot(ax, tg, (e1 + e2)/2, tg, em);  grid(ax, 'on');
legend(ax, 'true e_\Sigma', 'microphone estimate', 'Location', 'best');
title(ax, sprintf('microphone model: %g s window, %g Hz noise', P.sens.Tw_sigma, P.sens.sigma_e));
ylabel(ax, 'e_\Sigma [Hz]');
xlabel(tl, 't [s]');
save_fig(fig, 'ol_6_uncertainty_sensor');
end
