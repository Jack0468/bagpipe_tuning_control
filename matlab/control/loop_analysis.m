function LA = loop_analysis(P, eq, sys, ctrl, make_plots)
%LOOP_ANALYSIS  Loop gains, sensitivities S and T, margins and stability (Section 15.3).
%   Each loop is broken at its measurement, giving a SISO loop L (negative feedback):
%   * pressure loop: bag plant (q_b, F) -> p, observer-based controller p -> (q_b, F);
%   * tuning loops:  microphone error e_m -> integral -> slide reference -> position
%     servo -> slide -> e -> moving-average estimator -> e_m.
%   Stability is decided from the exact closed-loop poles (the moving average is
%   realised as an FIR filter), not from one phase margin. Margins are reported at the
%   first (lowest-frequency) crossover. The breath-gap feedforward is outside the loop.
if nargin < 5, make_plots = true; end
Ts = ctrl.Ts;
w = logspace(-3, log10(0.95*pi/Ts), 3000);

%% Pressure loop
bag = ctrl.bag;
Pb = c2d(ss(sys.A(1:3, 1:3), sys.B(1:3, 1:2), sys.C(1, 1:3), 0), Ts, 'zoh');
nx = 3;
Cb = ss([bag.Ad - bag.Bd(:, 1:2)*bag.Kx, -bag.Bd(:, 1:2)*bag.Ki; zeros(1, nx), 1], ...
        [bag.Bd(:, 3); Ts], [-bag.Kx, -bag.Ki], zeros(2, 1), Ts);
Lb = -Pb*Cb;
LA.pressure = summarise(w, squeeze(freqresp(Lb, w)).', all(abs(eig(feedback(Lb, 1))) < 1));

%% Tuning loops (common and differential)
modes = {'slideS', 'slideD'};
for q = 1:2
    K = ctrl.(modes{q});
    Tservo = ss(K.Phi - K.Gam*K.Kx, K.Gam*K.Kx(1), [1 0], 0, Ts);     % s_ref -> s
    Iz = tf(K.kI*Ts, [1 -1], Ts);                                     % e_m -> s_ref (x h)
    Lnoma = Iz*Tservo;                                                % loop without the sensor
    Lw = squeeze(freqresp(Lnoma, w)).'.*moving_average_fr(K.Tw, P.sens.f_mic, Ts, w);
    n_hold = round(1/(P.sens.f_mic*Ts));
    Nt = round(K.Tw*P.sens.f_mic)*n_hold;
    MA = ss(tf(ones(1, Nt)/Nt, [1, zeros(1, Nt - 1)], Ts));
    stable = all(abs(eig(feedback(Lnoma*MA, 1))) < 1);
    LA.(modes{q}) = summarise(w, Lw, stable);
    LA.(modes{q}).kI = K.kI;
end

%% Ramp drift: the tuning loop is type 1 (the servo closes the slide's own integrator)
LA.ramp_error_per_Hz_per_s = 1/ctrl.slideS.kI;       % steady error per Hz/s of drift

if make_plots
    fig = figure('Position', [100 100 950 380]);
    tl = tiledlayout(fig, 1, 2);
    plot_ST(nexttile(tl), w, LA.pressure, 'Pressure loop');
    plot_ST(nexttile(tl), w, LA.slideS, 'Common tuning loop (moving-average sensor)');
    save_fig(fig, 'loop_S_T');
end

fprintf('\n=== Loop analysis (Section 15.3) ===\n');
print_m('pressure loop', LA.pressure);
print_m('common tuning loop', LA.slideS);
print_m('differential tuning loop', LA.slideD);
fprintf('tuning-loop integral gains kI = %.3g / %.3g 1/s; ramp drift -> steady error %.3g Hz per Hz/s\n', ...
    LA.slideS.kI, LA.slideD.kI, LA.ramp_error_per_Hz_per_s);
end

function s = summarise(w, Lw, stable)
s.L = Lw;
s.S = 1./(1 + Lw);
s.T = Lw./(1 + Lw);
s.stable = stable;
[s.PM_deg, s.wc, s.GM_dB] = first_crossover_pm(w, Lw);
s.Smax_dB = 20*log10(max(abs(s.S)));
end

function plot_ST(ax, w, s, ttl)
semilogx(ax, w, 20*log10(abs(s.S)), w, 20*log10(abs(s.T)), w, 20*log10(abs(s.L)), ':');
grid(ax, 'on');  ylim(ax, [-60 30]);
xlabel(ax, '\omega [rad/s]');  ylabel(ax, 'magnitude [dB]');
title(ax, sprintf('%s: PM %.0f^\\circ, GM %.1f dB', ttl, s.PM_deg, s.GM_dB));
legend(ax, '|S|', '|T|', '|L|', 'Location', 'southwest');
end

function print_m(name, s)
yn = {'UNSTABLE', 'stable'};
fprintf('%-26s %-8s PM %6.1f deg at %.3g rad/s, GM %5.1f dB, max|S| %.1f dB\n', ...
    name, yn{s.stable + 1}, s.PM_deg, s.wc, s.GM_dB, s.Smax_dB);
end
