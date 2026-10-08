function R = run_estimator_demo(quick)
%RUN_ESTIMATOR_DEMO  Prototype of the one-microphone estimator on synthetic audio.
%   1. one worked example with spectra and band envelopes
%   2. accuracy vs window length Tw over random tuning errors and chanter pitches
%   3. sign of e_Delta by nudge-and-listen
%   The RMS errors found here are what P.sens.sigma_e and P.sens.Tw_* in bagpipe_params
%   should represent. Re-run on recordings of the real pipes when they exist.
if nargin < 1, quick = false; end
cfg0 = estimator_defaults();
if quick
    Tw_list = [2, 5];  n_trials = 12;  n_nudge = 6;
else
    Tw_list = [2, 5, 10];  n_trials = 40;  n_nudge = 20;
end

%% 1. Worked example
cfg = cfg0;  cfg.T = 5;  cfg.fc = 480.6;  e1 = 0.08;  e2 = 0.03;
cfg.f1 = cfg.fc/2 + e1;  cfg.f2 = cfg.fc/2 + e2;
[y, fs] = synth_bagpipe_audio(cfg);
est = estimate_tuning_errors(y, fs, cfg);
fprintf('\n=== Estimator prototype (Section 13.2) ===\n');
fprintf('example: true e_S = %.3f, |e_D| = %.3f Hz -> estimated e_S = %.3f, |e_D| = %.3f Hz\n', ...
    (e1 + e2)/2, abs(e1 - e2), est.eS, est.absD);
fig = figure('Position', [50 50 1100 650]);
tl = tiledlayout(fig, 2, 3);
Y = abs(fft(y.*hann(numel(y))))/numel(y);  fax = (0:numel(y) - 1)*fs/numel(y);
centres = [240, 480, 720];  labels = {'drone-only band (240 Hz)', 'shared band (480 Hz)', 'drone-only band (720 Hz)'};
for k = 1:3
    ax = nexttile(tl);
    sel = abs(fax - centres(k)) < 3;
    plot(ax, fax(sel), 20*log10(Y(sel)));  grid(ax, 'on');
    title(ax, labels{k});  xlabel(ax, 'f [Hz]');  ylabel(ax, 'dB');
end
for k = 1:3
    [z, fsb] = band_baseband(y, fs, centres(k), cfg.fs_b);
    ax = nexttile(tl);  plot(ax, (0:numel(z) - 1)/fsb, abs(z));  grid(ax, 'on');
    xlabel(ax, 't [s]');  ylabel(ax, '|baseband|');  title(ax, 'beat envelope');
end
title(tl, sprintf('Synthetic chanter + two tenors, e_1 = %.2f Hz, e_2 = %.2f Hz, T_w = %g s', e1, e2, cfg.T));
save_fig(fig, 'est_1_example');

%% 2. Accuracy vs window length
rs = RandStream('mt19937ar', 'Seed', 42);
rows = zeros(numel(Tw_list), 6);
scat = cell(numel(Tw_list), 1);
for i = 1:numel(Tw_list)
    errS = zeros(n_trials, 1);  errD = errS;  trueS = errS;  estS = errS;
    sign_ok = nan(n_trials, 1);  agree = [];
    for t = 1:n_trials
        cfg = cfg0;  cfg.T = Tw_list(i);  cfg.seed = 1000*i + t;
        cfg.fc = 480 + 2*rand(rs) - 1;
        eS = 0.3*rand(rs) - 0.15;  eD = 0.3*rand(rs) - 0.15;
        cfg.f1 = cfg.fc/2 + eS + eD/2;  cfg.f2 = cfg.fc/2 + eS - eD/2;
        [y, fs] = synth_bagpipe_audio(cfg);
        est = estimate_tuning_errors(y, fs, cfg);
        errS(t) = est.eS - eS;  errD(t) = est.absD - abs(eD);
        trueS(t) = eS;  estS(t) = est.eS;
        if abs(eS) > 0.05, sign_ok(t) = sign(est.eS) == sign(eS); end
        agree = [agree, est.passive_sign_agree]; %#ok<AGROW>
    end
    rows(i, :) = [Tw_list(i), sqrt(mean(errS.^2)), sqrt(mean(errD.^2)), ...
        mean(sign_ok, 'omitnan'), mean(agree), est.floorS];
    scat{i} = [trueS, estS];
end
R.accuracy = array2table(rows, 'VariableNames', {'Tw_s', 'rms_err_eS_Hz', 'rms_err_absD_Hz', ...
    'sign_correct_eS_gt_0p05', 'passive_sign_agrees', 'resolution_floor_eS_Hz'});
fprintf('accuracy over %d random trials per window (|e_S|, |e_D| <= 0.15 Hz, 30 dB SNR):\n', n_trials);
disp(R.accuracy);
fig = figure('Position', [50 50 1000 380]);
tl = tiledlayout(fig, 1, 2);
ax = nexttile(tl);  hold(ax, 'on');
for i = 1:numel(Tw_list)
    plot(ax, scat{i}(:, 1), scat{i}(:, 2), 'o', 'DisplayName', sprintf('T_w = %g s', Tw_list(i)));
end
plot(ax, [-0.15 0.15], [-0.15 0.15], 'k:', 'HandleVisibility', 'off');  grid(ax, 'on');
xlabel(ax, 'true e_\Sigma [Hz]');  ylabel(ax, 'estimated e_\Sigma [Hz]');  legend(ax, 'Location', 'best');
ax = nexttile(tl);  plot(ax, rows(:, 1), rows(:, 2), 'o-', rows(:, 1), rows(:, 3), 's-');  grid(ax, 'on');
xlabel(ax, 'window T_w [s]');  ylabel(ax, 'RMS error [Hz]');  legend(ax, 'e_\Sigma', '|e_\Delta|');
save_fig(fig, 'est_2_accuracy');

%% 3. Sign of e_Delta by nudging tenor 1 (+0.03 Hz)
nudge = 0.03;  ok = zeros(n_nudge, 1);
for t = 1:n_nudge
    cfg = cfg0;  cfg.T = 5;  cfg.seed = 5000 + t;
    eD = (0.04 + 0.08*rand(rs))*sign(rand(rs) - 0.5);
    cfg.f1 = cfg.fc/2 + eD/2;  cfg.f2 = cfg.fc/2 - eD/2;
    [y, fs] = synth_bagpipe_audio(cfg);  e0 = estimate_tuning_errors(y, fs, cfg);
    cfg.f1 = cfg.f1 + nudge;  cfg.seed = cfg.seed + 1;
    [y, fs] = synth_bagpipe_audio(cfg);  e1n = estimate_tuning_errors(y, fs, cfg);
    ok(t) = nudge_sign(e0.absD, e1n.absD, nudge) == sign(eD);
end
R.nudge_success = mean(ok);
fprintf('nudge-and-listen sign of e_Delta (|e_D| 0.04-0.12 Hz, nudge %.2f Hz): %d of %d correct\n', ...
    nudge, sum(ok), n_nudge);
save(fullfile(results_dir(), 'estimator_summary.mat'), 'R');
end
