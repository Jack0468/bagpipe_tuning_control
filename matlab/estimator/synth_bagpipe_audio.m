function [y, fs] = synth_bagpipe_audio(cfg)
%SYNTH_BAGPIPE_AUDIO  Synthetic microphone signal: chanter Low A + two tenor drones (+ noise).
%   cfg fields (defaults in estimator_defaults):
%     fs, T            sample rate [Hz] and duration [s]
%     fc, f1, f2       chanter and tenor pitches [Hz]: scalars or vectors of length fs*T
%     A_c, A_d, amp2   chanter and tenor 1 levels; tenor 2 level relative to tenor 1
%     chanter_rolloff  chanter partial n has amplitude A_c*n^-rolloff (conical bore: all n)
%     even_scale       even drone harmonics scaled by this (stopped pipe: odd ones dominate)
%     snr_db, seed     broadband SNR and random seed (phases and noise)
%   Replace with recordings of the real pipes once they exist (handover Section 10.5).
fs = cfg.fs;
N = round(cfg.T*fs);
rs = RandStream('mt19937ar', 'Seed', cfg.seed);
ph = @(f) 2*pi*cumsum(expand(f, N))/fs;
phc = ph(cfg.fc);  ph1 = ph(cfg.f1);  ph2 = ph(cfg.f2);
fmax = 0.45*fs;
y = zeros(N, 1);
for n = 1:floor(fmax/max(cfg.fc))
    y = y + cfg.A_c*n^(-cfg.chanter_rolloff)*cos(n*phc + 2*pi*rand(rs));
end
for k = 1:floor(fmax/max([cfg.f1(:); cfg.f2(:)]))
    a = cfg.A_d/k*(1 - (1 - cfg.even_scale)*(mod(k, 2) == 0));
    y = y + a*cos(k*ph1 + 2*pi*rand(rs)) + cfg.amp2*a*cos(k*ph2 + 2*pi*rand(rs));
end
sig = sqrt(mean(y.^2)/10^(cfg.snr_db/10));
y = y + sig*randn(rs, N, 1);
end

function v = expand(f, N)
if isscalar(f), v = repmat(f, N, 1); else, v = f(:); end
end
