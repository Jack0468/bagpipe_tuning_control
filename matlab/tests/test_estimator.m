function tests = test_estimator
%TEST_ESTIMATOR  One-microphone estimator building blocks (handover Section 13.2).
tests = functiontests(localfunctions);
end

function setupOnce(~)
addpath(fileparts(fileparts(mfilename('fullpath'))));  startup_paths();
end

function testMatrixPencilResolvesCloseTones(tc)
fs = 50;  n = (0:249).';                     % 5 s at 50 Hz, tones 0.12 Hz apart
z = 1.0*exp(1i*2*pi*0.30*n/fs) + 0.3*exp(1i*2*pi*0.42*n/fs);
[f, a] = matrix_pencil(z, fs, 2);
verifyEqual(tc, f, [0.30; 0.42], 'AbsTol', 1e-6);
verifyEqual(tc, abs(a), [1.0; 0.3], 'AbsTol', 1e-6);
end

function testEnvelopeFrequencySign(tc)
fs = 50;  t = (0:499).'/fs;
for d = [0.15, -0.15]                         % weaker tone above / below the stronger
    z = exp(1i*2*pi*1.0*t) + 0.4*exp(1i*2*pi*(1.0 + d)*t);
    verifyEqual(tc, envelope_if_sign(z, fs), sign(d));
end
end

function testEstimatorOnSyntheticAudio(tc)
cfg = estimator_defaults();  cfg.T = 5;  cfg.fc = 480.6;
cfg.f1 = cfg.fc/2 + 0.08;  cfg.f2 = cfg.fc/2 + 0.03;
[y, fs] = synth_bagpipe_audio(cfg);
est = estimate_tuning_errors(y, fs, cfg);
verifyEqual(tc, est.eS, 0.055, 'AbsTol', 0.03);
verifyEqual(tc, est.absD, 0.05, 'AbsTol', 0.01);
end

function testNudgeSign(tc)
verifyEqual(tc, nudge_sign(0.05, 0.08, 0.03), 1);     % f1 > f2: nudging f1 up widens the gap
verifyEqual(tc, nudge_sign(0.05, 0.02, 0.03), -1);
end
