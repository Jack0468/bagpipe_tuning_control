function cfg = estimator_defaults()
%ESTIMATOR_DEFAULTS  Default settings for the synthetic audio and the estimator.
%   Levels and spectral shapes are PLACEHOLDERS: measure them from recordings
%   (handover Section 10.5 item 9) - in particular which source is louder in each band.
cfg.fs = 8000;            % [Hz]
cfg.T = 5;                % analysis window Tw [s]
cfg.fd_nom = 240;         % nominal tenor pitch = band centre spacing [Hz]
cfg.fc = 480;  cfg.f1 = 240;  cfg.f2 = 240;
cfg.A_c = 1.0;            % chanter level
cfg.A_d = 0.5;            % tenor 1 level
cfg.amp2 = 0.85;          % tenor 2 relative to tenor 1
cfg.chanter_rolloff = 0.7;
cfg.even_scale = 0.5;
cfg.snr_db = 30;
cfg.seed = 1;
% Estimator
cfg.drone_bands = [1 3 5];     % odd multiples of fd_nom: tenors only
cfg.shared_bands = [1 2 3];    % n: chanter partial n with tenor partial 2n
cfg.fs_b = 50;                 % baseband sample rate [Hz]
cfg.trim = 0.1;                % fraction trimmed from each end (filter edge effects)
cfg.min_cycles = 0.3;          % beat cycles needed inside the window to resolve two tones
end
