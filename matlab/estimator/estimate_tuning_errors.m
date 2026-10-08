function est = estimate_tuning_errors(y, fs, cfg)
%ESTIMATE_TUNING_ERRORS  Common and differential tuning errors from one microphone window.
%   Proposed scheme of handover Section 13.2 (prototype):
%   * drone-only bands (odd multiples of fd): only the two tenors sound there; a
%     two-tone matrix-pencil fit gives |f1 - f2| = |e_Delta| (times the band number k);
%   * shared bands (centre 2n*fd): chanter partial n + tenor partials 2n. The strongest
%     component is taken as the chanter (check this in recordings); the mean offset of
%     the drone components from it, divided by 2n, is e_Sigma WITH its sign;
%   * if fewer than cfg.min_cycles beat cycles fit in the window, the tones are not
%     resolved and the error is reported as 0 ("within the resolution floor", Step 4).
%   est.eS [Hz], est.absD [Hz], est.resolvedS/D, est.floorS/D, est.passive_sign_agree,
%   est.bands (per-band diagnostics). The sign of e_Delta needs a nudge (nudge_sign).
fd = cfg.fd_nom;

%% Differential error from the drone-only bands
dk = [];  wk = [];
est.bands = struct('type', {}, 'k', {}, 'f', {}, 'a', {}, 'used', {});
for k = cfg.drone_bands
    [z, fsb] = band_baseband(y, fs, k*fd, cfg.fs_b);
    z = trim(z, cfg.trim);  Teff = numel(z)/fsb;
    [f, a] = matrix_pencil(z, fsb, 2);
    sep = abs(diff(f));
    used = sep*Teff >= cfg.min_cycles && min(abs(a))/max(abs(a)) > 0.2;
    if used
        dk(end+1) = sep/k;  wk(end+1) = k*min(abs(a)); %#ok<AGROW>
    end
    est.bands(end+1) = struct('type', 'drone', 'k', k, 'f', f, 'a', a, 'used', used);
end
est.floorD = cfg.min_cycles/(max(cfg.drone_bands)*Teff);
est.resolvedD = ~isempty(dk);
if est.resolvedD
    est.absD = sum(wk.*dk)/sum(wk);
else
    est.absD = 0;
end

%% Common error (signed) from the shared bands
en = [];  wn = [];  agree = [];
for n = cfg.shared_bands
    [z, fsb] = band_baseband(y, fs, 2*n*fd, cfg.fs_b);
    z = trim(z, cfg.trim);  Teff = numel(z)/fsb;
    M = 2 + (est.resolvedD && 2*n*est.absD*Teff >= cfg.min_cycles);  % drones split -> 3 tones
    [f, a] = matrix_pencil(z, fsb, M);
    [~, ic] = max(abs(a));
    idr = [1:ic-1, ic+1:M];                                  % drone components
    idr = idr(abs(a(idr)) >= 0.2*max(abs(a(idr))));          % drop weak spurious fits
    off = sum(abs(a(idr)).*f(idr))/sum(abs(a(idr))) - f(ic); % amplitude-weighted centroid
    used = abs(off)*Teff >= cfg.min_cycles && abs(off) < 2*n*0.5;   % plausible (<0.5 Hz)
    if used
        en(end+1) = off/(2*n);  wn(end+1) = n*sum(abs(a(idr))); %#ok<AGROW>
        if M == 2                                     % passive sign cross-check
            agree(end+1) = envelope_if_sign(z, fsb) == sign(off); %#ok<AGROW>
        end
    end
    est.bands(end+1) = struct('type', 'shared', 'k', n, 'f', f, 'a', a, 'used', used);
end
est.floorS = cfg.min_cycles/(2*max(cfg.shared_bands)*Teff);
est.resolvedS = ~isempty(en);
if est.resolvedS
    est.eS = sum(wn.*en)/sum(wn);
else
    est.eS = 0;
end
est.passive_sign_agree = agree;
end

function z = trim(z, frac)
n = round(frac*numel(z));
z = z(n+1:end-n);
end
