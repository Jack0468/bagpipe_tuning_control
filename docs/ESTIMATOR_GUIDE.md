# One-Microphone Estimator — Usage Guide

## What this is
A prototype of the signal processing in handover Section 13.2: from one microphone that
hears the chanter (Low A) and both tenor drones, estimate the common tuning error
e_Σ = ½(e₁ + e₂) with its sign and the differential error |e_Δ| = |f₁ − f₂|. It runs on
synthetic audio now and should be re-run on recordings of the real pipes.

## Access
MATLAB, `matlab/estimator/`. Run `run_estimator_demo` (or `run_all`).

## Common operations

```matlab
startup_paths();
cfg = estimator_defaults();               % audio and estimator settings
cfg.T = 5;  cfg.fc = 480.6;               % 5 s window, chanter pitch
cfg.f1 = cfg.fc/2 + 0.08;  cfg.f2 = cfg.fc/2 + 0.03;
[y, fs] = synth_bagpipe_audio(cfg);       % or: [y, fs] = audioread('recording.wav');
est = estimate_tuning_errors(y, fs, cfg)  % est.eS, est.absD, per-band diagnostics
run_estimator_demo                        % accuracy vs window length, nudge test
```

With a recording, set `cfg.fd_nom` to the measured tenor pitch and pass the audio
in windows of `Tw` seconds.

## How it works
1. **Band split.** Chanter partials sit at n·f_c ≈ 480n Hz and tenor partials at
   k·f_d ≈ 240k Hz. So odd multiples of 240 Hz hold the tenors only (*drone-only bands*), and
   even multiples hold a chanter partial plus both tenors (*shared bands*).
2. **Baseband.** Each band is shifted to 0 Hz and decimated to 50 Hz (`band_baseband`).
3. **Two- or three-tone fit.** A matrix-pencil fit (`matrix_pencil`) finds the tone
   frequencies, resolving gaps smaller than the FFT bin 1/T_w at good signal-to-noise ratio.
   - Drone-only band k: the two tones are k·|e_Δ| apart.
   - Shared band n: the strongest tone is taken as the chanter. The amplitude-weighted
     drone offset from it, divided by 2n, is e_Σ with its sign.
4. **Resolution floor.** If fewer than 0.3 beat cycles fit in the window, the error is
   reported as 0, meaning within the floor (Step 4).
5. **Sign checks.**
   - `envelope_if_sign`: the passive envelope/instantaneous-frequency correlation.
   - `nudge_sign`: nudge tenor 1 and see whether |e_Δ| grows (gives the sign of e_Δ).

## Results on synthetic audio (30 dB SNR, |e| ≤ 0.15 Hz)
Measured by `run_estimator_demo` (see `results/run_all_log.txt` for the latest numbers):

| Window | RMS error e_Σ | RMS error \|e_Δ\| | e_Σ sign (when \|e_Σ\| > 0.05 Hz) |
|---|---|---|---|
| 2 s | ≈ 0.04 Hz | ≈ 0.012 Hz | 100% |
| 5 s | ≈ 0.02 Hz | ≈ 0.0015 Hz | 100% |
| 10 s | ≈ 0.017 Hz | ≈ 0.001 Hz | 100% |

Nudge-and-listen recovered the sign of e_Δ in 20 of 20 trials.

## Data location
Synthetic audio is generated in memory. Put recordings outside git or in a gitignored folder,
because audio files are large.

## Troubleshooting / known limits
- **Synthetic sound is idealised.** It has steady harmonic partials, white noise and fixed
  levels. Real reeds have jitter, the drones' level depends on pressure, and the room adds
  reflections.
- **Chanter loudness.** The "strongest tone is the chanter" rule must be checked in each
  shared band of the recordings (handover risk 16).
- **Bass drone.** It is assumed stopped. Its partials (multiples of 120 Hz) fall in every
  band.
- **Merged tenors.** With e_Δ ≈ 0 the two tenors merge and can partly cancel in a band.
  Using several bands helps.
