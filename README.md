# Robotic Bagpipe Pressure & Drone-Tuning Control (ELEC3304 Assessment 2)

A robotic piper keeps a Highland bagpipe's bag pressure steady through random breath
gaps and keeps both tenor drones in tune with the chanter's Low A, using one pressure
sensor and one microphone. This repository holds the model, the controllers, the
simulations and a prototype of the one-microphone pitch estimator, all in MATLAB.

- Full specification: [bagpipe_tuning_project_handover_v2.md](bagpipe_tuning_project_handover_v2.md)
- Build status: [BLUEPRINT.md](BLUEPRINT.md) · decisions and bugs: [LOGBOOK.md](LOGBOOK.md)
- How to use the simulation: [docs/SIMULATION_GUIDE.md](docs/SIMULATION_GUIDE.md)
- Estimator prototype: [docs/ESTIMATOR_GUIDE.md](docs/ESTIMATOR_GUIDE.md)
- AI-use record for the declaration: [docs/AI_USE_LOG.md](docs/AI_USE_LOG.md)

> **All plant parameters are placeholders** until the students' measurements replace them
> (handover Section 10.5). Every number in `results/` is provisional.

## The system in one paragraph

Plant: 9 states (bag air mass, arm squeeze position and velocity, and position, velocity
and motor current of each tenor's tuning slide), 4 inputs (blowpipe flow, squeeze force,
two slide motor voltages), 3 measured outputs (bag pressure and the two tuning errors
e₁, e₂ = drone pitch − ½ chanter Low A). Target: at most one drone–chanter beat every
10 s, i.e. |e₁|, |e₂| ≤ 0.05 Hz.

## Requirements

- MATLAB R2025b (developed and tested there) with the Control System and Signal
  Processing toolboxes. The Symbolic Math Toolbox is **not** needed: Jacobians are checked
  by finite differences.
- No Python environment is needed (so no `environment.yml`).

## Quick start

In MATLAB, from the `matlab/` folder:

```matlab
run_tests          % 14 unit tests, about 1 minute
run_all('quick')   % whole pipeline with short runs, a few minutes
run_all            % full pipeline, about 10-15 minutes
```

Figures go to `results/figures/`, the console log to `results/run_all_log.txt`, and
summary tables to `results/*.mat`. `results/` is gitignored: regenerate it, don't commit it.

## Repository layout

```
matlab/
  run_all.m, run_tests.m, startup_paths.m
  params/      bagpipe_params.m         every parameter, PLACEHOLDERs flagged
  model/       nonlinear model, equilibrium, analytic + numeric linearisation,
               common/differential transform, scaling
  analysis/    structural properties (PBH, observability formula), feasibility checks
  control/     controller design, loop analysis (S, T, margins)
  sim/         open-loop tests, closed-loop simulator, scenarios, breath gaps, microphone model
  estimator/   synthetic audio + one-microphone tuning-error estimator prototype
  tests/       unit tests (matlab.unittest)
docs/          usage guides and the AI-use log
results/       generated output (gitignored)
```

## Approach and alternatives considered

| Question | Chosen | Alternatives and why not |
|---|---|---|
| Controller form | Pole placement (state feedback + observer + integral action), designed in discrete time | LQR is supported (`bag_method = 'lqr'`) but pole placement matches Lectures 8–9; a continuous design implemented at 5 ms went unstable (LOGBOOK) |
| Tuning loop | Cascade: slide position servo + integral of the microphone error, gain set by loop shaping for 55° phase margin | Observer of slide position from the microphone is impossible: slide offset and reed drift are indistinguishable (LOGBOOK) |
| Pitch estimation | Band split (drone-only / shared bands) + matrix-pencil two-tone fit | FFT peak picking cannot resolve 0.1 Hz in seconds; beat counting alone gives no sign; YIN-type pitch trackers report one pitch, not three overlapping sources |
| Breath-gap handling | Squeeze feedforward at gap onset + refill trajectory (2-DOF) | Pure feedback lets pressure fall ~2 kPa in a worst-case gap |

## Feasibility & AI-leverage assessment

| Measure | Score |
|---|---|
| Hardware / physical complexity | 2 / 5 (simulation only; measurements on real pipes) |
| Software / algorithmic complexity | 4 / 5 (stiff multi-rate model, delayed sensor, DSP) |
| Research uncertainty | 4 / 5 (reed sensitivities, drift rates, estimator on real sound) |
| AI-leverage split | 55% AI-executable · 30% human-prompted design · 15% human hands-on (measurements) |
| Overall difficulty | 7 / 10 |
| Size to first milestone | M |
