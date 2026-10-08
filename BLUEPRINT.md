# Robotic Bagpipe Control Blueprint

Status: **Building — the simulation pipeline runs end to end on placeholder parameters;
waiting on real-pipe measurements.** The design intent is in
[bagpipe_tuning_project_handover_v2.md](bagpipe_tuning_project_handover_v2.md). Decisions and
bugs are in [LOGBOOK.md](LOGBOOK.md).

## Build progress (updated as work happens)

**Stage 1 — Model**
- [x] Parameter file with every placeholder flagged (`matlab/params/bagpipe_params.m`)
- [x] Nonlinear 9-state model, outputs, p > 0 guard
- [x] Equilibrium and feasibility checks, including random breath gaps
- [x] Analytic LTI matrices, checked against finite differences (< 1e-10)
- [x] Common/differential transform, scaling
- [x] Structural properties: PBH reachability/observability 9/9, observability formula, p/F zero at 0
- [x] Open-loop tests of Section 14.1 (steps, ramp, sinusoids vs Bode, gaps, disturbances, linearity, uncertainty, sensor)
- [ ] Replace placeholders with measured values (students, handover Section 10.5)

**Stage 2 — Controller**
- [x] Pressure loop: state feedback + integral + observer + breath-gap feedforward (PM ≈ 53°, GM ≈ 17 dB)
- [x] Tuning loops: servo + integral on the microphone error, 55° PM against the moving average
- [x] Loop analysis: S, T, margins, exact closed-loop stability
- [x] Closed-loop studies: performance, worst case, Monte Carlo, robustness, window study
- [x] Estimator prototype on synthetic audio (accuracy vs window, sign by envelope/frequency and by nudge)
- [x] Unit tests (14, passing)
- [ ] Get the Assessment 2 specification and check the design against its criteria
- [ ] Test the estimator on recordings of the real pipes; update `sens.sigma_e` and `Tw`
- [ ] Optional: Coulomb friction/stiction in the slide model (hunting risk, handover risk 18)
- [ ] Report draft (students' own words; see docs/AI_USE_LOG.md)
