# Simulation — Usage Guide

## What this is
MATLAB model and simulations of the robotic bagpipe: the 9-state nonlinear plant, its LTI
model, the pressure and tuning controllers, and open- and closed-loop studies matching
handover Sections 6–15.

## Access
Open MATLAB in `matlab/` and run `startup_paths()` once per session (the run scripts do
this for you). No credentials or external services.

## Common operations

Whole pipeline:

```matlab
run_all('quick')        % smoke test, a few minutes
run_all                 % full, about 10-15 minutes
run_tests               % unit tests
```

Step by step:

```matlab
startup_paths();
P   = bagpipe_params();                  % edit this file with your measurements
eq  = compute_equilibrium(P);            % operating point (Section 7)
sys = lti_analytic(P, eq);               % A, B, C, D, E, Dd (Section 8)
structural_analysis(sys, P, eq);         % PBH reachability/observability, poles, zeros
feasibility_checks(P, eq);               % breath-gap checks (Section 7)
run_open_loop_tests(P, eq, sys);         % Section 14.1 figures

ctrl = design_controllers(P, eq, sys);   % pressure + tuning controllers
loop_analysis(P, eq, sys, ctrl);         % S, T, margins (Section 15.3)

scn = default_scenario(P, 'performance', 1, 600);   % kind, seed, length [s]
out = simulate_closed_loop(P, eq, ctrl, scn);
out.metrics                              % fraction in target, max |e|, pressure, ...
plot(out.log.t, [out.log.e1 out.log.e2]); yline([-0.05 0.05])
```

- **Scenarios** (`default_scenario`): `'performance'` (random gaps, reed drift, one drone
  warming faster, temperature steps), `'worst'` (longest gap then shortest interval,
  repeated), `'gaps_only'`, `'quiet'`.
- **Robustness:** pass a perturbed copy of the parameters as a fifth argument; the plant
  uses it, the controller keeps the nominal design:
  ```matlab
  Pt = P;  Pt.beta_c = 1.5*P.beta_c;
  out = simulate_closed_loop(P, eq, ctrl, scn, Pt);
  ```
- **Retuning:** pass options to `design_controllers` (defaults at the bottom of that file),
  e.g. `design_controllers(P, eq, sys, struct('tuning_pm_deg', 60))`. Check the result
  with `loop_analysis` and the unit tests.

## Data location
Everything generated goes to `results/` (gitignored): `figures/*.png`, `run_all_log.txt`,
`lti_model.mat`, `closed_loop_summary.mat`, `estimator_summary.mat`.

## How the simulator works
- Fixed step `Ts = 5 ms` (`ctrl.Ts`). The bag (nonlinear) is integrated with RK4 substeps;
  each slide (linear) uses its exact ZOH discretisation, including the motor current.
- The microphone is modelled as a moving average of each tuning error over `Tw` seconds,
  plus noise, updated at 10 Hz (`mic_sensor_init/step`). `sigma_e` = 0.02 Hz comes from
  the estimator prototype.
- Breath gaps force the blowpipe flow to 0; the controller knows when a gap starts and
  ends but not in advance.
- If bag pressure reaches 0 the run stops with `out.failed = true` (the model assumes
  p > 0 and does not clamp it).

## Troubleshooting
- *"Unrecognized function"*: run `startup_paths()` from `matlab/`.
- *A full-matrix reachability rank of 2*: expected, see LOGBOOK (2026-10-08, pole spread);
  use the PBH result that `structural_analysis` prints.
- *Short runs look worse than long ones*: disturbance rates are fixed per second, so a
  short run still sees realistic rates, but the start-up transient is a larger fraction.
