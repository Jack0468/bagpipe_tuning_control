# AI-Use Log (for the Assessment 2 declaration)

The students confirmed that AI-written code is allowed with a declaration. This log
records what the AI assistant (Claude, Anthropic) did, so the declaration can be truthful
and specific. Edit it if anything differs from what actually happened.

## What the AI did
- **Review of the handover specification:** found and corrected errors in the original
  document, as listed below.
- **Specification:** wrote the two-tenor revision
  (`bagpipe_tuning_project_handover_v2.md`) from decisions made by the students:
  - the tuning target (one beat per 10 s);
  - random breath gaps;
  - one microphone for both tenors;
  - chanter on Low A in just intonation;
  - direct measurement of parameters.

  The AI proposed the common/differential estimation scheme. The students chose to treat
  the signal processing as the project's engineering challenge.
- **All MATLAB code in `matlab/`:** model, linearisation, analysis, controller design,
  simulations, estimator prototype and unit tests.
- **Documentation:** README, BLUEPRINT, LOGBOOK and the usage guides.

## Errors the AI found in the original handover (and corrected)
- **Pitch formula:** f_c0 was defined as the pitch at "zero pressure deviation", but the
  formula used total pressure. Both pitch models are now written in deviation form.
- **Slide mass:** reflected rotor inertia (J_m·G², about 1–160 kg) dwarfs the bare slide
  mass. A gear ratio is now included.
- **Bag stiffness:** observability of the bag states from pressure, and the slowest bag pole,
  depend on k. det(O) = a_m·a_z²·k/M and det(A_bag) = −g·a_m·k/M.
- **Squeeze:** F → p has a zero at s = 0, so squeezing cannot hold a steady pressure. The
  single pressure integrator must act through the blowpipe.
- **Ramp drift:** the "constant error" claim for ramp drift was wrong, and so was the later
  "type 2" claim (see LOGBOOK). The tuning loop is type 1, and a ramp leaves r/k_I.
- **Audibility:** "5 cents is the audibility threshold for beating" was wrong. The target was
  re-based on beat rate.

## Errors the AI made and fixed while coding (see LOGBOOK)
- A continuous-time design implemented at 5 ms was unstable. It was replaced by a
  discrete-time design.
- An observer-based tuning loop was fragile and unstable against the real sensor. It was
  replaced by a servo plus integral cascade.
- A sign error in the matrix-pencil estimator. Found by testing; now covered by a unit test.
- The full reachability-matrix rank was misleading because of numerical conditioning. The
  PBH test is used instead.

## How the work was verified
- **Model:** analytic LTI matrices agree with finite-difference Jacobians of the nonlinear
  model to better than 1e-10.
- **Formulas:** the closed-form results (observability determinant, det A_bag, the zero of
  p/F, DC gain 1/g) are checked numerically in `test_model.m`.
- **Sinusoid test:** open-loop sinusoid amplitudes from the nonlinear model match Bode
  magnitudes to 0.1%.
- **Stability:** decided from exact closed-loop poles, including the full moving-average
  sensor.
- **Unit tests:** 14 tests (`run_tests`), all passing.

## What the students still need to do themselves
- Measure the parameters (handover Section 10.5) and replace every PLACEHOLDER in
  `bagpipe_params.m`.
- Re-run `run_all` and check that each result still makes sense with the measured values.
- Record the pipes and test the estimator on real sound.
- Understand and be able to defend every design choice in the report, and write the report
  in their own words.
