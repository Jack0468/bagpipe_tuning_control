# Troubleshooting

Issues that cross subsystem boundaries. Subsystem-specific notes are in
[SIMULATION_GUIDE.md](SIMULATION_GUIDE.md) and [ESTIMATOR_GUIDE.md](ESTIMATOR_GUIDE.md).

- **"Unrecognized function or variable"**: run from `matlab/` and call `startup_paths()`.
  The test files add their own path; `runtests` changes folder, so they must.
- **Running headless** (`matlab -batch run_all`): figures are created invisibly and saved to
  `results/figures/`. Nothing pops up; this is expected.
- **Run time**: `run_all` takes about 10–15 min (the Monte Carlo and robustness runs dominate);
  `run_all('quick')` takes a few minutes. Each closed-loop run is about 5 s per 10 simulated
  minutes.
- **After changing parameters**: re-run `run_tests` first. If the loop-margin test fails,
  retune the poles in `design_controllers` (defaults at the bottom of the file) and check
  with `loop_analysis`.
- **`out.failed = true`**: bag pressure reached 0. This is a real failure of the design or
  the parameters (for example q_b,max too small for the breath gaps; check
  `feasibility_checks`), not a numerical glitch.
- **Rank of `ctrb`/`obsv` looks wrong**: see LOGBOOK (pole spread); use the PBH result.
