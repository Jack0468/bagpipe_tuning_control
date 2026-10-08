# Robotic Bagpipe Control Logbook

Dated entries for things worth remembering later: design decisions (and reversals),
nontrivial bugs and their root cause, anything that cost real debugging time. The
checklist of what is done lives in [BLUEPRINT.md](BLUEPRINT.md).

Newest entries at the top.

## 2026-10-08 — Full reachability matrix reports rank 2; use the PBH test

**What happened:** `rank(ctrb(A,B))` on the 9-state model returned 2, not 9.

**Why it matters / root cause:** the open-loop poles span about 0.08 to 9800 1/s (the motor
current pole). In W = [B AB … A⁸B], the last columns are dominated by (−9800)⁸ ≈ 10³², so the
computed rank collapses even after scaling. The model *is* reachable and observable: the
PBH test (rank[λI − A, B] for each eigenvalue) gives 9, and the W-matrix rank of each
decoupled block (bag, slide 1, slide 2) is 3. Worth a sentence in the report, since the
course teaches the W test.

## 2026-10-08 — Matrix-pencil sign error (estimator)

**What happened:** the first estimator run gave e_Σ errors of about 1–2 Hz.

**Why it matters / root cause:** with Y = U·S·Vᴴ, the shift-invariant basis for the tones
is conj(V), not V. Using V returned conjugated frequencies (offsets with the wrong sign)
and poor amplitudes. After the fix, the RMS error of e_Σ is about 0.02 Hz at a 5 s window.
A unit test (`testMatrixPencilResolvesCloseTones`) now guards it.

## 2026-10-08 — Tuning loop redesigned as servo + integral (no slide observer from the microphone)

**What happened:** the first slide controller, an observer-based state feedback using the
microphone, was unstable against the moving-average sensor. A second version, with a
better sensor model, was stable in its design model but became unstable with tiny
modelling differences.

**Why it matters / root cause:** the microphone cannot separate a slide position offset
from reed drift, because both shift e by a constant (observability fails for that pair).
An observer of slide position from e therefore makes no sense. Placing the slow sensor
and integral poles exactly also forced a fragile inversion of the sensor model. The working
design is a cascade:
- an inner position servo on the motor-model slide estimate;
- an outer integral on the microphone error, with its gain set by loop shaping for 55° phase
  margin against the true moving average (crossover ≈ 0.24 rad/s at T_w = 5 s);
- pressure feedforward to the slide reference.

**Follow-up:** the handover said the tuning loop is type 2 and rejects ramp drift with zero
error. That is false once the servo closes the slide's own integrator: it is type 1, and a
ramp drift r [Hz/s] leaves a steady error r/k_I (≈ 4 s × r). The handover has been corrected.

## 2026-10-08 — Continuous-time design implemented at 5 ms went unstable

**What happened:** state feedback designed in continuous time with a −200 1/s slide pole
oscillated to the travel limits when run digitally at Ts = 5 ms.

**Why it matters / root cause:** |λ|·Ts = 1 is far too large for a continuous design to carry
over to a sampled implementation. All gains and observers are now placed on ZOH-discretised
models, with s-plane poles mapped by z = e^{sTs}. The report can still quote s-plane poles.

## 2026-10-08 — Pressure loop: fast observer needed for margins

**What happened:** the first pressure design was stable but had 16° phase margin and
1.3 dB gain margin.

**Why it matters / root cause:** an observer-based controller from one sensor (pressure)
has no guaranteed margins. Making the observer faster than the controller recovers them
(similar in spirit to loop-transfer recovery). Chosen poles: controller −2, −5, −15 ± 10j;
observer −50, −120 ± 60j → about 53° and 17 dB.

## 2026-10-08 — Project scope and decisions

**What happened:** decisions taken with the students:
- both tenor drones controlled from one microphone, with the bass drone stopped;
- chanter on Low A only (just intonation, constant air use);
- random breath gaps modelled on a real player;
- target of at most one beat every 10 s, i.e. |e| ≤ 0.05 Hz;
- parameters to be measured on real pipes;
- MATLAB code written by AI with a declaration (docs/AI_USE_LOG.md).

The original single-tenor handover was deleted; the two-tenor version
(`bagpipe_tuning_project_handover_v2.md`) is the specification. The first version stays
in git history (commit a35cfef).

**Why it matters:** these choices set the model size (9 states, 4 inputs) and the sensing
problem (common/differential estimation, handover Section 13.2).
