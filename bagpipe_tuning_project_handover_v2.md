# ELEC3304 Assessment 2 — Robotic Bagpipe Pressure & Drone-Tuning Control System
## Handover specification for the implementing agent

**Course:** ELEC3304 Control, Semester 2, 2026
**Group size:** 2 students (fixed for the rest of semester)
**Prepared:** 8 October 2026 (end of Week 9 practical)
**Revised:** 8 October 2026 — review corrections: pitch models in deviation form, effective slide mass and gear ratio, observability/stability conditions on bag stiffness k, pressure-input allocation (F → p zero at s = 0), random breath gaps (modelled on an actual player) as an input constraint, tuning target (at most one beat every 10 s, i.e. |e| ≤ 0.05 Hz), notation clashes. Scope then extended to **both tenor drones (9 states, 4 inputs)** tuned from **one microphone** using a common/differential estimation and control scheme; chanter plays Low A only (just intonation, constant air use); parameters to be measured directly on real pipes.
**Purpose of this document:** a complete, self-contained brief so another agent can derive the model, compute the LTI matrices, simulate the plant, and then design and evaluate the controller for Assessment 2.

> **Important gap:** The project folder contains the Week 9 practical sheet (which defines the plant requirements) and lecture notes, but **not the Assessment 2 specification itself** (marking criteria, report format, page limits, due date). Before writing the final report, ask the students for the Assessment 2 document. Everything below about the assessment is taken from the Week 9 sheet and the lectures.

---

## Contents

1. [Assignment specification (verbatim requirements)](#1-assignment-specification)
2. [Course context: methods the controller should use](#2-course-context)
3. [Project concept and how it was chosen](#3-project-concept)
4. [System description and scope](#4-system-description-and-scope)
5. [States, inputs, outputs, disturbances](#5-states-inputs-outputs-disturbances)
6. [Nonlinear model](#6-nonlinear-model)
7. [Equilibrium](#7-equilibrium)
8. [Linearisation and LTI state-space matrices](#8-linearisation-and-lti-matrices)
9. [Structural properties to verify](#9-structural-properties-to-verify)
10. [Parameters and measurement plan](#10-parameters)
11. [Constraints and uncertainties](#11-constraints-and-uncertainties)
12. [Handling the time-varying reed drift](#12-handling-the-time-varying-reed-drift)
13. [Sensor model and one-microphone estimation scheme](#13-sensor-model)
14. [Simulation requirements](#14-simulation-requirements)
15. [Controller design guidance](#15-controller-design-guidance)
16. [Known risks and how to handle them](#16-known-risks)
17. [Task list for the implementing agent](#17-task-list)
18. [Open questions for the students / tutor](#18-open-questions)

---

## 1. Assignment specification

Source: *Practical Week 9: Modelling a System in State-Space Form*, ELEC3304, October 2026.

### Objective
Choose a suitable plant and derive a model in preparation for Assessment 2, in which the group demonstrates how to design a controller for that plant. Work continues across the remaining three practicals (Weeks 9–11) and outside class time. In Week 10 the controller design begins.

### A1 — Find a suitable plant and derive an LTI state-space model
- Any class of system is allowed (mechanical, electrical, chemical, biological, …).
- Identify **inputs, outputs, disturbances and states**.
- The plant may be nonlinear, but it **must be linearised about a suitable equilibrium** to obtain an LTI state-space model.
- For complicated systems, use academic literature for modelling.
- **The system cannot be one discussed in lectures or tutorials.** (Lecture examples include: point-mass / uncrewed vehicle with and without drag, mass–spring–damper, cart / skateboard pendulum, the third-order plant 1/(s+1)³. The bagpipe system is not among them.)

**Hard requirements (all five must be met and explicitly shown in the report):**

| # | Requirement | How this project meets it |
|---|---|---|
| 1 | At least two states | 9 states (Section 5) |
| 2 | Fewer measurements (sensors) than states | 2 sensors (bag pressure sensor, one microphone) giving 3 measured outputs (p, e₁, e₂) vs 9 states |
| 3 | Reasonable model parameters, backed by research | Parameters **measured directly on real pipes** by the students (measurement plan, Section 10.5), cross-checked against literature; remaining values cited or justified as estimates |
| 4 | LTI state-space form using matrices | A, B, C, D (and disturbance matrices E, D_d) in Section 8 |
| 5 | At least one actuator, making the system reachable | 4 actuators: blowpipe flow, bag squeeze force, two slide motor voltages; reachability rank check in Section 9 |

**Additional instruction from the sheet:** think carefully about **input constraints** and **model uncertainties** while modelling — part of Assessment 2 requires discussing their effect on controller performance.

### A2 — Simulate the plant
- Use MATLAB (Simulink optional).
- Simulate (a) the **LTI model** that the controller will be designed on, and (b) a **more realistic model** with constraints, noise and nonlinearities.
- Apply **different types of inputs** (step, ramp, sinusoid, pulse, etc.) and examine the response to each.
- Simulate with **actuator constraints and model uncertainties** to see how performance is affected.

### Tutor checkpoint (Week 9 session)
The tutor records group members, details of the chosen system, and progress.

---

## 2. Course context

Topics in the uploaded lecture notes, which the controller design should draw on so the work matches the course:

- **Lectures 1–2:** feedback vs open-loop control; disturbances, uncertainties, constraints (actuators, sensors, delays); requirements for high-gain open-loop control.
- **Lecture 4:** transfer-function plant models; stability.
- **Lecture 5:** sensitivity S and complementary sensitivity T, S + T = 1; reference tracking and disturbance rejection; PI control; **two-degree-of-freedom architecture** with prefilter F(s).
- **Lecture 6 / Bode notes:** frequency response, Bode plots.
- **Lecture 7:** **feedback design via loop shaping**; **fundamental limitations due to RHP poles and zeros**; overcoming single-loop limitations; design objectives — tracking (T ≈ 1), disturbance rejection (S ≈ 0), robustness (|S| small), noise attenuation (|T| small), limiting control effort (|T/P| small).
- **Lecture 8:** state-space models, MIMO form ẋ = Ax + Bu, y = Cx + Du, state choice, homogeneous solution e^{At}x(0), internal stability via eigenvalues of A; outlook to controller–observer structure.
- **Lecture 9:** **reachability** (W = [B AB … A^{n−1}B], full rank), **state feedback u = −Kx with pole placement**, and (following on) **state observers** for estimating x from u and y.

The expected Assessment 2 design is therefore most likely: **state feedback + observer (estimator), with integral action for tracking**, supported by frequency-domain (Bode / S and T) analysis. Confirm against the Assessment 2 spec.

---

## 3. Project concept

The students wanted a **hard** project in **robotics**, connected to their interest in **music**. Several robotics-music plants were considered (theremin-playing arm, piano finger with series elastic actuator, robotic drummer, violin bowing arm). The students chose their own idea: **automatic tuning and pressure control of a Highland bagpipe**, operated by a "robotic piper".

### How a bagpipe works (relevant physics)
- The piper blows air through a **blowpipe** (with a one-way non-return valve) into a flexible **bag** held under the arm.
- The piper **squeezes the bag** with the arm to keep the pressure steady, especially while breathing in (when no air enters through the blowpipe).
- Air leaves the bag through the **reeds**: one **chanter** reed (the melody pipe, fingered) and three **drone** reeds (two tenor drones and one bass drone) that sound continuous notes.
- **Drones are tuned** by sliding a **tuning slide** (changing the drone's sounding length). The tenor drones sound one octave below the chanter's Low A; the bass drone two octaves below.
- **Chanter tuning** is done by applying tape over finger holes — a discrete, manual adjustment. The chanter scale is tuned in **just intonation** relative to the drones, so every chanter note has partials that coincide with drone partials.
- Pitch of every reed depends on bag **pressure**. Pitch also depends on **temperature** (speed of sound in the air column). As the reeds warm up and absorb moisture during playing, **pitch drifts upward** over time.

### Design decisions already made in discussion (with reasons)
1. **Pitch is the output; tape and drone length are actuators.** The original idea listed "tape on the chanter" and "drone length" as outputs; these are things you adjust, so they are inputs. Pitch is measured.
2. **Chanter tape is dropped as an actuator.** It is a discrete manual adjustment, hard to model as a continuous input. The chanter instead provides the tuning **reference**, and is assumed to be in tune with itself in just intonation.
3. **Both tenor drone tuning slides are motorised** (DC motor + leadscrew each). This is the robotics element along with the squeezing arm.
4. **Air enters through both the blowpipe and the bag squeeze** (student requirement). These are two separate inputs acting on the same pressure — a redundant (over-actuated) input pair.
5. **Pitch itself has no dynamics** on the control time-scale (acoustic transit ~1 ms). States come from bag air storage, arm/bag mechanics, and the two slide motors.
6. **Outputs are the tuning errors** of each tenor drone (drone pitch minus half the chanter Low A pitch), not absolute drone pitches, because bag pressure moves both the chanter and the drones. Using the errors avoids a hidden feedback loop through the reference, and the references become simply e₁ = e₂ = 0.
7. **Reed drift (time variance) is modelled as a slow disturbance**, keeping the model LTI as required; the controller rejects it with integral action. See Section 12.
8. **Scope:** the bag + **both tenor drones** (9 states). The **chanter plays Low A only, with constant air use** (no fingering changes; students' decision). The **bass drone is assumed stopped** during automatic tuning (open question 4). Every reed that sounds draws air from the bag.
9. **Blowpipe flow q_b is controlled by the robot, with random breath gaps modelled on an actual player** (confirmed by the students). Gap durations and the blowing intervals between them are drawn from distributions fitted to a real piper's breathing. Breath gaps are **not** a separate disturbance: they are a random input constraint, q_b,max(t) = 0 during each gap, not known to the controller in advance (Section 11).
10. **Tuning target: at most one drone–chanter beat every 10 s** for each tenor (beat frequency ≤ 0.1 Hz), which the students chose as reflective of the real situation. The beat is 2|e| (Section 6.7), so this is **|e₁|, |e₂| ≤ 0.05 Hz** on the drone-fundamental scale, ≈ 0.36 cents at f̄_d ≈ 240 Hz.
11. **Pitch sensing is a single microphone listening continuously** (confirmed by the students), used to tune **both** tenor drones. Extracting e₁ and e₂ from the combined sound is the project's main engineering challenge; the proposed approach is a **common/differential scheme** (Section 13.2).
12. **Parameters are measured directly** on real pipes by the students (requirement 3; measurement plan in Section 10.5).

### Framing for the report
The pipes are "played" by a robotic system: a blower/blowpipe flow source and a robotic arm squeezing the bag, plus two motorised tuning slides. Application: automatic drone tuning during long performances and as a practice aid. Note: real pipers tune before playing and only touch up between tunes, so the report should justify continuous tuning. Check with the tutor that this counts as robotics if that matters to the group.

The mechanics are deliberately simple — each slide is essentially a mass on a leadscrew, and the arm a mass–spring–damper. The difficulty and novelty lie in **estimating two tuning errors from one microphone** and in the over-actuated pressure control with random breath gaps; lead with these in the report.

---

## 4. System description and scope

```
                 q_b (blowpipe, one-way)
                       │
                       ▼
     F (arm) ──▶  [   BAG   ]   air mass m, volume V(z), pressure p
                       │
        ┌──────────────┼───────────────────┐
        ▼              ▼                   ▼
   Chanter reed   Tenor drone 1 reed   Tenor drone 2 reed
   (Low A only,   (flow ∝ √p)          (flow ∝ √p)
    flow ∝ √p)     │ slide s₁ ◀ V_m1    │ slide s₂ ◀ V_m2
        │          ▼                   ▼
   pitch f_c      pitch f₁            pitch f₂
        └──────────────┴─────────┬─────────┘
                                 ▼
               one microphone → e₁ = f₁ − ½f_c,  e₂ = f₂ − ½f_c
                       (via e_Σ and e_Δ, Section 13)
```

**In scope:** bag air mass, squeezing arm and bag wall (mass–spring–damper), two tenor drones each with a motorised leadscrew slide, chanter playing Low A as a pressure-dependent pitch source (algebraic, no states), outflow through all sounding reeds, one-microphone estimation of both tuning errors.

**Out of scope (possible extensions):** bass drone (assumed stopped; tuning it adds 3 slide states and it would also sound in every tenor analysis band, Section 13.2), chanter melody (other notes) and tape, reed acoustic dynamics (kHz, far faster than the control bandwidth), bag thermodynamics beyond an isothermal ideal-gas assumption, moisture.

**Modelling assumptions (state these in the report):**
- Air in the bag is an ideal gas at uniform temperature T; isothermal on the control time-scale.
- Bag volume changes only through arm squeeze displacement z: V = V₀ − A_c·z (A_c = effective arm–bag contact area).
- **Bag gauge pressure is strictly positive for all time (p > 0).** In normal operation p_min ≤ p ≤ p_max with p_min > 0, so √p in the outflow law is always defined.
- Reed outflow is orifice-like: mass flow ∝ √(gauge pressure).
- Chanter plays Low A only: its air use is constant at the operating pressure (no fingering changes) and still follows the reed flow law if pressure changes.
- Arm + bag wall behaves as a linear mass–spring–damper driven by the arm force and loaded by the bag pressure, with stiffness k > 0.
- Each tenor drone is a cylindrical pipe closed at the reed end: quarter-wave resonator.
- Pitch depends instantaneously (algebraically) on pressure, slide position and temperature.
- The two slide drives use the same motor and leadscrew (identical hardware); slides move horizontally (no gravity load); friction is viscous; each motor drives its leadscrew directly or through a gearbox of ratio N (ideal, no backlash).

---

## 5. States, inputs, outputs, disturbances

**Notation.** T is temperature throughout (T̄ nominal; T_d1, T_d2, T_c the two drone and chanter air columns). Where the course's complementary sensitivity or prefilter appear (Sections 2, 15), they are written with their Laplace argument, T(s) and F(s). A (no subscript) is the state matrix only; the arm–bag contact area is A_c. Arm/bag damping is b_a; c(T) is the speed of sound. F is the arm force input; the output disturbance matrix is D_d. Subscript j = 1, 2 indexes the tenor drones (i is motor current). Subscripts Σ and Δ denote the common mode (average) and differential mode (difference) of the two drones (Section 8.6). The drone end correction is L_e.

### State vector (n = 9)
x = [m, z, ż, s₁, ṡ₁, i₁, s₂, ṡ₂, i₂]ᵀ

| State | Symbol | Units | Meaning | Why it is a state |
|---|---|---|---|---|
| x₁ | m | kg | Mass of air in the bag | Stores mass; conservation law ṁ = in − out |
| x₂ | z | m | Arm squeeze displacement into the bag | Arm/bag-wall position (2nd-order mechanics) |
| x₃ | ż | m/s | Squeeze velocity | Arm/bag-wall momentum |
| x₄ | s₁ | m | Tenor drone 1 slide extension from reference position | Slide position (integrates velocity) |
| x₅ | ṡ₁ | m/s | Slide 1 velocity | Slide momentum |
| x₆ | i₁ | A | Slide 1 motor current | Motor inductance stores energy |
| x₇ | s₂ | m | Tenor drone 2 slide extension | as x₄ |
| x₈ | ṡ₂ | m/s | Slide 2 velocity | as x₅ |
| x₉ | i₂ | A | Slide 2 motor current | as x₆ |

Air mass is chosen over pressure because it satisfies a simple conservation law; pressure then follows algebraically from mass and volume, so **pressure is an output, not a state.**

### Input vector (4 inputs)
u = [q_b, F, V_m1, V_m2]ᵀ

| Input | Symbol | Units | Meaning | Constraint |
|---|---|---|---|---|
| u₁ | q_b | kg/s | Air mass flow in through blowpipe (manipulated) | 0 ≤ q_b ≤ q_b,max(t) (one-way valve; q_b,max(t) = 0 during each random breath gap, Section 11) |
| u₂ | F | N | Squeeze force applied by arm to bag | 0 ≤ F ≤ F_max (can only push) |
| u₃ | V_m1 | V | Slide 1 motor voltage | \|V_m1\| ≤ V_max |
| u₄ | V_m2 | V | Slide 2 motor voltage | \|V_m2\| ≤ V_max |

### Output vector (3 outputs)
y = [p, e₁, e₂]ᵀ

| Output | Symbol | Units | Sensor |
|---|---|---|---|
| y₁ | p | Pa (gauge) | Bag pressure sensor |
| y₂ | e₁ | Hz (target \|e₁\| ≤ 0.05 Hz) | Microphone + estimator (Section 13): tenor 1 pitch minus half the chanter Low A pitch |
| y₃ | e₂ | Hz (target \|e₂\| ≤ 0.05 Hz) | Same microphone + estimator: tenor 2 pitch minus half the chanter Low A pitch |

The estimator actually delivers the common and differential errors e_Σ = ½(e₁ + e₂) and e_Δ = e₁ − e₂ (Section 13.2), an invertible transform of (e₁, e₂).

2 sensors, 3 measured outputs < 9 states ✔ (requirement 2).

### Disturbances
d = [δT_bag, δT_d1, δT_d2, δT_c, δ_reed]ᵀ

| Disturbance | Symbol | Units | Effect | Time-scale |
|---|---|---|---|---|
| Bag air temperature | T_bag | K | Changes pressure (gas law) | minutes |
| Drone air-column temperatures | T_d1, T_d2 | K | Change each drone's pitch (speed of sound); drones can warm at different rates | minutes |
| Chanter air-column temperature | T_c | K | Changes chanter pitch | minutes |
| Chanter reed pitch drift | δ_reed | Hz | Chanter pitch creeps upward as the reed warms/wets | minutes (ramp-like) |

Optional: similar drift terms on each drone reed (δ_d1, δ_d2) if the measurements show drone-reed drift.

Breath gaps are **not** in this table: q_b is a manipulated input, and a breath gap is a random input constraint (q_b,max(t) = 0 for a random duration t_gap, separated by random blowing intervals t_blow, both modelled on an actual player; Section 11). There is no chanter-demand disturbance, because the chanter plays Low A only with constant air use.

---

## 6. Nonlinear model

### Constants and parameters
R = 287 J/(kg·K) (specific gas constant of air), p_atm = 101 325 Pa. Other symbols are defined in Section 10.

### 6.1 Bag pressure (algebraic)
Bag volume: V(z) = V₀ − A_c·z

Gauge pressure (ideal gas):

  p(m, z, T_bag) = m·R·T_bag / (V₀ − A_c·z) − p_atm

### 6.2 Air mass balance

  ṁ = q_b − Q_out(p)

  Q_out(p) = (K_d + K_c)·√p

K_d: **total** flow coefficient of the sounding drone reeds (K_d = K_d1 + K_d2 for the two tenors; add the bass reed if it sounds); K_c: chanter reed flow coefficient (units kg·s⁻¹·Pa^−½). The chanter plays Low A only, so there is no fingering term. √p is defined because p > 0 for all time (Section 4 assumption); the nonlinear simulation must check this assumption rather than clamp p (Section 14).

### 6.3 Arm and bag wall (mass–spring–damper loaded by pressure)

  M·z̈ = F − k·z − b_a·ż − A_c·p(m, z, T_bag)

M: effective moving mass of arm + bag wall; k: bag/arm stiffness (k > 0 — observability of the bag states from pressure and the stability margin of the bag block both depend on it, Section 9); b_a: arm/bag damping. The term A_c·p is the bag pushing back on the arm.

### 6.4 Tuning slides (leadscrew), j = 1, 2

  m_eff·s̈_j = G·K_t·i_j − b_s·ṡ_j,  with G = 2π·N/ℓ

ℓ: leadscrew lead (m/rev); N: gearbox ratio between motor and leadscrew (N = 1 for direct drive); K_t: motor torque constant; b_s: total viscous friction at the slide, including motor friction reflected through the drive (b_s = b_slide + b_m·G²).

m_eff: effective moving mass at the slide, **including reflected rotor inertia**:

  m_eff = m_s + J_m·G²   (add gearbox inertia if significant)

The rotor term usually dominates by orders of magnitude: with ℓ = 1 mm, N = 1 and J_m = 10⁻⁷ kg·m², J_m·G² ≈ 3.9 kg, compared with a bare slide mass m_s ≈ 0.02–0.1 kg. **Always use m_eff in the model, never m_s alone.** Both slides use the same hardware; if the measured values differ, give each slide its own parameters (the common/differential decoupling of Section 8.6 then becomes approximate).

### 6.5 Slide motors (armature circuit), j = 1, 2

  L·di_j/dt = V_mj − R_a·i_j − K_e·G·ṡ_j

R_a: armature resistance; L: armature inductance; K_e: back-EMF constant (K_e = K_t in SI units). G includes the gear ratio N, so G·ṡ_j is the motor shaft speed.

### 6.6 Pitch models (algebraic, deviation form)
Speed of sound: c(T) = c₀·√(T / 273.15), c₀ ≈ 331.3 m/s

Reed pressure sensitivity is written about the operating pressure p̄, so that the constant terms are the **playing pitches at the operating point** (which is what the measurements give).

Tenor drone j (quarter-wave, closed at reed end) plus reed pressure sensitivity:

  f_j = c(T_dj) / [4·(L_dj + s_j + L_e)] + β_dj·(p − p̄)

L_dj: sounding length of drone j at s_j = 0; L_e: end correction (≈ 0.6 × bore radius for an unflanged open end); β_dj: local slope of drone j pitch with pressure at p̄ (Hz/Pa). The two tenor reeds are never identical, so β_d1 ≠ β_d2 in general.

Chanter Low A:

  f_c = f_c0·c(T_c)/c(T̄) + β_c·(p − p̄) + δ_reed

f_c0: chanter Low A **at the operating point** (p = p̄, T_c = T̄, δ_reed = 0), i.e. the measured playing pitch; β_c: local slope of chanter pitch with pressure at p̄ (Hz/Pa).

### 6.7 Outputs

  y₁ = p(m, z, T_bag)
  y₂ = e₁ = f₁ − ½·f_c   (Hz)
  y₃ = e₂ = f₂ − ½·f_c   (Hz)

Common and differential errors (what the estimator measures, Section 13.2):

  e_Σ = ½(e₁ + e₂)  (the drone pair against the chanter)
  e_Δ = e₁ − e₂ = f₁ − f₂  (tenor against tenor)

**Tuning target: at most one beat every 10 s for each tenor against the chanter.** Each drone's 2nd harmonic and the chanter's Low A are 2|e_j| apart, so the beat frequency is |2f_j − f_c| = 2|e_j|. The target is therefore

  **|e₁| ≤ 0.05 Hz and |e₂| ≤ 0.05 Hz**

which also keeps the tenors within |e_Δ| ≤ 0.1 Hz of each other — at most one tenor–tenor beat every 10 s at their fundamental.

e is kept in Hz throughout because the target is specified in Hz. At f̄_d ≈ 235–240 Hz, 0.05 Hz ≈ 0.36 cents (e_cents = 1200·log₂( f_j / (½ f_c) ), for reporting only).

Note: the target is defined on the lowest coinciding partials. Higher coinciding partials beat proportionally faster (Section 13.2) — check with the recordings which beats are actually most audible.

### 6.8 Full nonlinear system (compact form)
ẋ₁ = q_b − (K_d + K_c)·√p(x₁, x₂, T_bag)
ẋ₂ = x₃
ẋ₃ = [F − k·x₂ − b_a·x₃ − A_c·p(x₁, x₂, T_bag)] / M
ẋ₄ = x₅
ẋ₅ = [G·K_t·x₆ − b_s·x₅] / m_eff
ẋ₆ = [V_m1 − R_a·x₆ − K_e·G·x₅] / L
ẋ₇ = x₈
ẋ₈ = [G·K_t·x₉ − b_s·x₈] / m_eff
ẋ₉ = [V_m2 − R_a·x₉ − K_e·G·x₈] / L

with p(x₁, x₂, T_bag) > 0 for all t.

Note the **block structure**: the bag subsystem (x₁–x₃) and the two slide subsystems (x₄–x₆, x₇–x₉) are dynamically decoupled; they couple only through the tuning-error outputs, where pressure affects all pitches.

---

## 7. Equilibrium

Choose operating values (from the measurements, Section 10):
- p̄: steady playing pressure (gauge)
- z̄: nominal squeeze displacement, leaving enough travel to cover a breath gap (z̄ + Δz_gap ≤ z_max, see below) and room to release
- T̄: nominal air temperature; δ̄_reed = 0

Then:

| Quantity | Equilibrium value | From |
|---|---|---|
| V̄ | V₀ − A_c·z̄ | volume definition |
| m̄ | (p̄ + p_atm)·V̄ / (R·T̄) | gas law |
| q̄_b | (K_d + K_c)·√p̄ | ṁ = 0 (blowpipe supplies all outflow; this is the **long-run mean** flow, averaged over the random breath gaps) |
| F̄ | k·z̄ + A_c·p̄ | z̈ = 0, ż = 0 |
| s̄_j | solves e_j = 0: c(T̄)/[4(L_dj + s̄_j + L_e)] = ½·f_c0 | tuning condition for each drone (pressure terms vanish at p̄ in deviation form) |
| ṡ̄_j, ī_j, V̄_mj | 0, 0, 0 | slides at rest, no static load |
| f̄_d | ½·f_c0 (both tenors in tune) | |

Choose L_dj so that each s̄_j sits near the middle of its slide travel.

Note that in steady state **only the blowpipe can supply air**; squeeze can only change pressure transiently (z is bounded, and the F → p channel has a zero at s = 0, Section 9.4).

**Feasibility checks (with margin), including random breath gaps.** Breath gaps are random (Section 11): gap durations t_gap and blowing intervals t_blow (from the end of one gap to the start of the next) are drawn from distributions fitted to an actual player. Use bounded distributions so that worst cases exist: longest gap t_gap,max and shortest blowing interval t_blow,min (e.g. 99th / 1st percentiles of the player data). Mean gap fraction: φ̄ = E[t_gap] / (E[t_gap] + E[t_blow]).
1. **Mean flow (necessary):** q_b is zero for a fraction φ̄ of the time on average, so while blowing it must average q̄_b / (1 − φ̄). Require q_b,max ≥ q̄_b / (1 − φ̄), plus margin.
2. **Refill before the next gap (worst case):** after a gap the bag is short by ≈ q̄_b·t_gap of air, which the blowpipe refills at up to (q_b,max − q̄_b) net. Worst case is the longest gap followed by the shortest interval:
   q̄_b·t_gap,max / (q_b,max − q̄_b) ≤ t_blow,min.
   This is stronger than check 1. If it fails, the air deficit (and the squeeze position) can ratchet over several close breaths — show in the Monte Carlo runs (Section 14) that z stays within travel.
3. **Squeeze travel over the longest gap:** to hold p ≈ p̄ while q_b = 0, the arm must shrink the bag by the volume that flows out. With volume outflow at bag conditions Q̄_v = q̄_b·R·T̄ / (p̄ + p_atm):
   Δz_gap ≈ Q̄_v·t_gap,max / A_c,  and require z̄ + Δz_gap ≤ z_max.
4. **Squeeze force over the longest gap:** F at the end of the gap ≈ k·(z̄ + Δz_gap) + A_c·p̄ ≤ F_max, and F̄ > 0.

Δz_gap is a large-signal excursion — confirm these checks in the nonlinear model, not only the LTI one.

---

## 8. Linearisation and LTI matrices

Deviation variables: δx = x − x̄, δu = u − ū, δd = d − d̄, δy = y − ȳ.

LTI model:

  δẋ = A·δx + B·δu + E·δd
  δy = C·δx + D·δu + D_d·δd

with δd = [δT_bag, δT_d1, δT_d2, δT_c, δ_reed]ᵀ.

### 8.1 Partial derivatives of pressure (evaluated at equilibrium)

  a_m = ∂p/∂m = R·T̄ / V̄
  a_z = ∂p/∂z = (p̄ + p_atm)·A_c / V̄
  a_T = ∂p/∂T_bag = (p̄ + p_atm) / T̄

### 8.2 Outflow derivative

  g = ∂Q_out/∂p = (K_d + K_c) / (2√p̄)

### 8.3 State and input matrices

Block form (9 states, 4 inputs):

```
      ⎡ A_bag    0      0   ⎤            ⎡ b_q   b_F    0     0  ⎤
A  =  ⎢   0     A_s     0   ⎥      B  =  ⎢  0     0    b_V    0  ⎥
      ⎣   0      0     A_s  ⎦            ⎣  0     0     0    b_V ⎦

          ⎡ −g·a_m        −g·a_z           0     ⎤
A_bag  =  ⎢   0              0             1     ⎥
          ⎣ −A_c·a_m/M  −(k + A_c·a_z)/M  −b_a/M  ⎦

          ⎡ 0       1              0        ⎤
A_s    =  ⎢ 0   −b_s/m_eff    G·K_t/m_eff   ⎥
          ⎣ 0   −K_e·G/L        −R_a/L      ⎦

b_q = [1, 0, 0]ᵀ,  b_F = [0, 0, 1/M]ᵀ,  b_V = [0, 0, 1/L]ᵀ
```

(If the two slides' measured parameters differ, use A_s1, A_s2.)

Physical notes:
- −g·a_m: more air → higher pressure → more outflow (self-regulating; stable pole).
- −(k + A_c·a_z)/M: squeezing raises pressure, which pushes back, so the trapped air acts as an extra spring of stiffness A_c·a_z.
- **Bag block:** characteristic polynomial (×M)
  M·s³ + (M·g·a_m + b_a)·s² + (g·a_m·b_a + k + A_c·a_z)·s + g·a_m·k,
  so det(A_bag) = −g·a_m·k / M. For k > 0 all three poles are in the LHP (Routh–Hurwitz holds for any positive parameters), but one pole moves toward 0 as k → 0. Physical reason: with k = 0, "more air + more squeeze" in the ratio that leaves p unchanged (δm : δz = a_z : −a_m) is a stationary mode — nothing pushes it back.
- Each slide has no restoring force (leadscrew), so each slide block has a **pole at 0** (integrator from velocity to position). These free integrators matter for disturbance rejection (Section 12).

### 8.4 Output matrices

Define, for j = 1, 2:
  β_ej = β_dj − ½·β_c   (net pressure sensitivity of tuning error e_j)
  h_sj = c(T̄) / [4·(L_dj + s̄_j + L_e)²] = f̄_d / (L_dj + s̄_j + L_e)   (drone j pitch sensitivity to slide extension, Hz/m)

```
      ⎡ a_m          a_z        0  │   0     0   0  │   0     0   0 ⎤
C  =  ⎢ β_e1·a_m    β_e1·a_z    0  │ −h_s1   0   0  │   0     0   0 ⎥
      ⎣ β_e2·a_m    β_e2·a_z    0  │   0     0   0  │ −h_s2   0   0 ⎦

D  =  0 (3×4)
```

e₁, e₂ stay in Hz (target 0.05 Hz each). For reporting in cents, multiply rows 2–3 of C and D_d by (1200/ln 2)/f̄_d.

**What the target |e_j| ≤ 0.05 Hz means for the hardware** (illustrative numbers with placeholder parameters — recompute with the measured values):
- **Slide resolution:** with c(298 K) ≈ 346 m/s and f̄_d ≈ 240 Hz, L_dj + s̄_j + L_e ≈ 0.36 m and h_s ≈ 0.67 Hz/mm, so 0.05 Hz ≈ **0.075 mm** of slide travel. Leadscrew backlash and positioning resolution must be well below this on both slides.
- **Pressure budget:** a pressure deviation δp shifts e_j by β_ej·δp. Without slide correction, staying within 0.05 Hz needs |δp| ≤ 0.05 / |β_ej| (e.g. β_e = 0.005 Hz/Pa would allow only 10 Pa). Unless β_e turns out small, pressure dips during breath gaps must be compensated by the slides — feedforward from the measured p is natural, and is needed because the microphone measurement is slow (Sections 13, 15.2).

### 8.5 Disturbance matrices

Columns δT_bag, δT_d1, δT_d2, δT_c, δ_reed:

```
E  (9×5):  row 1 = [ −g·a_T,      0, 0, 0, 0 ]
           row 3 = [ −A_c·a_T/M,  0, 0, 0, 0 ]
           all other rows zero

       ⎡ a_T          0             0              0          0  ⎤
D_d =  ⎢ β_e1·a_T   f̄_d/(2T̄)       0          −f̄_d/(2T̄)     −½  ⎥
       ⎣ β_e2·a_T     0          f̄_d/(2T̄)     −f̄_d/(2T̄)     −½  ⎦
```

A uniform temperature change (all air columns together) **cancels exactly** in each e_j, because f̄_d = ½·f_c0 at the equilibrium. What matters are temperature **differences**: f̄_d/(2T̄) ≈ 0.4 Hz/K, so a drone–chanter difference of only ≈ **0.12 K** uses the whole 0.05 Hz budget, and a tenor–tenor difference of 0.25 K uses the whole 0.1 Hz e_Δ budget. Chanter drift δ_reed enters both errors equally. Discuss this in the report.

### 8.6 Common/differential coordinates (two SISO slide problems)
Because both slides use the same hardware, transform the drone states, inputs and errors into common (Σ) and differential (Δ) modes:

  s_Σ = ½(s₁ + s₂),  s_Δ = s₁ − s₂   (likewise for ṡ and i)
  V_Σ = ½(V_m1 + V_m2),  V_Δ = V_m1 − V_m2
  e_Σ = ½(e₁ + e₂),  e_Δ = e₁ − e₂

Inverse: s₁ = s_Σ + ½s_Δ, s₂ = s_Σ − ½s_Δ (likewise for the voltages and errors).

The slide dynamics are then **exactly decoupled** and identical: ẋ_Σ = A_s·x_Σ + b_V·V_Σ and ẋ_Δ = A_s·x_Δ + b_V·V_Δ. With h_s1 = h_s2 = h_s, the outputs are

  δe_Σ = β̄_e·δp − h_s·δs_Σ + (f̄_d/2T̄)·[½(δT_d1 + δT_d2) − δT_c] − ½·δ_reed,   β̄_e = ½(β_e1 + β_e2)
  δe_Δ = (β_d1 − β_d2)·δp − h_s·δs_Δ + (f̄_d/2T̄)·(δT_d1 − δT_d2)

where δp = a_m·δm + a_z·δz + a_T·δT_bag. So:
- **e_Σ** carries the pressure coupling, the chanter drift and the drone–chanter temperature difference — this is where the pressure feedforward belongs.
- **e_Δ** is **immune to chanter drift and chanter temperature**, and sees pressure only through the reed mismatch β_d1 − β_d2 (small if the reeds are matched — measure it). It is driven only by differences between the two drones.

This turns the two-drone problem into two SISO loops (common and differential), matching how the microphone estimator measures them (Section 13.2). Input limits couple in these coordinates: |V_Σ ± ½V_Δ| ≤ V_max. If the measured slide or drone parameters differ between the two drones, the decoupling is approximate — include the mismatch in the robustness tests.

### 8.7 Scaling (do this before rank checks and pole placement)
Raw magnitudes differ enormously (m ~ 10⁻² kg, z ~ 10⁻² m, p ~ 10³ Pa, flows ~ 10⁻⁴ kg/s, e ~ 0.1–1 Hz). Scale each variable by its expected maximum deviation (x = S_x·x̃, u = S_u·ũ, d = S_d·d̃, y = S_y·ỹ) so all scaled variables are O(1):
  Ã = S_x⁻¹·A·S_x, B̃ = S_x⁻¹·B·S_u, Ẽ = S_x⁻¹·E·S_d,
  C̃ = S_y⁻¹·C·S_x, D̃ = S_y⁻¹·D·S_u, D̃_d = S_y⁻¹·D_d·S_d.
A natural output scale for each e_j is the target itself (0.05 Hz), so |ẽ_j| ≤ 1 means "within target". Scaling does not change rank in exact arithmetic; it is needed for numerical conditioning. Use the scaled model for rank tests, pole placement and observer design; report physical units.

### 8.8 MATLAB implementation note
Implement the nonlinear f(x,u,d) and h(x,u,d) symbolically (`syms`) and compute A, B, C, D, E, D_d with `jacobian`, then substitute equilibrium values. **Cross-check against the hand-derived matrices above.** Also verify numerically by finite differences on the nonlinear function.

---

## 9. Structural properties to verify

### 9.1 Reachability (requirement 5)
W = [B, AB, A²B, …, A⁸B]; require rank(W) = 9 (`rank(ctrb(A,B))`). *Found during implementation:* numerically this returns rank 2, because the poles span about 0.08 to 9800 1/s and A⁸B is swamped by the motor-current pole. Use the equivalent PBH test (rank[λI − A, B] = 9 for every eigenvalue λ), which gives 9, and the W rank of each decoupled block (3 each). Explain this in the report.
Expected: full rank. q_b drives m, F drives ż (hence z and, via pressure, m), V_mj drives i_j → ṡ_j → s_j.
Per-input reachability is meaningful **for the bag block (x₁–x₃) only**: check it from q_b alone and from F alone — useful for the input-allocation discussion. By the block structure, each slide block is reachable only from its own V_mj, and the full system is not reachable from any single input.

### 9.2 Observability (needed for observer design)
O = [C; CA; …; CA⁸]; require rank(O) = 9 (`rank(obsv(A,C))`).
- Slide blocks: slide j is observable through e_j (s_j is measured; ṡ_j and i_j follow through the chain). Equivalently, the common and differential slide modes are observable through e_Σ and e_Δ.
- Bag block from pressure alone (C_bag = [a_m, a_z, 0]):

  det(O_bag) = a_m·a_z²·k / M

  So the bag block is observable from p **if and only if k > 0**. The pressure sensor measures a_m·δm + a_z·δz and cannot by itself separate "more air" from "squeezed harder"; only the bag/arm stiffness k makes the two evolve differently. If k is small compared with the air-spring stiffness A_c·a_z, the bag block is **weakly observable** — check the conditioning (singular values) of the scaled observability matrix, not just its rank. If it is poorly conditioned: add a squeeze position sensor (arm encoder — realistic for a robot arm) and keep measurements < states (4 < 9).
- Check observability from each measured output alone too.

### 9.3 Stability of the open-loop plant
Compute eig(A). Expected: bag block stable for k > 0 (three LHP poles; product of poles = −g·a_m·k/M, so one pole approaches 0 as k → 0); each slide block one pole at 0 plus two stable motor/slide poles. Discuss internal stability (Lecture 8).

### 9.4 Transfer functions and zeros
Compute the 3×4 transfer matrix (`tf(ss(A,B,C,D))`), Bode plots of each channel, and zeros. The bag-block channels to pressure are (with den(s) = (M·s² + b_a·s + k)(s + g·a_m) + A_c·a_z·s):

  p/q_b = a_m·(M·s² + b_a·s + k) / den(s)   (DC gain 1/g; zeros at the arm's own resonance, M·s² + b_a·s + k = 0)
  p/F   = a_z·s / den(s)   (**zero at s = 0**)

Each slide channel is

  e_j/V_mj = −h_s·G·K_t / { s·[(m_eff·s + b_s)(L·s + R_a) + K_e·K_t·G²] }

(and identically for e_Σ/V_Σ and e_Δ/V_Δ).

**Squeeze cannot change steady-state pressure** — the F → p channel (and F → e_j, through β_ej) has a zero at the origin. Consequences: any pressure integral action must ultimately act through q_b (Section 15.2), and an integrator on p driven through F alone makes the augmented system unreachable. `tzero` on the full non-square 3×4 system will generally not show this; check the SISO channels individually (`zero(tf(...))`). Also check for any RHP zeros (Lecture 7 limitations). Note the pressure→tuning-error coupling (β_ej) — if significant, bag-pressure disturbances appear directly in the tuning errors (see the pressure budget in Section 8.4).

---

## 10. Parameters

**The students will measure the instrument parameters directly on real pipes** (requirement 3; plan in Section 10.5). Literature values serve as cross-checks; any value not measured must be cited or justified as an estimate. Values below marked "PLACEHOLDER" are order-of-magnitude starting points only, to be replaced by measurements.

### 10.1 Physical constants (reliable)
| Symbol | Value | Notes |
|---|---|---|
| R | 287 J/(kg·K) | specific gas constant, dry air (breath is humid and CO₂-rich, which shifts R slightly; a blower supplying ambient air matches the dry-air value better) |
| p_atm | 101 325 Pa | |
| c₀ | 331.3 m/s at 273.15 K | c(T) = c₀√(T/273.15); dry air. Humidity raises c slightly and CO₂ lowers it — this mainly affects the L_dj estimates; it largely cancels in e_j because drones and chanter share the same air |
| T̄ | ~298–308 K | warm air from breath; measure |

### 10.2 Instrument and acoustics — measure
| Symbol | Meaning | Guidance |
|---|---|---|
| f_c0 | Chanter Low A **at the operating point** (p = p̄, T̄) — the measured playing pitch | Measure (Section 10.5). Modern Highland pipe chanters are pitched well above concert A (often quoted around 470–480 Hz) — use as a cross-check. |
| f̄_d | Tenor drone frequency | One octave below chanter Low A: f̄_d = ½ f_c0 at equilibrium |
| L_d1, L_d2 | Effective sounding length of each tenor | Choose so each s̄_j is mid-travel: L_dj + s̄_j + L_e = c(T̄)/(4f̄_d); check against measured pitch vs slide position |
| L_e | End correction | ≈ 0.6 × bore radius (unflanged pipe); measure the tenor bore |
| h_s | Pitch sensitivity to slide extension (Hz/m) | **Measure directly** (pitch vs slide position), compare with f̄_d/(L_dj + s̄_j + L_e) |
| β_d1, β_d2, β_c | Local slope of each reed's pitch with pressure at p̄ (Hz/Pa) | **Key parameters — measure directly** (Section 10.5). β_ej = β_dj − ½β_c sets the pressure budget (Section 8.4); β_d1 − β_d2 sets how much pressure leaks into e_Δ (Section 8.6). |
| p̄ | Playing pressure (gauge) | **Measure** while playing (manometer or pressure sensor). A figure of 3–4 kPa was mentioned in discussion without a source; Highland pipes may play at noticeably higher pressure. |
| p_min, p_max | Reed sounding window | Measure: below p_min reeds stop; above p_max reeds choke/"double". Needed for constraints. p_min > 0. |
| K_d1, K_d2, K_c | Reed flow coefficients | Measure air consumption at p̄ with different reeds sounding (Section 10.5); K_d = K_d1 + K_d2 |
| V₀ | Bag volume | Measure |
| A_c | Effective arm–bag contact area | Measure / estimate from the arm-pad contact patch |

### 10.3 Robotic arm / bag wall — measure or estimate and justify
| Symbol | Meaning | Guidance |
|---|---|---|
| M | Effective moving mass (arm + bag wall) | PLACEHOLDER ~0.5–2 kg; depends on actuator design |
| k | Bag wall + arm stiffness | **Measure** force vs displacement with the bag **vented** (no trapped air), so the air spring A_c·a_z is excluded. **Must be > 0 and known with care:** observability of the bag states from pressure and the slowest bag pole both scale with k (Section 9). |
| b_a | Arm/bag damping | Choose for a plausible damping ratio (e.g. 0.3–0.7) or identify from a step test; justify |
| F_max | Max squeeze force | From actuator choice; must cover k·(z̄ + Δz_gap) + A_c·p̄ with margin (Section 7) |
| z range | Max squeeze travel | Bag geometry; must cover z̄ + Δz_gap (Section 7) |
| q_b,max | Max blowpipe/blower flow | Blower spec; must pass the mean-flow and refill checks (Section 7) with margin |
| t_gap, t_blow | Breath-gap duration and blowing interval between gaps — **random** | Fit distributions to an actual player (Section 10.5). Use bounded distributions (e.g. truncated normal or lognormal) and report mean, spread, t_gap,max and t_blow,min. Until data is collected, t_gap ≈ 1–2 s is a starting estimate. φ̄ = E[t_gap]/(E[t_gap] + E[t_blow]) |

### 10.4 Slide actuators (two identical units) — use a real datasheet
Pick a specific small DC motor + leadscrew (e.g. a miniature brushed DC gearmotor or linear actuator datasheet) and take values directly from it:
| Symbol | Meaning | Typical order (PLACEHOLDER) |
|---|---|---|
| R_a | Armature resistance | 1–10 Ω |
| L | Armature inductance | 0.1–2 mH |
| K_t = K_e | Torque / back-EMF constant | 0.005–0.03 N·m/A |
| J_m | Rotor inertia | 10⁻⁷–10⁻⁶ kg·m² |
| N | Gearbox ratio (motor turns per leadscrew turn) | 1 for direct drive; from datasheet for a gearmotor |
| ℓ | Leadscrew lead | 0.5–2 mm/rev |
| m_s | Bare slide + drone section mass | ~0.02–0.1 kg |
| m_eff | Effective mass used in the model: m_s + J_m·G², G = 2πN/ℓ | Dominated by rotor inertia: J_m·G² ≈ 1–160 kg for direct drive across the ranges above (≈ 3.9 kg for ℓ = 1 mm, J_m = 10⁻⁷ kg·m²); a gearbox multiplies it by N² |
| b_s | Total viscous friction at the slide (incl. motor friction × G²) | estimate; leadscrews often have significant friction |
| V_max | Supply voltage | from datasheet |
| s range | Slide travel | real tuning slides move a few cm |
| — | Backlash / positioning resolution | must be well below ≈ 0.075 mm (the slide motion equal to 0.05 Hz, Section 8.4) |

### 10.5 Measurement plan (real pipes)
Record pressure (sensor or manometer at the bag), audio (the same microphone used by the controller) and slide positions (callipers or marked slides) together. Suggested measurements:
1. **Operating point:** steady playing — p̄, f_c0, f̄_d, T̄.
2. **Sounding window:** slowly lower and raise pressure until reeds stop / choke — p_min, p_max.
3. **Pressure sensitivities β_c, β_d1, β_d2:** vary pressure slowly around p̄ and record pitch. Measure the chanter alone and **each tenor alone** (other reeds stopped), so the two drone values are separate.
4. **Slide sensitivity h_s and L_dj:** move each slide in known steps and record its pitch; check against the quarter-wave model.
5. **Air consumption K_c, K_d1, K_d2:** measure flow at p̄ with different combinations of reeds sounding (e.g. chanter only, chanter + one tenor, chanter + both), and split the total.
6. **Bag:** volume V₀; stiffness k from force vs displacement with the bag vented; contact area A_c.
7. **Breath gaps:** record an actual player (pressure trace or audio of the blowpipe valve); time each gap and blowing interval; fit the distributions.
8. **Temperatures and drift (optional but valuable):** long recording (15–30 min) from a cold start — drift rate of each pitch, and (with thermocouples) drone/chanter temperature differences.
9. **Spectra for the estimator (Section 13.2):** recordings of chanter + both tenors, including runs with the tenors deliberately offset by known slide amounts. Needed to find which partials are strong, which source is louder in each analysis band (used for the sign of the error), the noise floor, and to test the estimator on real sound.

---

## 11. Constraints and uncertainties

### Input constraints (must be simulated and discussed)
- **0 ≤ q_b ≤ q_b,max(t)** (non-return valve: the blowpipe can never remove air). **Breath gaps** are a random, time-varying limit: q_b,max(t) = 0 for a random duration t_gap, separated by random blowing intervals t_blow, with distributions fitted to an actual player (Section 10). Gaps are **not known in advance**; the controller knows a gap has started (and ended) only when it happens, because it is the limit on its own actuator. The controller must respect it, and q_b,max must pass the mean-flow and worst-case refill checks (Section 7).
- **0 ≤ F ≤ F_max**: arm can push but not pull.
- **Squeeze travel limit** z ∈ [0, z_max]: an empty bag cannot be squeezed further, so squeeze cannot sustain pressure indefinitely — over time the blowpipe must supply all outflow. The travel must cover Δz_gap (Section 7).
- **|V_mj| ≤ V_max** for each slide motor (in common/differential coordinates: |V_Σ ± ½V_Δ| ≤ V_max), and slide travel limits s_j ∈ [s_min, s_max].

### State/operating constraints
- **p > 0 for all time** (modelling assumption, Section 4). The nonlinear simulation must check it (stop with an error if p ≤ 0, Section 14) rather than clamp p.
- **p_min < p < p_max** (p_min > 0): reed sounding window. The LTI model is only valid for small deviations within this band. The nonlinear simulation must show behaviour at the edges (e.g. a breath gap with too little squeeze lets pressure fall below p_min).

### Model uncertainties (discuss effect on performance; test in simulation)
- Reed pressure sensitivities β_d1, β_d2, β_c (β_ej sets the pressure budget; β_d1 − β_d2 sets pressure leakage into e_Δ).
- Mismatch between the two drones and slides (β_d, h_s, L_d, slide friction) — breaks the exact common/differential decoupling.
- Reed flow coefficients K_d1, K_d2, K_c; flow law exponent (≈ ½ assumed).
- Bag stiffness k (also governs bag-state observability and the slowest bag pole), volume V₀, contact area A_c (bag material changes as it warms and wets).
- Temperature differences: drone–chanter (≈ 0.4 Hz/K in each e_j) and tenor–tenor (≈ 0.4 Hz/K in e_Δ).
- Reed drift rate (time variance).
- Breath-gap statistics (distribution of t_gap and t_blow; a different player breathes differently).
- Estimator noise, delay and resolution (Section 13).
- Air composition (humidity, CO₂) in R and c₀.
- Unmodelled: reed acoustic dynamics, moisture, leadscrew stiction/backlash, estimation artefacts.

Recommended robustness tests: vary each uncertain parameter by ±20–50 % (or across the measured spread) in the nonlinear simulation with the nominal controller, including k reduced toward the weakly observable case, drone-to-drone mismatch, and the estimator windows at the upper end of their sweep.

---

## 12. Handling the time-varying reed drift

The physical system is time-varying (reeds sharpen as they warm and absorb moisture). The assignment requires an **LTI** model, so:

1. **Design model:** keep the LTI plant; represent drift as an **additive slow disturbance** δ_reed(t) on the chanter pitch (and optionally similar terms on the drone reeds). It enters both e₁ and e₂ through D_d (column −½), i.e. only the **common** error e_Σ (Section 8.6).
2. **Controller:** include **integral action on the tuning errors** (∫e_Σ dt and ∫e_Δ dt). *Corrected during implementation:* the slide's own free integrator does **not** add a second integrator to the tuning loop, because the position servo closes it (feedback on slide position). Each tuning loop is therefore **type 1**: a constant drift is rejected with zero steady-state error, but a ramp drift of r Hz/s leaves a steady error r/k_I, where k_I is the integral gain of the tuning loop. With k_I ≈ 0.25 1/s (T_w = 5 s) this is about 4 s × r, e.g. 0.012 Hz for the placeholder drift (+3 Hz in 8 min on the chanter). Check this, and the transient error, against |e_j| ≤ 0.05 Hz with the measured drift rate. See LOGBOOK.md.
3. **Validation:** simulate the nonlinear plant with genuinely drifting parameters (e.g. f_c0 rising by a few Hz over several minutes — use the measured drift rate — and β coefficients changing) using the LTI-designed controller.
4. **Report:** state clearly that time variance is outside the LTI model, explain why treating it as a disturbance is valid (drift is much slower than closed-loop bandwidth), and mention **gain scheduling / adaptive control** as an extension.

**Confirm this framing with the tutor** (open question 1).

---

## 13. Sensor model

### 13.1 Pressure sensor (y₁)
Small additive Gaussian noise; bandwidth much faster than plant — treat as instantaneous.

### 13.2 One-microphone estimation of both tuning errors (proposed scheme)
**Students' decision:** one microphone, listening continuously, hears the chanter (Low A) and both tenor drones (bass stopped). Estimating e₁ and e₂ from it is the project's main engineering challenge. The scheme below is the proposed approach for the students to develop and validate on their recordings.

**What the microphone hears.** Chanter Low A has partials at n·f_c ≈ 480n Hz (n = 1, 2, 3, …). Each tenor has partials at k·f_j ≈ 240k Hz. So the spectrum splits into two kinds of band:
- **Drone-only bands** at **odd** multiples of f̄_d (≈ 240, 720, 1200 Hz, …): the chanter has no partial there. Only the two tenors sound, beating at k·|f₁ − f₂| = k·|e_Δ|.
- **Shared bands** at **even** multiples (≈ 480, 960, 1440 Hz, …): the chanter's n-th partial plus both tenors' 2n-th partials, beating at 2n·|e₁| and 2n·|e₂| (and 2n·|e_Δ| between the drones).

Narrow band-pass filters (or an STFT / filterbank) centred on these frequencies isolate each band.

**Step 1 — Common/differential decomposition.** Tune the tenors to each other and the pair to the chanter, as two separate SISO problems (Section 8.6):
- **e_Δ from the drone-only bands**, corrected with the differential slide input V_Δ. These bands contain only two tones (one per tenor), so the measurement is a clean two-tone beat, free of the chanter, its drift and (to the extent the reeds match) bag pressure.
- **e_Σ from the shared bands**, corrected with the common slide input V_Σ. Once e_Δ is small, the two tenors' partials in each shared band merge into one component, so each band again holds just **two** tones (chanter vs drone pair) and beats at 2n·|e_Σ|.

Without this decomposition, each shared band holds three tones with three interacting beat rates, and it is much harder to tell which drone is which.

**Step 2 — Recover the sign.** A beat rate gives the size of an error, not whether the drone is sharp or flat.
- *Passive method (shared bands):* for two beating tones of unequal strength, the band's instantaneous frequency (from the analytic signal / Hilbert transform) swings toward the weaker tone when the envelope is high and away from it when the envelope is low. For z(t) = A·e^{iω_a t} + B·e^{iω_b t} with A > B and Δω = ω_b − ω_a, the instantaneous-frequency deviation is Δω·B·(B + A·cos Δωt)/(A² + B² + 2AB·cos Δωt): +Δω·B/(A + B) at envelope peaks and −Δω·B/(A − B) at troughs. So the **correlation between envelope and instantaneous frequency has the sign of (weaker − stronger)**. If the chanter partial is the stronger one in a band (check in the recordings, Section 10.5 item 9), that sign is the sign of e_Σ. It needs about one beat period of signal.
- *Active method (drone-only bands, where the two tenors may be about equally loud and the passive method fails):* **nudge and listen**, as pipers do — make a small, known differential slide step (or a slow dither) and check whether the tenor–tenor beat slows down or speeds up. Keep the nudge below the target (≲ 0.05 Hz ≈ 0.075 mm of slide).

**Step 3 — Use higher partials for speed.** Coinciding partial pair n beats at 2n·|e|, so one beat at the target |e| = 0.05 Hz takes 10 s at n = 1, 5 s at n = 2, 3.3 s at n = 3 and 2 s at n = 5. Likewise drone-only band k beats at k·|e_Δ|. Combine several bands, weighted by their signal-to-noise ratio from the recordings, to shorten the measurement time.

**Step 4 — "In tune" without a sign.** If no beat is detected within a window T_w on pair n, then |e| < 1/(2n·T_w). With n = 1 and T_w = 10 s this is exactly the target, so the estimator can report "within target" and the loop holds — no sign is needed when the error is that small.

**Caveats to handle and discuss:**
- With e_Δ ≈ 0, the two tenors' partials add with a nearly fixed relative phase and can partly cancel in some bands. The relative phase differs between harmonics, so use several bands.
- The bass drone is assumed stopped: its partials (multiples of ≈ 120 Hz) fall in every tenor band. If it must sound, it becomes a third source (its odd harmonics give bass-only bands, so the scheme can extend).
- The target is defined on the lowest coinciding pair; higher pairs beat faster and may be what the ear hears (Section 6.7).
- Validate the estimator on recordings of the real pipes with known slide offsets (Section 10.5 item 9).

### 13.3 Simulation sensor model
In closed-loop simulation, use an idealised model of the estimator's output (implement the real DSP separately, on recorded or synthesised audio, as validation):

  e_Σ,meas(t) = moving average of e_Σ over T_w,Σ + noise
  e_Δ,meas(t) = moving average of e_Δ over T_w,Δ + noise

updated continuously, and convert to e₁ = e_Σ + ½e_Δ, e₂ = e_Σ − ½e_Δ where needed. Optionally add the Step 4 resolution floor (report 0 when |e| < 1/(2n·T_w)). The windows depend on which partials are used: baseline T_w ≈ 10 s for n = 1, shorter with higher partials; sweep T_w (e.g. 2–20 s) and the noise level (aim ≲ 0.01–0.015 Hz RMS, a small fraction of the 0.05 Hz target; the synthetic-audio prototype currently reaches ≈ 0.02 Hz RMS on e_Σ at T_w = 5 s, which the simulation uses). For LTI design, approximate each window by a delay of T_w/2 (`pade`) or a first-order lag, and discuss the phase-lag limit on bandwidth (Lecture 7 limitations).

**Consequence for control:** with effective delays of seconds, both microphone loops must be slow (crossover roughly ω_c ≲ 1/τ, e.g. ≈ 0.2 rad/s for τ = 5 s). That is fast enough for reed drift and temperature (minutes) but **too slow for pressure disturbances** from breath gaps (~1 s). Those must be handled through the fast pressure sensor: the pressure loop plus pressure feedforward to the common slide mode (Section 15.2). The differential loop needs little or no pressure feedforward if the reeds are matched.

---

## 14. Simulation requirements

### 14.1 Open-loop plant simulations (Week 9, A2)
On **both** the LTI model and the nonlinear model:
1. **Step** in each input separately (δq_b, δF, δV_m1, δV_m2; also δV_Σ and δV_Δ) — show p, e₁, e₂ and states.
2. **Ramp** in V_m1 or q_b.
3. **Sinusoids** at several frequencies (compare with Bode).
4. **Breath gaps**: q_b forced to 0 for each gap (the input constraint), with constant F — show how far p falls and confirm p stays > 0. Use (a) a **random gap sequence** drawn from the player-fitted distributions with a fixed seed (`rng(seed)`) so runs are reproducible, and (b) a **deterministic worst case**: longest gap t_gap,max followed by the shortest interval t_blow,min, repeated.
5. **Disturbances**: step in δT_d1 alone (one drone warming faster), δT_c, δT_bag; ramp δ_reed. Show that δT_d1 moves e_Δ while δ_reed moves only e_Σ.
6. **Linear vs nonlinear comparison** for small and large deviations — show where the linearisation breaks down (especially near p_min / p_max).
7. **With constraints** (saturations, travel limits), **noise**, **delay**, and **parameter uncertainty** (including drone-to-drone mismatch).

Use `ode15s` for the nonlinear model (stiff: motor electrical time constant ~ms vs drift ~minutes). Use an `Events` function to **stop the run with an error if p ≤ 0** (the p > 0 assumption is violated — do not clamp) and to flag z or s_j leaving their travel limits. Use `lsim` / `step` / `initial` for the LTI model.

### 14.2 Closed-loop simulations (Assessment 2)
Scenario "performance": start with both tenors in tune → random breath gaps (player-fitted) → slow chanter reed drift ramp over several minutes → one drone warming faster than the other (δT_d1 ramp) → temperature step. Report e₁, e₂ (and e_Δ) in Hz against the 0.05 Hz target (and the fraction of time inside it), the equivalent beat periods 1/(2|e_j|) against 10 s, pressure deviation, and input usage against limits.

Because the breath gaps are random, run the scenario as a **Monte Carlo** set (many seeds) and report statistics across runs — worst-case and percentile values of |e_j|, pressure deviation and squeeze travel — alongside the deterministic worst-case sequence from Section 14.1.

### 14.3 Plots to produce
Open-loop step responses; Bode plots of plant channels; pole–zero map; closed-loop responses with the ±0.05 Hz band marked on e₁ and e₂; S and T magnitude plots for the pressure, common and differential loops; input signals with saturation limits marked; robustness sweeps (overlaid responses for perturbed parameters).

---

## 15. Controller design guidance

These are suggestions consistent with the course; adapt to the Assessment 2 spec once available.

### 15.1 Control objectives
1. **At most one drone–chanter beat every 10 s for each tenor**, i.e. keep **|e₁|, |e₂| ≤ 0.05 Hz** (student requirement, chosen as reflective of the real situation; ≈ 0.36 cents). This also keeps the tenors within 0.1 Hz of each other. Justify it as a beat criterion: the ~5–10 cent just-noticeable difference applies to tones heard one after another, but beats between simultaneous tones are audible at far smaller mistuning, which is why pipers tune by ear to remove beats. The target is tight, so the report should include an **error budget** showing that estimator noise, slide positioning (0.05 Hz ≈ 0.075 mm), residual pressure coupling (β_ej·δp), temperature differences (≈ 0.4 Hz/K) and drift transients together stay below 0.05 Hz.
2. Keep bag pressure p ≈ p̄, within the sounding window, despite random breath gaps.
3. Respect all input constraints.
4. Reject slow drift and temperature changes.

### 15.2 Suggested architecture
The plant splits into a **bag/pressure** problem (q_b, F → p) and two **slide** problems (common V_Σ → e_Σ, differential V_Δ → e_Δ). Designing these as separate loops is natural because the dynamics are decoupled, and it gives SISO loops for the S/T analysis the course teaches.

- **State feedback with integral action:** augment with integrators on e_Σ, e_Δ and p (x_aug = [δx; ∫e_Σ; ∫e_Δ; ∫δp]), check reachability of the augmented system, place poles (`place`) or use LQR if allowed. Designing per block (bag, common, differential) keeps the gains easy to justify; a single MIMO `place` picks one of many possible gain matrices and may allocate inputs badly.
- **Observer:** Luenberger observer of the bag states from p (`place(A',C',…)'`), fed the **actual** (saturated) inputs, especially q_b = 0 during breath gaps, or its estimates drift. *Found during implementation:* the observer has to be clearly faster than the controller (controller −2, −5, −15 ± 10j; observer −50, −120 ± 60j) to give good margins (≈ 53°, 17 dB). **No observer of slide position from the microphone:** a slide offset and reed drift shift e by the same constant, so they cannot be told apart from e. The slide position is predicted from the motor model (an encoder would measure it), and the microphone error is integrated directly (below).
- **Implementation is digital:** design gains and observers on ZOH-discretised models with s-plane poles mapped by z = e^{sTs}. A continuous design implemented at Ts = 5 ms went unstable.
- **Pressure input allocation (redundant inputs q_b and F):** both act on pressure, but **F → p has a zero at s = 0** (Section 9.4): squeeze cannot hold a steady pressure offset. Therefore:
  - Use only **one** integrator on pressure, and it must act through **q_b** — an integrator on p driven through F alone makes the augmented system unreachable. In a full state-feedback design, check that the steady-state input share falls on q_b. If both inputs integrate pressure error they will drift against each other and one will saturate.
  - **Frequency split (recommended, mirrors real piping):** blowpipe handles low-frequency / average supply (the only input that adds air); squeeze handles fast corrections and covers breath gaps. Show on Bode plots that each input dominates a different band.
  - **Valve-position / mid-ranging control:** a slow loop moves q_b to return z (squeeze position) to z̄, keeping the fast squeeze actuator away from its limits and ready for the next breath gap. This needs z — measured by an arm encoder, or estimated (observable only if k > 0, Section 9.2).
  - Breath gaps are random and **not known in advance**, so the controller cannot pre-squeeze. It does know the instant a gap starts and ends (q_b is forced to 0 on its own actuator), so it can apply **squeeze feedforward at gap onset** (F ramp matched to the expected outflow) and refill with q_b once blowing resumes. Preview (pre-squeezing before a gap) would only be possible if the robot's breath generator announced the next gap — an optional extension; real pipers do anticipate their breaths.
- **Slide loops — two modes, two time-scales:**
  - **Common mode (V_Σ → e_Σ):** fast path = **feedforward from measured pressure** to both slides together, cancelling β̄_e·δp so pressure dips during breath gaps do not appear in e_Σ (the microphone is too slow to see them in time); slow path = e_Σ feedback with integral action to remove chanter drift, temperature effects and the feedforward's residual error (β̄_e is uncertain).
  - **Differential mode (V_Δ → e_Δ):** e_Δ feedback with integral action, from the drone-only bands. It sees no chanter drift and little pressure (only through β_d1 − β_d2), so it can run without feedforward; add a small differential pressure feedforward if the measured reed mismatch is significant.
  - **Implemented structure (cascade):** an inner position servo (state feedback on the motor-model slide position and velocity) tracks a slide reference. That reference is the pressure feedforward plus k_I/h_s·∫e_m dt, where e_m is the microphone estimate. k_I is set by loop shaping for a 55° phase margin against the moving-average sensor: crossover ≈ 0.24 rad/s at T_w = 5 s, consistent with ω_c ≲ 1/τ. An earlier attempt to place the slow sensor and integral poles with an observer was fragile and went unstable against the true moving average (LOGBOOK.md).
  - Respect the coupled voltage limits |V_Σ ± ½V_Δ| ≤ V_max.
- **Anti-windup** on every integrator (q_b ≥ 0 and q_b,max(t) including breath gaps, F limits, V_m1/V_m2 limits, slide travel).
- **Two-degree-of-freedom** (Lecture 5) prefilter F(s) optional for smooth re-tuning steps.

### 15.3 Frequency-domain analysis (Lecture 5–7)
- Bode / loop shaping of the pressure loop and the common and differential slide loops.
- Plot |S(s)| and |T(s)| for each loop; check tracking/disturbance rejection at low frequency and noise attenuation at high frequency. The 0.05 Hz target sets how small |S| must be at the drift and temperature frequencies.
- Discuss fundamental limitations: estimator delay, the F → p zero at the origin, any RHP zeros found in the channels (Padé approximations add RHP zeros by construction), actuator limits (including breath gaps).

---

## 16. Known risks

| # | Risk | Mitigation |
|---|---|---|
| 1 | Chanter pitch depends on pressure, so an absolute pitch reference would move with the plant | **Done:** outputs are tuning errors e_j = f_j − ½f_c, references e_j = 0 |
| 2 | Scarce published parameter data for Highland reeds/bags | **Mitigated:** students measure directly on real pipes (Section 10.5); literature as cross-check |
| 3 | Linearisation valid only in a narrow pressure window | Show nonlinear vs linear divergence; simulate edge cases |
| 4 | Redundant pressure inputs fight / wind up | F → p has a zero at s = 0: one pressure integrator, acting through q_b; frequency split or mid-ranging; anti-windup |
| 5 | One microphone for two tenors: three sources near the same partials, seconds per measurement, beat rate gives no sign | Common/differential scheme (Section 13.2): drone-only bands for e_Δ, shared bands for e_Σ, passive sign from envelope/frequency correlation, nudge-and-listen for e_Δ, higher partials for speed; validate on recordings |
| 6 | Bag states weakly observable from pressure if bag stiffness k is small (det O_bag = a_m·a_z²·k/M) | Measure k carefully; **check conditioning of obsv first**; add arm position encoder if needed |
| 7 | Badly scaled matrices (Pa vs kg vs m) | Scale variables (Section 8.7) before rank tests and design |
| 8 | Wide time-scale separation (ms to minutes) | Ignore acoustics; treat drift/temperature as slow disturbances; use ode15s; consider a current-controlled motor driver to remove the fastest states |
| 9 | Tutor may not accept drift-as-disturbance | Confirm in next practical |
| 10 | Scope growth (two drones, 9 states, Monte Carlo, estimator) in ~3 weeks | Common/differential decomposition reduces the slides to two identical SISO loops; prioritise the core deliverables (Section 17) before Monte Carlo and estimator validation |
| 11 | Realism: pipers normally tune before playing | Justify as automatic correction during long performances / practice aid; "robotic piper" framing |
| 12 | Uniform temperature cancels exactly in tuning errors | Model separate drone and chanter temperatures (Section 8.5) |
| 13 | Target \|e_j\| ≤ 0.05 Hz (one beat per 10 s) is tight (≈ 0.075 mm slide, ≈ 0.12 K drone–chanter temperature difference, 0.05/\|β_ej\| Pa of pressure) | Error budget (Section 15.1); pressure feedforward to the common mode; low-backlash leadscrews |
| 14 | Slide model wrong by orders of magnitude if rotor inertia is left out | Use m_eff = m_s + J_m·G² with G = 2πN/ℓ (Section 6.4) |
| 15 | Random breath gaps exceed squeeze capacity or blower flow — worst case is a long gap followed by a short blowing interval | Size q_b,max, z_max and F_max from the Section 7 worst-case checks; confirm with Monte Carlo runs and the deterministic worst-case sequence in the nonlinear model |
| 16 | Passive sign method needs the chanter partial to be clearly stronger (or weaker) than the drone pair in each shared band | Measure band amplitudes in the recordings; fall back to nudge-and-listen in bands where amplitudes are similar |
| 17 | Tenor partials partly cancel in some bands when e_Δ ≈ 0 | Use several bands; their relative phases differ |
| 18 | Slide stiction + integral action causes hunting at the 0.075 mm scale | Model static/Coulomb friction in the nonlinear simulation; consider an integrator deadband matched to the Step 4 resolution floor |
| 19 | Drone-to-drone mismatch (reeds, slides) breaks exact decoupling | Measure both drones; include mismatch in robustness tests |

---

## 17. Task list

**Stage 1 — Model (Week 9 deliverable)**
1. Carry out the measurement plan (Section 10.5) and fill in every parameter in Section 10. Resolve p̄, f_c0, β_d1, β_d2, β_c, h_s, K_d1, K_d2, K_c, V₀, k first, and use m_eff (not m_s) for the slides.
2. Choose and justify the equilibrium (Section 7); fit the breath-gap distributions to the player recordings; verify feasibility against input limits, including the breath-gap checks (mean flow, worst-case refill, squeeze travel and force).
3. Implement the nonlinear model f(x,u,d), h(x,u,d) in MATLAB, with an `ode15s` event that stops the run if p ≤ 0.
4. Derive A, B, C, D, E, D_d symbolically (`jacobian`), substitute numbers, cross-check against Section 8 and against finite differences. Form the common/differential model (Section 8.6).
5. Scale the model (Section 8.7).
6. Check reachability, observability (rank and conditioning), eigenvalues, channel zeros including the F → p zero at s = 0 (Section 9). **If observability fails or is poorly conditioned, stop and report** — add a squeeze-position sensor.
7. Run all open-loop simulations in Section 14.1 and produce plots.

**Stage 2 — Controller (Weeks 10–11, Assessment 2)**
8. Obtain the Assessment 2 specification from the students; align the design with its criteria.
9. Design the pressure loop with input allocation (Section 15.2) and anti-windup.
10. Design the common and differential slide loops (state feedback with integral action + observer), plus pressure feedforward to the common mode.
11. Frequency-domain analysis: Bode, S(s), T(s), limitations — per loop.
12. Closed-loop simulations on the nonlinear plant with constraints, noise, delay, random breath gaps (Monte Carlo + worst-case sequence), drift, temperature differences (Section 14.2), assessed against |e_j| ≤ 0.05 Hz with an error budget, sweeping the estimator windows.
13. Robustness sweeps over uncertain parameters, including drone-to-drone mismatch (Section 11).
14. Prototype the one-microphone estimator (Section 13.2) on the recordings: band extraction, beat rates, sign recovery, measurement time vs partial number. Use the results to set the window and noise values in the simulation sensor model.
15. Write up: system description, assumptions, model, measured parameters, linearisation, requirement checklist (Section 1 table), structural properties, estimator, simulations, controller design, discussion of constraints, uncertainty and time variance, limitations and extensions.

**Priority if time runs short:** tasks 1–7, then 9–11 and a basic version of 12; Monte Carlo statistics, robustness sweeps and the estimator prototype after that.

**Deliverables expected from the agent:** MATLAB scripts (parameters file, nonlinear model function, linearisation script, analysis script, simulation scripts / Simulink model, estimator prototype), figures, and a report draft.

---

## 18. Open questions

1. Does the tutor accept modelling reed drift as a slow disturbance (keeping the plant LTI)? Ask at the next practical.
2. Does a "robotic piper" (robotic arm squeezing the bag + two motorised drone slides) satisfy the group's robotics goal?
3. What exactly does the Assessment 2 specification require (report format, length, required design methods — e.g. pole placement vs LQR, observer required, frequency-domain analysis required)? Obtain the document.
4. Is the bass drone stopped during automatic tuning (assumed here)? If it must sound, it adds a third source to every tenor analysis band (Section 13.2) and its own slide states.

Resolved by the students: blowpipe flow is robot-controlled with random breath gaps modelled on an actual player; pitch sensing is a single microphone listening continuously, used for both tenor drones; the tuning target is at most one beat every 10 s (|e_j| ≤ 0.05 Hz); the chanter plays Low A only, in just intonation, with constant air use; both tenor drones are controlled; parameters are measured directly on real pipes.
