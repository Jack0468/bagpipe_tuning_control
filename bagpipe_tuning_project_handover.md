# ELEC3304 Assessment 2 — Robotic Bagpipe Pressure & Drone-Tuning Control System
## Handover specification for the implementing agent

**Course:** ELEC3304 Control, Semester 2, 2026
**Group size:** 2 students (fixed for the rest of semester)
**Prepared:** 8 October 2026 (end of Week 9 practical)
**Revised:** 8 October 2026 — review corrections: pitch models in deviation form, effective slide mass and gear ratio, observability/stability conditions on bag stiffness k, pressure-input allocation (F → p zero at s = 0), random breath gaps (modelled on an actual player) as an input constraint, tuning target (at most one beat every 10 s, i.e. |e| ≤ 0.05 Hz), single-microphone sensor model, notation clashes.
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
10. [Parameters (to research and verify)](#10-parameters)
11. [Constraints and uncertainties](#11-constraints-and-uncertainties)
12. [Handling the time-varying reed drift](#12-handling-the-time-varying-reed-drift)
13. [Sensor model](#13-sensor-model)
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
| 1 | At least two states | 6 states (Section 5) |
| 2 | Fewer measurements (sensors) than states | 2 sensors (bag pressure, tuning error from a microphone) vs 6 states |
| 3 | Reasonable model parameters, backed by research | Parameter table in Section 10 — **every value needs a cited source or a justified estimate** |
| 4 | LTI state-space form using matrices | A, B, C, D (and disturbance matrix E) in Section 8 |
| 5 | At least one actuator, making the system reachable | 3 actuators: blowpipe flow, bag squeeze force, slide motor voltage; reachability rank check in Section 9 |

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
- **Chanter tuning** is done by applying tape over finger holes — a discrete, manual adjustment.
- Pitch of every reed depends on bag **pressure**. Pitch also depends on **temperature** (speed of sound in the air column). As the reeds warm up and absorb moisture during playing, **pitch drifts upward** over time.

### Design decisions already made in discussion (with reasons)
1. **Pitch is the output; tape and drone length are actuators.** The original idea listed "tape on the chanter" and "drone length" as outputs; these are things you adjust, so they are inputs. Pitch is measured.
2. **Chanter tape is dropped as an actuator.** It is a discrete manual adjustment, hard to model as a continuous input. The chanter instead provides the tuning **reference**.
3. **Drone tuning slide is motorised** (DC motor + leadscrew). This is the robotics element along with the squeezing arm.
4. **Air enters through both the blowpipe and the bag squeeze** (student requirement). These are two separate inputs acting on the same pressure — a redundant (over-actuated) input pair.
5. **Pitch itself has no dynamics** on the control time-scale (acoustic transit ~1 ms). States come from bag air storage, arm/bag mechanics, and the slide motor.
6. **Output is the tuning error** (drone pitch minus half the chanter Low A pitch), not absolute drone pitch, because bag pressure moves both the chanter and the drone. Using the error avoids a hidden feedback loop through the reference, and the reference becomes simply e = 0.
7. **Reed drift (time variance) is modelled as a slow disturbance**, keeping the model LTI as required; the controller rejects it with integral action. See Section 12.
8. **Scope is limited** to the bag + one tenor drone (6 states). Extensions (all three drones, chanter dynamics) are optional only if time permits. Every reed that sounds still draws air from the bag, so the outflow model includes all sounding drones even though only one is tuned.
9. **Blowpipe flow q_b is controlled by the robot, with random breath gaps modelled on an actual player** (confirmed by the students). Gap durations and the blowing intervals between them are drawn from distributions fitted to a real piper's breathing. Breath gaps are **not** a separate disturbance: they are a random input constraint, q_b,max(t) = 0 during each gap, not known to the controller in advance (Section 11).
10. **Tuning target: at most one drone–chanter beat every 10 s** (beat frequency ≤ 0.1 Hz), which the students chose as reflective of the real situation. The beat is 2|e| (Section 6.7), so this is **|e| ≤ 0.05 Hz** on the drone-fundamental scale, ≈ 0.36 cents at f̄_d ≈ 240 Hz.
11. **Pitch sensing is a single microphone listening continuously** (confirmed by the students). Extracting e from the combined sound of chanter and drones is part of the project's engineering challenge (Section 13).

### Framing for the report
The pipes are "played" by a robotic system: a blower/blowpipe flow source and a robotic arm squeezing the bag, plus a motorised tuning slide. Application: automatic drone tuning during long performances and as a practice aid. Note: real pipers tune before playing and only touch up between tunes, so the report should justify continuous tuning. Check with the tutor that this counts as robotics if that matters to the group.

---

## 4. System description and scope

```
            q_b (blowpipe, one-way)
                  │
                  ▼
   F (arm) ──▶ [  BAG  ]  air mass m, volume V(z), pressure p
                  │
       ┌──────────┴──────────┐
       ▼                     ▼
   Chanter reed          Tenor drone reed
   (flow ∝ √p,           (flow ∝ √p)
    fingering w)              │
       │                 Drone pipe + motorised tuning slide s
       ▼                      ▼
   pitch f_c(p, T, δ)     pitch f_d(s, p, T)
       └──────────┬───────────┘
                  ▼
        e = f_d − ½ f_c   (microphone, Section 13)
```

**In scope:** bag air mass, squeezing arm and bag wall (mass–spring–damper), one tenor drone with motorised leadscrew slide, chanter as a pressure-dependent pitch source (algebraic, no states), outflow through all sounding reeds.

**Out of scope (possible extensions):** tuning slides of the second tenor drone and bass drone (each adds 3 slide states; their reeds' air consumption is still included in the outflow if they sound), chanter tape, reed acoustic dynamics (kHz, far faster than the control bandwidth), bag thermodynamics beyond an isothermal ideal-gas assumption, moisture.

**Modelling assumptions (state these in the report):**
- Air in the bag is an ideal gas at uniform temperature T; isothermal on the control time-scale.
- Bag volume changes only through arm squeeze displacement z: V = V₀ − A_c·z (A_c = effective arm–bag contact area).
- **Bag gauge pressure is strictly positive for all time (p > 0).** In normal operation p_min ≤ p ≤ p_max with p_min > 0, so √p in the outflow law is always defined.
- Reed outflow is orifice-like: mass flow ∝ √(gauge pressure).
- Arm + bag wall behaves as a linear mass–spring–damper driven by the arm force and loaded by the bag pressure, with stiffness k > 0.
- Drone is a cylindrical pipe closed at the reed end: quarter-wave resonator.
- Pitch depends instantaneously (algebraically) on pressure, slide position and temperature.
- Leadscrew slide moves horizontally (no gravity load); friction is viscous; motor drives the leadscrew directly or through a gearbox of ratio N (ideal, no backlash).

---

## 5. States, inputs, outputs, disturbances

**Notation.** T is temperature throughout (T̄ nominal; T_d, T_c drone and chanter air columns). Where the course's complementary sensitivity or prefilter appear (Sections 2, 15), they are written with their Laplace argument, T(s) and F(s). A (no subscript) is the state matrix only; the arm–bag contact area is A_c. Arm/bag damping is b_a; c(T) is the speed of sound. F is the arm force input; the output disturbance matrix is D_d.

### State vector (n = 6)
x = [m, z, ż, s, ṡ, i]ᵀ

| State | Symbol | Units | Meaning | Why it is a state |
|---|---|---|---|---|
| x₁ | m | kg | Mass of air in the bag | Stores mass; conservation law ṁ = in − out |
| x₂ | z | m | Arm squeeze displacement into the bag | Arm/bag-wall position (2nd-order mechanics) |
| x₃ | ż | m/s | Squeeze velocity | Arm/bag-wall momentum |
| x₄ | s | m | Drone tuning slide extension from reference position | Slide position (integrates velocity) |
| x₅ | ṡ | m/s | Slide velocity | Slide momentum |
| x₆ | i | A | Slide motor current | Motor inductance stores energy |

Air mass is chosen over pressure because it satisfies a simple conservation law; pressure then follows algebraically from mass and volume, so **pressure is an output, not a state.**

### Input vector (3 inputs)
u = [q_b, F, V_m]ᵀ

| Input | Symbol | Units | Meaning | Constraint |
|---|---|---|---|---|
| u₁ | q_b | kg/s | Air mass flow in through blowpipe (manipulated) | 0 ≤ q_b ≤ q_b,max(t) (one-way valve; q_b,max(t) = 0 during each random breath gap, Section 11) |
| u₂ | F | N | Squeeze force applied by arm to bag | 0 ≤ F ≤ F_max (can only push) |
| u₃ | V_m | V | Slide motor voltage | \|V_m\| ≤ V_max |

### Output vector (2 outputs)
y = [p, e]ᵀ

| Output | Symbol | Units | Sensor |
|---|---|---|---|
| y₁ | p | Pa (gauge) | Bag pressure sensor |
| y₂ | e | Hz (target \|e\| ≤ 0.05 Hz, i.e. beat ≤ 0.1 Hz) | Single microphone, listening continuously, + pitch/beat estimation (Section 13): tuning error between tenor drone and half the chanter Low A pitch |

2 sensors < 6 states ✔ (requirement 2).

### Disturbances
| Disturbance | Symbol | Units | Effect | Time-scale |
|---|---|---|---|---|
| Air temperature | T (deviation δT) | K | Changes pressure (gas law) and pitch (speed of sound) | minutes |
| Chanter air demand | w | – (fractional change) | Different fingered notes draw different flow from the bag | ~0.1–1 s (melody) |
| Reed pitch drift | δ_reed | Hz | Chanter pitch creeps upward as the reed warms/wets | minutes (ramp-like) |

Breath gaps are **not** in this table: q_b is a manipulated input, and a breath gap is a random input constraint (q_b,max(t) = 0 for a random duration t_gap, separated by random blowing intervals t_blow, both modelled on an actual player; Section 11).

Optional refinement: drones and chanter warm at different rates, so model separate temperatures T_d (drone air column) and T_c (chanter); only their **difference** strongly affects the tuning error (see Section 8.4).

---

## 6. Nonlinear model

### Constants and parameters
R = 287 J/(kg·K) (specific gas constant of air), p_atm = 101 325 Pa. Other symbols are defined in Section 10.

### 6.1 Bag pressure (algebraic)
Bag volume: V(z) = V₀ − A_c·z

Gauge pressure (ideal gas):

  p(m, z, T) = m·R·T / (V₀ − A_c·z) − p_atm

### 6.2 Air mass balance

  ṁ = q_b − Q_out(p, w)

  Q_out(p, w) = (K_d + K_c·(1 + w))·√p

K_d: **total** flow coefficient of all sounding drone reeds (all three drones if they play, even though only one tenor drone is tuned in this model); K_c: chanter reed flow coefficient (units kg·s⁻¹·Pa^−½). w scales chanter demand with fingering. √p is defined because p > 0 for all time (Section 4 assumption); the nonlinear simulation must check this assumption rather than clamp p (Section 14).

### 6.3 Arm and bag wall (mass–spring–damper loaded by pressure)

  M·z̈ = F − k·z − b_a·ż − A_c·p(m, z, T)

M: effective moving mass of arm + bag wall; k: bag/arm stiffness (k > 0 — observability of the bag states from pressure and the stability margin of the bag block both depend on it, Section 9); b_a: arm/bag damping. The term A_c·p is the bag pushing back on the arm.

### 6.4 Tuning slide (leadscrew)

  m_eff·s̈ = G·K_t·i − b_s·ṡ,  with G = 2π·N/ℓ

ℓ: leadscrew lead (m/rev); N: gearbox ratio between motor and leadscrew (N = 1 for direct drive); K_t: motor torque constant; b_s: total viscous friction at the slide, including motor friction reflected through the drive (b_s = b_slide + b_m·G²).

m_eff: effective moving mass at the slide, **including reflected rotor inertia**:

  m_eff = m_s + J_m·G²   (add gearbox inertia if significant)

The rotor term usually dominates by orders of magnitude: with ℓ = 1 mm, N = 1 and J_m = 10⁻⁷ kg·m², J_m·G² ≈ 3.9 kg, compared with a bare slide mass m_s ≈ 0.02–0.1 kg. **Always use m_eff in the model, never m_s alone.**

### 6.5 Slide motor (armature circuit)

  L·di/dt = V_m − R_a·i − K_e·G·ṡ

R_a: armature resistance; L: armature inductance; K_e: back-EMF constant (K_e = K_t in SI units). G includes the gear ratio N, so G·ṡ is the motor shaft speed.

### 6.6 Pitch models (algebraic, deviation form)
Speed of sound: c(T) = c₀·√(T / 273.15), c₀ ≈ 331.3 m/s

Reed pressure sensitivity is written about the operating pressure p̄, so that the constant terms are the **playing pitches at the operating point** (which is what measurements and literature report).

Tenor drone (quarter-wave, closed at reed end) plus reed pressure sensitivity:

  f_d = c(T_d) / [4·(L_d + s + Δ)] + β_d·(p − p̄)

L_d: drone sounding length at s = 0; Δ: end correction (≈ 0.6 × bore radius for an unflanged open end); β_d: local slope of drone pitch with pressure at p̄ (Hz/Pa).

Chanter Low A:

  f_c = f_c0·c(T_c)/c(T̄) + β_c·(p − p̄) + δ_reed

f_c0: chanter Low A **at the operating point** (p = p̄, T_c = T̄, δ_reed = 0), i.e. the measured playing pitch; β_c: local slope of chanter pitch with pressure at p̄ (Hz/Pa).

### 6.7 Outputs

  y₁ = p(m, z, T)
  y₂ = e = f_d − ½·f_c   (Hz)

**Tuning target: at most one beat every 10 s.** The drone's 2nd harmonic and the chanter's Low A are 2|e| apart, so the beat frequency is f_beat = |2f_d − f_c| = 2|e|. The target f_beat ≤ 0.1 Hz is therefore

  **|e| ≤ 0.05 Hz**

e is kept in Hz throughout because the target is specified in Hz. At f̄_d ≈ 235–240 Hz this is ≈ 0.36 cents (e_cents = 1200·log₂( f_d / (½ f_c) ) ≈ (1200 / ln 2)·(δf_d − ½δf_c) / f̄_d, for reporting only).

### 6.8 Full nonlinear system (compact form)
ẋ₁ = q_b − (K_d + K_c(1+w))·√p(x₁, x₂, T)
ẋ₂ = x₃
ẋ₃ = [F − k·x₂ − b_a·x₃ − A_c·p(x₁, x₂, T)] / M
ẋ₄ = x₅
ẋ₅ = [G·K_t·x₆ − b_s·x₅] / m_eff
ẋ₆ = [V_m − R_a·x₆ − K_e·G·x₅] / L

with p(x₁, x₂, T) > 0 for all t.

Note the **block structure**: the bag subsystem (x₁–x₃) and slide subsystem (x₄–x₆) are dynamically decoupled; they couple only through the tuning-error output, where pressure affects both pitches.

---

## 7. Equilibrium

Choose operating values (from research, Section 10):
- p̄: steady playing pressure (gauge)
- z̄: nominal squeeze displacement, leaving enough travel to cover a breath gap (z̄ + Δz_gap ≤ z_max, see below) and room to release
- T̄: nominal air temperature; w̄ = 0; δ̄_reed = 0

Then:

| Quantity | Equilibrium value | From |
|---|---|---|
| V̄ | V₀ − A_c·z̄ | volume definition |
| m̄ | (p̄ + p_atm)·V̄ / (R·T̄) | gas law |
| q̄_b | (K_d + K_c)·√p̄ | ṁ = 0 (blowpipe supplies all outflow; this is the **long-run mean** flow, averaged over the random breath gaps) |
| F̄ | k·z̄ + A_c·p̄ | z̈ = 0, ż = 0 |
| s̄ | solves e = 0: c(T̄)/[4(L_d + s̄ + Δ)] = ½·f_c0 | tuning condition (pressure terms vanish at p̄ in deviation form) |
| ṡ̄, ī, V̄_m | 0, 0, 0 | slide at rest, no static load |
| f̄_d | ½·f_c0 (in tune) | |

Choose L_d so that s̄ sits near the middle of the slide travel.

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

with δd = [δT, δw, δ_reed]ᵀ (or [δT_bag, δT_d, δT_c, δw, δ_reed]ᵀ if temperatures are separated — recommended, Section 8.5).

### 8.1 Partial derivatives of pressure (evaluated at equilibrium)

  a_m = ∂p/∂m = R·T̄ / V̄
  a_z = ∂p/∂z = (p̄ + p_atm)·A_c / V̄
  a_T = ∂p/∂T = (p̄ + p_atm) / T̄

### 8.2 Outflow derivatives

  g = ∂Q_out/∂p = (K_d + K_c) / (2√p̄)
  g_w = ∂Q_out/∂w = K_c·√p̄

### 8.3 State and input matrices

```
      ⎡ −g·a_m        −g·a_z               0        0     0               0          ⎤
      ⎢   0              0                 1        0     0               0          ⎥
A  =  ⎢ −A_c·a_m/M  −(k + A_c·a_z)/M    −b_a/M      0     0               0          ⎥
      ⎢   0              0                 0        0     1               0          ⎥
      ⎢   0              0                 0        0  −b_s/m_eff    G·K_t/m_eff     ⎥
      ⎣   0              0                 0        0  −K_e·G/L        −R_a/L        ⎦

      ⎡ 1     0      0   ⎤
      ⎢ 0     0      0   ⎥
B  =  ⎢ 0    1/M     0   ⎥
      ⎢ 0     0      0   ⎥
      ⎢ 0     0      0   ⎥
      ⎣ 0     0     1/L  ⎦
```

Physical notes:
- −g·a_m: more air → higher pressure → more outflow (self-regulating; stable pole).
- −(k + A_c·a_z)/M: squeezing raises pressure, which pushes back, so the trapped air acts as an extra spring of stiffness A_c·a_z.
- **Bag block (x₁–x₃):** characteristic polynomial (×M)
  M·s³ + (M·g·a_m + b_a)·s² + (g·a_m·b_a + k + A_c·a_z)·s + g·a_m·k,
  so det(A_bag) = −g·a_m·k / M. For k > 0 all three poles are in the LHP (Routh–Hurwitz holds for any positive parameters), but one pole moves toward 0 as k → 0. Physical reason: with k = 0, "more air + more squeeze" in the ratio that leaves p unchanged (δm : δz = a_z : −a_m) is a stationary mode — nothing pushes it back.
- s has no restoring force (leadscrew), so the slide block has a **pole at 0** (integrator from velocity to position). This free integrator matters for disturbance rejection (Section 12).

### 8.4 Output matrices

Define:
  β_e = β_d − ½·β_c   (net pressure sensitivity of the tuning error)
  h_s = c(T̄) / [4·(L_d + s̄ + Δ)²]   (drone pitch sensitivity to slide extension, Hz/m)

```
      ⎡ a_m        a_z       0     0     0    0 ⎤
C  =  ⎣ β_e·a_m    β_e·a_z   0   −h_s    0    0 ⎦

D  =  0 (2×3)
```

e stays in Hz (the target is |e| ≤ 0.05 Hz). For reporting in cents, multiply row 2 of C and D_d by (1200/ln 2)/f̄_d.

**What the target |e| ≤ 0.05 Hz means for the hardware** (illustrative numbers with placeholder parameters — recompute with the researched values):
- **Slide resolution:** h_s = f̄_d / (L_d + s̄ + Δ). With c(298 K) ≈ 346 m/s and f̄_d ≈ 240 Hz, L_d + s̄ + Δ ≈ 0.36 m and h_s ≈ 0.67 Hz/mm, so 0.05 Hz ≈ **0.075 mm** of slide travel. Leadscrew backlash and positioning resolution must be well below this.
- **Pressure budget:** a pressure deviation δp shifts e by β_e·δp. Without slide correction, staying within 0.05 Hz needs |δp| ≤ 0.05 / |β_e| (e.g. β_e = 0.005 Hz/Pa would allow only 10 Pa). Unless β_e turns out small, pressure dips (breath gaps, melody demand) must be compensated by the slide — feedforward from the measured p is natural, and is needed because the microphone measurement is slow (Sections 13, 15.2).

### 8.5 Disturbance matrices

**Single shared temperature** (columns δT, δw, δ_reed):

```
      ⎡ −g·a_T        −g_w     0 ⎤
      ⎢   0             0      0 ⎥
E  =  ⎢ −A_c·a_T/M      0      0 ⎥
      ⎢   0             0      0 ⎥
      ⎢   0             0      0 ⎥
      ⎣   0             0      0 ⎦

       ⎡ a_T          0     0  ⎤
D_d =  ⎣ β_e·a_T      0   −½   ⎦
```

With a single shared temperature the acoustic temperature terms **cancel exactly**: ∂e/∂T = f̄_d/(2T̄) − ½·f_c0/(2T̄) + β_e·a_T, and f̄_d = ½·f_c0 at the equilibrium, leaving only β_e·a_T (bag temperature acting through pressure). A uniform temperature change shifts drone and chanter together.

**Separate temperatures (recommended)** — columns δT_bag, δT_d, δT_c, δw, δ_reed:

```
      ⎡ −g·a_T       0    0    −g_w    0 ⎤
      ⎢   0          0    0      0     0 ⎥
E  =  ⎢ −A_c·a_T/M   0    0      0     0 ⎥
      ⎢   0          0    0      0     0 ⎥
      ⎢   0          0    0      0     0 ⎥
      ⎣   0          0    0      0     0 ⎦

       ⎡ a_T          0              0            0     0  ⎤
D_d =  ⎣ β_e·a_T   f̄_d/(2T̄)    −f̄_d/(2T̄)       0    −½   ⎦
```

(a_T here uses the bag temperature.) The tuning error is therefore driven by the **temperature difference** between drone and chanter air columns: f̄_d/(2T̄) ≈ 0.4 Hz/K, so a drone–chanter temperature difference of only ≈ **0.12 K** uses the whole 0.05 Hz budget. Discuss this in the report.

### 8.6 Scaling (do this before rank checks and pole placement)
Raw magnitudes differ enormously (m ~ 10⁻² kg, z ~ 10⁻² m, p ~ 10³ Pa, flows ~ 10⁻⁴ kg/s, e ~ 0.1–1 Hz). Scale each variable by its expected maximum deviation (x = S_x·x̃, u = S_u·ũ, d = S_d·d̃, y = S_y·ỹ) so all scaled variables are O(1):
  Ã = S_x⁻¹·A·S_x, B̃ = S_x⁻¹·B·S_u, Ẽ = S_x⁻¹·E·S_d,
  C̃ = S_y⁻¹·C·S_x, D̃ = S_y⁻¹·D·S_u, D̃_d = S_y⁻¹·D_d·S_d.
A natural output scale for e is the target itself (0.05 Hz), so |ẽ| ≤ 1 means "within target". Scaling does not change rank in exact arithmetic; it is needed for numerical conditioning. Use the scaled model for rank tests, pole placement and observer design; report physical units.

### 8.7 MATLAB implementation note
Implement the nonlinear f(x,u,d) and h(x,u,d) symbolically (`syms`) and compute A, B, C, D, E, D_d with `jacobian`, then substitute equilibrium values. **Cross-check against the hand-derived matrices above.** Also verify numerically by finite differences on the nonlinear function.

---

## 9. Structural properties to verify

### 9.1 Reachability (requirement 5)
W = [B, AB, A²B, …, A⁵B]; require rank(W) = 6 (`rank(ctrb(A,B))`).
Expected: full rank. q_b drives m, F drives ż (hence z and, via pressure, m), V_m drives i → ṡ → s.
Per-input reachability is meaningful **for the bag block (x₁–x₃) only**: check it from q_b alone and from F alone — useful for the input-allocation discussion. By the block structure, the slide block is reachable only from V_m, and the full 6-state system is not reachable from any single input.

### 9.2 Observability (needed for observer design)
O = [C; CA; …; CA⁵]; require rank(O) = 6 (`rank(obsv(A,C))`).
- Slide block: observable through e (s is measured; ṡ and i follow through the chain).
- Bag block from pressure alone (C_bag = [a_m, a_z, 0]):

  det(O_bag) = a_m·a_z²·k / M

  So the bag block is observable from p **if and only if k > 0**. The pressure sensor measures a_m·δm + a_z·δz and cannot by itself separate "more air" from "squeezed harder"; only the bag/arm stiffness k makes the two evolve differently. If k is small compared with the air-spring stiffness A_c·a_z, the bag block is **weakly observable** — check the conditioning (singular values) of the scaled observability matrix, not just its rank. If it is poorly conditioned: add a squeeze position sensor (arm encoder — realistic for a robot arm) and keep sensors < states (3 < 6).
- Check observability from each sensor alone too.

### 9.3 Stability of the open-loop plant
Compute eig(A). Expected: bag block stable for k > 0 (three LHP poles; product of poles = −g·a_m·k/M, so one pole approaches 0 as k → 0), slide block one pole at 0 plus two stable motor/slide poles. Discuss internal stability (Lecture 8).

### 9.4 Transfer functions and zeros
Compute the 2×3 transfer matrix (`tf(ss(A,B,C,D))`), Bode plots of each channel, and zeros. The bag-block channels to pressure are (with den(s) = (M·s² + b_a·s + k)(s + g·a_m) + A_c·a_z·s):

  p/q_b = a_m·(M·s² + b_a·s + k) / den(s)   (DC gain 1/g; zeros at the arm's own resonance, M·s² + b_a·s + k = 0)
  p/F   = a_z·s / den(s)   (**zero at s = 0**)

**Squeeze cannot change steady-state pressure** — the F → p channel (and F → e, through β_e) has a zero at the origin. Consequences: any pressure integral action must ultimately act through q_b (Section 15.2), and an integrator on p driven through F alone makes the augmented system unreachable. `tzero` on the full non-square 2×3 system will generally not show this; check the SISO channels individually (`zero(tf(...))`). Also check for any RHP zeros (Lecture 7 limitations). Note the pressure→tuning-error coupling (β_e) — if β_e is significant, bag-pressure disturbances appear directly in the tuning error (see the pressure budget in Section 8.4).

---

## 10. Parameters

**Every value must be researched and cited (or justified as an estimate) in the report** — requirement 3. Values below marked "PLACEHOLDER" are order-of-magnitude starting points only, not verified facts.

### Physical constants (reliable)
| Symbol | Value | Notes |
|---|---|---|
| R | 287 J/(kg·K) | specific gas constant, dry air (breath is humid and CO₂-rich, which shifts R slightly; a blower supplying ambient air matches the dry-air value better) |
| p_atm | 101 325 Pa | |
| c₀ | 331.3 m/s at 273.15 K | c(T) = c₀√(T/273.15); dry air. Humidity raises c slightly and CO₂ lowers it — this mainly affects the L_d estimate; it largely cancels in e because drone and chanter share the same air |
| T̄ | ~298–308 K | warm air from breath; choose and justify |

### Instrument and acoustics — need research
| Symbol | Meaning | Guidance |
|---|---|---|
| f_c0 | Chanter Low A **at the operating point** (p = p̄, T̄) — the measured playing pitch | Modern Highland pipe chanters are pitched well above concert A (often quoted around 470–480 Hz). **Verify with a source.** |
| f̄_d | Tenor drone frequency | One octave below chanter Low A: f̄_d = ½ f_c0 at equilibrium |
| L_d | Effective tenor drone sounding length | Choose so s̄ is mid-travel: L_d + s̄ + Δ = c(T̄)/(4f̄_d) |
| Δ | End correction | ≈ 0.6 × bore radius (unflanged pipe); find tenor drone bore |
| β_d, β_c | Local slope of reed pitch with pressure at p̄ (Hz/Pa) | **Key unknown.** Search bagpipe / cane-reed acoustics literature; clarinet/oboe reed models as fallback; or measure with a tuner app on real pipes. β_e = β_d − ½β_c sets the pressure budget for the 0.05 Hz target (Section 8.4). |
| p̄ | Playing pressure (gauge) | **Uncertain — verify.** A figure of 3–4 kPa was mentioned in discussion without a source; Highland pipes may play at noticeably higher pressure. Find a measured value (often quoted in inches/cm of water column). |
| p_min, p_max | Reed sounding window | Below p_min reeds stop; above p_max reeds choke/"double". Needed for constraints. p_min > 0. |
| K_d, K_c | Reed flow coefficients | Back-calculate from total airflow at p̄ (find literature on bagpipe air consumption, L/min) and split between drones and chanter. **K_d is the total for all sounding drone reeds** (three if all drones play). |
| V₀ | Bag volume | Find typical Highland bag size (synthetic or hide bags) |
| A_c | Effective arm–bag contact area | Estimate from forearm/arm-pad contact patch |

### Robotic arm / bag wall — estimate and justify
| Symbol | Meaning | Guidance |
|---|---|---|
| M | Effective moving mass (arm + bag wall) | PLACEHOLDER ~0.5–2 kg; depends on actuator design |
| k | Bag wall + arm stiffness | Estimate from bag compliance; PLACEHOLDER. **Must be > 0 and estimated with care:** observability of the bag states from pressure and the slowest bag pole both scale with k (Section 9). Compare it with the air-spring stiffness A_c·a_z. |
| b_a | Arm/bag damping | Choose for a plausible damping ratio (e.g. 0.3–0.7); justify |
| F_max | Max squeeze force | From actuator choice; must cover k·(z̄ + Δz_gap) + A_c·p̄ with margin (Section 7) |
| z range | Max squeeze travel | Bag geometry; must cover z̄ + Δz_gap (Section 7) |
| q_b,max | Max blowpipe/blower flow | Human lung or blower spec; must pass the mean-flow and refill checks (Section 7) with margin |
| t_gap, t_blow | Breath-gap duration and blowing interval between gaps — **random** | Fit distributions to an actual player: e.g. time the breaths in a recording or video of a solo piper, or measure directly if one of the students plays. Use bounded distributions (e.g. truncated normal or lognormal) and report mean, spread, t_gap,max and t_blow,min. Until data is found, t_gap ≈ 1–2 s is a starting estimate. φ̄ = E[t_gap]/(E[t_gap] + E[t_blow]) |

### Slide actuator — use a real datasheet
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

---

## 11. Constraints and uncertainties

### Input constraints (must be simulated and discussed)
- **0 ≤ q_b ≤ q_b,max(t)** (non-return valve: the blowpipe can never remove air). **Breath gaps** are a random, time-varying limit: q_b,max(t) = 0 for a random duration t_gap, separated by random blowing intervals t_blow, with distributions fitted to an actual player (Section 10). Gaps are **not known in advance**; the controller knows a gap has started (and ended) only when it happens, because it is the limit on its own actuator. The controller must respect it, and q_b,max must pass the mean-flow and worst-case refill checks (Section 7).
- **0 ≤ F ≤ F_max**: arm can push but not pull.
- **Squeeze travel limit** z ∈ [0, z_max]: an empty bag cannot be squeezed further, so squeeze cannot sustain pressure indefinitely — over time the blowpipe must supply all outflow. The travel must cover Δz_gap (Section 7).
- **|V_m| ≤ V_max**, and slide travel limits s ∈ [s_min, s_max].

### State/operating constraints
- **p > 0 for all time** (modelling assumption, Section 4). The nonlinear simulation must check it (stop with an error if p ≤ 0, Section 14) rather than clamp p.
- **p_min < p < p_max** (p_min > 0): reed sounding window. The LTI model is only valid for small deviations within this band. The nonlinear simulation must show behaviour at the edges (e.g. a breath gap with too little squeeze lets pressure fall below p_min).

### Model uncertainties (discuss effect on performance; test in simulation)
- Reed pressure sensitivities β_d, β_c (large uncertainty; β_e sets the pressure budget for the 0.05 Hz target).
- Reed flow coefficients K_d, K_c; flow law exponent (≈ ½ assumed).
- Bag stiffness k (also governs bag-state observability and the slowest bag pole), volume V₀, contact area A_c (bag material changes as it warms and wets).
- Temperature T and drone/chanter temperature difference (≈ 0.4 Hz/K in e).
- Reed drift rate (time variance).
- Breath-gap statistics (distribution of t_gap and t_blow; a different player breathes differently).
- Pitch-sensor noise and delay (Section 13).
- Air composition (humidity, CO₂) in R and c₀.
- Unmodelled: reed acoustic dynamics, moisture, leadscrew stiction/backlash, pitch-estimation artefacts.

Recommended robustness tests: vary each uncertain parameter by ±20–50 % in the nonlinear simulation with the nominal controller, including k reduced toward the weakly observable case and the microphone window T_w at the upper end of its sweep.

---

## 12. Handling the time-varying reed drift

The physical system is time-varying (reeds sharpen as they warm and absorb moisture). The assignment requires an **LTI** model, so:

1. **Design model:** keep the LTI plant; represent drift as an **additive slow disturbance** δ_reed(t) on the chanter pitch (and optionally a similar term on the drone reed). In the output equation it enters through D_d (column −½ for chanter drift).
2. **Controller:** include **integral action on the tuning error** (augment the state with ∫e dt). The slide channel already contains a free integrator (the pole at 0, Section 8.3), so with the added integrator the tuning loop is **type 2**: a ramp drift on the output should be rejected with **zero** steady-state error, and the plant integrator alone would already reject a constant drift. The added integrator is still needed for input-side disturbances (leadscrew friction/stiction, load) and robustness. What must be checked is the **transient** tuning error while the drift ramps — it depends on drift rate and loop bandwidth (which the slow microphone measurement limits, Section 13) and must stay within |e| ≤ 0.05 Hz. Confirm all of this in simulation.
3. **Validation:** simulate the nonlinear plant with genuinely drifting parameters (e.g. f_c0 rising by a few Hz over several minutes, β coefficients changing) using the LTI-designed controller.
4. **Report:** state clearly that time variance is outside the LTI model, explain why treating it as a disturbance is valid (drift is much slower than closed-loop bandwidth), and mention **gain scheduling / adaptive control** as an extension.

**Confirm this framing with the tutor** (open question 1).

---

## 13. Sensor model

### Pressure sensor (y₁)
Small additive Gaussian noise; bandwidth much faster than plant — treat as instantaneous.

### Tuning-error sensor (y₂, single microphone listening continuously)
**Students' decision:** one microphone hears the chanter and all drones together and runs continuously. Extracting e from it is part of the project's engineering challenge. The design must respect these facts (state them in the report):

- **Resolution time.** Near the target, the drone's 2nd harmonic and the chanter's Low A are only 2|e| ≤ 0.1 Hz apart. Separating two components that close — or timing the beat between them — takes on the order of 1/(2|e|) ≈ **10 s** of signal (one beat period). Listening continuously gives a continuously updated estimate, but each value summarises the last several seconds, so near the target the measurement behaves like a moving average with an effective delay of several seconds. Larger errors beat faster and are measured faster.
- **Sign.** A beat rate gives |2f_d − f_c| = 2|e|, not whether the drone is sharp or flat. The estimator must recover the sign for e to be a usable linear output. Pipers do this by nudging the drone and listening whether the beats slow down; a small deliberate slide perturbation could do the same.
- **Other sounding pipes.** The untuned drones also sound and share partials with the tuned tenor drone (the second tenor's fundamental, the bass drone's 2nd harmonic), which adds to the estimation challenge.
- **Noise vs delay.** The estimate's noise must be a small fraction of 0.05 Hz (aim ≲ 0.01–0.015 Hz RMS). Longer averaging lowers noise but adds delay; this trade-off is part of the design.

**Simulation sensor model** (idealised; no real DSP — DSP is report discussion only): e_meas(t) = moving average of e over a window T_w (or a first-order lag with time constant ≈ T_w/2) + noise, updated continuously. Treat T_w as a design parameter: baseline T_w ≈ 10 s (one beat period at the target), and sweep it (e.g. 2–20 s) together with the noise level. For LTI design, approximate the window by a delay of T_w/2 (`pade`) or a first-order lag, and discuss the phase-lag limit on bandwidth (Lecture 7 limitations).

**Consequence for control:** with an effective delay of ~5 s, the microphone loop must be slow (crossover roughly ω_c ≲ 1/τ ≈ 0.2 rad/s). That is fast enough for reed drift and temperature (minutes) but **too slow for pressure disturbances** from breath gaps and melody changes (~1 s). Those must be handled through the fast pressure sensor: the pressure loop plus pressure feedforward to the slide (Section 15.2).

---

## 14. Simulation requirements

### 14.1 Open-loop plant simulations (Week 9, A2)
On **both** the LTI model and the nonlinear model:
1. **Step** in each input separately (δq_b, δF, δV_m) — show p, e and states.
2. **Ramp** in V_m or q_b.
3. **Sinusoids** at several frequencies (compare with Bode).
4. **Breath gaps**: q_b forced to 0 for each gap (the input constraint), with constant F — show how far p falls and confirm p stays > 0. Use (a) a **random gap sequence** drawn from the player-fitted distributions with a fixed seed (`rng(seed)`) so runs are reproducible, and (b) a **deterministic worst case**: longest gap t_gap,max followed by the shortest interval t_blow,min, repeated.
5. **Disturbances**: step in δT (and δT_d vs δT_c), chanter-demand square wave δw (a melody), ramp δ_reed.
6. **Linear vs nonlinear comparison** for small and large deviations — show where the linearisation breaks down (especially near p_min / p_max).
7. **With constraints** (saturations, travel limits), **noise**, **delay**, and **parameter uncertainty**.

Use `ode15s` for the nonlinear model (stiff: motor electrical time constant ~ms vs drift ~minutes). Use an `Events` function to **stop the run with an error if p ≤ 0** (the p > 0 assumption is violated — do not clamp) and to flag z or s leaving their travel limits. Use `lsim` / `step` / `initial` for the LTI model.

### 14.2 Closed-loop simulations (Assessment 2)
Scenario "performance": start in tune → chanter melody (δw square-wave sequence) → random breath gaps (player-fitted) → slow reed drift ramp over several minutes → temperature step. Report tuning error in Hz against the |e| ≤ 0.05 Hz target (and the fraction of time inside it), the equivalent beat period 1/(2|e|) against 10 s, pressure deviation, input usage against limits.

Because the breath gaps are random, run the scenario as a **Monte Carlo** set (many seeds) and report statistics across runs — worst-case and percentile values of |e|, pressure deviation and squeeze travel — alongside the deterministic worst-case sequence from Section 14.1.

### 14.3 Plots to produce
Open-loop step responses; Bode plots of plant channels; pole–zero map; closed-loop responses with the ±0.05 Hz band marked on e; S and T magnitude plots for key loops; input signals with saturation limits marked; robustness sweeps (overlaid responses for perturbed parameters).

---

## 15. Controller design guidance

These are suggestions consistent with the course; adapt to the Assessment 2 spec once available.

### 15.1 Control objectives
1. **At most one drone–chanter beat every 10 s**, i.e. keep the tuning error within **|e| ≤ 0.05 Hz** (student requirement, chosen as reflective of the real situation; ≈ 0.36 cents). Justify it as a beat criterion: the ~5–10 cent just-noticeable difference applies to tones heard one after another, but beats between simultaneous tones are audible at far smaller mistuning, which is why pipers tune by ear to remove beats. The target is tight, so the report should include an **error budget** showing that sensor noise, slide positioning (0.05 Hz ≈ 0.075 mm), residual pressure coupling (β_e·δp), drone–chanter temperature difference (≈ 0.4 Hz/K) and drift transients together stay below 0.05 Hz.
2. Keep bag pressure p ≈ p̄, within the sounding window, despite breath gaps and chanter demand changes.
3. Respect all input constraints.
4. Reject slow drift and temperature changes.

### 15.2 Suggested architecture
- **State feedback with integral action:** augment with integrators on e and p (x_aug = [δx; ∫e; ∫δp]), check reachability of the augmented system, place poles (`place`) or use LQR if allowed.
- **Observer:** full-order Luenberger observer from y = [p, e] (`place(A',C',…)'`), poles ~3–10× faster than controller poles but slow enough not to amplify measurement noise / delay.
- **Pressure input allocation (redundant inputs q_b and F):** both act on pressure, but **F → p has a zero at s = 0** (Section 9.4): squeeze cannot hold a steady pressure offset. Therefore:
  - Use only **one** integrator on pressure, and it must act through **q_b** — an integrator on p driven through F alone makes the augmented system unreachable. In a full state-feedback design, check that the steady-state input share falls on q_b. If both inputs integrate pressure error they will drift against each other and one will saturate.
  - **Frequency split (recommended, mirrors real piping):** blowpipe handles low-frequency / average supply (the only input that adds air); squeeze handles fast corrections and covers breath gaps. Show on Bode plots that each input dominates a different band.
  - **Valve-position / mid-ranging control:** a slow loop moves q_b to return z (squeeze position) to z̄, keeping the fast squeeze actuator away from its limits and ready for the next breath gap. This needs z — measured by an arm encoder, or estimated (observable only if k > 0, Section 9.2).
  - Breath gaps are random and **not known in advance**, so the controller cannot pre-squeeze. It does know the instant a gap starts and ends (q_b is forced to 0 on its own actuator), so it can apply **squeeze feedforward at gap onset** (F ramp matched to the expected outflow) and refill with q_b once blowing resumes. Preview (pre-squeezing before a gap) would only be possible if the robot's breath generator announced the next gap — an optional extension; real pipers do anticipate their breaths.
- **Slide loop — two time-scales:**
  - **Fast path (pressure sensor):** **feedforward from measured pressure** to the slide, cancelling β_e·δp so pressure dips during breath gaps and melody changes do not appear in e. The microphone is too slow to see these in time (Section 13).
  - **Slow path (microphone):** tuning-error feedback with integral action trims the slide to remove reed drift, temperature effects and the feedforward's residual error (β_e is uncertain). Its bandwidth is limited by the microphone's effective delay (roughly ω_c ≲ 1/τ, Section 13); with drift over minutes, that is acceptable.
  - In the state-feedback + observer design, include the microphone lag in the design model (e.g. Padé or first-order-lag states) so the observer relies on pressure for fast estimates and on the microphone for slow corrections — check in simulation that it does.
- **Anti-windup** on every integrator (q_b ≥ 0 and q_b,max(t) including breath gaps, F limits, V_m limits, slide travel).
- **Two-degree-of-freedom** (Lecture 5) prefilter F(s) optional for smooth re-tuning steps.

### 15.3 Frequency-domain analysis (Lecture 5–7)
- Bode / loop shaping of the slide (tuning) loop and the pressure loop.
- Plot |S(s)| and |T(s)|; check tracking/disturbance rejection at low frequency and noise attenuation at high frequency. The 0.05 Hz target sets how small |S| must be at the drift and temperature frequencies.
- Discuss fundamental limitations: measurement delay from pitch estimation, the F → p zero at the origin, any RHP zeros found in the channels, actuator limits (including breath gaps).

---

## 16. Known risks

| # | Risk | Mitigation |
|---|---|---|
| 1 | Chanter pitch depends on pressure, so an absolute pitch reference would move with the plant | **Done:** output is tuning error e = f_d − ½f_c, reference e = 0 |
| 2 | Scarce published parameter data for Highland reeds/bags | Search acoustics literature; borrow from clarinet reed models; measure on real pipes if a student plays; document every source/estimate |
| 3 | Linearisation valid only in a narrow pressure window | Show nonlinear vs linear divergence; simulate edge cases |
| 4 | Redundant pressure inputs fight / wind up | F → p has a zero at s = 0: one pressure integrator, acting through q_b; frequency split or mid-ranging; anti-windup |
| 5 | Single-microphone sensing at the target: overlapping partials, ~10 s resolution time, beat rate gives no sign, other drones also sound (part of the engineering challenge) | Model the sensor as a moving average (window T_w) + noise and sweep T_w; slow microphone loop with fast pressure feedforward; discuss estimation methods in the report |
| 6 | Bag states weakly observable from pressure if bag stiffness k is small (det O_bag = a_m·a_z²·k/M) | Estimate k carefully; **check conditioning of obsv first**; add arm position encoder if needed |
| 7 | Badly scaled matrices (Pa vs kg vs m) | Scale variables (Section 8.6) before rank tests and design |
| 8 | Wide time-scale separation (ms to minutes) | Ignore acoustics; treat drift/temperature as slow disturbances; use ode15s |
| 9 | Tutor may not accept drift-as-disturbance | Confirm in next practical |
| 10 | Scope creep (3 drones + chanter = too much in ~4 weeks) | Lock scope to bag + one tenor drone; extensions only if time |
| 11 | Realism: pipers normally tune before playing | Justify as automatic correction during long performances / practice aid; "robotic piper" framing |
| 12 | Uniform temperature cancels exactly in tuning error | Model separate drone/chanter temperatures (Section 8.5) |
| 13 | Target \|e\| ≤ 0.05 Hz (one beat per 10 s) is tight (≈ 0.075 mm slide, ≈ 0.12 K drone–chanter temperature difference, 0.05/\|β_e\| Pa of pressure) | Error budget (Section 15.1); pressure feedforward to the slide; low-backlash leadscrew |
| 14 | Slide model wrong by orders of magnitude if rotor inertia is left out | Use m_eff = m_s + J_m·G² with G = 2πN/ℓ (Section 6.4) |
| 15 | Random breath gaps exceed squeeze capacity or blower flow — worst case is a long gap followed by a short blowing interval | Size q_b,max, z_max and F_max from the Section 7 worst-case checks; confirm with Monte Carlo runs and the deterministic worst-case sequence in the nonlinear model |
| 16 | No player data for breath-gap statistics | Time breaths in recordings/video of solo pipers or measure a student who plays; vary the distributions in robustness tests |

---

## 17. Task list

**Stage 1 — Model (Week 9 deliverable)**
1. Research and fill in every parameter in Section 10 with a citation or justified estimate. Resolve p̄, f_c0, β_d, β_c, K_d, K_c, V₀, k first, and use m_eff (not m_s) for the slide.
2. Choose and justify the equilibrium (Section 7); fit the breath-gap distributions to an actual player; verify feasibility against input limits, including the breath-gap checks (mean flow, worst-case refill, squeeze travel and force).
3. Implement the nonlinear model f(x,u,d), h(x,u,d) in MATLAB, with an `ode15s` event that stops the run if p ≤ 0.
4. Derive A, B, C, D, E, D_d symbolically (`jacobian`), substitute numbers, cross-check against Section 8 and against finite differences.
5. Scale the model (Section 8.6).
6. Check reachability, observability (rank and conditioning), eigenvalues, channel zeros including the F → p zero at s = 0 (Section 9). **If observability fails or is poorly conditioned, stop and report** — add a squeeze-position sensor.
7. Run all open-loop simulations in Section 14.1 and produce plots.

**Stage 2 — Controller (Weeks 10–11, Assessment 2)**
8. Obtain the Assessment 2 specification from the students; align the design with its criteria.
9. Design the pressure loop with input allocation (Section 15.2) and anti-windup.
10. Design state feedback with integral action + observer, plus pressure feedforward to the slide.
11. Frequency-domain analysis: Bode, S(s), T(s), limitations.
12. Closed-loop simulations on the nonlinear plant with constraints, noise, delay, random breath gaps (Monte Carlo + worst-case sequence), drift, temperature (Section 14.2), assessed against |e| ≤ 0.05 Hz (one beat per 10 s) with an error budget, sweeping the microphone window T_w.
13. Robustness sweeps over uncertain parameters (Section 11).
14. Write up: system description, assumptions, model, parameters with sources, linearisation, requirement checklist (Section 1 table), structural properties, simulations, controller design, discussion of constraints, uncertainty and time variance, limitations and extensions.

**Deliverables expected from the agent:** MATLAB scripts (parameters file, nonlinear model function, linearisation script, analysis script, simulation scripts / Simulink model), figures, and a report draft.

---

## 18. Open questions

1. Does the tutor accept modelling reed drift as a slow disturbance (keeping the plant LTI)? Ask at the next practical.
2. Does a "robotic piper" (robotic arm squeezing the bag + motorised drone slide) satisfy the group's robotics goal?
3. What exactly does the Assessment 2 specification require (report format, length, required design methods — e.g. pole placement vs LQR, observer required, frequency-domain analysis required)? Obtain the document.
4. Does either student play the pipes or have access to a set? Direct measurements of pitch vs slide position and pitch vs pressure would greatly strengthen requirement 3.
5. Which player's breathing will the breath-gap distributions be fitted to — a recording/video of a solo piper, or measurements of a student who plays? (Random, player-modelled gaps are decided — Section 3, decision 9 — but the distributions need a source.)

Resolved by the students: blowpipe flow is robot-controlled with random breath gaps modelled on an actual player; pitch sensing is a single microphone listening continuously; the tuning target is at most one beat every 10 s (|e| ≤ 0.05 Hz).
