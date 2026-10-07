# ELEC3304 Assessment 2 — Robotic Bagpipe Pressure & Drone-Tuning Control System
## Handover specification for the implementing agent

**Course:** ELEC3304 Control, Semester 2, 2026
**Group size:** 2 students (fixed for the rest of semester)
**Prepared:** 8 October 2026 (end of Week 9 practical)
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
8. **Scope is limited** to the bag + one tenor drone (6 states). Extensions (all three drones, chanter dynamics) are optional only if time permits.

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
        e = f_d − ½ f_c   (measured by microphone)
```

**In scope:** bag air mass, squeezing arm and bag wall (mass–spring–damper), one tenor drone with motorised leadscrew slide, chanter as a pressure-dependent pitch source (algebraic, no states), reed outflow.

**Out of scope (possible extensions):** second tenor drone and bass drone (each adds 3 slide states), chanter tape, reed acoustic dynamics (kHz, far faster than the control bandwidth), bag thermodynamics beyond an isothermal ideal-gas assumption, moisture.

**Modelling assumptions (state these in the report):**
- Air in the bag is an ideal gas at uniform temperature T; isothermal on the control time-scale.
- Bag volume changes only through arm squeeze displacement z: V = V₀ − A·z (A = effective contact area).
- Reed outflow is orifice-like: mass flow ∝ √(gauge pressure).
- Arm + bag wall behaves as a linear mass–spring–damper driven by the arm force and loaded by the bag pressure.
- Drone is a cylindrical pipe closed at the reed end: quarter-wave resonator.
- Pitch depends instantaneously (algebraically) on pressure, slide position and temperature.
- Leadscrew slide moves horizontally (no gravity load); friction is viscous.

---

## 5. States, inputs, outputs, disturbances

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

### Input vector (p = 3)
u = [q_b, F, V_m]ᵀ

| Input | Symbol | Units | Meaning | Constraint |
|---|---|---|---|---|
| u₁ | q_b | kg/s | Air mass flow in through blowpipe | 0 ≤ q_b ≤ q_b,max (one-way valve; periodic breath gaps for a human-like piper) |
| u₂ | F | N | Squeeze force applied by arm to bag | 0 ≤ F ≤ F_max (can only push) |
| u₃ | V_m | V | Slide motor voltage | \|V_m\| ≤ V_max |

### Output vector (q = 2)
y = [p, e]ᵀ

| Output | Symbol | Units | Sensor |
|---|---|---|---|
| y₁ | p | Pa (gauge) | Bag pressure sensor |
| y₂ | e | Hz (or cents — preferred for scaling) | Microphone + pitch estimation: tuning error between tenor drone and half the chanter Low A pitch |

2 sensors < 6 states ✔ (requirement 2).

### Disturbances
| Disturbance | Symbol | Units | Effect | Time-scale |
|---|---|---|---|---|
| Air temperature | T (deviation δT) | K | Changes pressure (gas law) and pitch (speed of sound) | minutes |
| Chanter air demand | w | – (fractional change) | Different fingered notes draw different flow from the bag | ~0.1–1 s (melody) |
| Reed pitch drift | δ_reed | Hz | Chanter pitch creeps upward as the reed warms/wets | minutes (ramp-like) |
| Breath gaps | (acts through q_b) | – | q_b → 0 while the piper inhales | ~1–3 s, periodic |

Optional refinement: drones and chanter warm at different rates, so model separate temperatures T_d (drone air column) and T_c (chanter); only their **difference** strongly affects the tuning error (see Section 8.4).

---

## 6. Nonlinear model

### Constants and parameters
R = 287 J/(kg·K) (specific gas constant of air), p_atm = 101 325 Pa. Other symbols are defined in Section 10.

### 6.1 Bag pressure (algebraic)
Bag volume: V(z) = V₀ − A·z

Gauge pressure (ideal gas):

  p(m, z, T) = m·R·T / (V₀ − A·z) − p_atm

### 6.2 Air mass balance

  ṁ = q_b − Q_out(p, w)

  Q_out(p, w) = (K_d + K_c·(1 + w))·√p

K_d: drone reed flow coefficient; K_c: chanter reed flow coefficient (units kg·s⁻¹·Pa^−½). w scales chanter demand with fingering.

### 6.3 Arm and bag wall (mass–spring–damper loaded by pressure)

  M·z̈ = F − k·z − c·ż − A·p(m, z, T)

M: effective moving mass of arm + bag wall; k: bag/arm stiffness; c: damping. The term A·p is the bag pushing back on the arm.

### 6.4 Tuning slide (leadscrew)

  m_s·s̈ = G·K_t·i − b_s·ṡ,  with G = 2π/ℓ

m_s: effective slide mass reflected through the leadscrew (include rotor inertia: m_s,eff = m_s + J_m·G²); ℓ: leadscrew lead (m/rev); K_t: motor torque constant; b_s: viscous friction.

### 6.5 Slide motor (armature circuit)

  L·di/dt = V_m − R_a·i − K_e·G·ṡ

R_a: armature resistance; L: armature inductance; K_e: back-EMF constant (K_e = K_t in SI units).

### 6.6 Pitch models (algebraic)
Speed of sound: c(T) = c₀·√(T / 273.15), c₀ ≈ 331.3 m/s

Tenor drone (quarter-wave, closed at reed end) plus reed pressure sensitivity:

  f_d = c(T_d) / [4·(L_d + s + Δ)] + β_d·p

L_d: drone sounding length at s = 0; Δ: end correction (≈ 0.6 × bore radius for an unflanged open end); β_d: drone reed pressure sensitivity (Hz/Pa).

Chanter Low A:

  f_c = f_c0·c(T_c)/c(T̄) + β_c·p + δ_reed

f_c0: nominal chanter Low A at reference temperature and zero pressure deviation; β_c: chanter reed pressure sensitivity (Hz/Pa).

### 6.7 Outputs

  y₁ = p(m, z, T)
  y₂ = e = f_d − ½·f_c

Optional cents form (preferred for scaling): e_cents = 1200·log₂( f_d / (½ f_c) ), which linearises to e_cents ≈ (1200 / ln 2)·(δf_d − ½δf_c) / f̄_d.

### 6.8 Full nonlinear system (compact form)
ẋ₁ = q_b − (K_d + K_c(1+w))·√p(x₁, x₂, T)
ẋ₂ = x₃
ẋ₃ = [F − k·x₂ − c·x₃ − A·p(x₁, x₂, T)] / M
ẋ₄ = x₅
ẋ₅ = [G·K_t·x₆ − b_s·x₅] / m_s
ẋ₆ = [V_m − R_a·x₆ − K_e·G·x₅] / L

Note the **block structure**: the bag subsystem (x₁–x₃) and slide subsystem (x₄–x₆) are dynamically decoupled; they couple only through the tuning-error output, where pressure affects both pitches.

---

## 7. Equilibrium

Choose operating values (from research, Section 10):
- p̄: steady playing pressure (gauge)
- z̄: nominal squeeze displacement (mid-range, leaving room to squeeze and release)
- T̄: nominal air temperature; w̄ = 0; δ̄_reed = 0

Then:

| Quantity | Equilibrium value | From |
|---|---|---|
| V̄ | V₀ − A·z̄ | volume definition |
| m̄ | (p̄ + p_atm)·V̄ / (R·T̄) | gas law |
| q̄_b | (K_d + K_c)·√p̄ | ṁ = 0 (blowpipe supplies all outflow) |
| F̄ | k·z̄ + A·p̄ | z̈ = 0, ż = 0 |
| s̄ | solves e = 0: c(T̄)/[4(L_d + s̄ + Δ)] + β_d p̄ = ½(f_c0 + β_c p̄) | tuning condition |
| ṡ̄, ī, V̄_m | 0, 0, 0 | slide at rest, no static load |
| f̄_d | ½·f̄_c (in tune) | |

Check q̄_b ≤ q_b,max and 0 < F̄ < F_max so the equilibrium is feasible with margin in both directions. Note that in steady state **only the blowpipe can supply air**; squeeze can only change pressure transiently (z is bounded).

---

## 8. Linearisation and LTI matrices

Deviation variables: δx = x − x̄, δu = u − ū, δd = d − d̄, δy = y − ȳ.

LTI model:

  δẋ = A·δx + B·δu + E·δd
  δy = C·δx + D·δu + F_d·δd

with δd = [δT, δw, δ_reed]ᵀ (or [δT_bag, δT_d, δT_c, δw, δ_reed]ᵀ if temperatures are separated).

### 8.1 Partial derivatives of pressure (evaluated at equilibrium)

  a_m = ∂p/∂m = R·T̄ / V̄
  a_z = ∂p/∂z = (p̄ + p_atm)·A / V̄
  a_T = ∂p/∂T = (p̄ + p_atm) / T̄

### 8.2 Outflow derivatives

  g = ∂Q_out/∂p = (K_d + K_c) / (2√p̄)
  g_w = ∂Q_out/∂w = K_c·√p̄

### 8.3 State and input matrices

```
      ⎡ −g·a_m        −g·a_z               0      0     0              0        ⎤
      ⎢   0              0                 1      0     0              0        ⎥
A  =  ⎢ −A·a_m/M   −(k + A·a_z)/M        −c/M     0     0              0        ⎥
      ⎢   0              0                 0      0     1              0        ⎥
      ⎢   0              0                 0      0   −b_s/m_s     G·K_t/m_s    ⎥
      ⎣   0              0                 0      0  −K_e·G/L      −R_a/L       ⎦

      ⎡ 1     0      0   ⎤
      ⎢ 0     0      0   ⎥
B  =  ⎢ 0    1/M     0   ⎥
      ⎢ 0     0      0   ⎥
      ⎢ 0     0      0   ⎥
      ⎣ 0     0     1/L  ⎦
```

Physical notes:
- −g·a_m: more air → higher pressure → more outflow (self-regulating; stable pole).
- −(k + A·a_z)/M: squeezing raises pressure, which pushes back, so the trapped air acts as an extra spring of stiffness A·a_z.
- s has no restoring force (leadscrew), so the slide block has a **pole at 0** (integrator from velocity to position).

### 8.4 Output matrices

Define:
  β_e = β_d − ½·β_c   (net pressure sensitivity of the tuning error)
  h_s = c(T̄) / [4·(L_d + s̄ + Δ)²]   (drone pitch sensitivity to slide extension, Hz/m)

```
      ⎡ a_m        a_z       0     0     0    0 ⎤
C  =  ⎣ β_e·a_m    β_e·a_z   0   −h_s    0    0 ⎦

D  =  0 (2×3)
```

(Multiply row 2 by (1200/ln 2)/f̄_d for an output in cents.)

### 8.5 Disturbance matrices

State disturbance matrix (columns δT, δw, δ_reed):

```
      ⎡ −g·a_T       −g_w     0 ⎤
      ⎢   0            0      0 ⎥
E  =  ⎢ −A·a_T/M       0      0 ⎥
      ⎢   0            0      0 ⎥
      ⎢   0            0      0 ⎥
      ⎣   0            0      0 ⎦
```

Output disturbance matrix:

```
       ⎡ a_T          0     0  ⎤
F_d =  ⎣ e_T          0   −½   ⎦
```

with e_T = ∂e/∂T. With a single shared temperature:

  e_T = f̄_d,ac/(2T̄) − ½·f_c0/(2T̄) + β_e·a_T,  where f̄_d,ac = c(T̄)/[4(L_d+s̄+Δ)]

Because the instrument is in tune (f̄_d ≈ ½f̄_c), the acoustic temperature terms nearly **cancel**: a uniform temperature change shifts drone and chanter together. The tuning error is therefore mainly driven by the **temperature difference** between drone and chanter air columns. Recommended: use separate T_d and T_c in the nonlinear simulation, giving

  ∂e/∂T_d = f̄_d,ac / (2T̄),  ∂e/∂T_c = −½·f_c0 / (2T̄)

and discuss this in the report.

### 8.6 Scaling (do this before rank checks and pole placement)
Raw magnitudes differ enormously (m ~ 10⁻² kg, z ~ 10⁻² m, p ~ 10³ Pa, flows ~ 10⁻⁴ kg/s, e ~ 1 Hz). Scale each variable by its expected maximum deviation (x = S_x·x̃, u = S_u·ũ, y = S_y·ỹ) so all scaled variables are O(1):
  Ã = S_x⁻¹·A·S_x, B̃ = S_x⁻¹·B·S_u, C̃ = S_y⁻¹·C·S_x.
Use the scaled model for rank tests, pole placement and observer design; report physical units.

### 8.7 MATLAB implementation note
Implement the nonlinear f(x,u,d) and h(x,u,d) symbolically (`syms`) and compute A, B, C, D, E, F_d with `jacobian`, then substitute equilibrium values. **Cross-check against the hand-derived matrices above.** Also verify numerically by finite differences on the nonlinear function.

---

## 9. Structural properties to verify

### 9.1 Reachability (requirement 5)
W = [B, AB, A²B, …, A⁵B]; require rank(W) = 6 (`rank(ctrb(A,B))`).
Expected: full rank. q_b drives m, F drives ż (hence z and, via pressure, m), V_m drives i → ṡ → s.
Also check reachability **per input** (e.g. from q_b alone, from F alone) — useful for the input-allocation discussion.

### 9.2 Observability (needed for observer design)
O = [C; CA; …; CA⁵]; require rank(O) = 6 (`rank(obsv(A,C))`).
- Slide block: observable through e (s is measured; ṡ and i follow through the chain).
- Bag block: the pressure sensor measures a_m·δm + a_z·δz. **Risk:** p alone might not separate "more air" from "squeezed harder". It should be observable because z has its own mechanical dynamics, but this must be checked numerically early. If rank-deficient: add a squeeze position sensor (arm encoder — realistic for a robot arm) and keep sensors < states (3 < 6).
- Check observability from each sensor alone too.

### 9.3 Stability of the open-loop plant
Compute eig(A). Expected: bag block stable (three poles in LHP), slide block one pole at 0 plus two stable motor/slide poles. Discuss internal stability (Lecture 8).

### 9.4 Transfer functions and zeros
Compute the 2×3 transfer matrix (`tf(ss(A,B,C,D))`), Bode plots of each channel, and transmission zeros (`tzero`). Check for any RHP zeros (Lecture 7 limitations). Note the pressure→tuning-error coupling (β_e) — if β_e is significant, bag-pressure disturbances appear directly in the tuning error.

---

## 10. Parameters

**Every value must be researched and cited (or justified as an estimate) in the report** — requirement 3. Values below marked "PLACEHOLDER" are order-of-magnitude starting points only, not verified facts.

### Physical constants (reliable)
| Symbol | Value | Notes |
|---|---|---|
| R | 287 J/(kg·K) | specific gas constant, dry air |
| p_atm | 101 325 Pa | |
| c₀ | 331.3 m/s at 273.15 K | c(T) = c₀√(T/273.15) |
| T̄ | ~298–308 K | warm air from breath; choose and justify |

### Instrument and acoustics — need research
| Symbol | Meaning | Guidance |
|---|---|---|
| f_c0 | Chanter Low A frequency | Modern Highland pipe chanters are pitched well above concert A (often quoted around 470–480 Hz). **Verify with a source.** |
| f̄_d | Tenor drone frequency | One octave below chanter Low A (≈ ½ f_c0) |
| L_d | Effective tenor drone sounding length | Estimate from quarter-wave: L_d + Δ ≈ c/(4f̄_d) |
| Δ | End correction | ≈ 0.6 × bore radius (unflanged pipe); find tenor drone bore |
| β_d, β_c | Reed pitch sensitivity to pressure (Hz/Pa) | **Key unknown.** Search bagpipe / cane-reed acoustics literature; clarinet/oboe reed models as fallback; or measure with a tuner app on real pipes. |
| p̄ | Playing pressure (gauge) | **Uncertain — verify.** A figure of 3–4 kPa was mentioned in discussion without a source; Highland pipes may play at noticeably higher pressure. Find a measured value (often quoted in inches/cm of water column). |
| p_min, p_max | Reed sounding window | Below p_min reeds stop; above p_max reeds choke/"double". Needed for constraints. |
| K_d, K_c | Reed flow coefficients | Back-calculate from total airflow at p̄ (find literature on bagpipe air consumption, L/min) and split between drones and chanter |
| V₀ | Bag volume | Find typical Highland bag size (synthetic or hide bags) |
| A | Effective arm–bag contact area | Estimate from forearm/arm-pad contact patch |

### Robotic arm / bag wall — estimate and justify
| Symbol | Meaning | Guidance |
|---|---|---|
| M | Effective moving mass (arm + bag wall) | PLACEHOLDER ~0.5–2 kg; depends on actuator design |
| k | Bag wall + arm stiffness | Estimate from bag compliance; PLACEHOLDER |
| c | Damping | Choose for a plausible damping ratio (e.g. 0.3–0.7); justify |
| F_max | Max squeeze force | From actuator choice; must exceed F̄ with margin |
| z range | Max squeeze travel | Bag geometry |
| q_b,max | Max blowpipe/blower flow | Human lung or blower spec |

### Slide actuator — use a real datasheet
Pick a specific small DC motor + leadscrew (e.g. a miniature brushed DC gearmotor or linear actuator datasheet) and take values directly from it:
| Symbol | Meaning | Typical order (PLACEHOLDER) |
|---|---|---|
| R_a | Armature resistance | 1–10 Ω |
| L | Armature inductance | 0.1–2 mH |
| K_t = K_e | Torque / back-EMF constant | 0.005–0.03 N·m/A |
| J_m | Rotor inertia | 10⁻⁷–10⁻⁶ kg·m² |
| ℓ | Leadscrew lead | 0.5–2 mm/rev |
| m_s | Slide + drone section mass | ~0.02–0.1 kg |
| b_s | Viscous friction | estimate; leadscrews often have significant friction |
| V_max | Supply voltage | from datasheet |
| s range | Slide travel | real tuning slides move a few cm |

---

## 11. Constraints and uncertainties

### Input constraints (must be simulated and discussed)
- **q_b ≥ 0** (non-return valve): the blowpipe can never remove air. Plus **periodic breath gaps** (q_b = 0 for ~1–2 s every few seconds) if modelling a human-like piper.
- **0 ≤ F ≤ F_max**: arm can push but not pull.
- **Squeeze travel limit** z ∈ [0, z_max]: an empty bag cannot be squeezed further, so squeeze cannot sustain pressure indefinitely — over time the blowpipe must supply all outflow.
- **|V_m| ≤ V_max**, and slide travel limits s ∈ [s_min, s_max].

### State/operating constraints
- **p_min < p < p_max**: reed sounding window. The LTI model is only valid for small deviations within this band. The nonlinear simulation must show behaviour at the edges (e.g. a breath gap with too little squeeze lets pressure fall below p_min).

### Model uncertainties (discuss effect on performance; test in simulation)
- Reed pressure sensitivities β_d, β_c (large uncertainty).
- Reed flow coefficients K_d, K_c; flow law exponent (≈ ½ assumed).
- Bag stiffness k, volume V₀, contact area A (bag material changes as it warms and wets).
- Temperature T and drone/chanter temperature difference.
- Reed drift rate (time variance).
- Unmodelled: reed acoustic dynamics, moisture, leadscrew stiction/backlash, pitch-estimation artefacts.

Recommended robustness tests: vary each uncertain parameter by ±20–50 % in the nonlinear simulation with the nominal controller.

---

## 12. Handling the time-varying reed drift

The physical system is time-varying (reeds sharpen as they warm and absorb moisture). The assignment requires an **LTI** model, so:

1. **Design model:** keep the LTI plant; represent drift as an **additive slow disturbance** δ_reed(t) on the chanter pitch (and optionally a similar term on the drone reed). In the output equation it enters through F_d (column −½ for chanter drift).
2. **Controller:** include **integral action on the tuning error** (augment the state with ∫e dt) so constant or slowly ramping drift is rejected with zero (or small) steady-state error. A ramp disturbance gives a constant steady-state error with a single integrator — quantify it and decide if it is acceptable (e.g. < 2–3 cents).
3. **Validation:** simulate the nonlinear plant with genuinely drifting parameters (e.g. f_c0 rising by a few Hz over several minutes, β coefficients changing) using the LTI-designed controller.
4. **Report:** state clearly that time variance is outside the LTI model, explain why treating it as a disturbance is valid (drift is much slower than closed-loop bandwidth), and mention **gain scheduling / adaptive control** as an extension.

**Confirm this framing with the tutor** (open question 1).

---

## 13. Sensor model

### Pressure sensor (y₁)
Small additive Gaussian noise; bandwidth much faster than plant — treat as instantaneous.

### Tuning-error sensor (y₂, microphone + pitch estimation)
- Pitch estimation needs an analysis window of roughly **20–50 ms**, which introduces a **measurement delay** of about half to one window length. Model as delay τ_m plus noise (e.g. ±0.5–1 cent RMS).
- For LTI design: either ignore the delay and check robustness afterwards, or include a Padé approximation (`pade`) and discuss the phase-lag limit on bandwidth (Lecture 7 limitations).
- **Practical difficulty to discuss (do not implement real DSP):** the drones are tuned in octaves to the chanter, so their harmonics overlap; separating each pitch from one microphone is hard. Alternatives: per-drone contact pickups, or tuning by measuring the **beat frequency** between drone and chanter (beats → 0 when in tune). This is report discussion only; the simulation uses the idealised e + noise + delay.

---

## 14. Simulation requirements

### 14.1 Open-loop plant simulations (Week 9, A2)
On **both** the LTI model and the nonlinear model:
1. **Step** in each input separately (δq_b, δF, δV_m) — show p, e and states.
2. **Ramp** in V_m or q_b.
3. **Sinusoids** at several frequencies (compare with Bode).
4. **Breath-gap pulse train**: q_b dropping to 0 periodically, with constant F.
5. **Disturbances**: step in δT (and δT_d vs δT_c), chanter-demand square wave δw (a melody), ramp δ_reed.
6. **Linear vs nonlinear comparison** for small and large deviations — show where the linearisation breaks down (especially near p_min / p_max).
7. **With constraints** (saturations, travel limits), **noise**, **delay**, and **parameter uncertainty**.

Use `ode15s` for the nonlinear model (stiff: motor electrical time constant ~ms vs drift ~minutes). Use `lsim` / `step` / `initial` for the LTI model.

### 14.2 Closed-loop simulations (Assessment 2)
Scenario "performance": start in tune → chanter melody (δw square-wave sequence) → periodic breath gaps → slow reed drift ramp over several minutes → temperature step. Report pitch error in cents, pressure deviation, input usage against limits.

### 14.3 Plots to produce
Open-loop step responses; Bode plots of plant channels; pole–zero map; closed-loop responses; S and T magnitude plots for key loops; input signals with saturation limits marked; robustness sweeps (overlaid responses for perturbed parameters).

---

## 15. Controller design guidance

These are suggestions consistent with the course; adapt to the Assessment 2 spec once available.

### 15.1 Control objectives
1. Keep tuning error e ≈ 0 (target: within ±2–3 cents; ~5 cents is a typical audibility threshold for beating — verify/justify).
2. Keep bag pressure p ≈ p̄, within the sounding window, despite breath gaps and chanter demand changes.
3. Respect all input constraints.
4. Reject slow drift and temperature changes.

### 15.2 Suggested architecture
- **State feedback with integral action:** augment with integrators on e and p (x_aug = [δx; ∫e; ∫δp]), check reachability of the augmented system, place poles (`place`) or use LQR if allowed.
- **Observer:** full-order Luenberger observer from y = [p, e] (`place(A',C',…)'`), poles ~3–10× faster than controller poles but slow enough not to amplify measurement noise / delay.
- **Pressure input allocation (redundant inputs q_b and F):** both act on pressure; if both loops integrate pressure error they will drift against each other and one will saturate. Options:
  - **Frequency split (recommended, mirrors real piping):** blowpipe handles low-frequency / average supply (the only input that adds air); squeeze handles fast corrections. Show on Bode plots that each input dominates a different band.
  - **Valve-position / mid-ranging control:** a slow loop moves q_b to return z (squeeze position) to its mid-range z̄, keeping the fast squeeze actuator away from its limits.
  - Only **one** integrator on pressure.
- **Slide loop:** tuning error → slide position (via motor). Bandwidth well below the pitch-measurement delay limit.
- **Anti-windup** on every integrator (q_b ≥ 0, F limits, V_m limits, slide travel).
- **Two-degree-of-freedom** (Lecture 5) prefilter optional for smooth re-tuning steps.

### 15.3 Frequency-domain analysis (Lecture 5–7)
- Bode / loop shaping of the slide (tuning) loop and the pressure loop.
- Plot |S| and |T|; check tracking/disturbance rejection at low frequency and noise attenuation at high frequency.
- Discuss fundamental limitations: measurement delay from pitch estimation, any RHP zeros found by `tzero`, actuator limits.

---

## 16. Known risks

| # | Risk | Mitigation |
|---|---|---|
| 1 | Chanter pitch depends on pressure, so an absolute pitch reference would move with the plant | **Done:** output is tuning error e = f_d − ½f_c, reference e = 0 |
| 2 | Scarce published parameter data for Highland reeds/bags | Search acoustics literature; borrow from clarinet reed models; measure on real pipes if a student plays; document every source/estimate |
| 3 | Linearisation valid only in a narrow pressure window | Show nonlinear vs linear divergence; simulate edge cases |
| 4 | Redundant pressure inputs fight / wind up | Frequency split or mid-ranging; one pressure integrator; anti-windup |
| 5 | Pitch sensing: overlapping harmonics; 20–50 ms estimation delay | Model sensor as e + noise + delay; discuss alternatives (pickups, beat frequency) |
| 6 | Bag states may be weakly observable from pressure alone | **Check rank(obsv) first**; add arm position encoder if needed |
| 7 | Badly scaled matrices (Pa vs kg vs m) | Scale variables (Section 8.6) before rank tests and design |
| 8 | Wide time-scale separation (ms to minutes) | Ignore acoustics; treat drift/temperature as slow disturbances; use ode15s |
| 9 | Tutor may not accept drift-as-disturbance | Confirm in next practical |
| 10 | Scope creep (3 drones + chanter = too much in ~4 weeks) | Lock scope to bag + one tenor drone; extensions only if time |
| 11 | Realism: pipers normally tune before playing | Justify as automatic correction during long performances / practice aid; "robotic piper" framing |
| 12 | Uniform temperature cancels in tuning error | Model separate drone/chanter temperatures (Section 8.5) |

---

## 17. Task list

**Stage 1 — Model (Week 9 deliverable)**
1. Research and fill in every parameter in Section 10 with a citation or justified estimate. Resolve p̄, f_c0, β_d, β_c, K_d, K_c, V₀ first.
2. Choose and justify the equilibrium (Section 7); verify feasibility against input limits.
3. Implement the nonlinear model f(x,u,d), h(x,u,d) in MATLAB.
4. Derive A, B, C, D, E, F_d symbolically (`jacobian`), substitute numbers, cross-check against Section 8 and against finite differences.
5. Scale the model (Section 8.6).
6. Check reachability, observability, eigenvalues, transmission zeros (Section 9). **If observability fails, stop and report** — add a squeeze-position sensor.
7. Run all open-loop simulations in Section 14.1 and produce plots.

**Stage 2 — Controller (Weeks 10–11, Assessment 2)**
8. Obtain the Assessment 2 specification from the students; align the design with its criteria.
9. Design the pressure loop with input allocation (Section 15.2) and anti-windup.
10. Design state feedback with integral action + observer.
11. Frequency-domain analysis: Bode, S, T, limitations.
12. Closed-loop simulations on the nonlinear plant with constraints, noise, delay, breath gaps, drift, temperature (Section 14.2).
13. Robustness sweeps over uncertain parameters (Section 11).
14. Write up: system description, assumptions, model, parameters with sources, linearisation, requirement checklist (Section 1 table), structural properties, simulations, controller design, discussion of constraints, uncertainty and time variance, limitations and extensions.

**Deliverables expected from the agent:** MATLAB scripts (parameters file, nonlinear model function, linearisation script, analysis script, simulation scripts / Simulink model), figures, and a report draft.

---

## 18. Open questions

1. Does the tutor accept modelling reed drift as a slow disturbance (keeping the plant LTI)? Ask at the next practical.
2. Does a "robotic piper" (robotic arm squeezing the bag + motorised drone slide) satisfy the group's robotics goal?
3. What exactly does the Assessment 2 specification require (report format, length, required design methods — e.g. pole placement vs LQR, observer required, frequency-domain analysis required)? Obtain the document.
4. Does either student play the pipes or have access to a set? Direct measurements of pitch vs slide position and pitch vs pressure would greatly strengthen requirement 3.
5. Human piper (blowpipe with breath gaps) or mechanical blower (continuous flow)? The handover assumes a breath-like blowpipe flow so the squeeze has a clear role; confirm.
