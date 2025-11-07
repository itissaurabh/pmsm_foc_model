# PMSM Motor Control with FOC and Hall Effect Sensors
## Theory and Implementation Guide

---

## Table of Contents
1. [Introduction](#introduction)
2. [PMSM Motor Theory](#pmsm-motor-theory)
3. [Field Oriented Control (FOC) Theory](#field-oriented-control-foc-theory)
4. [Hall Effect Sensors](#hall-effect-sensors)
5. [Mathematical Transformations](#mathematical-transformations)
6. [Control System Architecture](#control-system-architecture)
7. [Implementation Steps](#implementation-steps)
8. [Simulink Model Structure](#simulink-model-structure)
9. [Parameter Tuning Guide](#parameter-tuning-guide)
10. [Testing and Validation](#testing-and-validation)

---

## 1. Introduction

### What is PMSM?
A Permanent Magnet Synchronous Motor (PMSM) is a type of AC motor where the rotor contains permanent magnets that create a constant magnetic field. The stator contains three-phase windings that generate a rotating magnetic field when energized.

### What is FOC?
Field Oriented Control (FOC), also known as vector control, is an advanced control technique that allows independent control of torque and flux in AC motors, similar to DC motor control. This results in:
- High efficiency
- Precise speed and torque control
- Smooth operation
- Fast dynamic response

### Purpose of Hall Sensors
Hall effect sensors provide discrete position feedback (typically 60° electrical angle resolution) for:
- Commutation timing
- Speed estimation
- Initial rotor position detection
- Cost-effective position sensing (compared to encoders)

---

## 2. PMSM Motor Theory

### 2.1 Motor Equations

The voltage equations for a PMSM in the stator reference frame (abc) are:

```
Va = Rs*ia + La*(dia/dt) + ea
Vb = Rs*ib + Lb*(dib/dt) + eb
Vc = Rs*ic + Lc*(dic/dt) + ec
```

Where:
- Va, Vb, Vc: Phase voltages
- ia, ib, ic: Phase currents
- Rs: Stator resistance
- La, Lb, Lc: Stator inductances
- ea, eb, ec: Back-EMF voltages

### 2.2 Back-EMF

The back-EMF is proportional to rotor speed and is given by:

```
ea = -Ke * ωe * sin(θe)
eb = -Ke * ωe * sin(θe - 2π/3)
ec = -Ke * ωe * sin(θe + 2π/3)
```

Where:
- Ke: Back-EMF constant
- ωe: Electrical angular velocity
- θe: Electrical rotor position

### 2.3 Torque Equation

The electromagnetic torque is:

```
Te = (3/2) * P * λm * iq
```

Where:
- P: Number of pole pairs
- λm: Permanent magnet flux linkage
- iq: q-axis current (torque-producing current)

### 2.4 Mechanical Equations

```
Te - TL = J * (dωm/dt) + B * ωm

θm = ∫ωm dt

θe = P * θm
```

Where:
- TL: Load torque
- J: Moment of inertia
- B: Friction coefficient
- ωm: Mechanical angular velocity
- θm: Mechanical rotor position

---

## 3. Field Oriented Control (FOC) Theory

### 3.1 FOC Principle

FOC decouples the stator current into two components:
- **id (d-axis current)**: Flux-producing component
- **iq (q-axis current)**: Torque-producing component

This decoupling is achieved through coordinate transformations, allowing independent control similar to a DC motor where field and armature currents are naturally decoupled.

### 3.2 Control Strategy

For surface-mounted PMSM (the most common type):
- Set id = 0 (no additional flux needed, permanent magnets provide flux)
- Control iq to control torque

This is called **id = 0 control** and provides:
- Maximum torque per ampere
- Simplified control
- Optimal efficiency

### 3.3 FOC Block Diagram

```
Speed Reference → Speed PI → iq* Reference
                                ↓
                           Current Control
                           (id* = 0, iq*)
                                ↓
                        Inverse Park (dq→αβ)
                                ↓
                        Inverse Clarke (αβ→abc)
                                ↓
                             SVPWM
                                ↓
                            Inverter
                                ↓
                          PMSM Motor
                                ↓
                          Hall Sensors
                                ↓
        Clarke (abc→αβ) ← Current Sensing
                ↓
        Park (αβ→dq)
```

---

## 4. Hall Effect Sensors

### 4.1 Operating Principle

Hall effect sensors detect the presence of a magnetic field. In PMSM applications:
- 3 Hall sensors are placed 120° apart mechanically
- Each sensor outputs HIGH (1) or LOW (0) based on rotor magnet polarity
- This creates 6 distinct states per electrical revolution

### 4.2 Hall Sensor States

For a PMSM, the Hall sensor pattern repeats every electrical cycle:

| Electrical Angle | Hall A | Hall B | Hall C | Sector | Binary |
|------------------|--------|--------|--------|--------|--------|
| 0° - 60°         | 1      | 0      | 1      | 1      | 101    |
| 60° - 120°       | 1      | 0      | 0      | 2      | 100    |
| 120° - 180°      | 1      | 1      | 0      | 3      | 110    |
| 180° - 240°      | 0      | 1      | 0      | 4      | 010    |
| 240° - 300°      | 0      | 1      | 1      | 5      | 011    |
| 300° - 360°      | 0      | 0      | 1      | 6      | 001    |

### 4.3 Position Estimation

From Hall sensors, we can estimate:
- **Sector position**: 60° resolution (0°, 60°, 120°, 180°, 240°, 300°)
- **Speed**: By measuring time between Hall state changes
- **Direction**: By observing Hall state sequence

### 4.4 Position Calculation

```
Hall_State = [Ha, Hb, Hc]

Sector mapping:
101 → 0°
100 → 60°
110 → 120°
010 → 180°
011 → 240°
001 → 300°

Estimated Position θe = Sector_Angle + Interpolation
```

---

## 5. Mathematical Transformations

### 5.1 Clarke Transformation (abc → αβ)

Converts three-phase quantities to two-phase orthogonal stationary frame:

```
[iα]   [1      -1/2      -1/2    ] [ia]
[iβ] = [0   √3/2    -√3/2   ] [ib]
                                    [ic]
```

Or in normalized form (power invariant):

```
iα = (2/3) * (ia - ib/2 - ic/2)
iβ = (2/3) * (√3/2) * (ib - ic)
```

### 5.2 Park Transformation (αβ → dq)

Converts stationary frame to rotating frame aligned with rotor:

```
[id]   [cos(θe)   sin(θe)] [iα]
[iq] = [-sin(θe)  cos(θe)] [iβ]
```

Where θe is the electrical rotor position.

### 5.3 Inverse Park Transformation (dq → αβ)

```
[vα]   [cos(θe)  -sin(θe)] [vd]
[vβ] = [sin(θe)   cos(θe)] [vq]
```

### 5.4 Inverse Clarke Transformation (αβ → abc)

```
va = vα
vb = -vα/2 + (√3/2)*vβ
vc = -vα/2 - (√3/2)*vβ
```

---

## 6. Control System Architecture

### 6.1 Cascade Control Structure

The FOC system uses a cascade control structure:

1. **Outer Loop - Speed Control**
   - Input: Speed reference (ωref)
   - Feedback: Measured speed (ω)
   - Output: Torque reference (iq_ref)
   - Controller: PI controller

2. **Inner Loop - Current Control**
   - Input: Current references (id_ref = 0, iq_ref)
   - Feedback: Measured currents (id, iq)
   - Output: Voltage references (vd_ref, vq_ref)
   - Controller: Two PI controllers (one for d-axis, one for q-axis)

### 6.2 Control Hierarchy

```
Speed Control (Slower, ~1 kHz)
        ↓
Current Control (Faster, ~10 kHz)
        ↓
PWM Generation (Fastest, ~20 kHz)
```

### 6.3 PI Controller Equations

For a discrete PI controller:

```
u(k) = Kp * e(k) + Ki * Σe(k)
```

Where:
- u(k): Controller output at sample k
- e(k): Error at sample k
- Kp: Proportional gain
- Ki: Integral gain

With anti-windup for integral term.

---

## 7. Implementation Steps

### Step 1: Define Motor Parameters

Create a MATLAB script with motor specifications:
- Stator resistance (Rs)
- Stator inductance (Ls)
- Flux linkage (λm)
- Number of pole pairs (P)
- Moment of inertia (J)
- Friction coefficient (B)
- Rated voltage and current

### Step 2: Design Hall Sensor Model

Implement a function that:
- Takes rotor electrical position as input
- Outputs 3 binary Hall signals (Ha, Hb, Hc)
- Maps position to 6 sectors
- Includes optional noise/jitter simulation

### Step 3: Implement Clarke Transformation

Create function blocks:
- Input: Three-phase currents (ia, ib, ic)
- Output: Two-phase currents (iα, iβ)
- Use matrix transformation equations

### Step 4: Implement Park Transformation

Create function blocks:
- Input: (iα, iβ, θe)
- Output: (id, iq)
- Use rotation matrix with rotor position

### Step 5: Design Current Controllers

Implement two PI controllers:
- d-axis controller: Controls id to reference (typically 0)
- q-axis controller: Controls iq to reference (from speed loop)
- Include anti-windup
- Tune gains based on current loop bandwidth

### Step 6: Design Speed Controller

Implement PI controller:
- Input: Speed error
- Output: iq reference (torque command)
- Slower bandwidth than current loop
- Tune gains based on mechanical time constant

### Step 7: Implement Inverse Transformations

Create function blocks:
- Inverse Park: (vd, vq, θe) → (vα, vβ)
- Inverse Clarke: (vα, vβ) → (va, vb, vc)

### Step 8: Implement SVPWM

Space Vector PWM generation:
- Input: (vα, vβ, Vdc)
- Determine sector (1-6)
- Calculate duty cycles
- Output: Three-phase PWM signals

### Step 9: Create PMSM Motor Model

Build motor model using:
- Electrical equations (voltage, current, back-EMF)
- Mechanical equations (torque, speed, position)
- Use Simulink blocks or MATLAB Function

### Step 10: Position Estimation from Hall Sensors

Implement position estimator:
- Read Hall sensor states
- Decode to sector (0-5)
- Calculate electrical angle
- Optional: Interpolate within sector

### Step 11: Integrate Complete System

Connect all blocks in Simulink:
- Motor model
- Hall sensors
- Clarke and Park transformations
- Current and speed controllers
- SVPWM
- Feedback paths

### Step 12: Add Scopes and Visualization

Include monitoring for:
- Motor speed (reference vs actual)
- Phase currents (ia, ib, ic)
- dq currents (id, iq)
- Hall sensor signals
- Torque
- Position

---

## 8. Simulink Model Structure

### 8.1 Top-Level Architecture

```
[PMSM_FOC_Hall_Model.slx]
│
├── Input Block (Speed Reference)
│
├── Speed Controller Block
│   └── PI Controller
│
├── Current Controller Block
│   ├── Clarke Transform
│   ├── Park Transform
│   ├── d-axis PI Controller
│   └── q-axis PI Controller
│
├── Inverse Transforms Block
│   ├── Inverse Park
│   └── Inverse Clarke
│
├── SVPWM Block
│   └── Space Vector Modulation
│
├── Inverter Model
│   └── Three-phase VSI
│
├── PMSM Motor Model
│   ├── Electrical Subsystem
│   └── Mechanical Subsystem
│
├── Hall Sensor Model
│   └── Position to Hall State
│
├── Position Estimator
│   └── Hall State to Position
│
└── Measurement & Display
    ├── Current Scopes
    ├── Speed Scope
    ├── Position Scope
    └── Hall Signal Display
```

### 8.2 Recommended Simulink Settings

- **Solver**: Fixed-step (ode4 or ode3)
- **Step size**: 1e-5 to 1e-6 (10-100 kHz PWM frequency)
- **Simulation time**: 1-5 seconds for startup testing
- **Data logging**: Log key signals for analysis

### 8.3 Subsystem Organization

Organize your model into these subsystems:
1. **FOC_Controller**: Speed and current control
2. **Motor_and_Load**: PMSM model and mechanical load
3. **Hall_Sensors**: Sensor simulation
4. **Position_Estimation**: Hall decoding
5. **Measurements**: Current and voltage sensing
6. **Visualization**: Scopes and displays

---

## 9. Parameter Tuning Guide

### 9.1 Current Controller Tuning

Current loop bandwidth should be ~1/10 of PWM frequency.

For current PI controller:
```
ωcc = 2π * fcc  (current loop bandwidth)
τL = Ls/Rs      (electrical time constant)

Kp_current = Ls * ωcc
Ki_current = Rs * ωcc
```

Typical values:
- PWM frequency: 10-20 kHz
- Current loop bandwidth: 1-2 kHz
- Kp: 0.1 - 1.0
- Ki: 10 - 100

### 9.2 Speed Controller Tuning

Speed loop bandwidth should be ~1/10 of current loop bandwidth.

For speed PI controller:
```
ωsc = 2π * fsc  (speed loop bandwidth)
τm = J*Rs / (Kt^2)  (mechanical time constant)

Kp_speed = J * ωsc
Ki_speed = B * ωsc
```

Typical values:
- Speed loop bandwidth: 50-200 Hz
- Kp: 0.01 - 0.1
- Ki: 0.1 - 10

### 9.3 Tuning Procedure

1. **Start with conservative gains** (low values)
2. **Tune current loop first**:
   - Increase Kp until small oscillations appear
   - Reduce Kp by 20-30%
   - Increase Ki for steady-state accuracy
3. **Then tune speed loop**:
   - Similar procedure as current loop
   - Monitor for overshoot and oscillations
4. **Test with load variations**
5. **Verify stability margins**

---

## 10. Testing and Validation

### 10.1 Initial Tests

1. **Open-loop test**: Apply fixed voltages, verify motor spins
2. **Hall sensor test**: Verify correct state transitions
3. **Position estimation**: Compare estimated vs actual position
4. **Current control test**: Step id and iq references
5. **Speed control test**: Step speed reference

### 10.2 Performance Metrics

Evaluate:
- **Rise time**: Time to reach 90% of reference
- **Settling time**: Time to stay within 5% of reference
- **Overshoot**: Maximum deviation above reference
- **Steady-state error**: Final error after settling
- **Current ripple**: Peak-to-peak variation
- **Efficiency**: Output power / Input power

### 10.3 Common Issues and Solutions

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| Motor vibrates | Incorrect Hall sensor mapping | Verify Hall state sequence |
| Poor speed tracking | Speed controller gains too low | Increase Kp and Ki |
| Current oscillations | Current controller gains too high | Reduce Kp |
| Motor doesn't start | Wrong commutation sequence | Check Hall sensor polarity |
| Position jumps | Hall sensor noise | Add filtering or debouncing |
| Instability under load | Insufficient damping | Increase proportional gain |

### 10.4 Simulation Scenarios

Test your model with:
1. **No-load startup**: Ramp speed from 0 to rated speed
2. **Load step**: Apply sudden load torque
3. **Speed step**: Step change in speed reference
4. **Speed reversal**: Change direction of rotation
5. **Variable load**: Sinusoidal or random load torque

---

## 11. Advanced Topics

### 11.1 Sensorless Control

While this model uses Hall sensors, advanced FOC can estimate position from:
- Back-EMF observation
- Sliding mode observers
- Extended Kalman filters

### 11.2 Maximum Torque Per Ampere (MTPA)

For optimal efficiency, the current vector magnitude should be minimized:
```
id = 0 (for surface PMSM)
iq = desired torque / Kt
```

### 11.3 Field Weakening

For speeds above rated speed:
- Inject negative id current
- Reduces flux, allows higher speed
- Reduces available torque

### 11.4 Dead-time Compensation

Real inverters have dead-time to prevent shoot-through:
- Causes voltage distortion
- Can be compensated in software

---

## 12. References and Further Reading

### Books
1. "Permanent Magnet Synchronous and Brushless DC Motor Drives" - R. Krishnan
2. "Vector Control and Dynamics of AC Drives" - D.W. Novotny and T.A. Lipo
3. "Advanced Electrical Drives" - Rik De Doncker

### Application Notes
- Texas Instruments: "Field Oriented Control of 3-Phase AC-Motors"
- STMicroelectronics: "PMSM FOC SDK Documentation"
- Microchip: AN1078 "Sensorless Field Oriented Control"

### Standards
- IEC 60034: Rotating electrical machines
- IEEE 112: Standard Test Procedure for Polyphase Induction Motors

---

## Appendix A: Nomenclature

| Symbol | Description | Unit |
|--------|-------------|------|
| Rs | Stator resistance | Ω |
| Ls, Ld, Lq | Stator inductances | H |
| λm | Permanent magnet flux linkage | Wb |
| P | Number of pole pairs | - |
| J | Moment of inertia | kg·m² |
| B | Friction coefficient | N·m·s/rad |
| θe | Electrical rotor position | rad |
| θm | Mechanical rotor position | rad |
| ωe | Electrical angular velocity | rad/s |
| ωm | Mechanical angular velocity | rad/s |
| Te | Electromagnetic torque | N·m |
| TL | Load torque | N·m |
| id, iq | d-q axis currents | A |
| vd, vq | d-q axis voltages | V |
| Vdc | DC-link voltage | V |

---

## Appendix B: MATLAB Function Templates

### B.1 Clarke Transformation

```matlab
function [i_alpha, i_beta] = clarke_transform(i_a, i_b, i_c)
    % Clarke transformation (abc to alpha-beta)
    % Power invariant form
    i_alpha = (2/3) * (i_a - 0.5*i_b - 0.5*i_c);
    i_beta = (2/3) * (sqrt(3)/2) * (i_b - i_c);
end
```

### B.2 Park Transformation

```matlab
function [i_d, i_q] = park_transform(i_alpha, i_beta, theta_e)
    % Park transformation (alpha-beta to dq)
    cos_theta = cos(theta_e);
    sin_theta = sin(theta_e);

    i_d = cos_theta * i_alpha + sin_theta * i_beta;
    i_q = -sin_theta * i_alpha + cos_theta * i_beta;
end
```

### B.3 Hall Sensor Decoder

```matlab
function theta_est = hall_to_position(Ha, Hb, Hc)
    % Decode Hall sensor states to electrical position
    hall_state = 4*Ha + 2*Hb + Hc;

    switch hall_state
        case 5  % 101
            theta_est = 0;
        case 4  % 100
            theta_est = pi/3;
        case 6  % 110
            theta_est = 2*pi/3;
        case 2  % 010
            theta_est = pi;
        case 3  % 011
            theta_est = 4*pi/3;
        case 1  % 001
            theta_est = 5*pi/3;
        otherwise
            theta_est = 0; % Invalid state
    end
end
```

---

## Conclusion

This document provides a comprehensive foundation for understanding and implementing PMSM FOC control with Hall effect sensors. The combination of theoretical knowledge and practical implementation steps should enable you to:

1. Understand the physics and mathematics behind PMSM and FOC
2. Implement a complete FOC system in Simulink
3. Integrate Hall sensors for position feedback
4. Tune controller parameters for optimal performance
5. Test and validate your system

Remember that motor control is an iterative process - start simple, test thoroughly, and add complexity gradually.

**Good luck with your PMSM FOC implementation!**
