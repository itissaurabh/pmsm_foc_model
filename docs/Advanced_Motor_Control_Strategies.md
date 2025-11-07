# Advanced Motor Control Strategies for PMSM Drives

**Field Weakening, Regenerative Braking, Derating, and System Integration**

---

## Table of Contents

### 1. Field Weakening Control
   - 1.1 Theory and Mathematics of Field Weakening
   - 1.2 Control Implementation
   - 1.3 Electronics Considerations
   - 1.4 Vehicle Towing and Safety
   - 1.5 Practical Implementation Guidelines

### 2. Regenerative Braking
   - 2.1 Physics of Regenerative Braking
   - 2.2 Motor Behavior During Regeneration
   - 2.3 Regen Control Strategies
   - 2.4 Battery Management During Regen
   - 2.5 User-Selectable Regen Levels

### 3. Braking Strategy and Blending
   - 3.1 Algorithmic Braking Strategies
   - 3.2 Mechanical vs Electrical Braking
   - 3.3 Brake Blending for User Experience
   - 3.4 Safety Considerations

### 4. Hill Hold Control
   - 4.1 Hill Hold Theory
   - 4.2 Implementation Strategies
   - 4.3 Integration with Braking System

### 5. Temperature-Based Derating
   - 5.1 Motor Temperature Derating
   - 5.2 Controller Temperature Derating
   - 5.3 Derating Curves and Implementation
   - 5.4 Thermal Runaway Prevention

### 6. CAN Communication and Monitoring
   - 6.1 Essential CAN Parameters
   - 6.2 Debug and Diagnostic Messages
   - 6.3 Performance Monitoring
   - 6.4 Fault Reporting

### 7. References and Resources

---

## Introduction

This document builds upon the hardware design foundation to cover advanced motor control strategies essential for high-performance PMSM drives in electric vehicles and industrial applications. These topics address:

- **Extended speed range** through field weakening
- **Energy recovery** via regenerative braking
- **System protection** through intelligent derating
- **User experience** with advanced braking strategies
- **System integration** via comprehensive monitoring

These strategies are critical for:
- Electric vehicles (EVs, e-bikes, e-scooters)
- High-performance servo drives
- Robotics and autonomous systems
- Industrial automation

**Prerequisites:**
- Understanding of FOC (Field Oriented Control)
- Basic knowledge of PMSM operation
- Familiarity with power electronics (see Power Electronics Hardware Design document)

---

## 1. Field Weakening Control

### 1.1 Theory and Mathematics of Field Weakening

Field weakening is a control strategy that extends the motor's speed range beyond the base speed by reducing the magnetic flux in the motor. This allows higher speeds at the cost of reduced torque.

**The Fundamental Problem:**

```
At base speed, the motor reaches voltage limit:

V_limit = √(Vd² + Vq²) ≤ Vdc / √3

Where:
  Vd, Vq = voltage commands in dq frame
  Vdc = DC bus voltage

As speed increases:
  - Back-EMF increases proportionally: E = ω × λm
  - Required voltage to overcome back-EMF increases
  - At some speed (base speed), we hit the voltage limit
  - Cannot go faster with standard FOC!
```

**The Voltage Equation in dq Frame:**

```
Vd = Rs × id + Ld × (did/dt) - ω × Lq × iq
Vq = Rs × iq + Lq × (diq/dt) + ω × Ld × id + ω × λm

At steady state (did/dt = diq/dt = 0):

Vd = Rs × id - ω × Lq × iq
Vq = Rs × iq + ω × Ld × id + ω × λm

For surface-mounted PMSM (Ld ≈ Lq = L):

Vd = Rs × id - ω × L × iq
Vq = Rs × iq + ω × L × id + ω × λm

Voltage magnitude:
V = √(Vd² + Vq²)
```

**Base Speed Definition:**

```
Base speed (ωbase) is when voltage limit is reached with:
  - id = 0 (maximum torque per ampere, MTPA)
  - iq = I_rated

Vq_max = Rs × iq + ωbase × λm ≈ ωbase × λm (neglecting Rs drop)

At voltage limit:
  V_limit = Vq_max

Therefore:
  ωbase = V_limit / λm

Example:
  Vdc = 400V
  V_limit = 400 / √3 = 231V (line-to-neutral peak)
  λm = 0.0118 Wb (flux linkage)

  ωbase = 231 / 0.0118 = 19,576 rad/s = 186,900 RPM (electrical)

  For 4 pole pairs:
    ωbase_mech = 186,900 / 4 = 46,725 RPM (mechanical)
```

**Field Weakening Principle:**

The key insight: **Inject negative d-axis current to oppose the permanent magnet flux!**

```
Total flux linkage in d-axis:
  λd_total = λm + Ld × id

By making id negative:
  λd_total = λm - Ld × |id| < λm

This reduces the back-EMF:
  E_new = ω × λd_total = ω × (λm - Ld × |id|)

Now we can run at higher speed with same voltage limit!
```

**Voltage Circle and Current Circle:**

At any given speed, the operating point must satisfy:

```
1. Voltage constraint (voltage circle):
   Vd² + Vq² ≤ V_limit²

2. Current constraint (current circle):
   id² + iq² ≤ I_limit²

Where I_limit is the maximum current the inverter can supply.
```

**Graphical Representation in id-iq Plane:**

```
Current limit (circle):
  id² + iq² = I_limit²

Voltage limit (ellipse at speed ω):
  (Rs×id - ω×L×iq)² + (Rs×iq + ω×L×id + ω×λm)² = V_limit²

Simplifying (neglecting Rs):
  (ω×L×iq)² + (ω×L×id + ω×λm)² = V_limit²

  (L×iq)² + (L×id + λm)² = (V_limit/ω)²

This is an ellipse centered at (-λm/L, 0)

As speed increases:
  - The ellipse shrinks (radius ∝ 1/ω)
  - Operating point moves toward negative id axis
  - Must inject more negative id to stay within voltage limit
```

**Mathematical Derivation of Field Weakening Region:**

```
Define characteristic current:
  Ich = λm / Ld

Three operating regions:

Region 1: MTPA (Maximum Torque Per Ampere)
  Speed: ω < ωbase
  id = 0 (for surface PMSM)
  iq can be anything up to I_limit
  Torque: T = (3/2) × P × λm × iq

Region 2: Field Weakening (FW)
  Speed: ωbase < ω < ωmax
  id < 0 (negative, weakening field)
  Both voltage and current constraints active

  From voltage constraint:
    (L×id + λm) = -V_limit / ω

    id = -λm/L + V_limit/(ω×L) = -Ich + V_limit/(ω×L)

  From current constraint:
    iq = √(I_limit² - id²)

  Torque: T = (3/2) × P × [λm × iq + (Ld-Lq) × id × iq]
         ≈ (3/2) × P × λm × iq (for Ld ≈ Lq)

Region 3: MTPV (Maximum Torque Per Volt) - if Ld ≠ Lq
  Speed: ω > ω_MTPV
  Voltage constraint dominates
  Operating at voltage limit
  Further speed increase with reduced torque
```

**Torque in Field Weakening Region:**

```
Torque equation:
  T = (3/2) × P × λm × iq

Since iq decreases with speed in FW region:

  iq = √(I_limit² - id²)

And id becomes more negative as speed increases:

  id = -Ich + V_limit/(ω×L)

As ω increases:
  - id becomes more negative
  - iq must decrease (to stay on current circle)
  - Torque decreases hyperbolically

Power remains approximately constant:
  P = T × ω ≈ constant (in ideal case)

This is the "constant power region"
```

**Intuitive Understanding:**

Think of it this way:

1. **Permanent magnets create a fixed magnetic field** in the motor
2. **At high speeds, this creates large back-EMF** that opposes applied voltage
3. **We "fight" this permanent field** by injecting negative d-axis current
4. **This creates an opposing magnetic field** from the stator winding
5. **The net field is reduced**: Net flux = Permanent magnet flux - Induced flux
6. **Reduced flux → reduced back-EMF** → can run at higher speed
7. **Trade-off**: Less flux → less torque capability

**Analogy:**
- Like pressing the brake pedal while accelerating in a car
- You're applying force (id) to oppose the natural tendency (permanent magnet field)
- This allows higher speed but wastes energy (current flows but doesn't produce torque)
- Power is diverted from torque production to field weakening

---

### References for Section 1.1:

**Books:**
1. *"Permanent Magnet Synchronous and Brushless DC Motor Drives"* by R. Krishnan - Chapter 8: Field Weakening
2. *"Vector Control and Dynamics of AC Drives"* by Novotny and Lipo - Advanced control strategies
3. *"Advanced Electric Drives"* by Ned Mohan - Chapter 16: Field Weakening and MTPA

**Papers:**
1. "Field-Weakening in Permanent Magnet Synchronous Motors" by S. Morimoto et al., IEEE Trans. on Industry Applications
2. "Maximum Torque Per Ampere Control of PM Machines" - IEEE ECCE Conference
3. "Flux-Weakening Control for Electric Vehicle Applications" - SAE Technical Paper

**Application Notes:**
1. **Texas Instruments**: "Field Weakening and MTPA Control for PMSM" (SPRABQ7)
2. **Infineon**: "Field Oriented Control with Field Weakening" (AP32370)
3. **STMicroelectronics**: "PMSM Field Weakening Strategy" (AN4680)

---

