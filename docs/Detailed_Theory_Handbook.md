# Detailed Theory Handbook
## PMSM Motor Control with Field Oriented Control

**A Comprehensive Educational Guide for Understanding PMSM Motors, Position Sensors, and Field Oriented Control**

---

## About This Handbook

This handbook is designed as an introductory course notebook for fresh graduates who have basic knowledge of mathematics and electronics but need to build a solid foundation in motor control theory. Think of this as your companion guide through a laboratory course on electric motor drives.

### Who This Is For

- **Fresh graduates** in Electrical/Electronics Engineering
- **Students** taking motor drives or power electronics courses
- **Engineers** transitioning into motor control applications
- **Hobbyists** building motor control projects

### Prerequisites

You should be comfortable with:
- Basic calculus (derivatives, integrals)
- Complex numbers and phasor notation
- AC circuit analysis
- Basic control theory (transfer functions, feedback control)
- Differential equations (basic understanding)

### What You'll Learn

By the end of this handbook, you will understand:
1. How permanent magnet synchronous motors work physically and mathematically
2. Different position sensing methods (Hall sensors, resolvers, encoders)
3. The concept of reference frames and why they matter
4. Clarke and Park transformations and their physical meaning
5. Field Oriented Control algorithm and implementation
6. How to design and tune motor controllers

### How to Use This Handbook

- **Read sequentially** - Each section builds on previous concepts
- **Work through examples** - Don't skip the numerical examples
- **Draw diagrams** - Sketch the concepts as you read
- **Consult references** - Links to deeper materials are provided
- **Experiment** - Use the MATLAB code to visualize concepts

---

## Table of Contents

1. [Introduction and Motivation](#1-introduction-and-motivation)
2. [PMSM Motor Theory](#2-pmsm-motor-theory)
3. [Field Oriented Control (FOC) Theory](#3-field-oriented-control-theory)
4. [Position Sensors for Motor Control](#4-position-sensors)
5. [Mathematical Transformations and Reference Frames](#5-mathematical-transformations)
6. [Control System Architecture](#6-control-system-architecture)
7. [References and Further Reading](#7-references-and-further-reading)

---

## 1. Introduction and Motivation

### 1.1 Why Study Motor Control?

Electric motors are everywhere. They consume approximately **45% of global electricity** and are critical components in:
- Electric vehicles (Tesla, Nissan Leaf, etc.)
- Industrial automation (CNC machines, robots)
- Home appliances (washing machines, air conditioners)
- Aerospace (electric aircraft, drones)
- Medical devices (surgical robots, ventilators)

Modern motor control techniques enable:
- **High efficiency** (>95% in many applications)
- **Precise position/speed control** (micrometers, fractions of RPM)
- **Smooth operation** (low torque ripple, quiet operation)
- **Fast dynamic response** (millisecond-level changes)

### 1.2 Evolution of Motor Control

**1960s-1970s: DC Motor Era**
- Simple to control (voltage → speed)
- Separate field and armature windings
- Problems: Brushes wear out, sparking, limited speed, maintenance

**1980s: Introduction of Vector Control**
- AC motors with DC-like control
- Breakthrough: Clarke and Park transformations
- Complex calculations required powerful microcontrollers

**1990s-2000s: Digital Control Revolution**
- Fast DSPs and microcontrollers
- Real-time FOC implementations
- Cost-effective compared to DC motors

**2010s-Present: Modern Era**
- PMSMs dominate high-performance applications
- Integrated motor drives with advanced algorithms
- Sensorless control, AI-based optimization

### 1.3 Why PMSM?

**Advantages over other motor types:**

| Feature | PMSM | Induction Motor | DC Motor |
|---------|------|-----------------|----------|
| Efficiency | 95-98% | 85-92% | 80-90% |
| Power Density | Excellent | Good | Moderate |
| Maintenance | Low | Low | High (brushes) |
| Control Complexity | High | High | Low |
| Cost | Moderate-High | Low-Moderate | Moderate |
| Rotor Heat | Low (no windings) | High | Moderate |

**Why PMSMs won:**
- Rare-earth magnets (NdFeB) became affordable
- Power electronics advanced (fast IGBTs, MOSFETs)
- Microcontrollers powerful enough for real-time FOC
- Applications demand high efficiency (EVs, appliances)

### 1.4 What Makes Motor Control Challenging?

**Challenge 1: Nonlinear Dynamics**
- Motor equations involve trigonometric functions
- Magnetic saturation effects
- Temperature-dependent parameters

**Challenge 2: Multi-Variable System**
- Three phase currents to control
- Coupled electrical and mechanical dynamics
- Position, speed, and current all interdependent

**Challenge 3: Real-Time Requirements**
- Current control: 10-20 kHz update rate
- Fast ADCs, calculations, PWM updates
- Microsecond-level timing critical

**Challenge 4: Parameter Variations**
- Resistance changes with temperature
- Inductance varies with current (saturation)
- Magnet strength degrades over time

### 1.5 The FOC Solution

**Field Oriented Control transforms the problem:**
- Converts complex 3-phase AC system → Simple 2-phase DC system
- Decouples torque and flux control
- Enables use of simple PI controllers
- Makes AC motor control as easy as DC motor control

**The Magic:** Mathematical transformations (Clarke, Park) that rotate the reference frame to align with the rotor.

---

## 2. PMSM Motor Theory

### 2.1 What Is a PMSM?

A **Permanent Magnet Synchronous Motor (PMSM)** is an AC motor where:
- **Stator**: Contains three-phase windings (like all 3-phase AC motors)
- **Rotor**: Contains permanent magnets (not electromagnets or squirrel cage)
- **Synchronous**: Rotor rotates at exactly the same speed as the stator magnetic field

**Key Physical Principle:**
When you pass AC currents through the three stator windings, they create a rotating magnetic field. The permanent magnets on the rotor try to align with this field, causing rotation.

### 2.2 Physical Construction

#### 2.2.1 Stator Construction

The stator is the stationary outer part of the motor:

```
        North Pole Region
              ^
              |
    Phase A winding (0°)
              |
              |
Phase C ------+------ Phase B
(240°)        |        (120°)
              |
              v
        South Pole Region
```

**Key Features:**
- **Three phase windings** displaced by 120° spatially
- **Laminated steel core** to reduce eddy current losses
- **Distributed windings** to create sinusoidal flux distribution
- **Slots** machined into the core to hold copper windings

**Why three phases?**
- Minimum number for smooth rotating field
- Balanced system (sum of currents = 0)
- Efficient power transfer
- Self-starting capability

#### 2.2.2 Rotor Construction

Two main types of PMSM rotors:

**Surface-Mounted PMSM (SPM):**
```
    ┌─────────────────┐
    │    N  │  S      │  ← Magnets glued on surface
    │       │         │
    │   Iron Core     │
    │       │         │
    │    S  │  N      │
    └─────────────────┘
```

**Characteristics:**
- Magnets mounted on rotor surface
- Ld ≈ Lq (symmetrical inductance)
- Lower cost, simpler construction
- Limited high-speed capability (magnets can fly off)
- Used in: fans, pumps, low-speed applications

**Interior Permanent Magnet (IPM):**
```
    ┌─────────────────┐
    │  ╔═══╗   ╔═══╗  │  ← Magnets buried inside
    │  ║ N ║   ║ S ║  │
    │  ╚═══╝   ╚═══╝  │  ← Iron bridges hold magnets
    │                 │
    └─────────────────┘
```

**Characteristics:**
- Magnets embedded inside rotor
- Ld ≠ Lq (saliency - different inductances in d and q axes)
- More robust, can run at high speeds
- Reluctance torque in addition to magnet torque
- Used in: EVs, high-performance servo drives

**For this handbook, we focus on SPM motors (simpler analysis).**

#### 2.2.3 Permanent Magnets

**Magnet Materials:**

| Material | Remnant Field (Br) | Coercivity | Cost | Temp Rating |
|----------|-------------------|------------|------|-------------|
| Neodymium-Iron-Boron (NdFeB) | 1.0-1.4 T | Excellent | High | 80-200°C |
| Samarium-Cobalt (SmCo) | 0.8-1.1 T | Excellent | Very High | 250-350°C |
| Ferrite (Ceramic) | 0.2-0.4 T | Good | Low | 250°C |
| Alnico | 0.6-1.3 T | Poor | Moderate | 450°C |

**NdFeB magnets dominate modern PMSMs due to:**
- Highest magnetic field strength
- Good cost-performance ratio
- Sufficient temperature rating for most applications

**Magnet Pole Arrangement:**
- Typically 4-8 poles for industrial motors
- More poles → More torque, lower speed
- Fewer poles → Higher speed, less torque
- Pole pairs (P) = Number of poles / 2

### 2.3 Operating Principle

#### 2.3.1 Creating a Rotating Magnetic Field

When we apply three-phase currents to the stator windings:

```
ia(t) = I·cos(ωt)
ib(t) = I·cos(ωt - 120°)
ic(t) = I·cos(ωt - 240°)
```

Each phase creates a pulsating magnetic field along its axis. The **vector sum** of these three fields creates a **rotating magnetic field** with constant magnitude.

**Visualization:**
Imagine three people pushing a merry-go-round, each 120° apart, pushing in a sinusoidal pattern but with 120° phase shift. The combined effect is a smooth rotation.

#### 2.3.2 Synchronous Operation

The rotor permanent magnets try to **align with the stator's rotating field**. In steady-state:
- Rotor speed = Stator field speed (synchronous operation)
- Magnets "locked" to the rotating field
- Load torque causes a **load angle** (δ) between rotor and field

**Mathematical Relationship:**
```
Electrical frequency: fe = (P × n) / 60 Hz
where P = pole pairs, n = mechanical speed (RPM)
```

**Example:**
- Motor with 4 pole pairs
- Running at 1500 RPM
- Electrical frequency = (4 × 1500) / 60 = 100 Hz

**This is critical:** The electrical frequency is P times the mechanical frequency!

### 2.4 Mathematical Model - Voltage Equations

#### 2.4.1 Single Phase Equivalent Circuit

For one phase of the motor, we can write:

```
     Rs        Ls
  ───┬───────┬──────
     │       │
    Va      Ldi/dt    ea (back-EMF)
     │       │        ↓
  ───┴───────┴──────
```

**Voltage Equation:**
```
Va(t) = Rs·ia(t) + Ls·(dia/dt) + ea(t)
```

**Physical Meaning:**
- **Rs·ia**: Voltage drop across winding resistance (copper losses)
- **Ls·(dia/dt)**: Voltage to change the current (inductive effect)
- **ea**: Back-EMF generated by moving magnets

#### 2.4.2 Three-Phase Voltage Equations

For all three phases in the natural abc reference frame:

```
Va = Rs·ia + Ls·(dia/dt) + ea
Vb = Rs·ib + Ls·(dib/dt) + eb
Vc = Rs·ic + Ls·(dic/dt) + ec
```

**Assumptions Made (Important!):**
1. **Balanced three-phase system:** ia + ib + ic = 0
2. **Sinusoidal flux distribution:** No harmonics
3. **Linear magnetic circuit:** No saturation
4. **Symmetrical windings:** La = Lb = Lc = Ls
5. **No mutual inductance variation:** Constant with position
6. **Constant parameters:** Rs, Ls don't change

**What We're Ignoring (for now):**
- Magnetic saturation (Ls decreases at high currents)
- Iron losses (eddy currents, hysteresis)
- Slot harmonics (non-ideal flux distribution)
- Temperature effects (Rs increases with temperature)
- Cogging torque (due to slot geometry)

*We'll address these in advanced studies, but for FOC fundamentals, these assumptions are acceptable.*

#### 2.4.3 Back-EMF Generation

The back-EMF is generated by Faraday's law: a moving magnetic field induces voltage in conductors.

**Back-EMF Equations:**
```
ea(t) = -Ke·ωe·sin(θe)
eb(t) = -Ke·ωe·sin(θe - 2π/3)
ec(t) = -Ke·ωe·sin(θe + 2π/3)
```

**Where:**
- **Ke** = Back-EMF constant [V/(rad/s)] - depends on magnet strength and winding turns
- **ωe** = Electrical angular velocity [rad/s] = P × ωm
- **θe** = Electrical rotor position [rad] = P × θm
- **P** = Number of pole pairs

**Physical Interpretation:**
- Back-EMF is proportional to **speed** (faster → higher voltage)
- Back-EMF is sinusoidal in position (θe)
- Back-EMF **opposes** the applied voltage (Lenz's law)

**Key Insight:**
At no-load, the applied voltage must equal the back-EMF to maintain rotation. Under load, the voltage must also overcome the resistive and inductive drops.

### 2.5 Torque Production

#### 2.5.1 Electromagnetic Torque Equation

For a surface-mounted PMSM, the electromagnetic torque is:

```
Te = (3/2)·P·λm·iq
```

**This is the most important equation in FOC!**

**Where:**
- **Te** = Electromagnetic torque [N·m]
- **P** = Pole pairs
- **λm** = Permanent magnet flux linkage [Wb] - a property of the magnets
- **iq** = q-axis current [A] - the torque-producing current component

**Alternative form using Kt (torque constant):**
```
Te = Kt·iq

where Kt = (3/2)·P·λm
```

**Why this is beautiful:**
- Torque is **directly proportional** to one current component (iq)
- Simple linear relationship (like a DC motor: T = Kt·I)
- We can control torque by controlling iq!

#### 2.5.2 Physical Explanation of Torque

Torque is produced by the interaction between:
1. **Stator magnetic field** (created by three-phase currents)
2. **Rotor magnetic field** (from permanent magnets)

**Lorentz Force Law:**
```
F = B × I × L

Torque = Force × Radius
```

When the stator field and rotor field are:
- **Aligned (0°):** No torque (stable equilibrium)
- **Perpendicular (90°):** Maximum torque
- **Opposite (180°):** No torque (unstable equilibrium)

**The goal of FOC:** Keep the fields perpendicular for maximum torque per ampere.

#### 2.5.3 Relationship Between Electrical and Mechanical

**Electrical Domain:**
- Electrical position: θe = P × θm
- Electrical velocity: ωe = P × ωm
- Electrical frequency: fe = P × fm

**Mechanical Domain:**
- Mechanical position: θm [rad]
- Mechanical velocity: ωm [rad/s]
- Mechanical frequency: fm [Hz]

**Power Relationship:**
```
Electrical power = Mechanical power (assuming no losses)
Pe = Te × ωm
```

**Example Calculation:**

Given:
- Motor with P = 4 pole pairs
- Mechanical speed: 1500 RPM = 157.08 rad/s
- Torque: 2 N·m

Calculate:
- Electrical speed: ωe = 4 × 157.08 = 628.32 rad/s = 100 Hz
- Mechanical power: P = 2 × 157.08 = 314.16 W

### 2.6 Mechanical Dynamics

#### 2.6.1 Mechanical Equation of Motion

The motor's mechanical behavior follows Newton's second law for rotation:

```
Te - TL - Tf = J·(dωm/dt)
```

**Where:**
- **Te** = Electromagnetic torque (motor produces) [N·m]
- **TL** = Load torque (external load) [N·m]
- **Tf** = Friction torque = B·ωm [N·m]
- **J** = Moment of inertia [kg·m²]
- **B** = Viscous friction coefficient [N·m·s/rad]

**Physical Meaning:**
- Net torque = Motor torque - Load torque - Friction
- Net torque accelerates the rotor (J·α)
- Larger J → Slower acceleration (more sluggish)
- Larger B → More damping (more energy loss)

#### 2.6.2 Position Integration

Once we know the velocity, we can find position by integration:

```
θm(t) = θm(0) + ∫ωm(t)dt
```

**In discrete time (for simulation):**
```
θm(k+1) = θm(k) + ωm(k)·Δt
```

#### 2.6.3 Mechanical Time Constant

The **mechanical time constant** characterizes how fast the motor responds:

```
τm = J·Rs / (Kt²)

or approximately:

τm = J / B  (if friction dominates)
```

**Typical values:**
- Small servo motor: τm = 1-10 ms (fast response)
- Industrial motor: τm = 50-200 ms (moderate)
- Large motor with high inertia: τm > 1 s (slow)

**Design Implication:**
Speed control loop bandwidth must be **slower** than 1/τm to avoid instability.

### 2.7 Complete PMSM Model Summary

Putting it all together, the complete model consists of:

**Electrical Equations (three phases):**
```
Va = Rs·ia + Ls·(dia/dt) + ea
Vb = Rs·ib + Ls·(dib/dt) + eb
Vc = Rs·ic + Ls·(dic/dt) + ec

ea = -Ke·ωe·sin(θe)
eb = -Ke·ωe·sin(θe - 2π/3)
ec = -Ke·ωe·sin(θe + 2π/3)
```

**Electromagnetic Torque:**
```
Te = (3/2)·P·λm·iq
```

**Mechanical Equations:**
```
Te - TL - B·ωm = J·(dωm/dt)
θm = ∫ωm dt
θe = P·θm
```

**This is a coupled electromechanical system:** electrical currents produce torque, which affects mechanical speed, which affects back-EMF, which affects currents!

### 2.8 Numerical Example

Let's work through a complete example to make this concrete.

**Given Motor Parameters:**
- Rs = 0.285 Ω (stator resistance)
- Ls = 0.85 mH (stator inductance)
- λm = 11.8 mWb (flux linkage)
- P = 4 (pole pairs)
- J = 0.01 kg·m² (inertia)
- B = 0.01 N·m·s/rad (friction)

**Operating Condition:**
- Speed: 1000 RPM = 104.72 rad/s
- Load torque: TL = 1 N·m
- Phase current amplitude: 3 A

**Calculate:**

1. **Electrical frequency:**
   ```
   ωe = P × ωm = 4 × 104.72 = 418.88 rad/s
   fe = ωe/(2π) = 66.67 Hz
   ```

2. **Back-EMF constant:**
   ```
   Ke = P × λm = 4 × 0.0118 = 0.0472 V/(rad/s)
   ```

3. **Peak back-EMF:**
   ```
   Ea_peak = Ke × ωe = 0.0472 × 418.88 = 19.77 V
   ```

4. **Torque constant:**
   ```
   Kt = (3/2) × P × λm = 1.5 × 4 × 0.0118 = 0.0708 N·m/A
   ```

5. **Required iq for 1 N·m:**
   ```
   iq = Te / Kt = 1 / 0.0708 = 14.12 A
   ```

6. **Steady-state acceleration:**
   ```
   Te = Kt × iq = 0.0708 × 14.12 = 1 N·m
   Friction torque: Tf = B × ωm = 0.01 × 104.72 = 1.05 N·m

   Net torque = Te - TL - Tf = 1 - 1 - 1.05 = -1.05 N·m

   Acceleration = Net torque / J = -1.05 / 0.01 = -105 rad/s²

   (Motor is decelerating because load + friction exceed motor torque!)
   ```

**Lesson:** This example shows why proper current control is essential. We need to supply enough current (iq) to overcome both the load torque and friction torque.

### 2.9 Key Takeaways from PMSM Theory

1. **PMSM is an AC synchronous motor** with permanent magnets on the rotor
2. **Three-phase currents** create a rotating magnetic field
3. **Rotor speed equals stator field speed** (synchronous operation)
4. **Electrical frequency = P × mechanical frequency** (pole pairs matter!)
5. **Back-EMF is proportional to speed** (makes control at low speeds easier)
6. **Torque is proportional to one current component** (iq in FOC)
7. **Coupled electromechanical dynamics** require careful control design
8. **Many assumptions** make the model tractable (linear, balanced, sinusoidal)

### 2.10 Further Study - PMSM Theory

**Recommended Books:**
1. **"Permanent Magnet Synchronous and Brushless DC Motor Drives"** by R. Krishnan
   - Chapter 2: Construction and Principles
   - Chapter 3: Mathematical Models

2. **"Electric Motor Drives: Modeling, Analysis, and Control"** by R. Krishnan
   - Comprehensive treatment of motor modeling

3. **"Control of Electric Machine Drive Systems"** by Seung-Ki Sul
   - Excellent diagrams and explanations

**Application Notes:**
1. **Texas Instruments:** "Sensored Field Oriented Control of 3-Phase PMSM Motors" (SPRABQ2)
2. **Microchip:** AN1078 - "Sensorless Field Oriented Control of PMSM Motors"
3. **Infineon:** "Field Oriented Control (FOC) for PMSM" (AP32370)

**Video Lectures:**
1. **NPTEL - Electrical Drives** by Prof. Rik De Doncker (IIT Kharagpur)
2. **YouTube: "How a 3 Phase AC Induction Motor Works"** - Good for understanding rotating fields
3. **Mathworks Webinar:** "Field Oriented Control of PMSM Motors"

**Websites:**
1. **All About Circuits** - Motor Control Section
2. **Power Electronics Tips** - Motor drive articles
3. **EDN Network** - Motor control tutorials

---

*End of Section 2 - PMSM Motor Theory*

**Next section will cover Field Oriented Control (FOC) Theory. Please review this section before we proceed!**

