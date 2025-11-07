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

### 2.11 Application-Specific Considerations: Traction vs Industrial vs Servo

Different applications have vastly different requirements for PMSM drives. Understanding these differences is crucial for proper motor and controller design.

#### 2.11.1 Traction Applications (Electric Vehicles, Trains)

**Defining Characteristics:**
- Very wide speed range (0-15,000+ RPM)
- High torque at low speeds (hill climbing, acceleration)
- Constant power region at high speeds (highway cruising)
- Frequent start-stop cycles
- Regenerative braking capability essential
- Battery voltage varies (300-800V DC, can drop under load)
- Harsh environmental conditions (-40°C to +85°C ambient)
- Safety-critical application (ISO 26262 automotive safety)

**Typical Performance Profile:**
```
Torque
  ^
  │ ┌─────┐ Constant Torque Region
  │ │     │
  │ │     └──────────── Constant Power Region
  │ │
  │ └──────────────────────────────────────>
  0        Base Speed              Max Speed
```

**Design Priorities:**

1. **Wide Constant Power Speed Range (CPSR)**
   - Requirement: CPSR ratio of 3:1 to 5:1 or higher
   - Implementation: Field weakening control (inject negative id)
   - Motor design: Use Interior PMSMs (IPM) with saliency (Ld < Lq)
   - Why IPMs: Reluctance torque adds to magnet torque in field weakening

2. **High Peak-to-Continuous Torque Ratio**
   - Peak torque: 2-4× continuous torque (for acceleration)
   - Thermal management critical (liquid cooling common)
   - Short-term overload capability required

3. **Regenerative Braking**
   - Motor must operate as generator
   - Bidirectional power flow through inverter
   - Battery must accept charging current
   - Energy recovery: 15-25% improvement in range

4. **Efficiency Map Optimization**
   - Optimize for common driving cycles (WLTP, EPA)
   - Peak efficiency >95% at cruise conditions
   - Good part-load efficiency critical (city driving)

5. **High Power Density**
   - Target: 3-5 kW/kg for motor
   - Space and weight constrained (affects vehicle dynamics)
   - High-speed operation reduces motor size (P = T × ω)

6. **Robust Position Sensing**
   - Resolver or redundant sensors (safety)
   - Must survive vibration, temperature extremes
   - Low-cost Hall sensors often insufficient

**Example EV Motor Specifications:**
```
Application: Mid-size Electric Vehicle
Motor Type: IPM PMSM with oil cooling
Peak Power: 150 kW
Continuous Power: 75 kW
Peak Torque: 340 N·m (0-3000 RPM)
Base Speed: 3000 RPM
Max Speed: 15,000 RPM
CPSR: 5:1
Battery Voltage: 350-450 VDC
Efficiency: >95% at cruise
Weight: 50 kg (3 kW/kg)
Cooling: Oil spray + water jacket
```

**Control Challenges:**
- Battery voltage variation (50-100V range)
- Thermal derating at high temperatures
- Precise torque control for traction control systems
- NVH (noise, vibration, harshness) management
- Fault tolerance and safe degradation modes

#### 2.11.2 Industrial Applications (Pumps, Fans, Conveyors)

**Defining Characteristics:**
- Constant or slowly varying speed operation
- Moderate speed range (500-3000 RPM typical)
- Load torque varies with application
- Long continuous operation (24/7 in many cases)
- Cost-sensitive ($/kW is critical metric)
- Reliability and uptime critical

**Design Priorities:**

1. **Efficiency at Rated Load**
   - Optimized for one operating point (rated speed/torque)
   - Less concern about efficiency map breadth
   - Energy savings drive adoption (payback period important)

2. **Reliability and Lifetime**
   - Target: 20+ years, 100,000+ hours
   - Conservative thermal design
   - Air cooling often sufficient
   - Derating for ambient temperature

3. **Cost Optimization**
   - Ferrite magnets sometimes used (vs expensive NdFeB)
   - Induction motors still competitive at low power
   - Simple Hall sensors or even sensorless acceptable
   - Standard frame sizes, easy replacement

4. **Simple Speed Control**
   - V/f control often sufficient (no position sensor)
   - FOC used when better dynamic response needed
   - Speed accuracy: ±1-2% acceptable

**Typical Industrial Motor:**
```
Application: HVAC Fan Drive
Motor Type: SPM PMSM, air cooled
Rated Power: 7.5 kW
Rated Speed: 1500 RPM
Rated Torque: 48 N·m
Speed Range: 500-1800 RPM
Overload: 1.5× for 60 seconds
Efficiency: 92% at rated load
Cost Target: <$200 for motor + drive
Cooling: Natural convection
Sensor: Sensorless control
```

**Control Focus:**
- Energy efficiency (VFD energy savings)
- Smooth speed regulation
- Soft start/stop
- PLC integration, Modbus/Profinet communication

#### 2.11.3 Servo Applications (Robotics, CNC, Automation)

**Defining Characteristics:**
- Precise position control (micrometers, arc-seconds)
- Very high dynamic response (milliseconds)
- Frequent acceleration/deceleration
- Wide range of loads and speeds
- Positioning accuracy critical
- High bandwidth control loops

**Design Priorities:**

1. **High Bandwidth**
   - Current loop: 1-5 kHz bandwidth
   - Position loop: 100-500 Hz bandwidth
   - Fast torque response (<1 ms)

2. **Low Inertia**
   - Minimal rotor inertia for fast acceleration
   - Hollow shaft designs common
   - Direct drive (no gearbox) when possible

3. **Precise Position Feedback**
   - High-resolution encoders (16-20 bit absolute)
   - Sin-cos interpolation for sub-count resolution
   - Multi-turn absolute encoders

4. **Low Torque Ripple**
   - Cogging torque <1% of rated torque
   - Current harmonics minimized
   - Skewed magnets/stator slots

5. **Repeatability**
   - Position repeatability: ±1 arc-second
   - Backlash-free mechanical design
   - Temperature-stable components

**Typical Servo Motor:**
```
Application: CNC Axis Drive
Motor Type: SPM PMSM, frameless
Rated Power: 2 kW
Rated Speed: 3000 RPM
Peak Torque: 15 N·m
Continuous Torque: 6 N·m
Rotor Inertia: 0.5 kg·cm²
Torque Constant: 1.2 N·m/A
Sensor: 20-bit absolute encoder
Position Resolution: <1 arc-second
Acceleration: 10,000 rad/s²
```

**Control Requirements:**
- Cascade control: Position → Speed → Current
- Feedforward compensation
- Advanced algorithms: State feedback, observers
- Real-time Ethernet (EtherCAT, Profinet IRT)

#### 2.11.4 Comparison Table

| Feature | Traction (EV) | Industrial (HVAC) | Servo (CNC) |
|---------|---------------|-------------------|-------------|
| **Speed Range** | 0-15,000 RPM | 500-1800 RPM | 0-6000 RPM |
| **CPSR** | 3:1 to 5:1 | 1.5:1 | 2:1 |
| **Torque Ripple** | <5% acceptable | <3% | <1% critical |
| **Position Accuracy** | Not critical | Not required | ±1 arc-sec |
| **Sensor Type** | Resolver/Encoder | Sensorless/Hall | Abs. Encoder |
| **Current Loop BW** | 1-2 kHz | 500 Hz | 2-5 kHz |
| **Efficiency** | 95-97% peak | 92-94% rated | 90-93% |
| **Cost/kW** | $50-100 | $20-40 | $200-500 |
| **Cooling** | Liquid | Air | Air/Liquid |
| **Lifetime** | 10-15 years | 20+ years | 10-20 years |
| **Safety Level** | ASIL-C/D | Basic | PLd/PLe |
| **Overload** | 3-4× peak | 1.5-2× | 2-3× peak |
| **Thermal Time Const** | 10-30 min | 30-60 min | 5-15 min |

#### 2.11.5 Key Design Decisions

**Choosing Motor Type:**

**Use Surface PMSMs (SPM) when:**
- Moderate speed range (CPSR < 2:1)
- Cost is primary concern
- Air cooling sufficient
- Simpler control acceptable (Ld = Lq)
- Applications: Industrial drives, low-cost servo

**Use Interior PMSMs (IPM) when:**
- Wide speed range needed (CPSR > 3:1)
- Field weakening operation required
- High power density critical
- High-speed operation (mechanical robustness)
- Applications: Traction, high-performance servo

**Use Induction Motors when:**
- Very harsh environments (foundries, mining)
- Lowest cost critical
- Wide speed range (field weakening easier)
- Magnet demagnetization risk too high
- Applications: Heavy industrial, legacy systems

**Choosing Control Strategy:**

**Sensorless V/f Control:**
- Simplest, lowest cost
- ±2-5% speed accuracy
- No low-speed torque
- Applications: Fans, pumps (steady-state)

**Sensorless FOC:**
- Good efficiency
- ±1% speed accuracy
- Poor low-speed performance (<5% rated speed)
- Applications: Industrial VFDs, appliances

**Sensored FOC with Hall:**
- Low-cost position sensing
- Full torque at zero speed
- Limited accuracy (±30° electrical)
- Applications: E-bikes, tools, low-cost servo

**Sensored FOC with Encoder:**
- Excellent precision
- Full dynamic performance
- Higher cost and complexity
- Applications: Robotics, CNC, high-end servo

**Sensored FOC with Resolver:**
- Robust (vibration, temperature, EMI)
- Absolute position (within one revolution)
- Higher cost than Hall
- Applications: Automotive, aerospace, harsh environments

#### 2.11.6 Traction-Specific Control Features

Modern EV motor controllers include specialized features:

1. **Battery Voltage Compensation**
   - Modulation index adjustment for voltage sag
   - Overmodulation strategies
   - Voltage boost converters in some designs

2. **Thermal Management**
   - Real-time thermal models
   - Torque derating based on temperature
   - Coolant flow control integration

3. **Regenerative Braking Coordination**
   - Blending with friction brakes
   - Battery SOC and charging limits
   - Stability control integration

4. **NVH Management**
   - Torque ripple minimization
   - Acoustic noise shaping (shift frequencies)
   - Active damping of drivetrain resonances

5. **Functional Safety**
   - Redundant position sensors
   - Plausibility checks
   - Safe torque shutoff
   - ASIL-C or ASIL-D certification

6. **Traction Control Integration**
   - Rapid torque response (<10 ms)
   - Wheel slip control
   - Torque vectoring (multi-motor systems)

### 2.12 Key Takeaways - Application Considerations

1. **Traction demands wide speed range** → Use IPM motors with field weakening
2. **Industrial prioritizes efficiency at one point** → SPM or even induction OK
3. **Servo requires precision** → High-resolution encoders essential
4. **Cost-performance tradeoff varies** → Traction: performance, Industrial: cost, Servo: precision
5. **Sensor choice depends on application** → Resolver for harsh, encoder for precision, Hall for cost
6. **Control complexity scales with requirements** → V/f for simple, FOC for performance

---

*End of Section 2 - PMSM Motor Theory*

---

## 3. Understanding Reference Frames - The Foundation of FOC

### 3.1 What Is a Reference Frame?

A **reference frame** (or coordinate system) is a perspective from which we observe and describe physical quantities. Think of it like this:

**Analogy 1: Describing Position on Earth**
- You can describe your location using:
  - Latitude/Longitude (spherical coordinates)
  - X/Y on a map (Cartesian coordinates)
  - Distance/Direction from a landmark (polar coordinates)
- **Same position, different descriptions**
- Some coordinate systems make certain calculations easier

**Analogy 2: Merry-Go-Round**
- Standing on the ground (stationary frame): You see the horses going in circles
- Sitting on a horse (rotating frame): The horses appear stationary, the world spins around you
- **Same motion, different perspectives**

In motor control, we describe the same electrical quantities (currents, voltages, magnetic fields) using different reference frames. **The physics doesn't change - only our mathematical description.**

### 3.2 Why Do We Need Different Reference Frames?

The fundamental challenge in AC motor control:

**The Problem:**
- Three-phase AC currents create a rotating magnetic field
- This field is sinusoidal in time: i_a(t) = I·cos(ωt)
- Time-varying sinusoids are hard to control with simple controllers
- Traditional PI controllers work best with **DC (constant) signals**

**The Solution:**
- Transform to a reference frame that **rotates with the magnetic field**
- In this rotating frame, AC quantities become DC quantities
- Now we can use simple PI controllers!

**This is the KEY INSIGHT of Field Oriented Control.**

### 3.3 The Three Reference Frames in FOC

For PMSM control, we use three reference frames:

```
Natural Frame (abc) → Stationary Frame (αβ) → Rotating Frame (dq)
   3 phases              2 phases               2 phases
  Time-varying         Time-varying           DC (constant)
   Complex              Simpler               Simplest!
```

Let's understand each one in detail.

### 3.4 Natural Reference Frame (abc)

#### 3.4.1 Definition

This is the **physical reference frame** - the actual three-phase windings in the motor.

**Quantities in abc frame:**
- Three phase currents: ia, ib, ic
- Three phase voltages: Va, Vb, Vc
- Spatially separated by 120° (physical winding positions)
- Temporally sinusoidal (varying with electrical frequency)

**Equations:**
```
ia(t) = I·cos(ωe·t)
ib(t) = I·cos(ωe·t - 120°)
ic(t) = I·cos(ωe·t - 240°)

where:
ωe = electrical frequency
I = peak current
```

#### 3.4.2 Visualization

**Spatial View (looking at motor end):**
```
        Phase A (0°)
            ↑
            │
            │
Phase C ────┼──── Phase B
(240°)      │      (120°)
            │
            ↓
```

**Temporal View (currents vs time):**
```
Current
  ^
  │    ia
  │   ╱╲╱╲╱╲
  │  ╱  ╲  ╱╲
  │ ╱    ╲╱  ╲
  ├─────────────> Time
  │╲    ╱╲    ╱
  │ ╲  ╱  ╲  ╱ib
  │  ╲╱    ╲╱
  │    ic
```

#### 3.4.3 Characteristics

**Advantages:**
- ✓ Direct correspondence to physical hardware
- ✓ Easy to measure (current sensors in each phase)
- ✓ Natural for PWM generation (three switching legs)

**Disadvantages:**
- ✗ Three variables to track (redundant - ia + ib + ic = 0)
- ✗ Time-varying sinusoids (hard to control)
- ✗ Coupling between phases (changing one affects others)
- ✗ Torque/flux relationship is non-obvious

**When We Use It:**
- Current measurement (sensor outputs)
- PWM generation (inverter inputs)
- Fault detection (imbalance detection)

### 3.5 Stationary Reference Frame (αβ)

#### 3.5.1 Definition

The **αβ frame** (also called **stationary two-phase frame** or **Clarke frame**) reduces the three phases to two orthogonal components that are **stationary with respect to the stator**.

**Key Concept:**
- α-axis aligned with phase A
- β-axis 90° ahead of α-axis (perpendicular)
- Both axes fixed in space (don't rotate)

**Why 2 axes instead of 3?**
Because the three-phase system is **balanced**: ia + ib + ic = 0
- This constraint means we only have 2 independent variables
- The third phase is redundant information
- We can describe everything with just 2 components!

#### 3.5.2 Physical Interpretation

**The αβ frame represents the magnetic field as a vector:**

```
         β-axis
           ↑
           │
           │   B⃗ (magnetic field vector)
           │  ↗
           │ ╱
           │╱ θ
  ─────────┼─────────> α-axis
           │
           │
```

Instead of three sinusoidal currents, we now have:
- **A single rotating vector** with constant magnitude
- Position: angle θ from α-axis
- Magnitude: √(iα² + iβ²)

**This is called the "space vector" representation.**

#### 3.5.3 Mathematical Relationship

**From abc to αβ (Clarke Transformation):**
```
iα = (2/3) × (ia - ½·ib - ½·ic)
iβ = (2/3) × (√3/2)·(ib - ic)
```

Or in matrix form:
```
[iα]     2   [  1     -1/2      -1/2   ] [ia]
[iβ] = ───── [  0    √3/2     -√3/2   ] [ib]
       3                                  [ic]
```

**Example:**
At t=0, if ia = 10A, ib = -5A, ic = -5A:
```
iα = (2/3) × (10 - (-5)/2 - (-5)/2) = (2/3) × 15 = 10A
iβ = (2/3) × (√3/2) × (-5 - (-5)) = 0A
```
Result: Vector points along α-axis with magnitude 10A.

#### 3.5.4 Characteristics

**Advantages:**
- ✓ Only 2 variables instead of 3 (more efficient)
- ✓ Orthogonal components (α and β independent)
- ✓ Space vector representation (intuitive for field visualization)
- ✓ Still time-varying sinusoids (same frequency as abc)

**Disadvantages:**
- ✗ Still time-varying (not suitable for simple PI control)
- ✗ Coupling between torque and flux still exists
- ✗ Not directly measurable (must be calculated)

**When We Use It:**
- Intermediate step in transformation (abc → αβ → dq)
- Space vector PWM (SVPWM) calculations
- Flux estimation
- Motor modeling in simulation

#### 3.5.5 The "Rotating Vector" Insight

**Key Realization:**
Three balanced sinusoidal currents → Single rotating vector in αβ plane

**At any instant:**
```
iα(t) = I·cos(ωe·t)
iβ(t) = I·sin(ωe·t)

Magnitude: √(iα² + iβ²) = √(I²·cos²θ + I²·sin²θ) = I  (constant!)
Angle: θ(t) = atan2(iβ, iα) = ωe·t  (rotating at ωe)
```

**This is beautiful:** The mess of three sinusoidal currents becomes a clean rotating vector!

### 3.6 Rotating Reference Frame (dq)

#### 3.6.1 Definition

The **dq frame** (also called **synchronous rotating frame** or **Park frame**) is a coordinate system that **rotates with the rotor** at electrical speed ωe.

**Axes Definition:**
- **d-axis (direct axis):** Aligned with the rotor's magnetic field (North pole of magnets)
- **q-axis (quadrature axis):** 90° ahead of d-axis (perpendicular to rotor flux)

**Key Property:**
In this frame, **AC quantities become DC** (constant values)!

#### 3.6.2 Physical Interpretation

**Imagine you're sitting on the rotor:**
- The rotor appears stationary (you're rotating with it)
- The stator magnetic field, if properly controlled, also appears stationary
- Everything that was rotating now appears frozen!

**Like riding the merry-go-round:**
- On the ground: horses going in circles (abc frame)
- On a horse: horses appear still, scenery spins (dq frame)

```
        d-axis (aligned with rotor flux)
            ↑
            │ ⃝  (N pole of magnet)
            │
            │
            │
  ──────────┼──────────> q-axis
            │            (torque axis)
            │
            │ ⃝  (S pole of magnet)
            ↓
```

#### 3.6.3 Mathematical Relationship

**From αβ to dq (Park Transformation):**
```
id = iα·cos(θe) + iβ·sin(θe)
iq = -iα·sin(θe) + iβ·cos(θe)
```

Or in matrix form:
```
[id]   [ cos(θe)   sin(θe)] [iα]
[iq] = [-sin(θe)   cos(θe)] [iβ]
```

**This is a rotation matrix!** It rotates the αβ frame by angle θe to align with the rotor.

**Where does θe come from?**
- From position sensor (Hall, encoder, resolver)
- Or estimated (sensorless control)
- This is the rotor's electrical position

#### 3.6.4 Physical Meaning of d and q Currents

**d-axis current (id):**
- Component aligned with rotor magnets
- Affects the magnetic flux
- For surface PMSMs: set id = 0 (magnets provide sufficient flux)
- For IPMs: can use negative id for field weakening at high speed

**q-axis current (iq):**
- Component perpendicular to rotor magnets
- Produces torque!
- **Torque = Kt × iq** (simple linear relationship)
- This is what we control to regulate torque and speed

**Why is this powerful?**
- **Decoupled control:** id and iq are independent
- **Like a DC motor:** Flux (id) and torque (iq) controlled separately
- **Simple relationship:** Changing iq directly changes torque

#### 3.6.5 Characteristics

**Advantages:**
- ✓ **DC quantities** at steady-state (constant id, iq)
- ✓ **Decoupled flux and torque control** (id affects flux, iq affects torque)
- ✓ **Simple PI controllers** work well (designed for DC signals)
- ✓ **Direct torque control:** T = Kt·iq
- ✓ **Intuitive physical meaning**

**Disadvantages:**
- ✗ Requires accurate rotor position (θe)
- ✗ More complex transformations (computation needed)
- ✗ Not directly measurable (must be calculated)

**When We Use It:**
- Current control (PI controllers operate in dq frame)
- Torque control (set iq reference)
- Field weakening (adjust id)
- Motor modeling for control design

### 3.7 Comparison of Reference Frames

#### 3.7.1 Summary Table

| Feature | abc Frame | αβ Frame | dq Frame |
|---------|-----------|----------|----------|
| **Number of Variables** | 3 | 2 | 2 |
| **Nature of Signals** | Sinusoidal (AC) | Sinusoidal (AC) | Constant (DC)* |
| **Reference** | Stator windings | Stator fixed | Rotor position |
| **Rotation** | None | None | ωe (elec. speed) |
| **Directly Measurable** | Yes (current sensors) | No | No |
| **Used For** | Measurement, PWM | Intermediate, SVPWM | Control (PI loops) |
| **Complexity** | High (3 coupled vars) | Medium (2 orthogonal) | Low (2 decoupled) |
| **Torque-Flux Coupling** | Coupled | Coupled | **Decoupled** ✓ |
| **Control Difficulty** | Hard | Hard | **Easy** ✓ |

*At steady-state operation. During transients, id and iq vary.

#### 3.7.2 Visual Comparison

**Same current vector, three perspectives:**

```
ABC Frame (3-phase):
ia: ──╱╲╱╲──
ib: ─╱──╲╱──╲
ic: ╱────╲──╱

αβ Frame (2-phase stationary):
     β
     ↑
     │  ⤸ (rotating vector)
     │ ╱
  ───┼──> α
     │

dq Frame (2-phase rotating):
     q
     ↑
     │ • (stationary point!)
     │
  ───┼──> d
     │
```

### 3.8 The Transformation Chain

#### 3.8.1 Forward Path (Measurement → Control)

**What we measure → What we control:**

```
Current Sensors → abc currents → Clarke → αβ currents → Park → dq currents
   (hardware)      (3 AC)                    (2 AC)              (2 DC)
                                                                    ↓
                                                          PI Controllers
                                                          (operate here!)
```

**Each transformation simplifies the problem:**
1. abc → αβ: Reduce from 3 variables to 2
2. αβ → dq: Convert AC to DC (align with rotor)

#### 3.8.2 Reverse Path (Control → Actuation)

**What controllers output → What motor needs:**

```
PI Controllers → dq voltages → Inv.Park → αβ voltages → Inv.Clarke → abc voltages → PWM
  (vd*, vq*)      (2 DC)                     (2 AC)                      (3 AC)         ↓
                                                                                    Inverter
```

**Each inverse transformation prepares for hardware:**
1. dq → αβ: Convert DC commands back to AC
2. αβ → abc: Convert 2-phase to 3-phase for inverter

### 3.9 A Worked Example

Let's track one set of currents through all frames.

**Given:**
- Motor running at 1000 RPM, P=4 pole pairs
- Electrical frequency: fe = (4 × 1000)/60 = 66.67 Hz
- Peak current: I = 10A
- Time: t = 0
- Rotor position: θe(t=0) = 30°

**Step 1: abc frame at t=0**
```
ia = 10·cos(0°) = 10.0A
ib = 10·cos(-120°) = -5.0A
ic = 10·cos(-240°) = -5.0A

Verification: ia + ib + ic = 10 - 5 - 5 = 0 ✓
```

**Step 2: Clarke transformation (abc → αβ)**
```
iα = (2/3)·(10 - (-5)/2 - (-5)/2)
   = (2/3)·(10 + 2.5 + 2.5) = 10.0A

iβ = (2/3)·(√3/2)·(-5 - (-5))
   = (2/3)·(√3/2)·0 = 0A

Result: Space vector points along α-axis, magnitude 10A
```

**Step 3: Park transformation (αβ → dq) with θe=30°**
```
id = iα·cos(30°) + iβ·sin(30°)
   = 10·(√3/2) + 0·(1/2)
   = 8.66A

iq = -iα·sin(30°) + iβ·cos(30°)
   = -10·(1/2) + 0·(√3/2)
   = -5.0A

Result: id = 8.66A (flux-producing), iq = -5.0A (torque-producing)
```

**Step 4: Interpretation**
- Total current magnitude: √(id² + iq²) = √(75 + 25) = 10A ✓
- Flux component: 8.66A
- Torque: Te = Kt·iq (negative = regenerative braking)

**Key Insight:**
Same current, three descriptions:
- abc: Three 10A peak sinusoids, 120° apart
- αβ: 10A vector at 0° (pointing along α)
- dq: 8.66A along d-axis, -5A along q-axis

### 3.10 Why dq Frame is Special for Control

#### 3.10.1 The DC Quantity Property

**In steady-state operation:**

If the motor runs at constant speed with constant torque:
- abc currents: sinusoidal, frequency = ωe
- αβ currents: sinusoidal, frequency = ωe
- dq currents: **constant (DC)** ✓

**Why?**
- The dq frame rotates at ωe (same as field)
- Relative to this frame, field appears stationary
- Like watching a planet from the sun vs from Earth

**PI controllers love DC signals:**
```
Traditional PI: works well with DC, struggles with AC
FOC approach: Transform AC → DC, control, transform back
```

#### 3.10.2 The Decoupling Property

**In abc/αβ frames:**
- Torque depends on currents AND rotor position
- Complex nonlinear relationship
- Difficult to control independently

**In dq frame:**
```
Flux ≈ id     (for SPM: flux from magnets, so id→0)
Torque = Kt·iq  (simple, linear, direct)
```

**Decoupled means:**
- Changing id doesn't affect torque
- Changing iq doesn't affect flux
- Can control them independently!

**Like a DC motor:**
```
DC Motor:
  Field current → Flux
  Armature current → Torque

FOC PMSM:
  id → Flux (usually 0)
  iq → Torque
```

#### 3.10.3 Why Alignment with Rotor Matters

**Maximum Torque Per Ampere (MTPA):**

For maximum torque with minimum current:
- d-axis should align with rotor flux
- q-axis should be perpendicular
- All torque-producing current goes into iq
- Zero wasted current in id (for SPM)

**If misaligned:**
- Some current wasted (doesn't produce torque)
- Efficiency drops
- Torque ripple increases

**This is why accurate θe is critical!**

### 3.11 Common Misconceptions

**Misconception 1: "αβ and dq are different physical quantities"**
- ❌ Wrong: They're the same electrical quantities, different viewpoints
- ✓ Correct: Like describing velocity in different coordinate systems

**Misconception 2: "Transformations change the motor's behavior"**
- ❌ Wrong: Motor physics is unchanged
- ✓ Correct: Transformations only change our mathematical description

**Misconception 3: "dq currents are real currents flowing in the motor"**
- ❌ Wrong: Only abc currents physically flow
- ✓ Correct: dq currents are mathematical projections

**Misconception 4: "We need transformations to run the motor"**
- ❌ Wrong: Motor will run with abc voltages (like BLDC control)
- ✓ Correct: Transformations enable *optimal* control (FOC)

### 3.12 Key Takeaways - Reference Frames

1. **Three frames, same physics:** abc (natural), αβ (stationary), dq (rotating)

2. **Each transformation simplifies:**
   - abc → αβ: 3 variables → 2 variables
   - αβ → dq: AC signals → DC signals

3. **dq frame is special:**
   - Rotates with rotor
   - AC becomes DC (easy to control)
   - Flux and torque decoupled

4. **Transformations don't change physics:**
   - Same currents, voltages, power
   - Just different mathematical descriptions
   - Like changing units or coordinate systems

5. **Position (θe) is critical:**
   - Needed for Park transformation
   - From sensor or estimator
   - Accuracy affects control quality

6. **Forward and reverse paths:**
   - Forward: abc → αβ → dq (for control)
   - Reverse: dq → αβ → abc (for PWM)

7. **Why FOC works:**
   - Converts AC motor control problem → DC motor control problem
   - Enables simple PI controllers
   - Achieves optimal performance

### 3.13 Further Study - Reference Frames

**Books:**
1. **"Analysis of Electric Machinery and Drive Systems"** by Paul Krause
   - Chapter 3: Reference Frame Theory
   - The definitive mathematical treatment

2. **"Vector Control and Dynamics of AC Drives"** by Novotny and Lipo
   - Chapter 4: Coordinate Transformations
   - Excellent physical intuition

3. **"Control of Electric Machine Drive Systems"** by Seung-Ki Sul
   - Chapter 2: Reference Frame Transformations
   - Clear diagrams and examples

**Videos:**
1. **NPTEL - Electric Drives:** Lecture series on reference frames
2. **YouTube: "Understanding dq Transformation"** by MATLAB
3. **YouTube: "Clarke and Park Transforms Explained"** by Texas Instruments

**Application Notes:**
1. **Texas Instruments:** "Clarke & Park Transforms on the TMS320C2xx" (BPRA048)
2. **STMicroelectronics:** "Field Oriented Control of PMSM" (AN1946)
3. **Microchip:** "Sensored FOC for PMSM" - Chapter 3: Transformations

**Interactive Tools:**
1. **MATLAB/Simulink:** Build transformation blocks and visualize
2. **Desmos/GeoGebra:** Plot rotating vectors in different frames
3. **Scope tool in Simulink:** View abc, αβ, dq simultaneously

---

*End of Section 3 - Understanding Reference Frames*

---

## 4. Position Sensors for Motor Control

### 4.1 Why Do We Need Position Sensors?

For Field Oriented Control, we need to know the rotor's **electrical position (θe)** in real-time. Here's why:

**1. Coordinate Transformations Require Position**
- Park transformation: Rotate from stationary (αβ) to rotor-synchronized (dq) frame
- Requires θe to calculate cos(θe) and sin(θe)
- Without accurate θe, the dq frame is misaligned → poor control

**2. Commutation (Switching the Right Phase)**
- We must energize the correct stator phase at the correct time
- Like firing cylinders in an engine at the right piston position
- Wrong commutation → torque ripple, vibration, efficiency loss

**3. Speed Calculation**
- Speed = dθ/dt (derivative of position)
- Needed for speed control loop feedback
- Can be calculated from position sensor signals

**4. Initial Rotor Position Detection**
- At startup, we need to know where the rotor is
- Prevents wrong initial torque direction
- Critical for smooth, reliable startup

**Resolution vs Accuracy:**
- **Resolution:** Smallest position change that can be detected
- **Accuracy:** How close the measured position is to true position
- **Different sensors have different tradeoffs**

### 4.2 Hall Effect Sensors

Hall sensors are the most cost-effective position sensing solution for PMSM FOC. They provide discrete (digital) position information.

#### 4.2.1 Hall Effect Physical Principle

**Hall Effect (discovered by Edwin Hall, 1879):**

When a current-carrying conductor is placed in a magnetic field perpendicular to the current, a voltage appears across the conductor perpendicular to both current and field.

```
        Magnetic Field (B)
              ↓↓↓
    ┌───────────────────┐
    │                   │  ← VH (Hall voltage)
  I→│   Semiconductor   │→
    │                   │
    └───────────────────┘
```

**Hall Voltage:**
```
VH = (RH × I × B) / t

where:
RH = Hall coefficient (material property)
I = Current through sensor
B = Magnetic field strength
t = Thickness of conductor
```

**In motor applications:**
- Hall IC contains: Hall element + amplifier + comparator
- Output: Digital HIGH when North pole detected, LOW for South pole
- Threshold: Typically switches at ±5-20 mT (50-200 Gauss)
- Supply: 3.3V or 5V logic-level output

#### 4.2.2 Hall Sensor Placement in PMSM

**Standard Configuration:**
- **3 Hall sensors** placed inside the motor
- **120° mechanical spacing** (for most motors)
- Located in the airgap region, sensing rotor magnets
- Mounted on PCB or directly on stator

**Sensor Positioning:**
```
        North Pole Region
              ↑
        Hall Sensor A (0°)
              │
              │
Hall C ───────┼─────── Hall B
(240°)        │         (120°)
              │
              ↓
        South Pole Region
```

**Critical Alignment:**
- Hall sensors must be precisely aligned with motor phases
- Typical tolerance: ±5° electrical
- Misalignment causes:
  - Increased torque ripple
  - Reduced efficiency
  - Higher acoustic noise
  - Potential startup issues

#### 4.2.3 Hall Sensor State Pattern

For a motor with **P pole pairs**, the Hall pattern repeats **P times** per mechanical revolution.

**6 States Per Electrical Revolution:**

| Electrical Angle | Hall A | Hall B | Hall C | Decimal | Binary | Sector |
|------------------|--------|--------|--------|---------|--------|--------|
| 0° - 60°         | 1      | 0      | 1      | 5       | 101    | 1      |
| 60° - 120°       | 1      | 0      | 0      | 4       | 100    | 2      |
| 120° - 180°      | 1      | 1      | 0      | 6       | 110    | 3      |
| 180° - 240°      | 0      | 1      | 0      | 2       | 010    | 4      |
| 240° - 300°      | 0      | 1      | 1      | 3      | 011    | 5      |
| 300° - 360°      | 0      | 0      | 1      | 1       | 001    | 6      |

**Invalid States (should never occur):**
- 000 (all low) - indicates sensor fault or weak magnets
- 111 (all high) - indicates sensor fault or misalignment

**Visualization Over Time:**
```
Hall A: ──┐  ┌─────────┐  ┌────
          └──┘         └──┘

Hall B: ─────┐  ┌─────────┐  ┌
             └──┘         └──┘

Hall C: ┐  ┌─────────┐  ┌────
        └──┘         └──┘

Sector:  1  2   3   4   5   6  1
```

#### 4.2.4 Position Estimation from Hall Sensors

**Sector-Based Estimation:**

Each Hall state corresponds to a 60° sector. We estimate position as the **center of the current sector:**

```
Hall State (Binary) → Sector → Estimated θe
    101             →    1    →    30° (π/6 rad)
    100             →    2    →    90° (π/2 rad)
    110             →    3    →   150° (5π/6 rad)
    010             →    4    →   210° (7π/6 rad)
    011             →    5    →   270° (3π/2 rad)
    001             →    6    →   330° (11π/6 rad)
```

**Resolution Limitation:**
- Position known only to ±30° electrical
- For a motor with P=4 pole pairs: ±30°/4 = ±7.5° mechanical
- Good enough for FOC torque control
- Not sufficient for precise position control

**Interpolation Methods (Advanced):**

To improve resolution between Hall transitions:

1. **Time-based interpolation:**
   - Measure time between Hall transitions (ΔT)
   - Estimate position based on time elapsed in current sector
   - Assumes constant speed within sector
   - Works reasonably well at steady-state

2. **Observer-based estimation:**
   - Use motor model to predict position
   - Update estimate when Hall transition occurs
   - Better dynamic performance
   - More complex implementation

3. **Hybrid with sensorless:**
   - Use Hall sensors for low speed
   - Switch to back-EMF observer at higher speeds
   - Combines benefits of both methods

#### 4.2.5 Speed Estimation from Hall Sensors

**Method 1: Transition Counting**

Each Hall transition represents 60° electrical movement:

```
Speed calculation:
Δθe = 60° = π/3 rad (per transition)
ΔT = Time between transitions [seconds]

ωe = Δθe / ΔT = (π/3) / ΔT rad/s

ωm = ωe / P  (convert to mechanical speed)

RPM = ωm × (60/2π)
```

**Example:**
- Hall transition every 5 ms
- ωe = (π/3) / 0.005 = 209.44 rad/s
- For P=4: ωm = 209.44 / 4 = 52.36 rad/s = 500 RPM

**Method 2: Frequency Measurement**

Count Hall transitions over a fixed time window:

```
Measure period T (e.g., 100 ms)
Count N transitions in time T

Electrical revolutions = N / 6 (6 transitions per elec. revolution)
Electrical frequency fe = N / (6 × T)  Hz

Mechanical frequency fm = fe / P  Hz
RPM = fm × 60
```

**Noise and Filtering:**
- Hall signals can have glitches near switching points
- Use digital filtering (moving average, median filter)
- Debouncing logic to reject very short pulses
- Plausibility checks (reject impossible accelerations)

#### 4.2.6 Advantages and Disadvantages of Hall Sensors

**Advantages:**
- ✓ Very low cost (< $1 per sensor)
- ✓ Simple interface (digital I/O pins)
- ✓ Works from zero speed (no minimum speed requirement)
- ✓ Robust to EMI (digital signal)
- ✓ Absolute position within sector (no homing needed)
- ✓ Small size, easy integration
- ✓ Reliable in harsh environments

**Disadvantages:**
- ✗ Low resolution (±30° electrical)
- ✗ Discrete position (not continuous)
- ✗ Temperature sensitivity (Hall IC characteristics change)
- ✗ Magnet strength variation affects switching point
- ✗ Requires motor disassembly for sensor replacement
- ✗ Cannot achieve high-precision position control
- ✗ Torque ripple higher than with high-resolution sensors

**Best Applications:**
- E-bikes, e-scooters
- Power tools
- Appliances (washing machines, dryers)
- HVAC fans and pumps
- Low-cost industrial drives
- Any cost-sensitive application where ±1° position error is acceptable

### 4.3 Resolvers (Sin-Cos Encoders)

Resolvers are analog position sensors that provide continuous, high-accuracy position information. They are the gold standard for harsh environments.

#### 4.3.1 Resolver Construction and Principle

**Physical Construction:**

A resolver is essentially a **rotating transformer** with:
- **Stator:** Two windings 90° apart (sin and cos windings)
- **Rotor:** One excitation winding (energized with AC carrier)
- **Magnetic coupling** varies with rotor position

```
        Stator Sin Winding
              ↑
              │
     ┌────────┼────────┐
     │        │        │
Cos──┼───╭────┼────╮───┼──Cos
     │   │  Rotor  │   │
     │   ╰─────────╯   │
     │                 │
     └─────────────────┘
              │
         Carrier Input
```

**Operating Principle:**

1. **Excitation:** AC carrier (e.g., 10 kHz, 7V RMS) applied to rotor winding
2. **Coupling:** Magnetic field from rotor couples into stator windings
3. **Modulation:** Coupling strength varies with rotor position (θ)
4. **Outputs:** Two sinusoidal signals

**Output Signals:**
```
Excitation:  Vex = V0 × sin(ωc × t)

Sin output:  Vsin = K × V0 × sin(θ) × sin(ωc × t)
Cos output:  Vcos = K × V0 × cos(θ) × sin(ωc × t)

where:
ωc = Carrier frequency (e.g., 2π × 10 kHz)
θ = Mechanical rotor position
K = Transformation ratio (typically 0.5)
```

**The carrier signal is *modulated* by sin(θ) and cos(θ).**

#### 4.3.2 Resolver Signal Processing

**Demodulation Process:**

To extract position from resolver signals:

1. **Synchronous Demodulation**
   - Multiply sin and cos outputs by reference carrier
   - Low-pass filter to remove carrier frequency
   - Result: DC levels proportional to sin(θ) and cos(θ)

2. **Arctangent Calculation**
   ```
   θ = atan2(Vsin, Vcos)
   ```
   - Provides position in range -180° to +180°
   - Full 360° coverage

3. **Digital Resolver-to-Digital Converter (RDC)**
   - Dedicated ICs: AD2S1210, AD2S1205 (Analog Devices)
   - Performs all demodulation and angle calculation
   - Outputs: Parallel or serial digital angle
   - Accuracy: 12-16 bits typical (0.088° to 0.0055°)

**Tracking Loop (Type II Servo):**

Most RDCs use a tracking loop:

```
Vsin, Vcos → Phase Detector → Loop Filter → VCO → θ_estimated
                ↑                                      │
                └──────────── Feedback ────────────────┘
```

**Benefits:**
- Noise filtering inherent in loop
- Velocity output available (from VCO frequency)
- Robust tracking even with noisy signals

#### 4.3.3 Resolver Specifications

**Key Parameters:**

1. **Transformation Ratio**
   - Typical: 0.5 ± 0.05
   - Defines output voltage amplitude
   - Vout_max = Transformation Ratio × Vexcitation

2. **Accuracy**
   - Electrical accuracy: ±5 to ±30 arc-minutes
   - Mechanical accuracy also depends on mounting
   - Temperature drift: ±1 arc-minute per °C typical

3. **Electrical Null**
   - Residual voltage when rotor at null position
   - Typically < 30 mV
   - Lower is better (less position error)

4. **Carrier Frequency**
   - Range: 400 Hz to 20 kHz
   - Higher frequency → faster response, smaller transformer
   - Lower frequency → simpler electronics, more robust

5. **Max Speed**
   - Limited by mechanical construction
   - Typical: 10,000 - 15,000 RPM max
   - Brushless resolvers can go higher

**Pole Pairs:**

Resolvers can have multiple pole pairs (P):
- **Single-speed:** P = 1, one electrical cycle per mechanical revolution
- **Multi-speed:** P > 1, multiple cycles per revolution
- Higher P → finer resolution but absolute position ambiguity

For motor control, typically use P=1 resolver.

#### 4.3.4 Advantages and Disadvantages of Resolvers

**Advantages:**
- ✓ Extremely robust (survive -55°C to +155°C)
- ✓ Immune to vibration and shock
- ✓ No electronic components in sensing element
- ✓ Long lifetime (no wear parts in brushless types)
- ✓ Intrinsically absolute (no homing needed)
- ✓ Radiation tolerant (nuclear, space applications)
- ✓ High accuracy (down to ±5 arc-minutes)
- ✓ Analog signal less susceptible to EMI-induced errors

**Disadvantages:**
- ✗ Higher cost ($20-100+ vs $1 for Hall)
- ✗ Larger size and weight
- ✗ Requires excitation circuitry
- ✗ Requires RDC IC for signal processing
- ✗ Lower resolution than optical encoders (typically 12-16 bit)
- ✗ Analog signals need careful PCB routing
- ✗ Calibration may be needed

**Best Applications:**
- Automotive (steering, throttle, EV traction)
- Aerospace (actuators, flight controls)
- Defense (turrets, missile systems)
- Industrial robots in harsh environments
- Oil & gas (downhole tools, pumps)
- Anywhere extreme reliability is required

### 4.4 Optical Encoders

Optical encoders provide the highest resolution position sensing using light and photodetectors.

#### 4.4.1 Incremental Encoders

**Construction:**
- **Code disc:** Glass or metal disc with alternating transparent/opaque patterns
- **Light source:** LED
- **Photodetectors:** Phototransistors or photodiodes
- **Two channels:** A and B, 90° phase shifted (quadrature)
- **Optional index:** Z channel, one pulse per revolution

**Quadrature Signals:**

```
Channel A: ──┐  ┌──┐  ┌──┐  ┌──
             └──┘  └──┘  └──┘

Channel B: ────┐  ┌──┐  ┌──┐  ┌
               └──┘  └──┘  └──┘

Index Z:  ───┐                  ┌
             └──────────────────┘
             (one pulse per rev)
```

**Direction Detection:**
- If A leads B: Clockwise rotation
- If B leads A: Counter-clockwise rotation

**Position Counting:**
- Count A and B edges (up to 4× multiplication)
- **1× decoding:** Count A rising edges only (N counts/rev)
- **2× decoding:** Count A rising + falling edges (2N counts/rev)
- **4× decoding:** Count all A and B edges (4N counts/rev)

**Example:**
- 1024 lines per revolution (PPR - Pulses Per Revolution)
- With 4× decoding: 4096 counts per revolution
- Resolution: 360° / 4096 = 0.088° = 5.27 arc-minutes

**Advantages:**
- High resolution (1000 to 10,000+ PPR common)
- Low cost (for moderate resolution)
- Simple interface (digital quadrature signals)
- Fast response time

**Disadvantages:**
- Relative position only (need index for absolute reference)
- Position lost if power cycled (must re-home)
- Susceptible to dust, contamination on disc
- Count errors accumulate (no error correction)

#### 4.4.2 Absolute Encoders

**Principle:**

Each position has a unique code pattern. Reading the code gives absolute position immediately on power-up.

**Code Patterns:**

1. **Binary Code:**
   - Natural binary representation
   - Problem: Multiple bits change simultaneously → glitch risk
   ```
   Position 3: 011
   Position 4: 100  (all 3 bits change!)
   ```

2. **Gray Code:**
   - Only one bit changes between adjacent positions
   - Eliminates transition errors
   ```
   Position 3: 010
   Position 4: 110  (only 1 bit changes)
   ```

**Multi-turn Encoders:**
- Track both position within revolution AND number of revolutions
- Typical: 12-bit single-turn + 12-bit multi-turn = 24-bit total
- Battery-backed or energy-harvesting to maintain count
- Total range: 4096 counts × 4096 revolutions = 16.8 million counts

**Communication Protocols:**
- **Parallel:** Multiple output lines (one per bit)
- **SSI (Serial Synchronous Interface):** Clock + data
- **BiSS:** Bidirectional serial (allows configuration)
- **EnDat:** Bidirectional with error detection
- **Profinet, EtherCAT:** Industrial Ethernet protocols

**Advantages:**
- ✓ Absolute position immediately on power-up
- ✓ No homing sequence required
- ✓ Very high resolution (up to 26-bit = 67 million counts)
- ✓ No accumulation of errors
- ✓ Multi-turn capability

**Disadvantages:**
- ✗ Higher cost ($100-$1000+)
- ✗ More complex interface
- ✗ Requires battery or energy harvesting for multi-turn
- ✗ Still susceptible to contamination

#### 4.4.3 Magnetic Encoders

Modern alternative to optical, using Hall sensors and magnetized patterns.

**Principle:**
- Multi-pole magnetic ring on rotor
- Array of Hall sensors or GMR (Giant Magnetoresistance) sensors
- Digital signal processing to interpolate between poles

**Advantages over Optical:**
- More robust (no optical path to contaminate)
- Smaller size possible
- Lower cost
- Immune to oil, dust, condensation

**Disadvantages:**
- Lower resolution than optical (but improving)
- Susceptible to external magnetic fields
- Temperature sensitivity

**Popular ICs:**
- AS5047 (AMS): 14-bit, SPI interface
- MA732 (Monolithic Power): 14-bit, ABZ outputs
- TLE5012 (Infineon): 15-bit, SPI interface

### 4.5 Sensor Comparison and Selection

#### 4.5.1 Comparison Table

| Feature | Hall Sensors | Resolver | Incremental Encoder | Absolute Encoder |
|---------|--------------|----------|---------------------|------------------|
| **Resolution** | ±30° elec. | 12-16 bit | 10-20 bit | 12-26 bit |
| **Cost** | $1-3 | $20-100 | $10-50 | $100-1000 |
| **Absolute Position** | Per sector | Yes | No (need index) | Yes |
| **Environmental Ruggedness** | Good | Excellent | Moderate | Moderate-Good |
| **Temperature Range** | -40 to +125°C | -55 to +155°C | -10 to +85°C | -10 to +100°C |
| **Vibration Resistance** | Excellent | Excellent | Moderate | Moderate |
| **EMI Immunity** | Good | Excellent | Moderate | Good |
| **Signal Type** | Digital | Analog | Digital | Digital |
| **Interface Complexity** | Very Simple | Moderate | Simple | Moderate-Complex |
| **Size** | Very Small | Large | Moderate | Moderate-Large |
| **Accuracy** | ±3° mech. | ±5-30 arc-min | ±1-5 arc-min | ±5-30 arc-sec |
| **Speed Limit** | 15,000 RPM | 10,000 RPM | 10,000+ RPM | 10,000+ RPM |
| **Typical Application** | Cost-sensitive | Harsh environment | Servo drives | Robotics, CNC |

#### 4.5.2 Selection Guidelines

**Choose Hall Sensors when:**
- Budget is tight (cost is primary constraint)
- Application doesn't require precise position
- Torque smoothness can tolerate ±30° error
- Operating environment is moderate
- Zero-speed operation required
- Examples: E-bikes, fans, pumps, tools

**Choose Resolver when:**
- Operating environment is harsh (extreme temp, vibration, radiation)
- High reliability is critical (automotive, aerospace)
- Long lifetime required (20+ years)
- Moderate resolution is sufficient (12-16 bit)
- Budget allows ($50-100 per axis acceptable)
- Examples: EV traction, steering, flight controls

**Choose Incremental Encoder when:**
- High resolution needed at moderate cost
- Homing to index pulse is acceptable
- Clean environment (no heavy dust or liquids)
- Speed or position control (servo applications)
- Examples: CNC, robotics, test equipment

**Choose Absolute Encoder when:**
- Position must be known immediately on power-up
- No homing sequence acceptable
- Highest precision required
- Multi-turn position tracking needed
- Budget allows (high-end applications)
- Examples: Robotics, precision automation, medical devices

### 4.6 Signal Processing and Interface

#### 4.6.1 Hall Sensor Interfacing

**Hardware:**
```
Hall Sensor → Microcontroller GPIO
   (3 pins)      (Digital inputs with pull-ups)
```

**Software - Hall State Decoding:**
```c
// Read Hall sensors (assuming active-high)
uint8_t hall_a = GPIO_Read(HALL_A_PIN);
uint8_t hall_b = GPIO_Read(HALL_B_PIN);
uint8_t hall_c = GPIO_Read(HALL_C_PIN);

// Combine into state (0-7)
uint8_t hall_state = (hall_a << 2) | (hall_b << 1) | hall_c;

// Decode to sector (1-6)
const uint8_t hall_to_sector[8] = {0, 6, 4, 5, 2, 1, 3, 0};
uint8_t sector = hall_to_sector[hall_state];

// Estimate electrical angle
const float sector_angles[7] = {0, 30, 90, 150, 210, 270, 330};
float theta_e_deg = sector_angles[sector];
```

**Speed Measurement:**
```c
// Using timer capture on Hall transitions
void hall_transition_isr(void) {
    static uint32_t last_time = 0;
    uint32_t current_time = timer_get_microseconds();
    uint32_t delta_t = current_time - last_time;

    if (delta_t > 100) {  // Debounce (100 µs min)
        // 60 degrees electrical per transition
        float omega_e = (PI / 3.0) / (delta_t * 1e-6);  // rad/s
        float omega_m = omega_e / pole_pairs;
        speed_rpm = omega_m * (60.0 / (2 * PI));
    }

    last_time = current_time;
}
```

#### 4.6.2 Resolver Interfacing

**Using RDC IC (e.g., AD2S1210):**

```
Resolver → AD2S1210 RDC IC → Microcontroller
(Sin/Cos)   (Demodulation)    (SPI interface)
```

**SPI Communication:**
```c
// Read 16-bit position from AD2S1210
uint16_t read_resolver_position(void) {
    uint16_t position;

    CS_LOW();  // Chip select
    position = spi_transfer_16bit(0xFFFF);  // Dummy write, read data
    CS_HIGH();

    // Convert to angle (16-bit = 0-360 degrees)
    float angle_deg = (position / 65536.0) * 360.0;
    return angle_deg;
}

// Read 12-bit velocity (if RDC supports)
int16_t read_resolver_velocity(void) {
    int16_t velocity;

    CS_LOW();
    spi_send_command(READ_VELOCITY_CMD);
    velocity = spi_transfer_16bit(0xFFFF);
    CS_HIGH();

    return velocity;  // LSB typically in RPM or rad/s
}
```

#### 4.6.3 Encoder Interfacing

**Incremental Encoder with Quadrature Decoder:**

Most microcontrollers have hardware quadrature decoders:

```c
// Initialize hardware encoder peripheral
void encoder_init(void) {
    // Configure TIM2 in encoder mode
    TIM2->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;  // TI1 and TI2 as inputs
    TIM2->CCER = 0;  // Non-inverted
    TIM2->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;  // Encoder mode 3 (count both edges)
    TIM2->ARR = 0xFFFF;  // Auto-reload value
    TIM2->CNT = 0;  // Reset counter
    TIM2->CR1 = TIM_CR1_CEN;  // Enable counter
}

// Read position
int32_t read_encoder_position(void) {
    return (int32_t)TIM2->CNT;
}

// Calculate speed
float calculate_speed(void) {
    static int32_t last_position = 0;
    int32_t current_position = read_encoder_position();
    int32_t delta_position = current_position - last_position;
    last_position = current_position;

    // Speed in rad/s (assuming 1 ms sample time)
    float counts_per_rev = 4096;  // 1024 PPR × 4 (quadrature)
    float speed_rad_s = (delta_position / counts_per_rev) * (2 * PI) / 0.001;

    return speed_rad_s;
}
```

### 4.7 Further Study - Position Sensors

**Books:**
1. **"Rotary Encoder Handbook"** by Dynapar
   - Comprehensive coverage of encoders
   - Free download from Dynapar website

2. **"Resolver and Encoder Conversion Handbook"** by Analog Devices
   - RDC ICs and signal processing
   - Application circuits

3. **"Sensors and Actuators in Mechatronics"** by François Ebrahimi

**Application Notes:**
1. **Texas Instruments:** "Position Sensors for Motor Control" (SLYT527)
2. **Analog Devices:** "A Resolver-to-Digital Converter for UAV Applications" (AN-1333)
3. **Allegro MicroSystems:** "Hall-Effect IC Applications Guide"
4. **AMS:** "AS5047P Rotary Position Sensor" datasheet and app notes

**Videos:**
1. **YouTube: "How Encoders Work"** by Accu-Coder
2. **YouTube: "Resolver Tutorial"** by Parker Hannifin
3. **YouTube: "Hall Effect Sensor Tutorial"** by AddOhms

**Standards:**
- **IEC 61800-5-2:** Encoders for servo drives
- **MIL-STD-3162:** Resolvers for defense applications

---

*End of Section 4 - Position Sensors*

---

## 5. Mathematical Transformations - Detailed Derivations

Now that we understand reference frames conceptually, let's dive into the mathematical details of the transformations that make FOC possible.

### 5.1 The Clarke Transformation (abc → αβ)

#### 5.1.1 Derivation from First Principles

**Goal:** Transform three-phase balanced quantities into two-phase orthogonal quantities while preserving physical relationships.

**Starting Point:**

Three-phase balanced system:
```
ia + ib + ic = 0  (Kirchhoff's Current Law for balanced system)

ia(t) = I·cos(ωt)
ib(t) = I·cos(ωt - 120°)
ic(t) = I·cos(ωt - 240°)
```

**Geometric Approach:**

Imagine the three phase windings as unit vectors in 2D space:

```
Phase A: axis at 0°
Phase B: axis at 120°
Phase C: axis at 240°
```

Unit vectors:
```
a⃗ = [1, 0]ᵀ
b⃗ = [cos(120°), sin(120°)]ᵀ = [-1/2, √3/2]ᵀ
c⃗ = [cos(240°), sin(240°)]ᵀ = [-1/2, -√3/2]ᵀ
```

**Resultant vector:**
```
i⃗_αβ = ia·a⃗ + ib·b⃗ + ic·c⃗

iα = ia·1 + ib·(-1/2) + ic·(-1/2)
iβ = ia·0 + ib·(√3/2) + ic·(-√3/2)
```

**Normalization:**

To preserve power/amplitude, multiply by 2/3:

```
iα = (2/3)·[ia - (1/2)ib - (1/2)ic]
iβ = (2/3)·[(√3/2)ib - (√3/2)ic]
```

Simplified:
```
iα = (2/3)·[ia - (1/2)(ib + ic)]
iβ = (2/3)·(√3/2)·(ib - ic)
```

Using ia + ib + ic = 0, we get ib + ic = -ia:
```
iα = (2/3)·[ia + (1/2)ia] = (2/3)·(3/2)ia = ia  (for balanced system)
```

**Matrix Form:**

```
[iα]     2   [  1      -1/2     -1/2   ] [ia]
[iβ] = ───── [  0     √3/2    -√3/2   ] [ib]
[i0]     3   [ 1/2      1/2      1/2   ] [ic]
```

For balanced systems, i0 (zero-sequence) = 0, so we only need α and β.

#### 5.1.2 Power Invariant vs Amplitude Invariant Forms

**Two common normalizations exist:**

**1. Power Invariant (we use this):**
```
K = 2/3

[iα]     2   [  1      -1/2     -1/2   ] [ia]
[iβ] = ───── [  0     √3/2    -√3/2   ] [ib]
       3                                  [ic]
```

**Properties:**
- Power in abc = Power in αβ
- |i_αβ| = |i_abc| (magnitude preserved)
- Used in most FOC implementations

**2. Amplitude Invariant:**
```
K = √(2/3)

[iα]         [  1      -1/2     -1/2   ] [ia]
[iβ] = √(2/3) [  0     √3/2    -√3/2   ] [ib]
                                          [ic]
```

**Properties:**
- Peak αβ values = peak abc values
- Power NOT preserved (factor of 2/3)
- Sometimes used in theoretical analysis

**We use power-invariant form throughout this handbook.**

#### 5.1.3 Inverse Clarke Transformation

To go back from αβ to abc:

**Derivation:**

We need to project the αβ vector back onto the three phase axes.

```
ia = iα·cos(0°) + iβ·sin(0°) = iα

ib = iα·cos(120°) + iβ·sin(120°)
   = iα·(-1/2) + iβ·(√3/2)
   = -(1/2)iα + (√3/2)iβ

ic = iα·cos(240°) + iβ·sin(240°)
   = iα·(-1/2) + iβ·(-√3/2)
   = -(1/2)iα - (√3/2)iβ
```

**Matrix Form:**
```
[ia]   [    1         0    ] [iα]
[ib] = [ -1/2      √3/2    ] [iβ]
[ic]   [ -1/2     -√3/2    ]
```

**Verification:**
ia + ib + ic = 1·iα + (-1/2)iα + (-1/2)iα + 0 + (√3/2)iβ + (-√3/2)iβ = 0 ✓

### 5.2 The Park Transformation (αβ → dq)

#### 5.2.1 Derivation as Rotation Matrix

**Goal:** Rotate the αβ frame by angle θ to align with the rotor.

**Standard 2D Rotation:**

To rotate a vector by angle θ clockwise:
```
[x']   [ cos(θ)   sin(θ)] [x]
[y'] = [-sin(θ)   cos(θ)] [y]
```

**For Park Transformation:**

We want to rotate the αβ frame to align with the rotor at electrical angle θe:

```
[id]   [ cos(θe)   sin(θe)] [iα]
[iq] = [-sin(θe)   cos(θe)] [iβ]
```

**Physical Interpretation:**

- **id:** Component of current aligned with d-axis (rotor flux direction)
- **iq:** Component of current aligned with q-axis (perpendicular to rotor flux)

When θe = 0 (rotor aligned with α-axis):
```
id = iα,  iq = -iβ
```

When θe = 90° (rotor aligned with β-axis):
```
id = iβ,  iq = iα
```

#### 5.2.2 Expanded Form

```
id = iα·cos(θe) + iβ·sin(θe)
iq = -iα·sin(θe) + iβ·cos(θe)
```

**Why the negative sign on iq?**

By convention, we define q-axis as 90° **ahead** of d-axis in the direction of rotation. The negative sign in the transformation ensures this orientation.

Alternative convention (some textbooks):
```
id = iα·cos(θe) + iβ·sin(θe)
iq = iα·sin(θe) - iβ·cos(θe)  (different sign convention)
```

Both work, but sign affects how we interpret iq (motoring vs generating). We use the first convention.

#### 5.2.3 Inverse Park Transformation

To rotate back from dq to αβ:

```
[iα]   [ cos(θe)  -sin(θe)] [id]
[iβ] = [ sin(θe)   cos(θe)] [iq]
```

Expanded:
```
iα = id·cos(θe) - iq·sin(θe)
iβ = id·sin(θe) + iq·cos(θe)
```

**Verification (Round-trip):**

Starting with (id, iq), apply inverse Park, then Park:

```
iα = id·cos(θe) - iq·sin(θe)
iβ = id·sin(θe) + iq·cos(θe)

id' = iα·cos(θe) + iβ·sin(θe)
    = (id·cos²(θe) - iq·sin(θe)cos(θe)) + (id·sin²(θe) + iq·sin(θe)cos(θe))
    = id·(cos²(θe) + sin²(θe)) + iq·(sin(θe)cos(θe) - sin(θe)cos(θe))
    = id ✓

iq' = -iα·sin(θe) + iβ·cos(θe)
    = -(id·cos(θe)sin(θe) - iq·sin²(θe)) + (id·sin(θe)cos(θe) + iq·cos²(θe))
    = iq·(sin²(θe) + cos²(θe)) + id·(-cos(θe)sin(θe) + sin(θe)cos(θe))
    = iq ✓
```

### 5.3 Complete Transformation Chain

#### 5.3.1 abc → dq (Forward)

Combining Clarke and Park:

```
Step 1: [iα, iβ] = Clarke(ia, ib, ic)
Step 2: [id, iq] = Park(iα, iβ, θe)
```

**Combined Matrix:**
```
[id]     2   [ cos(θe)   cos(θe-120°)   cos(θe-240°)] [ia]
[iq] = ───── [-sin(θe)  -sin(θe-120°)  -sin(θe-240°)] [ib]
       3                                                [ic]
```

This is rarely used directly; we compute Clarke then Park separately for clarity.

#### 5.3.2 dq → abc (Reverse)

```
Step 1: [vα, vβ] = InvPark(vd, vq, θe)
Step 2: [va, vb, vc] = InvClarke(vα, vβ)
```

**Combined Matrix:**
```
[va]   [   cos(θe)     -sin(θe)  ] [vd]
[vb] = [ cos(θe-120°) -sin(θe-120°)] [vq]
[vc]   [ cos(θe-240°) -sin(θe-240°)]
```

### 5.4 Space Vector Pulse Width Modulation (SVPWM)

#### 5.4.1 Voltage Space Vectors

**The 8 Basic Vectors:**

A three-phase inverter has 2³ = 8 possible switching states:

| State | Sa | Sb | Sc | Vα | Vβ | Name | Sector |
|-------|----|----|----|----|----|----|--------|
| V0 | 0 | 0 | 0 | 0 | 0 | Zero | - |
| V1 | 1 | 0 | 0 | 2Vdc/3 | 0 | Active | 1 |
| V2 | 1 | 1 | 0 | Vdc/3 | Vdc/√3 | Active | 2 |
| V3 | 0 | 1 | 0 | -Vdc/3 | Vdc/√3 | Active | 3 |
| V4 | 0 | 1 | 1 | -2Vdc/3 | 0 | Active | 4 |
| V5 | 0 | 0 | 1 | -Vdc/3 | -Vdc/√3 | Active | 5 |
| V6 | 1 | 0 | 1 | Vdc/3 | -Vdc/√3 | Active | 6 |
| V7 | 1 | 1 | 1 | 0 | 0 | Zero | - |

**Visualization:**
```
         V3 (010)
          ╱ ↑ ╲
    V4 ──╱──┼──╲── V2
   (011) ╲  │  ╱ (110)
          ╲ │ ╱
     ───────┼───────V1 (100)
            │╲
            │ ╲
      V5    │  ╲ V6
     (001)  ↓  (101)
```

**Maximum Voltage Circle:**

The largest circle that fits inside the hexagon has radius:
```
V_max = Vdc / √3 ≈ 0.577·Vdc
```

This is 15% larger than the maximum achievable with sinusoidal PWM!

#### 5.4.2 SVPWM Algorithm

**Goal:** Synthesize desired voltage vector V* using adjacent vectors and zero vectors.

**For reference voltage (Vα*, Vβ*):**

1. **Determine Sector:**
```
θ = atan2(Vβ*, Vα*)
sector = floor(θ / (π/3)) + 1
```

2. **Calculate angle within sector:**
```
θ_sector = θ - (sector-1)·(π/3)
```

3. **Calculate duty cycles for adjacent vectors:**

For sector 1 (using V1 and V2):
```
T1 = m·Ts·sin(π/3 - θ_sector) / sin(π/3)
T2 = m·Ts·sin(θ_sector) / sin(π/3)
T0 = Ts - T1 - T2

where m = |V*| / V_max  (modulation index)
```

4. **Map to three-phase duty cycles:**

Using centered PWM (symmetric placement of zero vectors):

**Sector 1:**
```
Ta = (T1 + T2 + T0/2) / Ts
Tb = (T2 + T0/2) / Ts
Tc = (T0/2) / Ts
```

**General formula for sector n:**
Pattern rotates through sectors following switching sequence for minimum switching losses.

#### 5.4.3 SVPWM vs Sinusoidal PWM

**Sinusoidal PWM:**
```
ma = M·sin(ωt)
mb = M·sin(ωt - 120°)
mc = M·sin(ωt - 240°)

V_max = M·Vdc/2  (for M ≤ 1)
```

**SVPWM:**
```
V_max = Vdc/√3 = 0.577·Vdc
```

**Comparison:**
```
SVPWM advantage = 0.577/(0.5) = 1.15 = 15% more voltage
```

**Why SVPWM is better:**
- Higher DC bus utilization
- Lower harmonic distortion
- Better suited for digital implementation
- Optimal switching sequence (fewer transitions)

### 5.5 Implementation Considerations

#### 5.5.1 Computational Efficiency

**Trig Function Approximations:**

For real-time systems, sin/cos calculations can be expensive.

**Options:**

1. **Lookup Tables:**
```c
// Pre-calculated table (e.g., 360 entries for 1° resolution)
const float sin_table[360] = {...};
const float cos_table[360] = {...};

float sin_lookup(float angle_deg) {
    int index = ((int)angle_deg) % 360;
    return sin_table[index];
}
```

2. **CORDIC Algorithm:**
- Iterative approximation
- Only uses shifts and adds
- Hardware-friendly
- ~10-15 iterations for good precision

3. **Taylor Series Approximation:**
```
sin(x) ≈ x - x³/6 + x⁵/120  (for small x near 0)
cos(x) ≈ 1 - x²/2 + x⁴/24
```

4. **Hardware Acceleration:**
- ARM Cortex-M4F: Hardware FPU with sin/cos
- DSP processors: Dedicated trig units
- Modern MCUs: 1-2 cycles for trig functions

#### 5.5.2 Fixed-Point vs Floating-Point

**Floating-Point (recommended for modern MCUs):**
```c
float clarke_alpha = (2.0f/3.0f) * (ia - 0.5f*ib - 0.5f*ic);
float clarke_beta = (2.0f/3.0f) * (0.866f) * (ib - ic);  // 0.866 ≈ √3/2
```

**Fixed-Point (for embedded without FPU):**
```c
// Q15 format: 16-bit signed, 15 fractional bits
#define Q15(x) ((int16_t)((x) * 32768.0f))

int16_t TWO_THIRDS_Q15 = Q15(2.0/3.0);   // 21845
int16_t SQRT3_2_Q15 = Q15(0.866);        // 28378

int16_t clarke_alpha_q15 = (TWO_THIRDS_Q15 * (ia - ((ib + ic) >> 1))) >> 15;
```

#### 5.5.3 Saturation and Limiting

**Voltage Limits:**

After inverse transformations, ensure voltages don't exceed hardware limits:

```c
// After inverse Clarke
float v_magnitude = sqrtf(v_alpha*v_alpha + v_beta*v_beta);
float v_max = vdc / sqrtf(3.0f);

if (v_magnitude > v_max) {
    // Scale down proportionally
    float scale = v_max / v_magnitude;
    v_alpha *= scale;
    v_beta *= scale;
}
```

### 5.6 Worked Examples

#### Example 5.1: Complete Transformation

**Given:**
- Three-phase currents at t=0
- ia = 10A, ib = -5A, ic = -5A
- Rotor position: θe = 45°
- Calculate id and iq

**Solution:**

**Step 1: Clarke Transformation**
```
iα = (2/3)·[10 - 0.5·(-5) - 0.5·(-5)]
   = (2/3)·[10 + 2.5 + 2.5]
   = (2/3)·15
   = 10 A

iβ = (2/3)·(√3/2)·[(-5) - (-5)]
   = (2/3)·(0.866)·0
   = 0 A
```

**Step 2: Park Transformation with θe = 45°**
```
id = iα·cos(45°) + iβ·sin(45°)
   = 10·(0.707) + 0·(0.707)
   = 7.07 A

iq = -iα·sin(45°) + iβ·cos(45°)
   = -10·(0.707) + 0·(0.707)
   = -7.07 A
```

**Interpretation:**
- Magnitude: √(id² + iq²) = √(50 + 50) = 10 A ✓ (conserved)
- Torque: T = Kt·iq = Kt·(-7.07) → Negative torque (generating/braking)
- Flux component: id = 7.07 A (non-zero d-axis current)

#### Example 5.2: SVPWM Duty Cycle Calculation

**Given:**
- Desired voltage: Vα* = 12V, Vβ* = 8V
- DC bus: Vdc = 28V
- PWM period: Ts = 100µs

**Solution:**

**Step 1: Calculate magnitude and angle**
```
|V*| = √(12² + 8²) = √(144 + 64) = 14.42 V

θ = atan2(8, 12) = 33.69° = 0.588 rad
```

**Step 2: Determine sector**
```
sector = floor(33.69° / 60°) + 1 = 1
```

**Step 3: Angle within sector**
```
θ_sector = 33.69° - 0° = 33.69° = 0.588 rad
```

**Step 4: Modulation index**
```
V_max = 28 / √3 = 16.17 V
m = 14.42 / 16.17 = 0.892
```

**Step 5: Calculate T1, T2, T0**
```
T1 = 0.892 × 100µs × sin(60° - 33.69°) / sin(60°)
   = 89.2µs × sin(26.31°) / 0.866
   = 89.2µs × 0.444 / 0.866
   = 45.75 µs

T2 = 0.892 × 100µs × sin(33.69°) / sin(60°)
   = 89.2µs × 0.555 / 0.866
   = 57.17 µs

T0 = 100µs - 45.75µs - 57.17µs = -2.92 µs ≈ 0
```

(Small negative value due to rounding; set to 0 in practice)

**Step 6: Duty cycles (Sector 1)**
```
Ta = (T1 + T2 + T0/2) / Ts = (45.75 + 57.17 + 0) / 100 = 1.029 ≈ 1.0
Tb = (T2 + T0/2) / Ts = (57.17 + 0) / 100 = 0.572
Tc = (T0/2) / Ts = 0 / 100 = 0
```

**Note:** Ta slightly exceeds 1.0 due to rounding. In practice, all duties would be scaled to fit [0, 1].

### 5.7 Key Takeaways - Mathematical Transformations

1. **Clarke reduces variables:** 3-phase abc → 2-phase αβ (exploit balanced system constraint)

2. **Park aligns with rotor:** αβ (stationary) → dq (rotating), making AC quantities appear DC

3. **Power is conserved:** Using 2/3 normalization ensures |i_abc| = |i_αβ| = |i_dq|

4. **Transformations are reversible:** Can go back and forth without information loss

5. **SVPWM is optimal:** 15% better voltage utilization than sinusoidal PWM

6. **Implementation tradeoffs:** Trig functions vs tables vs approximations

7. **Saturation must be handled:** Voltage requests can exceed hardware capability

### 5.8 Further Study - Mathematical Transformations

**Books:**
1. **"Power Electronics and Motor Drives"** by Bimal K. Bose
   - Chapter 6: Space Vector PWM
   - Detailed SVPWM implementation

2. **"Advanced Electric Drives"** by Rik De Doncker
   - Appendix A: Mathematical Transformations
   - Rigorous derivations

3. **"Vector Control and Dynamics of AC Drives"** by Novotny & Lipo
   - Chapter 4: Reference Frame Theory
   - The classic treatment

**Application Notes:**
1. **Texas Instruments:** "Clarke & Park Transforms on C2000" (SPRAAA3)
2. **STMicroelectronics:** "SVPWM Generation" (AN4776)
3. **Microchip:** "Space Vector Modulation" (AN908)

**Papers:**
1. **"Generalised Theory of Direct Torque Control for AC Drives"** by Habetler et al.
2. **"Modeling and Analysis of Space Vector Modulation"** by Holtz

**Videos:**
1. **MATLAB:** "Understanding Clarke and Park Transforms"
2. **Texas Instruments:** "Introduction to Space Vector PWM"

**Interactive Tools:**
1. **PLECS Demo:** SVPWM visualization
2. **Simulink:** Power Electronics Blockset examples
3. **Python/Jupyter:** Transformation visualizations

---

*End of Section 5 - Mathematical Transformations*

---

## Section 6: Field Oriented Control (FOC) Theory

### Introduction

Field Oriented Control (FOC), also known as **vector control**, is the revolutionary control strategy that transformed AC motor drives in the 1970s and 1980s. Before FOC, AC motors were difficult to control precisely - they had sluggish torque response and couldn't match the performance of DC motors. FOC changed everything by making an AC motor behave like a separately-excited DC motor.

**The FOC breakthrough:** By transforming AC motor variables into the rotating dq reference frame and controlling them independently, we can achieve:
- Instantaneous torque control (like a DC motor)
- Decoupled flux and torque control
- Maximum efficiency operation
- Fast dynamic response (millisecond-level torque changes)
- Wide speed range with field weakening

This section explains **how FOC works**, **why it works**, and **how to implement it** for PMSM motors. We'll build from conceptual understanding to practical implementation details.

### 6.1 FOC Algorithm Overview - The Big Picture

#### 6.1.1 The FOC Block Diagram

Let's start with the complete FOC system and understand each block:

```
Reference Inputs                    Measured Feedback
  ω_ref (speed)                     θe (rotor position)
  τ_ref (torque)                    ia, ib, ic (phase currents)
       ↓                                      ↓
┌──────────────────────────────────────────────────────────────┐
│                     FOC CONTROLLER                            │
│                                                               │
│  ┌─────────────┐         ┌──────────────┐                   │
│  │   Speed     │  i*q    │   Current    │  V*d, V*q         │
│  │ Controller  │────────→│  Controller  │────────┐          │
│  │   (PI)      │         │   (2×PI)     │        │          │
│  └─────────────┘         └──────────────┘        │          │
│        ↑                        ↑                 │          │
│        │ ω_measured             │ id, iq          ↓          │
│        │                        │          ┌────────────┐    │
│        │                 ┌──────┴──────┐   │  Inverse   │    │
│        │                 │    Park     │   │   Park     │    │
│        │                 │ Transform   │   │ Transform  │    │
│        │                 │  (αβ→dq)    │   │  (dq→αβ)   │    │
│        │                 └──────────────┘   └────────────┘    │
│        │                        ↑                 │          │
│        │                  iα, iβ│                 │ Vα, Vβ   │
│        │                 ┌──────┴──────┐          ↓          │
│        │                 │   Clarke    │    ┌──────────┐    │
│        │                 │ Transform   │    │  SVPWM   │    │
│        │                 │  (abc→αβ)   │    │          │    │
│        │                 └─────────────┘    └──────────┘    │
│        │                        ↑                 │          │
└────────┼────────────────────────┼─────────────────┼──────────┘
         │                        │                 │
         │                    ia,ib,ic          Ta,Tb,Tc
         │                        │             (duty cycles)
         │                        ↓                 ↓
      ┌──┴────┐           ┌─────────────┐   ┌─────────────┐
      │ Speed │           │   Current   │   │  3-Phase    │
      │ Calc  │           │   Sensors   │   │  Inverter   │
      └───────┘           └─────────────┘   └─────────────┘
         ↑                        ↑                 │
         │                        │                 ↓
      ┌──┴────────────────────────┴────────────────────┐
      │              PMSM MOTOR                        │
      │         (electromagnetic torque                │
      │          drives mechanical load)                │
      └────────────────────────────────────────────────┘
               ↑
         Position Sensor
         (Hall/Encoder)
```

#### 6.1.2 Information Flow in FOC

Let's trace the information flow through the system:

**Forward Path (Command to Motor):**

1. **Speed Reference** → Speed PI controller → **Torque command (i*q)**
2. **Torque command** → Current PI controller → **Voltage command in dq (V*d, V*q)**
3. **dq voltages** → Inverse Park transform (using θe) → **αβ voltages (V*α, V*β)**
4. **αβ voltages** → SVPWM → **PWM duty cycles (Ta, Tb, Tc)**
5. **Duty cycles** → Gate drivers → **Inverter** → **3-phase voltages (Va, Vb, Vc)**
6. **3-phase voltages** → **Motor** → **Electromagnetic torque** → **Mechanical rotation**

**Feedback Path (Sensing and Transformation):**

1. **Motor rotation** → Position sensor → **Rotor angle (θe)**
2. **Phase currents** → ADC → **Digital values (ia, ib, ic)**
3. **abc currents** → Clarke transform → **αβ currents (iα, iβ)**
4. **αβ currents** → Park transform (using θe) → **dq currents (id, iq)**
5. **dq currents** → Current controllers → Close current loop
6. **Rotor angle** → Speed calculation → **Measured speed (ω_measured)**
7. **Measured speed** → Speed controller → Close speed loop

**Key Observations:**

- **Two control loops:** Outer speed loop (slow, ~1 kHz) and inner current loop (fast, ~10-20 kHz)
- **Transformations are bidirectional:** Forward path uses inverse Park/Clarke; feedback uses Clarke/Park
- **Position is critical:** θe is needed for Park/inverse Park transforms
- **Everything happens in dq:** Control laws operate on DC quantities (id, iq)

#### 6.1.3 Cascade Control Architecture

FOC uses a **cascade control structure** with two loops:

**Inner Loop - Current Control:**
- **Objective:** Force id and iq to track their references (i*d, i*q)
- **Sample rate:** Fast (10-20 kHz, matches PWM frequency)
- **Bandwidth:** High (~1 kHz, limited by electrical time constant)
- **Controller:** Two PI controllers (one for d-axis, one for q-axis)
- **Time constant:** τ_elec = Ld/Rs ≈ 1-5 ms for typical PMSMs

**Outer Loop - Speed Control:**
- **Objective:** Force speed ω to track reference ω_ref
- **Sample rate:** Slower (1-5 kHz)
- **Bandwidth:** Lower (~50-200 Hz, limited by mechanical time constant)
- **Controller:** One PI controller
- **Time constant:** τ_mech = J/B ≈ 20-200 ms for typical systems

**Why cascade control?**

1. **Separation of timescales:** Electrical dynamics (ms) are much faster than mechanical dynamics (tens of ms)
2. **Current limiting:** Inner loop can enforce current limits for motor protection
3. **Improved disturbance rejection:** Inner loop rejects electrical disturbances before they affect speed
4. **Simplified tuning:** Can tune loops independently (inner first, then outer)

**Design rule:** Inner loop should be **5-10 times faster** than outer loop to ensure proper separation.

```
Speed loop bandwidth: ~50 Hz → Current loop bandwidth: ~500 Hz
```

### 6.2 Why FOC Works - The Decoupling Principle

#### 6.2.1 The Problem with abc Frame Control

Why can't we just control the motor in the natural abc frame? Let's see what happens:

**Motor voltage equations in abc frame:**
```
Va = Rs·ia + d(ψa)/dt
Vb = Rs·ib + d(ψb)/dt
Vc = Rs·ic + d(ψc)/dt
```

Where the flux linkages ψa, ψb, ψc depend on:
- Stator currents (ia, ib, ic) through self and mutual inductances
- Rotor position (θe) through magnet flux
- Rotor speed (ωe) through back-EMF

**The coupling problem:**
```
d(ψa)/dt = La·dia/dt + M·dib/dt + M·dic/dt + d(λm·cos(θe))/dt
         = La·dia/dt + M·dib/dt + M·dic/dt - λm·ωe·sin(θe)
```

Notice the issues:
1. **Time-varying coefficients:** sin(θe), cos(θe) change constantly as motor spins
2. **Cross-coupling:** Current ia affects flux in phases b and c through mutual inductance M
3. **Speed dependency:** Back-EMF term ωe·sin(θe) increases with speed
4. **AC quantities:** All currents and voltages are sinusoidal at electrical frequency

**Result:** Trying to control ia, ib, ic directly is like trying to hit three moving targets that are coupled together and oscillating at high frequency. Very difficult!

#### 6.2.2 The Magic of the dq Transformation

Now let's see the same system in the dq (rotor reference) frame:

**Voltage equations in dq frame:**
```
Vd = Rs·id + Ld·did/dt - ωe·Lq·iq
Vq = Rs·iq + Lq·diq/dt + ωe·Ld·id + ωe·λm
```

**Torque equation:**
```
τ = (3/2)·P·[λm·iq + (Ld - Lq)·id·iq]
```

**What changed? Everything got better:**

1. **DC quantities:** In steady state, id and iq are constants (not sinusoids!)
2. **Clear separation:**
   - id controls flux (magnetization)
   - iq controls torque
3. **Speed terms become predictable:** ωe·Lq·iq and ωe·Ld·id can be compensated (feedforward)
4. **Linear control possible:** PI controllers work excellently on DC signals

#### 6.2.3 The Physical Interpretation

Let's understand what's happening physically:

**In the abc frame:**
- We're standing still while the rotor magnetic field spins past us
- The magnetic field looks like a rotating wave
- To control it, we have to generate 3-phase sinusoidal currents at exactly the right frequency, phase, and amplitude
- It's like trying to push a spinning merry-go-round by running alongside it

**In the dq frame:**
- We're riding on the rotor (rotating with it)
- The rotor magnetic field is now stationary relative to us
- To control it, we just maintain constant DC currents in d and q directions
- It's like sitting on the merry-go-round and simply pushing in a fixed direction

**Analogy:** Imagine you're trying to paint a specific spot on a spinning wheel:
- **abc frame:** You stand still and try to time your brush strokes as the spot spins past - very hard!
- **dq frame:** You rotate with the wheel and the spot is now stationary - easy!

#### 6.2.4 Decoupling in Detail

For a **surface-mounted PMSM** (SPMSM), where Ld ≈ Lq ≈ L:

```
Vd = Rs·id + L·did/dt - ωe·L·iq       [d-axis voltage equation]
Vq = Rs·iq + L·diq/dt + ωe·L·id + ωe·λm   [q-axis voltage equation]

τ = (3/2)·P·λm·iq                      [torque equation]
```

**Notice:**
1. **id affects Vq:** Through the term ωe·L·id
2. **iq affects Vd:** Through the term -ωe·L·iq
3. **Only iq affects torque:** τ is proportional to iq (if id = 0)

**The FOC solution:**

We can achieve **complete decoupling** by:

1. **Set id* = 0:** This maximizes torque per ampere for SPMSMs
2. **Control iq to control torque:** τ = (3/2)·P·λm·iq
3. **Add feedforward compensation:**
   ```
   Vd_ff = -ωe·L·iq     [compensate iq effect on d-axis]
   Vq_ff = ωe·L·id + ωe·λm  [compensate id effect and back-EMF on q-axis]
   ```

**With feedforward compensation:**
```
Vd_total = Vd_PI + Vd_ff = Rs·id + L·did/dt
Vq_total = Vq_PI + Vq_ff = Rs·iq + L·diq/dt
```

Now the equations are **completely decoupled!**
- Vd controls id independently
- Vq controls iq independently
- iq controls torque independently

**This is why FOC works:** We've transformed a complex, coupled, time-varying AC system into two simple, decoupled, time-invariant DC systems.

### 6.3 The id=0 Control Strategy for Surface PMSMs

#### 6.3.1 Why Set id = 0?

For surface-mounted PMSMs (SPMSMs) where Ld ≈ Lq, the torque equation is:

```
τ = (3/2)·P·λm·iq
```

Notice that torque depends **only on iq**, not on id. So what does id do?

**The d-axis current id:**
- Aligns with the rotor magnetic field direction
- Creates flux in the same direction as the permanent magnets
- Does **not** contribute to torque (for SPMSMs)
- Increases copper losses: P_loss = Rs·(id² + iq²)

**Conclusion:** For maximum efficiency, we should set **id = 0** and use **only iq for torque control**.

**Physical interpretation:**
- id = 0 means no current in the direction of the rotor field
- All current is perpendicular to the rotor field (iq direction)
- This creates maximum torque per ampere
- It's like pushing a door: push perpendicular to the hinge (efficient) vs. pushing toward the hinge (wasteful)

#### 6.3.2 Control Law for id=0 Strategy

**Step 1: Set d-axis reference to zero**
```
i*d = 0  (always)
```

**Step 2: Calculate q-axis reference from torque command**
```
τ* = (3/2)·P·λm·i*q

Therefore:
i*q = τ* / [(3/2)·P·λm]
     = 2·τ* / (3·P·λm)
```

Or, if we have a speed controller:
```
i*q = Speed_PI(ω_ref - ω_measured)
```

**Step 3: Current controllers force id and iq to track references**
```
V*d = PI_d(i*d - id) + Vd_ff
V*q = PI_q(i*q - iq) + Vq_ff
```

Where the feedforward terms are:
```
Vd_ff = -ωe·Lq·iq
Vq_ff = ωe·Ld·id + ωe·λm ≈ ωe·λm  (since id ≈ 0)
```

#### 6.3.3 Maximum Torque Per Ampere (MTPA)

The id=0 strategy is actually a special case of **Maximum Torque Per Ampere (MTPA)** control.

**Total current magnitude:**
```
I_total = √(id² + iq²)
```

**For SPMSM with id=0:**
```
I_total = |iq|
τ = (3/2)·P·λm·iq

Torque per ampere = τ / I_total = (3/2)·P·λm
```

This is the **maximum possible** torque per ampere for an SPMSM!

**Graphical interpretation:**

```
      iq
       ↑
       │
       │   ×  Operating point (id=0, iq>0)
       │   │
       │   │ I_total
       │   │
───────┼───┴──────→ id
       │

Torque contours (hyperbolas): τ = (3/2)·P·λm·iq
Current limit circle: id² + iq² = I²_max

For SPMSM, MTPA trajectory is the vertical line id=0
```

#### 6.3.4 Limitations of id=0 Strategy

The id=0 strategy works perfectly **below base speed**, but has limitations:

**1. Voltage limit at high speed:**

As speed increases, back-EMF increases:
```
Vq_required ≈ ωe·λm + Rs·iq + Lq·diq/dt
```

Eventually, Vq_required exceeds available inverter voltage:
```
√(Vd² + Vq²) ≤ Vdc/√3
```

**Solution:** Field weakening (inject negative id to reduce flux)

**2. Not optimal for IPMSMs:**

For interior PMSMs where Ld < Lq, the torque equation is:
```
τ = (3/2)·P·[λm·iq + (Ld - Lq)·id·iq]
                     └─ Reluctance torque
```

Negative id can actually **increase** total torque due to reluctance torque.

**Solution:** MTPA optimization algorithm (covered in advanced topics)

**3. Position detection at standstill:**

With id=0, there's no excitation in the d-axis, making sensorless position estimation difficult at zero speed.

**Solution:** Inject small id during startup, or use high-frequency injection methods

#### 6.3.5 Practical Implementation Considerations

**Current limit handling:**

Even with id*=0, the actual id might not be exactly zero. We need to limit total current:

```
I_max = √(i*d² + i*q²)

If I_max > I_rated:
    scale_factor = I_rated / I_max
    i*d = i*d × scale_factor
    i*q = i*q × scale_factor
```

**Anti-windup:**

When voltage saturates, PI integrators must not wind up:

```matlab
% Current PI with anti-windup
error = i_ref - i_measured
proportional = Kp * error
integral = integral + Ki * error * Ts

% Calculate desired voltage
v_desired = proportional + integral

% Apply voltage limit
v_limited = saturate(v_desired, V_max)

% Back-calculate to prevent windup
if v_limited != v_desired:
    integral = v_limited - proportional
```

**Startup sequence:**

1. **Align rotor:** Apply dc current in phase A for 100-500 ms to align rotor
2. **Ramp speed:** Slowly increase ω_ref from 0 to target
3. **Monitor currents:** Ensure id stays near 0, iq follows command
4. **Check position:** Verify Hall states match expected sequence

---

### 6.4 Current Control Loop Design

The current control loop is the **inner, fast loop** that forces the actual motor currents (id, iq) to track their references (i*d, i*q). This is the heart of FOC - without good current control, everything else fails.

#### 6.4.1 Current Loop Plant Model

First, let's understand what we're controlling. The PMSM electrical equations in dq frame are:

```
Vd = Rs·id + Ld·did/dt - ωe·Lq·iq       [d-axis]
Vq = Rs·iq + Lq·diq/dt + ωe·Ld·id + ωe·λm   [q-axis]
```

If we add feedforward compensation to cancel the cross-coupling terms:

```
Vd_ff = -ωe·Lq·iq
Vq_ff = ωe·Ld·id + ωe·λm
```

Then the decoupled equations become:

```
Vd_PI = Rs·id + Ld·did/dt
Vq_PI = Rs·iq + Lq·diq/dt
```

**Transfer function for d-axis:**
```
         id(s)           1/Rs              1/τd
Gd(s) = ------- = ---------------- = -------------
        Vd(s)     1 + (Ld/Rs)·s      1 + τd·s

where τd = Ld/Rs  (electrical time constant)
```

**Transfer function for q-axis:**
```
         iq(s)           1/Rs              1/τq
Gq(s) = ------- = ---------------- = -------------
        Vq(s)     1 + (Lq/Rs)·s      1 + τq·s

where τq = Lq/Rs
```

**This is a first-order system!** Very simple to control with PI.

**Typical values for a small PMSM:**
```
Rs = 0.5 Ω
Ld = Lq = 1 mH
τ = L/Rs = 1e-3/0.5 = 2 ms
Corner frequency: fc = 1/(2π·τ) = 80 Hz
```

#### 6.4.2 PI Controller Design

For each axis (d and q), we use a PI controller:

```
         Kp·s + Ki
C(s) = ------------ = Kp + Ki/s
             s
```

**Continuous-time control law:**
```
V*(t) = Kp·e(t) + Ki·∫e(τ)dτ

where e(t) = i*(t) - i(t)  (current error)
```

**Discrete-time implementation (more common):**
```
V*[k] = Kp·e[k] + Vi[k]

where:
  e[k] = i*[k] - i[k]
  Vi[k] = Vi[k-1] + Ki·Ts·e[k]  (integrator state)
```

#### 6.4.3 PI Gain Calculation - Bandwidth Method

The most common tuning method is to **specify desired bandwidth** and calculate gains accordingly.

**Closed-loop transfer function with PI controller:**

```
                C(s)·G(s)              (Kp·s + Ki)·(1/Rs)
T(s) = ------------------------- = ---------------------------
        1 + C(s)·G(s)            s² + (Rs/L + Kp/L)·s + Ki/L
```

**For a second-order system, we want:**
```
T(s) = ωn² / (s² + 2·ζ·ωn·s + ωn²)

where:
  ωn = natural frequency (rad/s)
  ζ = damping ratio (typically 0.707 for critical damping)
```

**Matching coefficients:**

```
2·ζ·ωn = Rs/L + Kp/L    →    Kp = 2·ζ·ωn·L - Rs
ωn² = Ki/L              →    Ki = ωn²·L
```

**For critical damping (ζ = 0.707) and bandwidth ωbw:**

```
ωn = ωbw / √(1 - 2·ζ² + √(4·ζ⁴ - 4·ζ² + 2))
   ≈ ωbw  (for ζ = 0.707)

Therefore:
Kp = 2·0.707·ωbw·L - Rs ≈ √2·ωbw·L - Rs
Ki = ωbw²·L
```

**Simplified formulas (commonly used):**

For bandwidth fbw in Hz:
```
ωbw = 2π·fbw

Kp = L·ωbw
Ki = Rs·ωbw
```

These simplified formulas give good performance and are widely used in industry!

#### 6.4.4 Design Example - Current Loop

Let's design a current controller for our example motor.

**Given parameters:**
```
Rs = 0.5 Ω
Ld = Lq = 1 mH = 0.001 H
P = 4 pole pairs
Vdc = 24 V
fsw = 10 kHz (PWM frequency)
```

**Step 1: Choose current loop bandwidth**

Rule of thumb: **Current loop bandwidth = 1/10 of PWM frequency**

```
fbw_current = fsw / 10 = 10000 / 10 = 1000 Hz
ωbw_current = 2π·1000 = 6283 rad/s
```

This ensures the controller can respond within one PWM cycle.

**Step 2: Calculate PI gains**

```
Kp = L·ωbw = 0.001 × 6283 = 6.28
Ki = Rs·ωbw = 0.5 × 6283 = 3142
```

**Step 3: Convert to discrete time**

For sampling period Ts = 1/fsw = 100 μs:

```matlab
% Discrete PI implementation
error = i_ref - i_measured
proportional = Kp * error
integrator = integrator + Ki * Ts * error

v_out = proportional + integrator
```

**Step 4: Add anti-windup**

When voltage saturates, prevent integrator windup:

```matlab
% Calculate voltage limit
V_max = Vdc / sqrt(3) = 24 / 1.732 = 13.86 V

% Apply saturation
v_limited = saturate(v_out, -V_max, V_max)

% Back-calculate integrator
if v_limited ~= v_out
    integrator = v_limited - proportional
end
```

#### 6.4.5 Advanced Current Control Techniques

**1. Feedforward Decoupling**

Add cross-coupling compensation for perfect decoupling:

```matlab
% Measure speed
omega_e = speed_measured * pole_pairs

% Calculate feedforward terms
Vd_ff = -omega_e * Lq * iq
Vq_ff = omega_e * Ld * id + omega_e * lambda_m

% Total voltage command
Vd_total = Vd_PI + Vd_ff
Vq_total = Vq_PI + Vq_ff
```

This dramatically improves dynamic response!

**2. Active Damping**

Add derivative term to reduce overshoot:

```matlab
% PID controller (optional)
derivative = Kd * (e[k] - e[k-1]) / Ts

v_out = proportional + integrator + derivative
```

Typically not needed if bandwidth is chosen correctly.

**3. Current Limiting**

Protect motor and inverter:

```matlab
% Calculate total current
I_total = sqrt(id^2 + iq^2)

% If over limit, scale references
if I_total > I_max
    scale = I_max / I_total
    id_ref = id_ref * scale
    iq_ref = iq_ref * scale
end
```

**4. Voltage Limiting**

Ensure voltage stays within hexagon boundary:

```matlab
% Calculate voltage magnitude
V_total = sqrt(Vd^2 + Vq^2)

% Maximum voltage (linear region)
V_max = Vdc / sqrt(3)

% If over limit, scale voltages
if V_total > V_max
    scale = V_max / V_total
    Vd = Vd * scale
    Vq = Vq * scale
end
```

#### 6.4.6 Current Loop Performance Analysis

**Step response characteristics:**

With our designed controller (fbw = 1 kHz):
```
Rise time: tr ≈ 0.35/fbw = 0.35 ms
Settling time: ts ≈ 4.6/ωbw = 0.73 ms
Overshoot: MP ≈ 4% (for ζ = 0.707)
```

**Bode plot analysis:**

```
Low frequency gain: 40 dB (100:1 error reduction)
Bandwidth: 1000 Hz
Phase margin: ~65° (good stability)
Gain margin: >10 dB (robust)
```

**Disturbance rejection:**

The closed-loop system can reject:
- Load torque changes
- Speed variations
- Voltage fluctuations
- Parameter variations (within limits)

### 6.5 Speed Control Loop Design

The speed control loop is the **outer, slow loop** that generates the torque/current command (i*q) to track the speed reference.

#### 6.5.1 Speed Loop Plant Model

**Mechanical equation:**
```
J·dω/dt = τ - τ_load - B·ω

where:
  J = moment of inertia (kg·m²)
  ω = mechanical speed (rad/s)
  τ = electromagnetic torque (N·m)
  τ_load = load torque (N·m)
  B = viscous friction coefficient (N·m·s)
```

**Transfer function from torque to speed:**

```
         ω(s)            1/B              1/τm
G_speed(s) = ------- = ------------- = -------------
            τ(s)      J/B·s + 1       τm·s + 1

where τm = J/B  (mechanical time constant)
```

**With current loop in feedback:**

If the current loop is fast (bandwidth ≫ speed loop), we can approximate it as unity gain:
```
iq ≈ i*q  (current loop tracks perfectly)
```

Then:
```
τ = (3/2)·P·λm·iq = Kt·iq

where Kt = (3/2)·P·λm  (torque constant)
```

**Complete speed loop plant:**
```
         ω(s)         Kt/B         Kt/(B·τm)
G(s) = -------- = ------------- = -------------
        i*q(s)    J/B·s + 1       s + 1/τm
```

This is also a **first-order system** (assuming current loop is fast).

**Typical values:**
```
J = 0.00005 kg·m²  (small motor)
B = 0.00001 N·m·s  (low friction)
τm = J/B = 5 s  (mechanical time constant)
fc = 1/(2π·τm) = 0.032 Hz  (very slow!)
```

Notice: Mechanical dynamics are **much slower** than electrical dynamics (5000 ms vs 2 ms).

#### 6.5.2 Speed PI Controller Design

**PI controller structure:**
```
         Kp_speed·s + Ki_speed
C_speed(s) = ----------------------
                   s

i*q(t) = Kp_speed·eω(t) + Ki_speed·∫eω(τ)dτ

where eω(t) = ω_ref(t) - ω_measured(t)
```

**Discrete implementation:**
```matlab
% Speed error (mechanical rad/s)
error_speed = omega_ref - omega_measured

% PI calculation
proportional = Kp_speed * error_speed
integrator_speed = integrator_speed + Ki_speed * Ts_speed * error_speed

% Current command (q-axis)
iq_ref = proportional + integrator_speed

% Limit output
iq_ref = saturate(iq_ref, -Iq_max, Iq_max)
```

#### 6.5.3 Speed Loop Gain Calculation

Using the bandwidth method:

**Desired bandwidth:** Choose to be **5-10 times slower** than current loop:
```
fbw_current = 1000 Hz
fbw_speed = 100 Hz (10× slower)
ωbw_speed = 2π·100 = 628 rad/s
```

**Calculate gains:**

With current loop approximated as unity and plant G(s) = Kt/(τm·s + 1):

```
Kp_speed = J·ωbw / Kt
Ki_speed = B·ωbw / Kt
```

**Alternative formulation:**

For mechanical systems, a common approach is:

```
Kp_speed = 2·ζ·ωn·J / Kt
Ki_speed = ωn²·J / Kt

where ζ = 0.707, ωn = ωbw
```

#### 6.5.4 Design Example - Speed Loop

Continuing with our example motor:

**Given parameters:**
```
J = 0.00005 kg·m²
B = 0.00001 N·m·s
P = 4 pole pairs
λm = 0.1 Wb
Kt = (3/2)·P·λm = 1.5 × 4 × 0.1 = 0.6 N·m/A
Iq_max = 10 A (current limit)
```

**Step 1: Choose speed loop bandwidth**
```
fbw_speed = 100 Hz
ωbw_speed = 2π·100 = 628 rad/s
```

**Step 2: Calculate PI gains**
```
Kp_speed = J·ωbw / Kt = 0.00005 × 628 / 0.6 = 0.0523
Ki_speed = B·ωbw / Kt = 0.00001 × 628 / 0.6 = 0.0105
```

**Step 3: Choose speed loop sampling rate**

Speed loop can run slower than current loop:
```
fs_speed = 1000 Hz (every 10th current loop cycle)
Ts_speed = 0.001 s
```

**Step 4: Implement with anti-windup**

```matlab
% Speed loop (runs at 1 kHz)
function iq_ref = speed_controller(omega_ref, omega_measured)
    persistent integrator_speed
    if isempty(integrator_speed)
        integrator_speed = 0;
    end

    % Parameters
    Kp_speed = 0.0523;
    Ki_speed = 0.0105;
    Ts_speed = 0.001;
    Iq_max = 10;  % Maximum q-axis current

    % Speed error (rad/s)
    error_speed = omega_ref - omega_measured;

    % PI calculation
    proportional = Kp_speed * error_speed;
    integrator_speed = integrator_speed + Ki_speed * Ts_speed * error_speed;

    % Current command
    iq_ref = proportional + integrator_speed;

    % Anti-windup: limit and back-calculate
    iq_ref_limited = max(min(iq_ref, Iq_max), -Iq_max);

    if iq_ref ~= iq_ref_limited
        integrator_speed = iq_ref_limited - proportional;
    end

    iq_ref = iq_ref_limited;
end
```

#### 6.5.5 Cascade Loop Tuning Procedure

**Step-by-step tuning process:**

**Step 1: Tune Current Loop First**

1. Set speed reference to zero (motor stationary)
2. Apply step change to i*q (e.g., 0 → 2 A)
3. Measure iq response
4. Adjust Kp and Ki until:
   - Rise time: ~0.3-0.5 ms
   - Overshoot: <10%
   - No oscillation
5. Repeat for i*d (usually same gains work)

**Step 2: Verify Current Loop Bandwidth**

1. Inject sine wave into i*q reference
2. Sweep frequency from 10 Hz to 2 kHz
3. Measure -3 dB bandwidth
4. Should be ~1 kHz (1/10 of PWM frequency)

**Step 3: Tune Speed Loop**

1. With current loop working, apply step to ω_ref
2. Start with low gains (Kp_speed/10, Ki_speed/10)
3. Gradually increase gains until:
   - Good tracking (low steady-state error)
   - Fast response (settling time ~0.02-0.05 s)
   - No overshoot (or <10% overshoot)
   - No oscillation

**Step 4: Test Combined System**

1. Apply various speed commands (steps, ramps, sinusoids)
2. Apply load torque disturbances
3. Verify current limits are enforced
4. Check for any instability

**Common tuning issues:**

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| Slow rise time | Gains too low | Increase Kp |
| Overshoot | Kp too high or Ki too low | Reduce Kp, increase Ki |
| Oscillation | Bandwidth too high | Reduce gains |
| Steady-state error | Ki too low | Increase Ki |
| Noisy current | Speed loop too fast | Reduce speed loop bandwidth |
| Poor disturbance rejection | Gains too low | Increase Ki |

#### 6.5.6 Speed Loop Performance Analysis

**With our designed controller:**

```
Bandwidth: 100 Hz
Rise time: ~3.5 ms
Settling time: ~20 ms
Overshoot: <5%
Steady-state error: <1% (with load)
```

**Load disturbance rejection:**

With integral action, steady-state error due to constant load is zero!

```
At steady state with load τ_load:
  i*q = τ_load/Kt  (PI automatically compensates)
```

**Frequency response:**

```
Low frequency gain: 60 dB (1000:1 error reduction)
Bandwidth: 100 Hz
Phase margin: ~65°
Stability: Good
```

#### 6.5.7 Advanced Speed Control Topics

**1. Acceleration Feedforward**

Improve transient response by compensating inertia:

```matlab
% Reference acceleration
alpha_ref = (omega_ref[k] - omega_ref[k-1]) / Ts

% Feedforward torque
tau_ff = J * alpha_ref

% Add to PI output
iq_ref = iq_PI + tau_ff / Kt
```

**2. Load Observer**

Estimate load torque for better disturbance rejection:

```matlab
% Simple load observer
tau_load_est = tau_load_est + K_obs * (omega_measured - omega_predicted)

% Use in feedforward
iq_ff = tau_load_est / Kt
```

**3. Reference Filtering**

Add ramp limiter to prevent sudden accelerations:

```matlab
% Limit acceleration
max_accel = 1000;  % rad/s²
delta_omega = omega_ref - omega_ref_prev;
delta_omega = saturate(delta_omega, -max_accel*Ts, max_accel*Ts);
omega_ref_filtered = omega_ref_prev + delta_omega;
```

**4. Adaptive Gains**

Adjust gains based on operating point:

```matlab
% Increase gains at low speed for better response
if abs(omega_measured) < 10  % rad/s
    Kp_speed = Kp_speed * 2;
    Ki_speed = Ki_speed * 2;
end
```

---

### 6.6 Field Weakening and High-Speed Operation

#### 6.6.1 The Voltage Constraint Problem

As motor speed increases, the back-EMF increases proportionally:

```
E = ωe·λm  (back-EMF voltage)
```

At some speed, called the **base speed**, the required voltage reaches the inverter's maximum capability:

```
√(Vd² + Vq²) ≤ V_max = Vdc/√3
```

**Below base speed:**
- Voltage limit is not reached
- id=0 control works perfectly
- Motor operates in constant torque region

**Above base speed:**
- Voltage limit is reached
- Cannot maintain id=0 and full torque
- Must trade off torque for speed
- Motor operates in constant power region

#### 6.6.2 Field Weakening Concept

The solution is to inject **negative d-axis current** (id < 0) to reduce the effective magnetic flux:

**Effect of negative id:**

```
Total flux = λm + Ld·id

With id < 0:
  Total flux decreases → Back-EMF decreases → Can run faster
```

**Physical interpretation:**
- Permanent magnet flux cannot be changed
- But we can create opposing flux with stator current
- This "weakens" the effective field
- Trade-off: Reduces available torque but extends speed range

**Torque with field weakening:**

For SPMSM (Ld ≈ Lq):
```
τ = (3/2)·P·λm·iq

For IPMSM (Ld < Lq):
τ = (3/2)·P·[λm·iq + (Ld - Lq)·id·iq]
    └─ PM torque     └─ Reluctance torque

With id < 0 and Ld < Lq:
  Reluctance torque is negative (opposes PM torque)
  But for IPMSMs this can still increase speed range significantly
```

#### 6.6.3 Field Weakening Control Strategy

**Basic strategy:**

1. **Below base speed:** Use id=0 control
2. **At base speed:** Voltage limit is reached
3. **Above base speed:** Inject negative id to reduce flux

**Implementation:**

```matlab
% Calculate voltage magnitude
V_measured = sqrt(Vd^2 + Vq^2)
V_max = Vdc / sqrt(3)

% Field weakening PI controller
error_voltage = V_max - V_measured

if omega > omega_base
    % Activate field weakening
    id_ref = FW_PI(error_voltage)
    id_ref = min(id_ref, 0)  % Only negative id
else
    % Below base speed
    id_ref = 0
end
```

**Alternative: Voltage-based method**

More sophisticated approach uses voltage constraints directly:

```matlab
% Current constraint (circular)
id² + iq² ≤ I_max²

% Voltage constraint (elliptical)
(Rs·id - ωe·Lq·iq)² + (Rs·iq + ωe·Ld·id + ωe·λm)² ≤ V_max²

% At high speed (Rs·iq << ωe·Ld·id):
(ωe·Lq·iq)² + (ωe·Ld·id + ωe·λm)² ≈ V_max²

% Simplified voltage ellipse:
(Lq·iq)² + (Ld·id + λm)² ≤ (V_max/ωe)²
```

#### 6.6.4 MTPA with Field Weakening Regions

Complete control strategy for IPMSMs:

**Region 1: MTPA (Low speed)**
```
Constraint: id² + iq² ≤ I_max²
Objective: Maximize torque per ampere

For SPMSM: id = 0
For IPMSM: id = [λm - √(λm² + 8(Lq-Ld)²·iq²)] / [4(Lq-Ld)]
```

**Region 2: MTPV (Medium speed)**
```
Constraint: Voltage limit becoming active
Objective: Maximum Torque Per Volt
```

**Region 3: Field Weakening (High speed)**
```
Constraint: Both current and voltage limits
Objective: Maximize speed while staying within limits
```

**Transition diagram:**

```
Torque
  ↑
  │╲
  │ ╲  Region 1: MTPA
  │  ╲  (constant torque)
  │   ╲
  │    ╲______
  │           ╲____  Region 2: MTPV
  │                ╲____
  │                     ╲____  Region 3: Field Weakening
  │                          ╲____ (constant power)
  └─────────────────────────────────→ Speed
        ω_base               ω_max
```

#### 6.6.5 Field Weakening Design Example

**Given motor:**
```
λm = 0.1 Wb
Ld = Lq = 1 mH (SPMSM)
Rs = 0.5 Ω
P = 4 pole pairs
I_max = 10 A
Vdc = 24 V
V_max = 24/√3 = 13.86 V
```

**Step 1: Calculate base speed**

At base speed with id=0, maximum torque:
```
iq = I_max = 10 A
Vq_required = ωe_base·λm + Rs·iq
13.86 = ωe_base × 0.1 + 0.5 × 10
ωe_base = (13.86 - 5) / 0.1 = 88.6 rad/s (electrical)

ω_base = ωe_base / P = 88.6 / 4 = 22.15 rad/s (mechanical)
N_base = ω_base × 60/(2π) = 211 RPM
```

**Step 2: Operate above base speed**

At 2× base speed (ωe = 177.2 rad/s):

Without field weakening:
```
Vq_required = 177.2 × 0.1 + 0.5 × 10 = 22.72 V  ❌ Exceeds 13.86 V!
```

With field weakening (solve for id):
```
Vd = Rs·id - ωe·L·iq
Vq = Rs·iq + ωe·L·id + ωe·λm

√(Vd² + Vq²) = V_max = 13.86 V

Assuming iq = 8 A (reduced torque), solve for id:
id ≈ -4.2 A

Check: id² + iq² = 4.2² + 8² = 81.64 < 100 ✓

Torque: τ = (3/2) × 4 × 0.1 × 8 = 4.8 N·m (reduced from 6 N·m at base speed)
```

#### 6.6.6 Practical Field Weakening Implementation

**Complete MATLAB function:**

```matlab
function [id_ref, iq_ref] = field_weakening_control(omega_ref, omega_measured, tau_ref)
    % Motor parameters
    lambda_m = 0.1;
    Ld = 0.001;
    Lq = 0.001;
    Rs = 0.5;
    P = 4;
    I_max = 10;
    Vdc = 24;
    V_max = Vdc / sqrt(3);

    % Electrical speed
    omega_e = omega_measured * P;

    % Base speed (approximate)
    omega_e_base = (V_max - Rs*I_max) / lambda_m;

    % Calculate iq from torque reference
    Kt = 1.5 * P * lambda_m;
    iq_ref = tau_ref / Kt;

    % Field weakening
    if omega_e > omega_e_base
        % Estimate required negative id
        % From voltage constraint: (ωe·Lq·iq)² + (ωe·Ld·id + ωe·λm)² ≤ V_max²
        % Solve for id:
        V_margin = V_max - 1;  % 1V safety margin

        % Approximate (neglecting Rs terms):
        term1 = (omega_e * Lq * iq_ref)^2;
        term2 = V_margin^2 - term1;

        if term2 > 0
            id_ref = (sqrt(term2) - omega_e * lambda_m) / (omega_e * Ld);
            id_ref = max(id_ref, -I_max);  % Limit negative id
        else
            % Cannot reach this speed-torque point
            id_ref = -I_max;
            % Must reduce iq
            max_iq = sqrt(V_margin^2 / (omega_e * Lq)^2);
            iq_ref = min(iq_ref, max_iq);
        end
    else
        % Below base speed
        id_ref = 0;
    end

    % Current limit (circular constraint)
    I_total = sqrt(id_ref^2 + iq_ref^2);
    if I_total > I_max
        scale = I_max / I_total;
        id_ref = id_ref * scale;
        iq_ref = iq_ref * scale;
    end
end
```

### 6.7 Complete FOC Implementation Algorithm

Now let's put everything together into a complete, real-time FOC algorithm.

#### 6.7.1 Initialization Phase

```c
// Motor parameters (constant)
#define RS          0.5f      // Stator resistance (Ω)
#define LD          0.001f    // d-axis inductance (H)
#define LQ          0.001f    // q-axis inductance (H)
#define LAMBDA_M    0.1f      // Flux linkage (Wb)
#define POLE_PAIRS  4         // Number of pole pairs
#define J           0.00005f  // Inertia (kg·m²)

// Inverter parameters
#define VDC         24.0f     // DC bus voltage (V)
#define V_MAX       (VDC/1.732f)  // Max phase voltage (V)
#define I_MAX       10.0f     // Max phase current (A)
#define FSW         10000     // PWM frequency (Hz)
#define TS          (1.0f/FSW) // Sample time (s)

// Control parameters (tuned)
#define KP_D        6.28f     // d-axis current Kp
#define KI_D        3142.0f   // d-axis current Ki
#define KP_Q        6.28f     // q-axis current Kp
#define KI_Q        3142.0f   // q-axis current Ki
#define KP_SPEED    0.0523f   // Speed Kp
#define KI_SPEED    0.0105f   // Speed Ki

// State variables (initialized to zero)
float integrator_id = 0.0f;
float integrator_iq = 0.0f;
float integrator_speed = 0.0f;
float theta_e = 0.0f;           // Electrical angle (rad)
float omega_e = 0.0f;           // Electrical speed (rad/s)
float omega_m = 0.0f;           // Mechanical speed (rad/s)

// Hall sensor state
uint8_t hall_state_prev = 0;
uint32_t hall_time_prev = 0;
```

#### 6.7.2 Main Control Loop (10 kHz)

```c
void FOC_MainLoop(void) {
    // This function is called at PWM frequency (10 kHz)

    // ==========================================
    // STEP 1: Read Sensors
    // ==========================================

    // Read phase currents from ADC
    float ia = ADC_ReadCurrent(ADC_CHANNEL_A);  // Amps
    float ib = ADC_ReadCurrent(ADC_CHANNEL_B);  // Amps
    float ic = -(ia + ib);  // Reconstruct (assumes balanced)

    // Read Hall sensors
    uint8_t hall_a = GPIO_Read(HALL_A_PIN);
    uint8_t hall_b = GPIO_Read(HALL_B_PIN);
    uint8_t hall_c = GPIO_Read(HALL_C_PIN);
    uint8_t hall_state = (hall_a << 2) | (hall_b << 1) | hall_c;

    // Decode Hall state to angle
    theta_e = Hall_DecodeAngle(hall_state);  // Electrical angle

    // Calculate speed (every 10th cycle to reduce noise)
    static uint16_t speed_counter = 0;
    if (++speed_counter >= 10) {
        speed_counter = 0;
        omega_m = Speed_Calculate(hall_state, hall_state_prev,
                                    hall_time_prev, micros());
        omega_e = omega_m * POLE_PAIRS;
        hall_state_prev = hall_state;
        hall_time_prev = micros();
    }

    // ==========================================
    // STEP 2: Clarke Transform (abc → αβ)
    // ==========================================

    float i_alpha = (2.0f/3.0f) * (ia - 0.5f*ib - 0.5f*ic);
    float i_beta = (2.0f/3.0f) * (0.866f*ib - 0.866f*ic);

    // ==========================================
    // STEP 3: Park Transform (αβ → dq)
    // ==========================================

    float cos_theta = arm_cos_f32(theta_e);  // Use CMSIS-DSP
    float sin_theta = arm_sin_f32(theta_e);

    float id = cos_theta * i_alpha + sin_theta * i_beta;
    float iq = -sin_theta * i_alpha + cos_theta * i_beta;

    // ==========================================
    // STEP 4: Speed Controller (1 kHz)
    // ==========================================

    static float iq_ref = 0.0f;
    static uint16_t speed_loop_counter = 0;

    if (++speed_loop_counter >= 10) {  // Every 10th cycle
        speed_loop_counter = 0;

        float omega_ref = GetSpeedReference();  // From user/profile
        float error_speed = omega_ref - omega_m;

        // Speed PI controller
        float prop_speed = KP_SPEED * error_speed;
        integrator_speed += KI_SPEED * 0.001f * error_speed;  // Ts=1ms

        iq_ref = prop_speed + integrator_speed;

        // Limit and anti-windup
        float iq_ref_limited = CLAMP(iq_ref, -I_MAX, I_MAX);
        if (iq_ref != iq_ref_limited) {
            integrator_speed = iq_ref_limited - prop_speed;
        }
        iq_ref = iq_ref_limited;
    }

    float id_ref = 0.0f;  // id=0 control for SPMSM

    // ==========================================
    // STEP 5: Current Controllers (10 kHz)
    // ==========================================

    // d-axis PI controller
    float error_d = id_ref - id;
    float prop_d = KP_D * error_d;
    integrator_id += KI_D * TS * error_d;
    float Vd_PI = prop_d + integrator_id;

    // q-axis PI controller
    float error_q = iq_ref - iq;
    float prop_q = KP_Q * error_q;
    integrator_iq += KI_Q * TS * error_q;
    float Vq_PI = prop_q + integrator_iq;

    // ==========================================
    // STEP 6: Feedforward Decoupling
    // ==========================================

    float Vd_ff = -omega_e * LQ * iq;
    float Vq_ff = omega_e * LD * id + omega_e * LAMBDA_M;

    float Vd = Vd_PI + Vd_ff;
    float Vq = Vq_PI + Vq_ff;

    // ==========================================
    // STEP 7: Voltage Limiting with Anti-Windup
    // ==========================================

    float V_mag = sqrtf(Vd*Vd + Vq*Vq);

    if (V_mag > V_MAX) {
        float scale = V_MAX / V_mag;
        Vd *= scale;
        Vq *= scale;

        // Back-calculate to prevent integrator windup
        integrator_id = Vd - prop_d - Vd_ff;
        integrator_iq = Vq - prop_q - Vq_ff;
    }

    // ==========================================
    // STEP 8: Inverse Park Transform (dq → αβ)
    // ==========================================

    float V_alpha = cos_theta * Vd - sin_theta * Vq;
    float V_beta = sin_theta * Vd + cos_theta * Vq;

    // ==========================================
    // STEP 9: SVPWM (αβ → PWM duties)
    // ==========================================

    float Ta, Tb, Tc;
    SVPWM_Calculate(V_alpha, V_beta, VDC, &Ta, &Tb, &Tc);

    // ==========================================
    // STEP 10: Update PWM
    // ==========================================

    PWM_SetDuty(PWM_CHANNEL_A, Ta);
    PWM_SetDuty(PWM_CHANNEL_B, Tb);
    PWM_SetDuty(PWM_CHANNEL_C, Tc);
}
```

#### 6.7.3 FOC Algorithm Flowchart

```
START (PWM interrupt at 10 kHz)
   │
   ├─→ Read ADC (ia, ib) ─→ Calculate ic = -(ia+ib)
   │
   ├─→ Read Hall sensors (Ha, Hb, Hc) ─→ Decode θe
   │
   ├─→ Calculate ωm, ωe (every 10th cycle)
   │
   ├─→ Clarke Transform: (ia, ib, ic) → (iα, iβ)
   │
   ├─→ Park Transform: (iα, iβ, θe) → (id, iq)
   │
   ├─→ [Every 10th cycle] Speed Loop:
   │   ω_ref - ωm → Speed PI → i*q
   │
   ├─→ Set i*d = 0 (or field weakening)
   │
   ├─→ Current Loop d-axis:
   │   i*d - id → PI → Vd_PI
   │   Add feedforward: Vd = Vd_PI + Vd_ff
   │
   ├─→ Current Loop q-axis:
   │   i*q - iq → PI → Vq_PI
   │   Add feedforward: Vq = Vq_PI + Vq_ff
   │
   ├─→ Voltage Limiting:
   │   if √(Vd²+Vq²) > Vmax: scale both
   │   Back-calculate integrators (anti-windup)
   │
   ├─→ Inverse Park: (Vd, Vq, θe) → (Vα, Vβ)
   │
   ├─→ SVPWM: (Vα, Vβ, Vdc) → (Ta, Tb, Tc)
   │
   ├─→ Update PWM registers
   │
   └─→ END (wait for next interrupt)
```

### 6.8 Key Takeaways - Field Oriented Control

1. **FOC transforms complex AC control into simple DC control** via the dq reference frame

2. **Decoupling is key:** Feedforward compensation eliminates cross-coupling between d and q axes

3. **Two-loop cascade structure:** Fast inner current loop (10 kHz) + slower outer speed loop (1 kHz)

4. **id=0 strategy is optimal for SPMSMs** below base speed (maximum torque per ampere)

5. **PI controllers with anti-windup** are sufficient for good performance

6. **Bandwidth selection matters:**
   - Current loop: ~1/10 of PWM frequency
   - Speed loop: ~1/10 of current loop bandwidth

7. **Field weakening extends speed range** by injecting negative id above base speed

8. **Rotor position is critical:** Accurate θe is needed for Park/inverse Park transforms

9. **Real-time constraints:** FOC must execute within one PWM period (typically 50-100 μs)

10. **Systematic tuning:** Always tune current loop first, then speed loop

### 6.9 Further Study - Field Oriented Control

**Foundational Books:**

1. **"Vector Control of AC Drives"** by Peter Vas
   - The classic comprehensive treatment
   - Chapters 3-5: FOC theory and implementation

2. **"Control of Electric Machine Drive Systems"** by Seung-Ki Sul
   - Chapter 7: Vector Control of PMSM
   - Excellent mathematical rigor

3. **"Advanced Electric Drives"** by Rik De Doncker
   - Chapter 8: Field-Oriented Control
   - Modern perspective with DSP implementation

4. **"Power Electronics and Motor Drives"** by Bimal K. Bose
   - Chapter 9: Vector Control of AC Drives
   - Practical industrial perspective

**Application Notes (Essential Reading):**

1. **Texas Instruments:**
   - SPRA588: "Field Orientated Control of 3-Phase AC-Motors"
   - SPRAAB7: "Sensorless Field Oriented Control of 3-Phase PMSMs"
   - SPRABQ2: "InstaSPIN-FOC and InstaSPIN-MOTION"

2. **STMicroelectronics:**
   - AN1078: "FOC Motor Control for PMSM Motors"
   - AN4277: "PMSM FOC Motor Control SDK"
   - AN5051: "Sensorless PMSM Field Oriented Control"

3. **Microchip:**
   - AN1078: "Sensorless Field Oriented Control of PMSM Motors"
   - AN1299: "Single-Shunt Three-Phase Current Reconstruction Algorithm"
   - AN2520: "Field Weakening Operation"

4. **Infineon:**
   - AP32370: "Field Oriented Control of PMSMs"
   - AP32371: "Sensorless FOC for PMSM using Sliding Mode Observer"

**Papers (Advanced Topics):**

1. **"Field Weakening in PMSM Drives"** by Morimoto et al.
   - MTPA and field weakening strategies for IPMSMs

2. **"Sensorless Control of PMSMs"** by Holtz
   - Overview of position estimation methods

3. **"Digital Control Strategies for Brushless PM Drives"** by Jahns
   - Practical digital implementation considerations

**Video Courses:**

1. **MATLAB/Simulink:**
   - "Introduction to Field-Oriented Control"
   - "PMSM Control Design with Simulink"

2. **Texas Instruments Training:**
   - "Motor Control Fundamentals" series
   - Practical lab exercises with C2000 DSPs

3. **YouTube - "Zach Star":** "Vector Control Explained"

4. **Coursera:** "Power Electronics Specialization" by University of Colorado

**Interactive Tools:**

1. **PLECS Demo Models:**
   - FOC_PMSM_Basic.plecs
   - Field_Weakening_IPMSM.plecs

2. **MATLAB/Simulink Examples:**
   - Motor Control Blockset → PMSM FOC examples
   - Includes code generation for TI, STM32, NXP targets

3. **Open-source implementations:**
   - VESC Project (vedderb/bldc on GitHub)
   - SimpleFOC library (simplefoc.com)
   - ODrive firmware (odriverobotics/ODrive)

**Standards and References:**

1. **IEC 61800-7-201:** "Adjustable speed electrical power drive systems"
2. **IEEE Std 1566:** "Standard for Performance of Adjustable Speed Drives"

---

*End of Section 6 - Field Oriented Control Theory*

