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

