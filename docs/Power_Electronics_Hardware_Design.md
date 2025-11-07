# Power Electronics Hardware Design for PMSM Motor Controllers

**A Comprehensive Guide to Motor Controller Circuit Design and Implementation**

---

## Table of Contents

### 1. Introduction and Overview
   - 1.1 Motor Controller Architecture
   - 1.2 Key Design Requirements
   - 1.3 Safety and Reliability Considerations

### 2. Power Stage Fundamentals
   - 2.1 Three-Phase H-Bridge (Six-Switch Inverter)
   - 2.2 Operating Modes and Switching States
   - 2.3 Power Flow and Regeneration
   - 2.4 Common Topologies and Variants

### 3. Power Semiconductor Selection
   - 3.1 MOSFET Fundamentals and Key Parameters
   - 3.2 Silicon MOSFETs vs SiC vs GaN
   - 3.3 Device Selection Criteria
   - 3.4 Parallel Operation Considerations

### 4. Gate Driver Design
   - 4.1 Gate Driver Requirements
   - 4.2 Discrete Gate Driver Circuits
   - 4.3 Integrated Gate Driver ICs
   - 4.4 Bootstrap vs Isolated Power Supply
   - 4.5 Gate Resistance Selection

### 5. Current Sensing
   - 5.1 Current Measurement Requirements for FOC
   - 5.2 Shunt Resistor Sensing (Low-Side, High-Side, Inline)
   - 5.3 Hall Effect Current Sensors
   - 5.4 Single Shunt vs Three Shunt Topologies
   - 5.5 Current Sensor Placement and FOC Algorithm Impact

### 6. Switching Dynamics and dv/dt Effects
   - 6.1 Switching Transients in Power MOSFETs
   - 6.2 dv/dt Effects on System Performance
   - 6.3 Gate Resistance Trade-offs
   - 6.4 Snubber Circuits
   - 6.5 Ringing, Overshoot, and EMI

### 7. Thermal Management
   - 7.1 Power Loss Calculations
   - 7.2 Thermal Resistance and Heat Transfer
   - 7.3 Heatsink Design and Capacity Calculations
   - 7.4 Air Cooling vs Liquid Cooling
   - 7.5 Thermal Simulation and Testing

### 8. PCB Layout and Design
   - 8.1 High-Current Trace Width Calculations
   - 8.2 Low-Inductance Layout Techniques
   - 8.3 Power Loop Minimization
   - 8.4 Gate Drive Layout Best Practices
   - 8.5 Grounding and Layer Stack-up
   - 8.6 Component Placement Guidelines

### 9. EMI/EMC Considerations
   - 9.1 EMI Sources in Motor Drives
   - 9.2 Conducted and Radiated Emissions
   - 9.3 EMI Filtering and Suppression
   - 9.4 Shielding and Cable Design
   - 9.5 Standards and Compliance Testing

### 10. Testing and Validation
   - 10.1 Motor Controller Test Setup
   - 10.2 Oscilloscope Measurements
   - 10.3 Key Parameters to Monitor
   - 10.4 Failure Modes and Troubleshooting
   - 10.5 Reliability Testing

### 11. Mechanical Design
   - 11.1 IP67 Enclosure Design
   - 11.2 Connector Selection
   - 11.3 Vibration and Shock Resistance
   - 11.4 Thermal Interface Materials

### 12. References and Resources

---

## 1. Introduction and Overview

### 1.1 Motor Controller Architecture

A PMSM motor controller consists of several key subsystems working together to achieve precise torque and speed control:

**Block Diagram:**
```
DC Bus ──→ Power Stage ──→ Motor (3-phase)
  ↑           (Inverter)        ↓
  │              ↓              │
  │         Gate Drivers    Position/
  │              ↓           Speed Sensor
  │         Control Board   (Hall/Encoder)
  │         - MCU/DSP           ↓
  │         - ADCs              │
  │         - Current Sense ←───┘
  │         - Voltage Sense ←───┘
  │              ↓
  └────── Power Supply
          (Aux 12V/5V)
```

**Key Subsystems:**

1. **Power Stage**: Three-phase inverter with 6 power switches (MOSFETs or IGBTs)
2. **Gate Drivers**: Drive circuits for high-side and low-side switches
3. **Current Sensing**: Measure phase or DC bus currents for FOC algorithm
4. **Voltage Sensing**: Monitor DC bus voltage and phase voltages
5. **Control Unit**: MCU/DSP running FOC algorithms (Clarke, Park, PI controllers, SVPWM)
6. **Position/Speed Sensor Interface**: Hall sensors, encoders, or resolver
7. **Protection Circuits**: Overcurrent, overvoltage, overtemperature protection
8. **Auxiliary Power Supply**: Generate isolated gate drive power and logic supply

### 1.2 Key Design Requirements

**Power Requirements:**
- **Voltage Range**: Typical automotive: 200-450 V DC (400V nominal battery)
  - Industrial: 300-800 V DC
  - Low voltage: 24-48 V DC (e-bikes, AGVs)
- **Current Rating**: Continuous and peak current capability
  - Peak current typically 2-3× continuous for acceleration
  - Derating considerations for ambient temperature
- **Power Level**: 1 kW to 200+ kW depending on application

**Performance Requirements:**
- **Efficiency**: Target >95% at rated load, >90% across operating range
- **Control Bandwidth**: Current loop 1-5 kHz, speed loop 100-500 Hz
- **PWM Frequency**: 10-20 kHz typical (balance between switching loss and current ripple)
- **Torque Ripple**: <5% for smooth operation
- **Response Time**: Current response <1 ms, torque response <10 ms

**Environmental Requirements:**
- **Operating Temperature**: -40°C to +85°C ambient (junction temp <150-175°C)
- **Ingress Protection**: IP67 for automotive/outdoor (dust-tight, waterproof)
- **Vibration**: 10-20 G for automotive applications
- **Altitude**: Up to 3000m for most automotive applications
- **Humidity**: 5-95% non-condensing

**Safety and Reliability:**
- **Isolation**: >1000 V DC between power and control circuits
- **Fault Detection**: Overcurrent, overvoltage, short circuit, overtemperature
- **Fault Response**: Safe shutdown within 10 μs
- **MTBF**: >10,000 hours for automotive, >100,000 hours for industrial
- **Functional Safety**: ISO 26262 ASIL-C/D for automotive applications

### 1.3 Safety and Reliability Considerations

**Critical Protection Features:**

1. **Overcurrent Protection**:
   - Hardware desaturation detection (DESAT) for short circuit
   - Current sense threshold monitoring
   - Response time: <2-5 μs for device protection

2. **Overvoltage Protection**:
   - Clamp circuits for regeneration voltage spikes
   - Active shunt regulation if needed
   - Typical clamp: 450-480 V for 400V system

3. **Overtemperature Protection**:
   - MOSFET/IGBT case temperature monitoring
   - Coolant temperature monitoring (for liquid-cooled)
   - Thermal derating curves

4. **Gate Drive Fault Detection**:
   - Undervoltage lockout (UVLO) on gate supplies
   - Shoot-through prevention (deadtime)
   - Desaturation monitoring

5. **Isolation Monitoring**:
   - High voltage isolation integrity
   - Ground fault detection
   - Isolation resistance >500 Ω/V

**Failure Modes and Effects Analysis (FMEA):**

| Component | Failure Mode | Effect | Mitigation |
|-----------|--------------|--------|------------|
| Power MOSFET | Short circuit | Phase-to-phase or phase-to-DC | Fast fault detection (<5 μs), fuses |
| Gate driver | Loss of supply | No switching, motor coasts | UVLO detection, redundant supplies |
| Current sensor | Open circuit | Loss of control | Sensor fault detection, safe shutdown |
| Control MCU | Software crash | Loss of control | Watchdog timer, safe state |
| DC link capacitor | Capacitance loss | High ripple, overvoltage | Ripple monitoring, ESR measurement |

**Design for Reliability Best Practices:**
- Component derating: 50-70% of maximum ratings
- Redundant protection mechanisms (belt and suspenders)
- Graceful degradation where possible
- Comprehensive fault logging for diagnostics
- Temperature cycling and vibration testing during development

**Standards and Certifications:**
- **Automotive**: ISO 26262 (functional safety), AEC-Q100 (components)
- **Industrial**: IEC 61800-5-1 (adjustable speed drives safety)
- **EMC**: CISPR 25, SAE J1113 (automotive), IEC 61800-3 (industrial)
- **Environmental**: IP ratings (IEC 60529), vibration (IEC 60068)

---

### References for Section 1:

**Books:**
1. *"Power Electronics for Motor Drives"* by R. Krishnan - Chapter 3: Motor Drive Power Circuits
2. *"Advanced Electric Drives: Analysis, Control, and Modeling Using MATLAB/Simulink"* by Ned Mohan - Chapter 4: Inverters
3. *"Electric Vehicle Technology Explained"* by James Larminie and John Lowry - Chapter 5: Electric Motors and Controllers

**Application Notes:**
1. **Texas Instruments**: "System Design Guidelines for Motor Drive Applications" (SLVA890)
2. **Infineon**: "Design Guide for Motor Control Inverters" (Application Note AN2019-06)
3. **STMicroelectronics**: "Power MOSFET Selection Guide for Motor Control" (AN4612)

**Articles and Papers:**
1. "Reliability Considerations for Motor Drive Power Electronics" - IEEE Transactions on Power Electronics
2. "Fault Detection and Protection Strategies for PMSM Drives" - EPE Journal

**Videos:**
1. **TI Training**: "Motor Drive Power Stage Design Fundamentals" (YouTube - TI Precision Labs)
2. **Infineon**: "Introduction to Motor Control Inverters" (YouTube - Infineon Technologies)

**Industry Standards:**
1. ISO 26262: Road vehicles - Functional safety
2. IEC 61800-5-1: Adjustable speed electrical power drive systems - Safety requirements
3. AEC-Q100: Failure Mechanism Based Stress Test Qualification for Automotive Grade ICs

---

## 2. Power Stage Fundamentals

### 2.1 Three-Phase H-Bridge (Six-Switch Inverter)

The three-phase inverter, also called a six-switch bridge or three-phase H-bridge, is the heart of the motor controller. It converts DC voltage from the battery/power supply into three-phase AC voltages with variable amplitude and frequency.

**Circuit Topology:**
```
                    VDC+
                     │
         ┌───────────┼───────────┼───────────┐
         │           │           │           │
        Q1          Q3          Q5          │
    (High-side)  (High-side)  (High-side)   │
         │           │           │           │
    ├────┼────── ├───┼────── ├───┼──────     │
    │    A       │   B       │   C          │  DC Link
    │    Phase   │   Phase   │   Phase      │  Capacitor
    ├────┼────── ├───┼────── ├───┼──────     │   (Cdc)
         │           │           │           │
        Q2          Q4          Q6          │
    (Low-side)   (Low-side)   (Low-side)    │
         │           │           │           │
         └───────────┴───────────┴───────────┘
                     │
                    VDC-
                     │
    To Motor:     Phase A    Phase B    Phase C
```

**Key Components:**

1. **Power Switches (Q1-Q6)**:
   - MOSFETs for <1000V applications
   - IGBTs for >600V high-power applications
   - SiC MOSFETs for high-efficiency designs
   - Each switch rated for full DC bus voltage and phase current

2. **DC Link Capacitor (Cdc)**:
   - Stores energy and filters ripple current
   - Typical: 100-500 μF per kW of power
   - Must handle high RMS ripple current (0.3-0.5× motor current)
   - Low ESR and ESL critical for voltage stability

3. **Gate Drivers**:
   - Isolated drivers for high-side switches (Q1, Q3, Q5)
   - Non-isolated or low-side drivers for Q2, Q4, Q6
   - Provide sufficient gate current for fast switching

4. **Current Sensing**:
   - Inline shunt resistors in phase lines (3 shunts)
   - Low-side shunt resistors (3 shunts in Q2, Q4, Q6 source)
   - Single DC bus shunt (1 shunt in VDC- return)
   - Hall effect sensors (isolated, no power loss)

### 2.2 Operating Modes and Switching States

**Switching States:**

The inverter has **8 possible switching states** (2³ = 8):

| State | Q1 | Q3 | Q5 | Q2 | Q4 | Q6 | Va | Vb | Vc | Description |
|-------|----|----|----|----|----|----|----|----|----|--------------------|
| V0    | 0  | 0  | 0  | 1  | 1  | 1  | 0  | 0  | 0  | Zero vector (all low) |
| V1    | 1  | 0  | 0  | 0  | 1  | 1  | 2/3Vdc | -1/3Vdc | -1/3Vdc | Active vector |
| V2    | 1  | 1  | 0  | 0  | 0  | 1  | 1/3Vdc | 1/3Vdc | -2/3Vdc | Active vector |
| V3    | 0  | 1  | 0  | 1  | 0  | 1  | -1/3Vdc | 2/3Vdc | -1/3Vdc | Active vector |
| V4    | 0  | 1  | 1  | 1  | 0  | 0  | -2/3Vdc | 1/3Vdc | 1/3Vdc | Active vector |
| V5    | 0  | 0  | 1  | 1  | 1  | 0  | -1/3Vdc | -1/3Vdc | 2/3Vdc | Active vector |
| V6    | 1  | 0  | 1  | 0  | 1  | 0  | 1/3Vdc | -2/3Vdc | 1/3Vdc | Active vector |
| V7    | 1  | 1  | 1  | 0  | 0  | 0  | 0  | 0  | 0  | Zero vector (all high) |

**Note**:
- State "1" = switch ON, "0" = switch OFF
- Va, Vb, Vc are phase voltages relative to DC bus midpoint
- V0 and V7 are "zero vectors" (no net voltage applied)
- V1-V6 are "active vectors" (apply voltage to motor)

**Shoot-Through Prevention:**

Critical safety requirement: **Never turn on both switches in the same leg simultaneously!**

```
PROHIBITED:  Q1=ON and Q2=ON  ──→ Short circuit across DC bus!
```

**Deadtime Implementation:**
- Insert 0.5-2 μs deadtime between high-side OFF and low-side ON
- Insert deadtime between low-side OFF and high-side ON
- Typical deadtime: 1-2 μs for Si MOSFETs, 200-500 ns for SiC/GaN

**Deadtime Side Effects:**
- Voltage error: ΔV ≈ Vdc × (Td / Ts)
- More pronounced at low speeds and high currents
- Requires compensation in FOC algorithm (see Section 5.4.5 in main handbook)

### 2.3 Power Flow and Regeneration

**Motoring Mode (Power from DC Bus to Motor):**
```
Battery ──→ Inverter ──→ Motor
(Discharging)            (Accelerating)
```
- Positive torque (accelerating)
- Current flows from DC+ through high-side switches to motor
- Returns through low-side switches to DC-

**Regeneration Mode (Power from Motor to DC Bus):**
```
Battery ←── Inverter ←── Motor
(Charging)            (Braking)
```
- Negative torque (braking)
- Motor acts as generator
- Current flows through freewheeling diodes back to DC bus
- DC bus voltage rises unless battery can absorb power

**Regeneration Considerations:**

1. **DC Bus Overvoltage**:
   - Battery has limited charge acceptance
   - DC bus can rise to dangerous levels during heavy braking
   - Mitigation: Active braking resistor (shunt regulator) or reduced regen torque

2. **Braking Resistor Sizing**:
   ```
   P_brake = (m × v² / 2) / t_stop

   Example: 1500 kg vehicle, 60 km/h → 0 in 3 seconds
   P_brake = (1500 × 16.67² / 2) / 3 = 69.4 kW peak!
   ```

3. **Battery Charge Current Limits**:
   - Lithium batteries typically limited to 0.5-1C charge rate
   - 20 kWh battery (50Ah nominal): Max charge ~25-50 kW
   - Must limit regen torque if power exceeds battery capability

### 2.4 Common Topologies and Variants

**Standard Six-Switch Inverter:**
- Most common topology
- One leg per phase
- Full four-quadrant operation (both motoring and regeneration)

**Asymmetric Half-Bridge (Four-Switch Inverter):**
```
         VDC+
          │
    ┌─────┼─────┐
    │     │     │
   Q1    Q3   C1/C2
    │     │   (Split)
    A     B     │
    │     │   C1/C2
   Q2    Q4   (Split)
    │     │     │
    └─────┴─────┘
         VDC-

  Phase C connected to capacitor midpoint
```
- Lower cost (4 switches instead of 6)
- Reduced performance and efficiency
- Requires balanced split capacitors
- Used in low-cost fans, pumps (not automotive)

**Dual Motor Drive:**
```
  DC Bus ──→ Inverter 1 ──→ Motor 1 (Front axle)
    │
    └──────→ Inverter 2 ──→ Motor 2 (Rear axle)
```
- Two independent inverters from single DC bus
- Individual torque control for each motor
- Used in dual-motor EVs (AWD)
- Requires careful power sharing and bus voltage management

**Multi-Level Inverters (3-Level, 5-Level):**
- Use multiple DC voltage levels (e.g., Vdc/2, Vdc)
- Reduced voltage stress on switches
- Lower dv/dt (reduced EMI)
- More complex and expensive
- Used in high-power industrial drives (>100 kW)

**Key Design Parameters:**

| Parameter | Typical Value | Notes |
|-----------|---------------|-------|
| DC Bus Voltage | 200-450 V (EV), 300-800 V (Industrial) | Nominal battery voltage |
| DC Bus Capacitance | 100-500 μF/kW | Film or ceramic, low ESR |
| Switching Frequency | 10-20 kHz | Trade-off: losses vs ripple |
| Deadtime | 1-2 μs (Si), 0.2-0.5 μs (SiC) | Prevent shoot-through |
| Maximum Modulation Index | 0.907 (linear SVPWM) | Can go to 0.952 (overmodulation) |

**Component Selection Guidelines:**

1. **MOSFET Voltage Rating**:
   - V_rated ≥ 1.5 × Vdc_max (50% margin)
   - Consider regen spikes and transients
   - Example: 400V system → Use 650V or 750V MOSFETs

2. **MOSFET Current Rating**:
   - I_continuous ≥ 1.5 × I_phase_RMS
   - I_peak ≥ 2 × I_phase_peak (short duration)
   - Account for thermal derating

3. **DC Link Capacitor**:
   - Minimum capacitance: C ≥ (P_motor / (2π × f_line × Vdc × ΔV_ripple))
   - RMS ripple current: I_ripple_RMS ≈ 0.4 × I_motor_RMS
   - Use low-ESR film capacitors (polypropylene)

---

### References for Section 2:

**Books:**
1. *"Power Electronics: Converters, Applications, and Design"* by Ned Mohan, Tore M. Undeland, William P. Robbins - Chapter 8: Inverters
2. *"Modern Power Electronics and AC Drives"* by Bimal K. Bose - Chapter 4: Voltage Source Inverters
3. *"Electric Motor Drives: Modeling, Analysis, and Control"* by R. Krishnan - Chapter 5: Power Electronic Converters

**Application Notes:**
1. **Infineon**: "Six-Switch Three-Phase Inverter for Motor Control" (Application Note AN2017-12)
2. **Texas Instruments**: "Understanding the Basics of a Three-Phase Motor Drive" (SLVA954)
3. **ON Semiconductor**: "Three-Phase Inverter Design and Control Techniques" (AND9126/D)
4. **Microchip**: "BLDC Motor Control Using dsPIC30F2010" (AN957) - Section on inverter topology

**Articles and Papers:**
1. "Comparative Study of Three-Phase Inverter Topologies for PMSM Drives" - IEEE PESC Conference
2. "Deadtime Effects in Voltage Source Inverters and Compensation Methods" - IEEE Transactions on Industry Applications

**Videos:**
1. **Infineon**: "Three-Phase Inverter Operation and Control" (YouTube)
2. **TI Precision Labs**: "Motor Drive Inverter Fundamentals" (YouTube series)
3. **STM32 Motor Control**: "Power Stage Design for Motor Drives" (YouTube - STMicroelectronics)

**App Notes - Regeneration and Braking:**
1. **Tesla Motors**: "Regenerative Braking in Electric Vehicles" (SAE Paper 2013-01-1457)
2. **Infineon**: "Braking Resistor Selection for Motor Drives" (Application Note AN2018-07)

**Datasheets (Reference Examples):**
1. Infineon IPT60R028G7 (650V SiC MOSFET for automotive inverters)
2. ON Semiconductor NVH4L050N120SC1 (1200V SiC MOSFET, 50A)
3. TDK B32774 series (DC link film capacitors for motor drives)

---

## 3. Power Semiconductor Selection

### 3.1 MOSFET Fundamentals and Key Parameters

**MOSFET Structure and Operation:**

A power MOSFET (Metal-Oxide-Semiconductor Field-Effect Transistor) is a voltage-controlled switch used in motor drive inverters. When voltage is applied to the gate (VGS > Vth), a conductive channel forms, allowing current to flow from drain to source.

**Critical MOSFET Parameters:**

1. **Voltage Rating (VDS_max)**:
   - Maximum drain-source voltage the device can block
   - Common ratings: 100V, 150V, 650V, 750V, 1200V
   - **Selection rule**: VDS_rated ≥ 1.5 × VDC_max (50% margin for transients)
   - Example: 400V DC bus → Use 600V-750V rated MOSFETs

2. **Current Rating**:
   - **Continuous Drain Current (ID)**: At 25°C case temperature
   - **Pulsed Drain Current (IDM)**: Short duration (<1ms)
   - **RMS Current**: Actual current handling depends on thermal design
   - **Selection rule**: ID_rated ≥ 1.5 × I_phase_RMS at operating temperature

3. **On-Resistance (RDS_on)**:
   - Resistance when MOSFET is fully ON
   - **Conduction Loss**: P_cond = I²_RMS × RDS_on
   - Temperature dependent: RDS_on increases ~50-80% from 25°C to 150°C
   - **Lower RDS_on = lower conduction loss but higher cost and gate charge**

4. **Gate Charge (Qg, Qgs, Qgd)**:
   - Charge required to turn ON the MOSFET
   - **Total Gate Charge (Qg)**: Total charge from 0V to VGS_target
   - **Gate-Source Charge (Qgs)**: Charge to reach Miller plateau
   - **Gate-Drain Charge (Qgd)**: Miller charge (causes switching delay)
   - **Switching Loss**: P_sw ∝ Qg × VGS × f_sw
   - **Trade-off**: Low Qg = fast switching but may have higher RDS_on

5. **Switching Times**:
   - **Turn-on delay (td_on)**: Gate voltage rise to threshold
   - **Rise time (tr)**: Current rises from 10% to 90%
   - **Turn-off delay (td_off)**: Gate voltage fall from VGS to Vth
   - **Fall time (tf)**: Current falls from 90% to 10%
   - Total switching time ≈ 50-200 ns for Si, 20-50 ns for SiC

6. **Body Diode Characteristics**:
   - Intrinsic diode (parasitic diode) in MOSFET structure
   - **Forward Voltage (VF)**: 0.7-1.2V for Si, 2-4V for SiC
   - **Reverse Recovery Time (trr)**: 50-200 ns for Si, ~0 for SiC
   - **Reverse Recovery Charge (Qrr)**: Energy loss during diode turn-off
   - Poor body diode in SiC → often use external fast diodes in parallel

7. **Thermal Resistance (Rth_JC)**:
   - Junction-to-case thermal resistance
   - Typical: 0.3-1.0 °C/W for TO-247 packages
   - Lower Rth → better heat dissipation

**Power Loss Calculation:**

```
Total MOSFET loss = Conduction loss + Switching loss

P_cond = I²_RMS × RDS_on(Tj)

P_sw = (E_on + E_off) × f_sw
     ≈ (1/6) × VDS × ID × (tr + tf) × f_sw

Where:
  tr, tf = rise and fall times
  f_sw = switching frequency
```

**Example Calculation:**
```
50 kW motor, 400V DC bus, 150A phase current RMS
Using 650V, 15mΩ Si MOSFET at 150°C junction temp (RDS_on ≈ 23mΩ)

Per MOSFET:
  P_cond = (150)² × 0.023 = 517 W
  P_sw (at 10 kHz) ≈ 200 W
  Total per MOSFET = 717 W

Six MOSFETs total: 6 × 717 = 4.3 kW loss
Efficiency = 50 / (50 + 4.3) = 92%

Using 650V SiC MOSFET with 10mΩ, faster switching:
  P_cond = (150)² × 0.010 = 225 W
  P_sw (at 20 kHz) ≈ 80 W
  Total per MOSFET = 305 W
  Total loss = 1.83 kW
  Efficiency = 50 / (50 + 1.83) = 96.5%

Efficiency gain: 4.5 percentage points!
```

### 3.2 Silicon MOSFETs vs SiC vs GaN

**Technology Comparison:**

| Parameter | Silicon (Si) | Silicon Carbide (SiC) | Gallium Nitride (GaN) |
|-----------|--------------|------------------------|------------------------|
| **Bandgap** | 1.1 eV | 3.2 eV | 3.4 eV |
| **Max Junction Temp** | 150-175°C | 175-200°C | 150-200°C |
| **Voltage Range** | 30-1700V | 650-1700V | 100-650V |
| **RDS_on (relative)** | Baseline | 30-40% lower | 40-60% lower |
| **Switching Speed** | Moderate (50-200 ns) | Fast (20-50 ns) | Very Fast (5-20 ns) |
| **Gate Charge** | Baseline | 30-50% lower | 50-70% lower |
| **Body Diode VF** | 0.7-1.2V | 2.5-4.5V | 1.5-2.5V |
| **Reverse Recovery** | Significant (Qrr) | Negligible | Negligible |
| **Cost (relative)** | 1× (baseline) | 2-4× | 3-5× |
| **Maturity** | Very mature | Mature | Emerging |
| **Availability** | Excellent | Good | Moderate |

**Silicon (Si) MOSFETs:**

**Advantages:**
- Lowest cost and widely available
- Mature technology with extensive application notes
- Good for low to medium voltage (<650V)
- Well-understood failure modes and reliability

**Disadvantages:**
- Higher RDS_on at elevated temperatures
- Slower switching (higher switching losses)
- Poor body diode reverse recovery
- Limited to ~1000V practical voltage

**Best for:**
- Cost-sensitive applications (e-bikes, scooters, power tools)
- Low voltage systems (24-48V)
- Low to medium power (<20 kW)
- PWM frequencies <15 kHz

**Silicon Carbide (SiC) MOSFETs:**

**Advantages:**
- 60-70% lower RDS_on than equivalent Si
- 3-5× faster switching (lower switching losses)
- Negligible reverse recovery (fast body diode)
- Higher temperature operation (200°C junction)
- Enables higher switching frequencies (20-50 kHz)
- Better efficiency (2-5 percentage points gain)

**Disadvantages:**
- 2-4× higher cost than Si
- Higher gate oxide stress (more sensitive to overvoltage)
- Poor body diode forward drop (3-4V)
- Gate driver design requires care (negative VGS for noise immunity)

**Best for:**
- High-performance EVs (>50 kW)
- High-efficiency applications where cost is justified
- High DC bus voltage (>400V)
- High switching frequency designs (>15 kHz)
- Thermal-constrained applications (high ambient temp)

**Typical Applications:**
- Tesla Model 3 (SiC inverter, ~98% efficiency)
- Premium EVs (Porsche Taycan, Audi e-tron GT)
- Fast chargers (>100 kW)

**Gallium Nitride (GaN) FETs:**

**Advantages:**
- Lowest RDS_on and gate charge
- Fastest switching speed (5-20 ns)
- Smallest package size (high power density)
- Zero reverse recovery
- Excellent for very high frequency (50-500 kHz)

**Disadvantages:**
- Most expensive (3-5× Si cost)
- Limited voltage range (<650V practical)
- Normally-ON topology requires special gate drivers
- Less mature technology (fewer long-term reliability data)
- Sensitive to overvoltage and overcurrent

**Best for:**
- Ultra-high switching frequency (>50 kHz)
- Compact, high-power-density designs
- Moderate voltage systems (200-400V)
- Low to medium power (<50 kW currently)

**Emerging Applications:**
- High-performance drones
- Compact EV chargers
- Server power supplies
- Future: High-RPM motors requiring low inductance

### 3.3 Device Selection Criteria

**Decision Tree for Power Semiconductor Selection:**

```
START: Define application requirements
  │
  ├─ DC Bus Voltage?
  │   ├─ <100V → Si MOSFET (low voltage)
  │   ├─ 100-400V → Si, SiC, or GaN
  │   ├─ 400-800V → SiC or high-voltage Si
  │   └─ >800V → SiC or IGBT
  │
  ├─ Power Level?
  │   ├─ <5 kW → Si MOSFET (cost effective)
  │   ├─ 5-50 kW → Si or SiC (depends on efficiency target)
  │   ├─ 50-200 kW → SiC preferred
  │   └─ >200 kW → SiC or IGBT modules
  │
  ├─ Efficiency Target?
  │   ├─ >96% → SiC required
  │   ├─ 93-96% → SiC or optimized Si
  │   └─ <93% → Si acceptable
  │
  ├─ Switching Frequency?
  │   ├─ <10 kHz → Si adequate
  │   ├─ 10-20 kHz → Si or SiC
  │   ├─ 20-50 kHz → SiC preferred
  │   └─ >50 kHz → GaN or SiC
  │
  ├─ Cost Sensitivity?
  │   ├─ High (consumer) → Si
  │   ├─ Medium (automotive) → SiC
  │   └─ Low (aerospace) → SiC or GaN
  │
  └─ Thermal Constraints?
      ├─ Unconstrained → Si or SiC
      ├─ Limited cooling → SiC (lower losses)
      └─ High ambient temp → SiC (200°C capable)
```

**Practical Selection Guidelines:**

| Application | Voltage | Power | Recommended Device |
|-------------|---------|-------|---------------------|
| E-bike/Scooter | 48V | 1-5 kW | 100V Si MOSFET |
| Golf cart, AGV | 48-72V | 5-10 kW | 150V Si MOSFET |
| Small EV (Tata Nano EV) | 72-144V | 10-30 kW | 200-300V Si MOSFET |
| Mid-size EV (Nissan Leaf) | 350-400V | 80 kW | 650V SiC MOSFET |
| Premium EV (Tesla Model 3) | 350-400V | 150 kW | 650V SiC MOSFET |
| Heavy truck/bus | 600-800V | 200+ kW | 1200V SiC modules |
| Industrial servo | 540-680V | 5-50 kW | 1200V SiC or Si |
| High-speed spindle | 400V | 20-100 kW | 650V SiC (high freq) |

**Key Selection Criteria Summary:**

1. **Start with voltage rating**: This immediately narrows options
2. **Consider power losses**: Calculate conduction and switching losses
3. **Evaluate thermal design**: Can you cool it adequately?
4. **Calculate efficiency**: Does SiC cost justify efficiency gain?
5. **Check availability and supply chain**: Can you source it reliably?
6. **Review gate driver requirements**: Do you have suitable drivers?

### 3.4 Parallel Operation Considerations

**Why Parallel MOSFETs?**

1. **Current Capability**: Increase beyond single device rating
2. **Thermal Performance**: Distribute power dissipation across multiple devices
3. **Reduced RDS_on**: N devices in parallel → RDS_on / N (lower conduction loss)
4. **Junction Temperature**: Lower Tj for same total power (improved reliability)
5. **Redundancy**: One device failure may not stop entire inverter (graceful degradation)
6. **Cost Optimization**: Multiple smaller MOSFETs may be cheaper than single large device

**Thermal Performance Benefits:**

```
Example: 500W dissipation per phase leg

Single MOSFET:
  P_loss = 500W
  Rth_JC = 0.5°C/W
  ΔT_junction = 500 × 0.5 = 250°C rise!
  Tj = 25 + 250 = 275°C → FAILURE!

Two MOSFETs in parallel (assuming perfect current sharing):
  P_loss per device = 250W
  Rth_JC = 0.5°C/W (same device)
  ΔT_junction = 250 × 0.5 = 125°C rise
  Tj = 25 + 125 = 150°C → Safe operation

Three MOSFETs in parallel:
  P_loss per device = 167W
  ΔT_junction = 167 × 0.5 = 83.5°C rise
  Tj = 25 + 83.5 = 108.5°C → Excellent thermal margin
```

**Thermal derating benefits:**
- Lower junction temperature → longer MTBF (halving Tj doubles lifetime)
- Reduced thermal stress on solder joints and PCB
- Less demanding cooling requirements (smaller heatsink or lower airflow)
- Better efficiency at elevated ambient temperatures

**Challenges in Parallel Operation:**

1. **Static Current Imbalance**:
   - MOSFETs never have exactly the same RDS_on (±10% tolerance typical)
   - Device with lower RDS_on carries more current
   - Variation increases with temperature coefficient mismatch
   - Can lead to thermal runaway in hottest device

2. **Dynamic Current Imbalance**:
   - Different gate threshold voltages (Vth variation ±20%)
   - Unequal gate drive loop inductances cause timing skew
   - One device turns on/off faster → carries transient surge current
   - High-frequency current imbalance even if DC sharing is good

3. **Layout-Induced Imbalance**:
   - Different trace lengths → different resistances and inductances
   - Unequal source inductance causes negative feedback imbalance
   - Different gate loop impedances → different switching speeds
   - Unequal thermal coupling to heatsink

4. **Thermal Coupling Effects**:
   - Positive temperature coefficient of RDS_on helps balance (hotter device increases R, takes less current)
   - BUT during switching, negative temperature coefficient of Vth can cause runaway
   - Adjacent devices on heatsink thermally couple (one hot device heats its neighbor)

**Gate Driver Design for Parallel MOSFETs:**

**Option 1: Individual Gate Resistors (Recommended)**

```
           ┌── Rg1 ──┬─── MOSFET 1 Gate
           │         │
 Gate ─────┼── Rg2 ──┼─── MOSFET 2 Gate
 Driver    │         │
 Output    └── Rg3 ──┴─── MOSFET 3 Gate

Where: Rg1 = Rg2 = Rg3 (matched values)
```

**Advantages:**
- Each MOSFET has matched gate impedance
- Prevents oscillation between paralleled gates
- Allows fine-tuning of individual switching speed
- Isolates gate-drain capacitance coupling

**Design Guidelines:**
- Use 1-5Ω per MOSFET for high power, 5-10Ω for medium power
- Tolerance: ±1% matched resistors recommended
- Low inductance resistors (surface mount, short leads)

**Option 2: Common Gate Drive with Star Topology**

```
                   Rg_common
 Gate Driver ────────RRR──────┬───── MOSFET 1 Gate
                               │
                               ├───── MOSFET 2 Gate
                               │
                               └───── MOSFET 3 Gate

Star point at physical center of parallel MOSFETs
```

**Less recommended** - can cause gate oscillations between devices.

**Option 3: Separate Isolated Drivers (Best Performance)**

```
 PWM Signal ──┬─→ Isolated Driver 1 ──→ Rg1 ──→ MOSFET 1
              │
              ├─→ Isolated Driver 2 ──→ Rg2 ──→ MOSFET 2
              │
              └─→ Isolated Driver 3 ──→ Rg3 ──→ MOSFET 3
```

**Advantages:**
- Complete electrical isolation between parallel devices
- Independent control of each MOSFET switching
- Can implement active current balancing
- Best for high-power applications (>50kW)

**Cost:** High ($10-15 per driver × N devices)

**PCB Layout Considerations for Parallel MOSFETs:**

**1. Power Loop Symmetry:**

```
                    DC+ Bus
                       │
        ┌──────────────┼──────────────┐
        │              │              │
     ┌──┴──┐        ┌──┴──┐        ┌──┴──┐
     │ Q1  │        │ Q2  │        │ Q3  │  (3 parallel high-side)
     └──┬──┘        └──┬──┘        └──┬──┘
        │              │              │
        └──────────────┼──────────────┘
                       │
                   Phase Out
                       │
        ┌──────────────┼──────────────┐
        │              │              │
     ┌──┴──┐        ┌──┴──┐        ┌──┴──┐
     │ Q4  │        │ Q5  │        │ Q6  │  (3 parallel low-side)
     └──┬──┘        └──┬──┘        └──┬──┘
        │              │              │
        └──────────────┼──────────────┘
                       │
                    DC- Bus

Critical: Equal trace lengths from DC+/DC- to each MOSFET
```

**Key Requirements:**
- **Equal drain trace lengths**: ±5mm tolerance from DC+ to all drains
- **Equal source trace lengths**: ±5mm tolerance from all sources to DC-/Phase
- Minimize total loop inductance: <10 nH for high-performance designs
- Wide, thick copper traces (4-6 oz copper) for power paths

**2. Kelvin Source Connection:**

```
        Drain
          │
      ┌───┴───┐
      │MOSFET │
      │       │
      └───┬───┘
          │ Source (Power)
          ├────────→ To DC- or Phase (heavy trace)
          │
          └─────────→ To Gate Driver Return (separate Kelvin trace)

Purpose: Eliminates voltage drop in source inductance from affecting gate drive
```

**Implementation:**
- Separate sense trace from source pin directly to gate driver ground
- Keep Kelvin trace short (<25mm) and away from noisy switching nodes
- Use 4-layer PCB minimum: Layer 1 (signals), Layer 2 (GND), Layer 3 (power), Layer 4 (signals)

**3. Gate Drive Routing:**

```
For 3 parallel MOSFETs, use equal-length serpentine routing:

Gate Driver ──┬── (serpentine, 50mm) ──→ Rg1 ──→ Q1
              │
              ├── (serpentine, 50mm) ──→ Rg2 ──→ Q2
              │
              └── (serpentine, 50mm) ──→ Rg3 ──→ Q3

All gate traces exactly same length ±2mm
```

**Best Practices:**
- Match gate drive trace lengths to ±2mm
- Use 0.5-1.0mm trace width for gate drives (not critical for current, but impedance controlled)
- Route gate traces away from drain nodes (high dv/dt) to prevent coupling
- Ground plane under gate traces for shielding

**4. Thermal Layout:**

**Inline Configuration:**
```
   [Q1] [Q2] [Q3]    ← All MOSFETs in a row
   ════════════════   ← Common heatsink
```
**Pros:** Simple routing, good for low current imbalance
**Cons:** End devices run cooler than center (airflow gradient)

**Triangular Configuration:**
```
      [Q2]
     /    \
   [Q1]  [Q3]
   ══════════   ← Common heatsink
```
**Pros:** More symmetric thermal coupling
**Cons:** More complex PCB routing

**Spacing:**
- Minimum 10mm between parallel MOSFETs for thermal isolation
- Maximum 30mm to keep power loop inductance low
- Use thermal vias under each MOSFET (50-100 vias, 0.3mm diameter)

**Thermal Considerations for Parallel MOSFETs:**

**1. Heatsink Mounting:**

```
Incorrect (causes thermal imbalance):
  Q1    Q2    Q3
  ↓     ↓     ↓
 TIM   TIM   TIM   ← Different thickness!
 ═══════════════   Heatsink

Correct (equal thermal coupling):
  Q1    Q2    Q3
  ↓     ↓     ↓
  ───────────────  ← Precision machined mounting surface
 ═══════════════   Heatsink with flatness <0.05mm
```

**Best Practices:**
- Use precision TIM application (screen printing or pre-applied pads)
- Heatsink flatness specification: <0.05mm across MOSFET mounting area
- Equal torque on all mounting screws (use torque wrench: 0.5-0.8 Nm typical)
- TIM thickness tolerance: ±10 μm for critical applications

**2. Thermal Monitoring:**

```
Implementation options:

Option A: NTC thermistor on heatsink between MOSFETs
Option B: Temperature sense diode in MOSFETs (if available)
Option C: Infrared thermal camera during development

Derating curve:
  Tj < 100°C: 100% current rating
  Tj = 125°C: 80% current rating  (start derating)
  Tj = 150°C: 50% current rating  (aggressive derating)
  Tj > 175°C: Shutdown (protection)
```

**3. Airflow Management:**

For air-cooled parallel MOSFETs:
- Orient MOSFETs in line with airflow direction
- First MOSFET sees coolest air, last sees heated air
- Compensate by making downstream devices carry slightly less current (via Rg tuning)
- CFD simulation recommended for >10kW designs

**Current Sharing Analysis:**

```
Two MOSFETs in parallel:
  Device 1: RDS_on = 10 mΩ
  Device 2: RDS_on = 11 mΩ  (10% higher, typical production spread)

Total current: 200A

Current distribution (static):
  I1 = I_total × (RDS2 / (RDS1 + RDS2))
     = 200 × (11 / 21) = 105A  (52.5%)

  I2 = I_total × (RDS1 / (RDS1 + RDS2))
     = 200 × (10 / 21) = 95A   (47.5%)

Power dissipation:
  P1 = I1² × RDS1 = 105² × 0.010 = 110W  (+11% from average)
  P2 = I2² × RDS2 = 95² × 0.011 = 99W    (-11% from average)

Thermal effect:
  If P1 > P2, then Tj1 > Tj2
  Higher Tj1 → higher RDS1 → I1 decreases slightly (self-balancing)

Positive temperature coefficient helps: dRDS/dT ≈ +0.5%/°C
```

**Imbalance Mitigation Strategies:**

1. **Device Binning**:
   - Test RDS_on of batch at same temperature
   - Select devices within ±3% RDS_on
   - Costs more but greatly improves sharing

2. **Source Inductance Balancing**:
   - Add small resistor in source of "faster" MOSFET (0.5-2 mΩ)
   - Creates negative feedback (higher current → higher voltage drop → less drive)
   - Improves dynamic current sharing during switching

3. **Active Gate Drive Control** (advanced):
   - Sense individual MOSFET currents
   - Adjust gate drive voltage to balance currents
   - Used in >100kW industrial drives
   - Adds significant cost and complexity

**When to Parallel MOSFETs:**

**Good candidates:**
- High current phase (>300A) where single device is not available
- Thermal constraints (limited heatsink size, high ambient temperature)
- Need graceful degradation (N+1 redundancy)
- Cost optimization (3× $10 MOSFETs cheaper than 1× $50 MOSFET)

**When NOT to Parallel:**

- If single device with adequate margin is available
- Cost of N smaller devices + complex layout > one larger device
- PCB space is severely constrained
- Cannot achieve symmetrical layout
- Switching frequency >50 kHz (dynamic imbalance becomes severe)

**Better Alternative: Use Larger Single Device or Module**
- Power modules (e.g., Infineon HybridPACK, Wolfspeed CAB-series) have internal paralleling optimized
- Single large die better than multiple small dies (no imbalance)
- Factory-optimized internal layout
- Lower inductance (integrated design)
- Simplifies gate drive and external layout
- **Cost premium:** 1.3-1.5× cost of equivalent discrete paralleling, but saves design/test time

---

### References for Section 3:

**Books:**
1. *"Power MOSFET Basics"* by Vishay Siliconix Application Note
2. *"Fundamentals of Power Semiconductor Devices"* by B. Jayant Baliga - Comprehensive theory
3. *"SiC Power Devices and Applications"* - Wide Bandgap Semiconductors (IEEE Press)

**Application Notes:**
1. **Infineon**: "MOSFET Power Losses Calculation Using the Datasheet Parameters" (AN2019-16)
2. **ON Semiconductor**: "Paralleling Power MOSFETs" (AND9094/D)
3. **Texas Instruments**: "Understanding the Basics of SiC MOSFETs" (SNOAA36)
4. **Wolfspeed (Cree)**: "SiC MOSFET Gate Driver Design Considerations" (Application Note)
5. **STMicroelectronics**: "Silicon vs Silicon Carbide: A Comparison" (AN5089)
6. **ROHM Semiconductor**: "SiC Power Devices and Modules - Application Manual"

**Articles and Papers:**
1. "Comparison of Si IGBT and SiC MOSFET-Based Inverters for Electric Vehicle Traction" - IEEE Transactions on Transportation Electrification
2. "GaN Power Devices for Automotive Applications" - SAE International
3. "Body Diode Reverse Recovery and its Effects in High-Performance Motor Drives" - PCIM Europe

**Videos:**
1. **Wolfspeed**: "SiC vs Si: Which Should You Choose?" (YouTube)
2. **Infineon**: "Power MOSFET Basics and Selection" (YouTube training series)
3. **EEVblog**: "Power MOSFET Tutorial" (YouTube)
4. **Texas Instruments**: "GaN FET Technology Overview" (TI Training)

**Datasheets (Comparative Study):**
1. **Si**: Infineon IPW65R019C7 (650V, 114A, 19mΩ Si CoolMOS)
2. **SiC**: Wolfspeed C3M0021120K (1200V, 108A, 21mΩ SiC MOSFET)
3. **GaN**: GaN Systems GS66516T (650V, 30A, 50mΩ GaN FET)

**Industry Whitepapers:**
1. **Tesla**: "Model 3 Drive Unit: Full SiC Inverter" (teardown analysis by Munro & Associates)
2. **Yole Développement**: "SiC and GaN Power Semiconductor Market Report 2024"

---

## 4. Gate Driver Design

### 4.1 Gate Driver Requirements

The gate driver provides the voltage and current needed to charge/discharge the MOSFET gate capacitance for fast, controlled switching.

**Key Requirements:**

1. **Voltage Levels**:
   - **Turn-ON voltage (VGS_on)**: +12V to +18V (Si), +15V to +20V (SiC)
   - **Turn-OFF voltage (VGS_off)**: 0V or negative (-2V to -5V for noise immunity)
   - SiC MOSFETs benefit from negative turn-off voltage to prevent false triggering

2. **Drive Current Capability**:
   ```
   Peak gate current = Qg / t_switch

   Example: Qg = 150 nC, target switching time = 50 ns
   I_gate_peak = 150 nC / 50 ns = 3A
   ```

   - Typical requirements: 2-5A for medium power, 5-10A for high power
   - Higher current → faster switching → lower switching losses (but more EMI)

3. **Propagation Delay**:
   - Time from control signal to MOSFET switching
   - Critical for deadtime accuracy
   - Target: <50 ns for high-performance drives
   - Matched delays across all 6 drivers (<10 ns mismatch)

4. **Isolation**:
   - **High-side drivers**: Must be isolated (floating potential)
   - **Low-side drivers**: Can be non-isolated (referenced to power ground)
   - Isolation voltage: >1000V minimum, >2500V for automotive
   - Isolation methods: Optocoupler, magnetic (transformer), capacitive

5. **Protection Features**:
   - Undervoltage lockout (UVLO): Prevents operation with insufficient gate voltage
   - Desaturation (DESAT) detection: Short-circuit protection
   - Miller clamp: Prevents false turn-on from dv/dt
   - Overcurrent shutdown

### 4.2 Discrete Gate Driver Circuits

**Basic Discrete Driver (Low-Side):**

```
MCU PWM ───┐
           │
          ┌▼┐ 74HC04         ┌──────┐
          │ │  Inverter      │ Gate │  Rg   ┌──┐
          │ ├───────────┬────┤Driver├───RRR─┤Q1│
          └─┘           │    │ IC   │       └┬─┘
                        │    │(e.g.,│        │
                    ┌───▼─┐  │IR2110│      ──┴── Phase A
                    │     │  └──────┘      (Motor)
                    │Logic│
                    │Supply│
                    │+15V │
                    └─────┘
```

**Components:**

1. **Buffer/Level Shifter**:
   - 74HC series logic for 3.3V to 5V conversion
   - Texas Instruments SN74LVC series for fast switching
   - Ensures clean logic transitions

2. **Gate Driver IC** (e.g., Infineon 2ED020I12-F2, Texas Instruments UCC27714):
   - Provides high peak current (2-10A)
   - Level shifting for high-side
   - Built-in deadtime generation
   - UVLO protection

3. **Gate Resistor (Rg)**:
   - Controls turn-on/turn-off speed
   - Typical: 1-10Ω for Si MOSFETs, 2-20Ω for SiC
   - Trade-off: Fast switching (low Rg) vs EMI/ringing (high Rg)

4. **Turn-off Diode and Resistor (Optional)**:
   ```
         ┌────Rg_on───┐
   Gate  │            │  MOSFET Gate
   Drive ├──┐    ┌────┤
         │  │    │    │
         │  D    Rg_off
         │  │    │
         └──┴────┴────┘
   ```
   - Allows asymmetric switching (slow turn-on, fast turn-off)
   - Reduces turn-on EMI while maintaining low turn-off loss

**High-Side Bootstrap Circuit:**

```
                  VDD (+15V)
                   │
                   D_boot (Fast diode)
                   │
    Low ───────────┴──────── C_boot ──┬──── VCC (Driver supply)
    Side                     (1-10µF)  │
    Switch                             │
    OFF                         ┌──────┴───────┐
                                │   High-Side  │
    MCU ───────────────────────┤ Gate Driver  │
    PWM                         │     IC       │
                                └──────┬───────┘
                                       │
                                      Gate ─── High-Side MOSFET
```

**Bootstrap Operation:**
- When low-side switch is ON, bootstrap capacitor charges through diode
- Provides floating supply for high-side driver
- Simple and low-cost solution

**Bootstrap Limitations:**
- Requires periodic low-side conduction to recharge capacitor
- Not suitable for >95% duty cycle or DC operation
- Capacitor must be sized for gate charge and leakage

**Bootstrap Capacitor Sizing:**
```
C_boot ≥ 10 × Qg / ΔV_ripple

Example:
  Qg = 150 nC, acceptable ripple = 1V
  C_boot ≥ 10 × 150 nC / 1V = 1.5 µF

Use 2.2 µF or 4.7 µF ceramic (X7R) for margin
```

### 4.3 Integrated Gate Driver ICs

**Popular Gate Driver ICs:**

| IC Family | Manufacturer | Features | Isolation | Use Case |
|-----------|--------------|----------|-----------|----------|
| **IR2110/IR2301** | Infineon | Bootstrap high-side, 2A | No | Low-cost Si MOSFET |
| **UCC27714** | TI | 4A peak, advanced protection | No | Medium power Si/SiC |
| **SI827x** | Silicon Labs | Isolated, 4A, up to 5kV | Yes (capacitive) | High-performance |
| **ACPL-33GT** | Broadcom | Isolated, 2.5A gate drive | Yes (optocoupler) | Industrial |
| **1ED3491** | Infineon | Coreless transformer isolation | Yes (magnetic) | Automotive SiC |
| **UCC21750** | TI | 10A, reinforced isolation | Yes (capacitive) | High-power SiC |

**Infineon IR2110 (Classic Bootstrap Driver):**

Features:
- 500V max floating voltage
- 2A source/sink current
- Built-in UVLO
- Low propagation delay (~150 ns)
- Low cost (~$1-2)

Limitations:
- No galvanic isolation
- Basic protection only
- Not suitable for SiC (limited current)

**Texas Instruments UCC27714 (Advanced Non-Isolated):**

Features:
- 4A peak drive current
- Split outputs (separate high/low side)
- Programmable deadtime
- DESAT protection input
- Faster propagation (~50 ns)
- Cost: ~$3-5

**Infineon 1ED3491 (Isolated, Automotive-Grade):**

Features:
- Coreless transformer isolation (1500V)
- 10A peak current (excellent for SiC)
- Miller clamping
- Advanced diagnostics
- Automotive qualified (AEC-Q100)
- Cost: ~$8-12

Best for: High-performance SiC motor drives

### 4.4 Bootstrap vs Isolated Power Supply

**Comparison:**

| Aspect | Bootstrap | Isolated DC-DC |
|--------|-----------|----------------|
| **Cost** | Low ($2-5 per phase) | High ($15-30 per phase) |
| **Complexity** | Simple | Complex |
| **Duty Cycle** | <95% | 0-100% |
| **Startup** | Requires low-side ON first | Immediate |
| **Reliability** | Good | Excellent |
| **Switching Frequency** | >5 kHz recommended | Any |
| **Best for** | Standard motor drives | Servo, special applications |

**When to Use Bootstrap:**
- Standard FOC motor drive (alternating PWM)
- Cost-sensitive designs
- PWM frequency >5 kHz
- Normal duty cycle range (20-80%)

**When to Use Isolated Supply:**
- High duty cycle operation (>95%)
- Low switching frequency (<5 kHz)
- DC operation required
- Safety-critical applications (medical, aerospace)
- Multiple paralleled inverters sharing control

**Isolated Supply Options:**

1. **Isolated DC-DC Modules**:
   - RECOM R1.5P-xxx series (1.5W, 1kV isolation)
   - MORNSUN B0515XT-1WR3 (1W, 3kV isolation)
   - Cost: $5-10 per module × 3 high-side = $15-30

2. **Custom Flyback Transformer**:
   - Single transformer with multiple secondary windings
   - Lower cost for production volumes
   - Requires careful design (leakage inductance, isolation)

3. **Isolated Gate Driver ICs** (as shown in 4.3):
   - Self-contained solution
   - Built-in isolation (no external DC-DC needed)
   - Higher IC cost but simpler BOM

### 4.5 Gate Resistance Selection

**Trade-offs:**

```
Low Rg (Fast Switching):          High Rg (Slow Switching):
✓ Lower switching losses          ✓ Reduced EMI
✓ Lower junction temperature      ✓ Less ringing/overshoot
✓ Higher efficiency               ✓ Safer for layout issues
✗ Higher EMI                      ✗ Higher switching losses
✗ More ringing/overshoot          ✗ Higher temperature
✗ Requires excellent PCB layout   ✗ Lower efficiency
```

**Calculation Method:**

```
Target switching time = t_sw
MOSFET gate charge = Qg
Gate drive voltage = VGS_drive

Rg ≈ (VGS_drive × t_sw) / Qg

Example:
  Qg = 150 nC
  VGS_drive = 15V
  Target t_sw = 100 ns

  Rg = (15 × 100n) / 150n = 10Ω
```

**Practical Guidelines:**

| MOSFET Type | Typical Rg | Switching Time | Application |
|-------------|------------|----------------|-------------|
| Si MOSFET (low power) | 10-47Ω | 100-300 ns | Consumer, <5kW |
| Si MOSFET (high power) | 2-10Ω | 50-150 ns | Automotive, 5-50kW |
| SiC MOSFET | 5-15Ω | 30-80 ns | High-efficiency, >20kW |
| GaN FET | 1-5Ω | 10-30 ns | High-frequency, compact |

**External vs Internal Rg:**

Some gate driver ICs have internal gate resistance (e.g., 2-4Ω). If using these:

```
Total Rg = Rg_internal + Rg_external

If IC has 3Ω internal and you want 10Ω total:
  Rg_external = 10 - 3 = 7Ω  (use standard 6.8Ω)
```

**Turn-on vs Turn-off Resistance:**

For reduced EMI with minimal loss penalty:
- **Turn-on**: Slower (higher Rg) → reduces di/dt and EMI
- **Turn-off**: Faster (lower Rg) → reduces switching loss

**Asymmetric Gate Drive Circuit:**
```
        Rg_on (10Ω)
         ─RRR─
            │
   Gate ────┼──── MOSFET Gate
   Drive    │
         ───┤>├─── Diode (allows bypass of Rg_on during turn-off)
            │
        Rg_off (2Ω)
         ─RRR─
            │
           GND
```

Result: Turn-on time = 100 ns, turn-off time = 20 ns

**Negative Gate Voltage (for SiC):**

SiC MOSFETs benefit from negative turn-off voltage:
- VGS_on = +15V to +20V
- VGS_off = -3V to -5V
- Prevents false turn-on from high dv/dt
- Improves noise immunity

Use split-rail gate driver (e.g., UCC21732, SI8271) or discrete negative supply.

---

### References for Section 4:

**Books:**
1. *"Gate Drive Circuits for Power Semiconductor Devices"* by Dr. Ulrich Nicolai (SEMIKRON)
2. *"Power Electronics Design Handbook"* by Nihal Kularatna - Chapter on gate drives

**Application Notes:**
1. **Infineon**: "Gate Resistance - What is the Right Value?" (Application Note AN2017-06)
2. **Texas Instruments**: "Gate Driver Fundamentals and Selection Criteria" (SLUA618)
3. **Wolfspeed**: "Driving SiC MOSFET: Gate Driver Design Considerations" (Application Note)
4. **ON Semiconductor**: "Understanding IGBT/MOSFET Gate Driver Circuits" (AND9083/D)
5. **Microchip**: "Gate Drive Considerations for SiC MOSFETs" (Application Note)
6. **Silicon Labs**: "Isolated Gate Driver Design for SiC MOSFETs" (AN1048)

**Articles and Papers:**
1. "Gate Drive Optimization for SiC MOSFET-Based Motor Drives" - IEEE APEC Conference
2. "Bootstrap Gate Driver Design Challenges and Solutions" - PCIM Europe
3. "Negative Gate Voltage Effects on SiC MOSFET Reliability" - IEEE Transactions on Power Electronics

**Videos:**
1. **TI Precision Labs**: "Gate Driver Design for Motor Control" (YouTube series)
2. **Wolfspeed**: "Best Practices for SiC MOSFET Gate Drive Design" (YouTube)
3. **Infineon**: "Gate Driver ICs Explained" (YouTube)
4. **Microchip**: "Isolated vs Non-Isolated Gate Drivers" (Webinar)

**Datasheets:**
1. Infineon IR2110 (600V Half-Bridge Driver)
2. Texas Instruments UCC27714 (Advanced 4A Driver with Protection)
3. Silicon Labs Si8271 (Isolated 4A Gate Driver IC)
4. Infineon 1ED3491 (Automotive Isolated Driver for SiC)

**Design Tools:**
1. **Texas Instruments**: WEBENCH Power Designer (includes gate driver calculator)
2. **Infineon**: IPOSIM (Inverter Power Stage Simulator)
3. **Wolfspeed**: SiC MOSFET Gate Driver Design Tool (online calculator)

---

## 5. Current Sensing

### 5.1 Current Measurement Requirements for FOC

Field Oriented Control requires accurate, fast current measurements to regulate motor torque and flux. The FOC algorithm transforms three-phase currents (ia, ib, ic) into the rotating dq reference frame where they appear as DC quantities suitable for PI control.

**Key Requirements:**

1. **Accuracy**:
   - ±1-2% over full operating range
   - Temperature coefficient: <100 ppm/°C
   - Linearity error: <1% of full scale
   - Example: 300A motor → ±3-6A accuracy needed

2. **Bandwidth**:
   - Must capture PWM current ripple (harmonics up to 5× PWM frequency)
   - Minimum: 100 kHz for 10 kHz PWM
   - Recommended: 200-500 kHz for clean measurements
   - Anti-aliasing filter cutoff: 0.3-0.5× PWM frequency

3. **Sampling Synchronization**:
   - Sample at PWM center or valley (when current is stable)
   - Avoid sampling during switching transients
   - Triggered by PWM timer in microcontroller
   - Aperture time: <100 ns for low jitter

4. **Dynamic Range**:
   - Must measure from near-zero to peak transient current
   - Typical: 0.1A to 3× rated current
   - Example: 100A rated, 300A peak → 3000:1 dynamic range
   - Requires 12-bit ADC minimum (16-bit preferred)

5. **Galvanic Isolation** (for inline/high-side sensing):
   - Required for safety in high-voltage systems
   - Isolation rating: >1000V minimum
   - Common-mode rejection: >80 dB

6. **Offset and Drift**:
   - Zero-current offset: <±0.5% of full scale
   - Offset drift: <50 mV over temperature
   - Auto-zeroing during startup or idle periods
   - Regular calibration in firmware

### 5.2 Shunt Resistor Sensing (Low-Side, High-Side, Inline)

**Shunt Resistor Principle:**

A low-value precision resistor in the current path generates a voltage proportional to current:

```
V_shunt = I_phase × R_shunt

Example: 100A current, 0.5 mΩ shunt
V_shunt = 100 × 0.0005 = 50 mV
```

**Low-Side Shunt Sensing:**

```
               VDC+
                │
           ┌────┼────┐
           │    │    │
          Q1   Q3   Q5  (High-side switches)
           │    │    │
        ───┼────┼────┼─── To Motor
           A    B    C
           │    │    │
          Q2   Q4   Q6  (Low-side switches)
           │    │    │
        ───┼────┼────┼───
           │    │    │
          Rsa  Rsb  Rsc  (Shunt resistors)
           │    │    │
           └────┴────┴─── VDC-
```

**Advantages:**
- Simple and low-cost
- Ground-referenced (easy to amplify)
- No isolation required
- Direct ADC connection possible

**Disadvantages:**
- Only measures current when low-side switch is ON
- Cannot measure during freewheeling (high-side ON, low-side OFF)
- Requires reconstruction algorithm for missing samples
- DC bus current includes ripple and switching noise

**Design Considerations:**

1. **Shunt Resistor Value Selection**:
   ```
   Trade-off:
   - Too low (e.g., 0.1 mΩ): Small signal, needs high amplification, noise susceptible
   - Too high (e.g., 5 mΩ): Large power loss, voltage drop affects efficiency

   Typical: 0.3-1.0 mΩ for high current (>100A)
            1-5 mΩ for medium current (10-100A)

   Power dissipation:
   P_shunt = I²_RMS × R_shunt

   Example: 150A RMS, 0.5 mΩ shunt
   P_shunt = 150² × 0.0005 = 11.25 W per shunt!

   Must use high-power shunt (≥15W rating with heatsinking)
   ```

2. **Shunt Resistor Specifications**:
   - **Power rating**: 2-3× calculated power for derating
   - **Tolerance**: ±1% or better
   - **Temperature coefficient**: ±50 ppm/°C or better
   - **Inductance**: <10 nH (use wide, flat design)
   - **Type**: Metal foil (Vishay WSL, Isabellenhuette, Ohmite)

3. **Kelvin Sensing Connection**:
   ```
                Current Flow
                    ───→
         ┌──────────────────────┐
         │   Shunt Resistor     │
         │    (Low Inductance)  │
         └──┬──────────────┬────┘
            │              │
       Power│              │Power
    Connection        Connection
            │              │
            │              │
         ───┴──         ───┴──  Sense leads (separate)
          +Sense        -Sense  (to amplifier)

   Purpose: Eliminates voltage drop in power connections
   ```

**High-Side Shunt Sensing:**

```
            VDC+
             │
        ┌────┼────┬────┐
        │    │    │    │
       Rsa  Rsb  Rsc  │   (Shunt resistors in DC+ feed)
        │    │    │    │
        │    │    │    │
       Q1   Q3   Q5   │   (High-side switches)
        │    │    │    │
     ───┼────┼────┼────┤
        A    B    C    │
        │    │    │    │
       Q2   Q4   Q6   │   (Low-side switches)
        │    │    │    │
        └────┴────┴────┘
             │
            VDC-
```

**Advantages:**
- Continuous current measurement (not switching-dependent)
- Better for certain control algorithms

**Disadvantages:**
- **High common-mode voltage** (floating at DC+ potential)
- Requires isolated amplifier or high CMRR amplifier
- More expensive
- Complex protection circuit design

**Inline Shunt Sensing (Between Inverter and Motor):**

```
    Inverter              Motor
    ┌──────┐             ┌─────┐
    │  Q1  │    Rsa      │     │
    ├──┬───┤───┤  ├─────┤  A  │
    │  │Q2 │             │     │
    ├──┼───┤    Rsb      │     │
    │  │Q4 │───┤  ├─────┤  B  │
    ├──┼───┤             │     │
    │  │Q6 │    Rsc      │     │
    └──┴───┘───┤  ├─────┤  C  │
                          └─────┘
```

**Advantages:**
- True phase current measurement
- Bidirectional (motoring and regeneration)
- Not affected by PWM switching state

**Disadvantages:**
- Higher common-mode voltage (floating at phase voltage)
- Requires 3 isolated amplifiers
- Most expensive option
- Longer current path (additional inductance)

### 5.3 Hall Effect Current Sensors

Hall effect sensors provide galvanic isolation and can measure AC and DC currents without inserting resistance in the power path.

**Operating Principle:**

```
    Primary Current (Motor phase)
         │
         ▼
    ════════  (Conductor or PCB trace)
         │
         │    Magnetic Field ⊙
      ┌──┴──┐
      │ Hall│  ← Senses magnetic field
      │Sensor│
      └──┬──┘
         │
       Output (Proportional to current)
```

**Types:**

1. **Open-Loop Hall Sensors** (e.g., Allegro ACS series):
   - Direct Hall element output
   - Lower cost ($2-5)
   - Accuracy: ±1-3%
   - Bandwidth: 100-200 kHz
   - Offset drift: Moderate (±50 mV over temp)

2. **Closed-Loop (Compensated) Hall Sensors** (e.g., LEM HASS, LA-H):
   - Feedback coil nulls magnetic field
   - Higher accuracy: ±0.5-1%
   - Better linearity and lower drift
   - Higher cost ($10-25)
   - Bandwidth: 200-500 kHz
   - Best for precision applications

**Advantages:**
- No power dissipation in measurement
- Galvanic isolation (>3kV typical)
- Measures DC to high frequency
- No voltage drop in power path
- Excellent for high current (up to 1000A+)

**Disadvantages:**
- More expensive than shunt + amplifier
- Larger physical size (requires space near conductor)
- Offset drift with temperature
- Susceptible to external magnetic fields (motor, cables)
- Requires careful placement and shielding

**Design Considerations:**

1. **Sensor Selection**:
   ```
   Current Range:
   - 0-50A: ACS712 (±2%), ACS770 (±1%)
   - 0-200A: ACS758 (±1%), LEM HTFS series
   - >200A: LEM HASS, LA-H series, or custom PCB trace + open-loop
   ```

2. **PCB Layout for Integrated Hall Sensors**:
   - Route current through sensor's integrated conductor
   - Minimize stray magnetic fields from nearby traces
   - Keep return currents away from sensor
   - Use ground plane cutouts if needed

3. **External Magnetic Field Immunity**:
   - Shield sensor with ferromagnetic material (mu-metal)
   - Differential sensing (two sensors, opposite polarity)
   - Mount away from motor housing and high-current cables
   - Twist motor phase cables to reduce radiated field

4. **Calibration and Offset Compensation**:
   - Auto-zero during startup (motor off, no current)
   - Store offset in EEPROM
   - Periodic re-calibration during idle periods
   - Temperature compensation table in firmware

### 5.4 Single Shunt vs Three Shunt Topologies

**Three-Shunt Topology:**

```
Low-side shunt in each phase leg

Advantages:
✓ Direct measurement of all three phase currents
✓ Simple reconstruction (ia, ib, ic directly measured)
✓ Works at all duty cycles (0-100%)
✓ Redundancy (can calculate 3rd current from two: ia + ib + ic = 0)
✓ Best accuracy and reliability

Disadvantages:
✗ Higher cost (3 shunts + 3 amplifiers)
✗ More complex analog circuitry
✗ Larger PCB area
```

**Single-Shunt Topology (DC Bus Shunt):**

```
         VDC+
          │
     ┌────┼────┬────┐
     │    │    │    │
    Q1   Q3   Q5   │
     │    │    │    │
     A    B    C    │
     │    │    │    │
    Q2   Q4   Q6   │
     │    │    │    │
     └────┴────┴────┘
          │
         Rsh  (Single shunt in DC bus)
          │
         VDC-

Current reconstruction based on switching states
```

**Advantages:**
- Lowest cost (1 shunt, 1 amplifier)
- Simplest analog design
- Minimal PCB area

**Disadvantages:**
- Complex reconstruction algorithm required
- Limited measurable duty cycle range (typically 10-90%)
- Cannot measure at very low or very high modulation indices
- Requires two ADC samples per PWM cycle
- Lower accuracy than three-shunt
- Algorithm complexity increases software load

**Single-Shunt Current Reconstruction:**

The DC bus current contains information about phase currents depending on which switches are ON:

```
Example switching states:

State: Q1=ON, Q2=OFF, Q3=OFF, Q4=ON, Q5=OFF, Q6=ON
  Current path: ia flows through Q1 (positive)
                ib and ic flow through Q4, Q6 (return)
  DC bus current: I_dc = ia

State: Q1=ON, Q2=OFF, Q3=ON, Q4=OFF, Q5=OFF, Q6=ON
  DC bus current: I_dc = ia + ib
```

By sampling at specific times and knowing the switching state:
1. Sample during first active vector → get one current
2. Sample during second active vector → get another current
3. Calculate third current: ic = -(ia + ib)

**Challenges:**
- Switching transients require delay before sampling (dead zone)
- At high/low modulation, insufficient time for stable measurement
- Requires precise timing and fast ADC

**When to Use Each Topology:**

| Application | Recommended Topology | Reason |
|-------------|---------------------|--------|
| Cost-sensitive (e-bike, tools) | Single-shunt | Lowest cost, acceptable performance |
| General EV, industrial | Three-shunt | Best balance of cost/performance |
| High-performance servo | Three-shunt or Hall | Accuracy and full duty cycle range |
| Very high current (>300A) | Hall effect | No power loss, isolation |
| Safety-critical (automotive) | Three-shunt | Redundancy, proven reliability |

### 5.5 Current Sensor Placement and FOC Algorithm Impact

**Placement Options:**

1. **Low-Side Shunt (Most Common)**:
   ```
   Location: Between low-side MOSFET source and ground

   Measurement timing:
   - Sample when low-side switch is ON
   - Typical: At PWM valley (center of on-time)
   - ADC trigger from PWM timer
   ```

2. **Inline Shunt**:
   ```
   Location: Between inverter output and motor terminal

   Advantages for FOC:
   - True phase current (not affected by switching)
   - Can use slower ADC (no switching noise)
   - Better for sensorless FOC (cleaner back-EMF measurement)
   ```

3. **DC Link Shunt**:
   ```
   Location: DC bus (positive or negative rail)

   FOC considerations:
   - Requires current reconstruction algorithm
   - Software must track switching states
   - PWM pattern may need modification at extremes
   - Higher CPU load
   ```

**ADC Sampling Strategy for FOC:**

**Synchronous Sampling (Recommended):**

```
PWM Cycle:
     ┌──────┐                    ┌──────┐
     │      │                    │      │
─────┘      └────────────────────┘      └─────
     ↑                           ↑
   Sample                      Sample
   (valley)                   (valley)

Timing:
- Trigger ADC at PWM valley (or peak for center-aligned)
- Current is stable (mid-pulse, no switching)
- Sample all three phases simultaneously (if possible)
- Conversion time: <2 μs for 12-bit
```

**Oversampling for Noise Reduction:**

For high-noise environments:
- Sample 4× per PWM cycle
- Average results (digital filtering)
- Improves SNR by √N (2× better with 4 samples)
- Trade-off: Higher CPU load

**Offset Calibration:**

```
Firmware calibration routine:

1. At startup (motor off):
   FOR i = 1 to 1000:
     Sample ADC channels (ia, ib, ic)
     Accumulate samples

   offset_a = average(samples_a)
   offset_b = average(samples_b)
   offset_c = average(samples_c)

2. During runtime:
   i_measured_a = ADC_a - offset_a
   i_measured_b = ADC_b - offset_b
   i_measured_c = ADC_c - offset_c

3. Periodic re-cal (every 10 minutes idle):
   Update offsets during zero-current periods
```

**Anti-Aliasing Filter Design:**

Required before ADC to prevent high-frequency noise aliasing:

```
         R (10-100Ω)
Sensor ──┤  ├────┬──── ADC Input
               │
              ┴ C (1-10nF)
              ─
              ─
               │
              GND

Cutoff frequency:
  f_c = 1 / (2π × R × C)

Design rule:
  f_c = 0.3 to 0.5 × f_PWM

Example: 10 kHz PWM
  f_c = 3-5 kHz
  Use R=10kΩ, C=4.7nF → f_c ≈ 3.4 kHz
```

**Impact on FOC Performance:**

| Current Sensing Aspect | Impact on FOC | Recommendation |
|------------------------|---------------|----------------|
| Accuracy (±1-2%) | Torque ripple, efficiency | Use ±1% shunts, calibrate offsets |
| Bandwidth (>100 kHz) | Current loop stability | 200+ kHz for clean control |
| Offset drift | d-axis current error | Auto-zero every startup |
| Phase matching | Current imbalance | Match all 3 channels within ±0.5% |
| Sampling delay | Phase lag in current loop | Minimize, compensate in controller |
| Noise (SNR >60dB) | Control jitter | Good filtering, layout, grounding |

---

### References for Section 5:

**Books:**
1. *"Current Sensing Techniques: A Review"* by Carsten Klumpner - IEEE Industrial Electronics Magazine
2. *"Motor Control Sensors and Actuators"* by Kenjo and Nagamori - Chapter 4

**Application Notes:**
1. **Texas Instruments**: "Current Sensing for Motor Control" (SLVA959)
2. **Infineon**: "Shunt-Based Current Sensing Techniques for Motor Control" (Application Note AN2018-11)
3. **STMicroelectronics**: "Single-Shunt Current Sensing in Motor Control" (AN4946)
4. **Allegro Microsystems**: "Hall Effect Current Sensing Application Guide" (AN296151)
5. **LEM**: "Current Transducers Selection Guide and Application Notes"
6. **Microchip**: "Single-Shunt Three-Phase Current Reconstruction Algorithm" (AN1299)

**Articles and Papers:**
1. "Comparison of Current Sensing Techniques for Motor Drives" - IEEE APEC Conference
2. "Single Shunt Current Measurement for PMSM Drives" - IEEE Transactions on Industry Applications
3. "Precision Current Measurement Techniques for High-Performance Motor Control" - PCIM Europe

**Videos:**
1. **TI Precision Labs**: "Current Sensing in Motor Drives" (YouTube series)
2. **Allegro Microsystems**: "Hall Effect Current Sensors Explained" (YouTube)
3. **STM32 Motor Control**: "Current Sensing Methods Comparison" (YouTube)

**Datasheets (Representative Examples):**
1. **Shunt Resistors**: Vishay WSL3637, Isabellenhuette PBV, Ohmite LVK12
2. **Hall Sensors**: Allegro ACS772, ACS730; LEM HASS 50-S, LA 55-P
3. **Shunt Amplifiers**: Texas Instruments INA240, INA181; Analog Devices AD8417

**Design Tools:**
1. **Texas Instruments**: Current Sense Amplifier Design Calculator
2. **Allegro**: Hall Sensor Selection Tool
3. **LEM**: Current Transducer Selector

---

## 6. Switching Dynamics and dv/dt Effects

### 6.1 Switching Transients in Power MOSFETs

When a MOSFET switches, the rapid change in voltage and current creates transients that affect system performance, efficiency, and EMI.

**MOSFET Turn-ON Sequence:**

```
Phase 1: Gate charging (0 to Vth)
  - Gate voltage rises from 0V to threshold
  - No drain current yet
  - Duration: td_on (turn-on delay)

Phase 2: Miller plateau (Vth to Vgs_miller)
  - Drain current rises rapidly
  - Gate voltage plateaus (Miller effect)
  - VDS still high (MOSFET in active region)
  - Duration: tri (current rise time)
  - High power dissipation: P = VDS × ID

Phase 3: Voltage fall
  - Drain voltage falls to RDS_on × ID
  - Gate continues charging through Cgd (Miller capacitance)
  - Duration: tfv (voltage fall time)
  - Still high power dissipation

Phase 4: Final gate charging
  - Gate voltage rises to final value (VGS_drive)
  - MOSFET fully ON
  - Low conduction loss: P = ID² × RDS_on
```

**MOSFET Turn-OFF Sequence:**

Reverse of turn-on:
1. Gate discharge begins
2. VDS rises (voltage rise time: trv)
3. Miller plateau
4. Current falls (current fall time: tfi)
5. Gate fully discharged

**Switching Loss Calculation:**

```
Energy loss per switching cycle:

E_on = ∫(VDS × ID × dt) during turn-on
     ≈ (1/6) × VDS × ID × (tri + tfv)

E_off = ∫(VDS × ID × dt) during turn-off
      ≈ (1/6) × VDS × ID × (trv + tfi)

Total switching loss:
P_sw = (E_on + E_off) × f_sw

Example:
  VDS = 400V, ID = 150A
  tri + tfv = 100 ns (turn-on)
  trv + tfi = 80 ns (turn-off)
  f_sw = 10 kHz

  E_on = (1/6) × 400 × 150 × 100n = 1 mJ
  E_off = (1/6) × 400 × 150 × 80n = 0.8 mJ
  P_sw = (1 + 0.8) mJ × 10 kHz = 18W per MOSFET

  Six MOSFETs: 108W total switching loss!
```

### 6.2 dv/dt Effects on System Performance

**What is dv/dt?**

Rate of change of voltage during switching:

```
dv/dt = ΔVDS / Δt

Example:
  VDS changes from 400V to 0V in 50 ns
  dv/dt = 400V / 50ns = 8000 V/μs = 8 V/ns
```

**Problems Caused by High dv/dt:**

1. **Gate Coupling and False Turn-ON**:
   ```
   High dv/dt on drain couples through Cgd (Miller capacitance)

   IG_induced = Cgd × (dv/dt)

   Example:
     Cgd = 200 pF, dv/dt = 10 V/ns
     IG_induced = 200p × 10V/ns = 2A!

   This current can charge the gate of an OFF MOSFET → false turn-on
   ```

   **Mitigation:**
   - Use negative gate voltage when OFF (-3V to -5V for SiC)
   - Low-impedance gate driver (strong pull-down)
   - Add Miller clamp circuit

2. **Common-Mode Noise in Isolated Systems**:
   - High dv/dt couples through isolation barrier capacitance
   - Creates common-mode current in ground loops
   - Corrupts low-level signals (current sensing, position sensors)

   **Mitigation:**
   - Use high CMRR amplifiers (>80 dB)
   - Proper grounding (star ground topology)
   - Twisted pair for sensor signals
   - Common-mode chokes on signal lines

3. **Motor Bearing Currents**:
   ```
   Motor winding to ground capacitance (Cwg) charges through bearings

   I_bearing = Cwg × (dv/dt)

   High dv/dt → high bearing current → bearing erosion → premature failure
   ```

   **Mitigation:**
   - Slower switching (higher Rg)
   - Common-mode chokes on motor cables
   - Insulated bearings
   - Motor frame grounding (with proper CM filter)
   - Use SiC with controlled di/dt and dv/dt

4. **Conducted and Radiated EMI**:
   - High dv/dt creates high-frequency harmonics
   - Couples to cables and radiates

   **Mitigation:**
   - See Section 9 (EMI/EMC Considerations)
   - RC snubbers
   - Proper shielding

### 6.3 Gate Resistance Trade-offs

**Effect of Gate Resistance on Switching Speed:**

```
Higher Rg → Slower switching → Lower dv/dt, di/dt

Advantages of high Rg (slow switching):
✓ Lower EMI
✓ Reduced ringing and overshoot
✓ Less stress on MOSFETs
✓ Lower bearing currents
✓ More forgiving of layout imperfections

Disadvantages:
✗ Higher switching losses
✗ Higher junction temperature
✗ Lower efficiency
✗ May require larger heatsink

Advantages of low Rg (fast switching):
✓ Lower switching losses
✓ Higher efficiency
✓ Lower junction temperature
✓ Can use higher PWM frequency

Disadvantages:
✗ Higher EMI
✗ More ringing and overshoot
✗ Requires excellent PCB layout
✗ May cause false turn-on
✗ Higher bearing currents
```

**Optimal Rg Selection Process:**

1. Start with manufacturer recommendation (datasheet)
2. Calculate based on target switching time (see Section 4.5)
3. Test with oscilloscope:
   - Measure VDS and ID waveforms
   - Check for overshoot (<20% acceptable)
   - Check for ringing (should damp within 2-3 cycles)
   - Measure EMI (near-field probe or spectrum analyzer)
4. Iterate:
   - If too much ringing → increase Rg
   - If losses too high → decrease Rg
   - Use asymmetric Rg (different for turn-on and turn-off)

**Asymmetric Gate Drive:**

```
Recommended for low EMI with acceptable efficiency:

Rg_on = 10-20Ω  (slow turn-on, low EMI)
Rg_off = 2-5Ω   (fast turn-off, low switching loss)

Circuit:
        Rg_on (15Ω)
         ─RRR─
            │
   Gate ────┼──── MOSFET Gate
   Drive    │
         ───┤>├─── Diode (bypass Rg_on during turn-off)
            │
        Rg_off (3Ω)
         ─RRR─
            │
           GND
```

### 6.4 Snubber Circuits

Snubbers absorb energy during switching transients to reduce voltage overshoot and ringing.

**RC Snubber (Most Common):**

```
         Drain
          │
      ┌───┴───┐
      │MOSFET │
      │       │
      └───┬───┘
          │ Source
          ├────────→ (Power connection)
          │
       ┌──┴──┐
       │     │
      Rs    Cs   (Snubber across MOSFET)
       │     │
       └──┬──┘
          │
```

**Design Equations:**

```
Snubber capacitor:
  Cs = (Ls × ID²) / (2 × VDS × ΔV_allowed)

Where:
  Ls = parasitic inductance in power loop
  ΔV_allowed = acceptable overshoot (e.g., 50V)

Snubber resistor:
  Rs = √(Ls / Cs) / 2  (critically damped)

Example:
  Ls = 50 nH (typical with good layout)
  ID = 150A
  VDS = 400V
  ΔV_allowed = 50V

  Cs = (50n × 150²) / (2 × 400 × 50) = 2.8 nF → use 3.3 nF

  Rs = √(50n / 3.3n) / 2 = 1.95Ω → use 2.2Ω

Power dissipation in snubber:
  P_snubber = Cs × VDS² × f_sw
  P_snubber = 3.3n × 400² × 10k = 5.3W

  Use 2.2Ω, 10W resistor
```

**RCD Snubber (More Efficient):**

```
         Drain
          │
      ┌───┴───┐
      │MOSFET │
      └───┬───┘
          │
       ┌──┴──┐
      Ds   Rs
       │     │
       └──┬──┘
          │
         Cs
          │
         GND

Advantage: Energy returned to DC bus (via Ds)
Disadvantage: More complex, requires fast recovery diode
```

**When to Use Snubbers:**

- **Always**: For >500W systems and VDS >200V
- **Critical**: For SiC MOSFETs (fast switching creates more ringing)
- **Optional**: For <100W low-voltage systems with good PCB layout

**Alternative to RC Snubbers:**

- **Layout optimization** (minimize Ls) - most effective!
- **Gate resistance tuning** (slow down switching)
- **Active snubbers** (complex, used in high-power >100kW)

### 6.5 Ringing, Overshoot, and EMI

**Cause of Ringing:**

```
Parasitic inductance (Ls) + MOSFET output capacitance (Coss) form LC resonator

Resonant frequency:
  f_ring = 1 / (2π × √(Ls × Coss))

Example:
  Ls = 50 nH, Coss = 500 pF
  f_ring = 1 / (2π × √(50n × 500p)) = 31.8 MHz

This 30+ MHz ringing creates:
  - EMI in AM radio band
  - False triggering of logic circuits
  - Voltage stress on MOSFETs
```

**Voltage Overshoot:**

```
ΔV_overshoot = Ls × (di/dt)

Where di/dt during turn-off:
  di/dt = ID / t_fall

Example:
  Ls = 50 nH, ID = 150A, t_fall = 50 ns
  di/dt = 150A / 50ns = 3000 A/μs
  ΔV_overshoot = 50n × 3000A/μs = 150V!

Total voltage stress = VDC + ΔV_overshoot
  = 400V + 150V = 550V

Must ensure MOSFET rated for this (650V MOSFET minimum)
```

**Mitigation Strategies:**

1. **Minimize Loop Inductance** (MOST IMPORTANT):
   - Wide, short PCB traces
   - Multi-layer PCB with power planes
   - DC link capacitors close to MOSFETs (<10mm)
   - Laminated bus bar for high power
   - Target: Ls <20 nH for excellent performance, <50 nH acceptable

2. **Gate Resistance Optimization**:
   - Slow down switching to reduce di/dt
   - Use 5-15Ω for SiC, 10-47Ω for Si
   - Asymmetric drive (slow turn-on, moderate turn-off)

3. **Snubber Circuits**:
   - RC snubber across each MOSFET
   - Sized as shown in Section 6.4

4. **Controlled dv/dt Gate Drivers**:
   - Some ICs have programmable dv/dt control
   - Examples: UCC21750 (TI), 1ED3491 (Infineon)
   - Trade-off: Slower switching, but controlled EMI

5. **Low-Inductance DC Link Capacitors**:
   - Use ceramic or film capacitors with low ESL
   - Multiple smaller caps better than one large cap
   - Mount very close to MOSFET drain/source

**Acceptable Levels:**

```
Voltage overshoot: <20% of VDC (e.g., <80V for 400V system)
Ringing amplitude: Should decay to <10% within 3 cycles
Ringing frequency: 10-100 MHz typical (minimize with layout)
```

---

### References for Section 6:

**Books:**
1. *"Switching Power Supply Design"* by Abraham Pressman - Chapter 6: Snubber Circuits
2. *"Power Electronics: Converters, Applications, and Design"* by Mohan et al. - Chapter on switching dynamics

**Application Notes:**
1. **Infineon**: "Understanding and Minimizing Switching Losses" (AN2015-10)
2. **Texas Instruments**: "Understanding and Applying Snubber Circuits for Power Switches" (SLVA298)
3. **Wolfspeed**: "Managing dv/dt and di/dt in SiC Designs" (Application Note)
4. **ON Semiconductor**: "Reducing Ringing and EMI in MOSFET Circuits" (AND9083/D)
5. **STMicroelectronics**: "MOSFET Switching Loss and dv/dt Modeling" (AN5293)

**Articles and Papers:**
1. "Impact of Fast Switching SiC MOSFETs on Motor Drive dv/dt and Bearing Currents" - IEEE ECCE Conference
2. "Snubber Design for High-Power Motor Drives" - PCIM Europe
3. "Gate Driver Optimization for Reduced EMI in Motor Controllers" - IEEE Transactions on Power Electronics

**Videos:**
1. **TI Precision Labs**: "Understanding MOSFET Switching Losses" (YouTube)
2. **Wolfspeed**: "dv/dt and di/dt Control in SiC Designs" (YouTube)
3. **Keysight Technologies**: "How to Measure Power MOSFET Switching Waveforms" (YouTube)

**Test Equipment Application Notes:**
1. **Tektronix**: "Power MOSFET Switching Loss Measurement" (Application Note 243W-18447-1)
2. **Rohde & Schwarz**: "EMI Debugging Techniques for Power Electronics" (Application Note)

---

## 7. Thermal Management

Thermal management is critical for reliability and performance of motor controllers. MOSFETs generate significant heat that must be removed to prevent thermal runaway and ensure long-term reliability.

### 7.1 Power Loss Calculations

**Total Power Loss in Inverter:**

```
P_total = P_conduction + P_switching + P_gate + P_other

Where:
  P_conduction = Conduction losses in MOSFETs
  P_switching = Switching losses
  P_gate = Gate drive power
  P_other = DC link cap ESR, shunt resistor, etc.
```

**Conduction Loss (Per MOSFET):**

```
P_cond = I²_RMS × RDS_on(Tj)

Key considerations:
- RDS_on increases with temperature (typ. +0.5%/°C)
- RDS_on at 150°C ≈ 1.6-1.8× RDS_on at 25°C

Example:
  I_RMS = 100A per MOSFET
  RDS_on @ 25°C = 10 mΩ
  RDS_on @ 150°C = 17 mΩ (70% increase)

  P_cond @ 150°C = 100² × 0.017 = 170W per MOSFET
```

**Switching Loss (Per MOSFET):**

```
P_sw = (E_on + E_off) × f_sw

From Section 6.1:
  E_on ≈ (1/6) × VDS × ID × (tri + tfv)
  E_off ≈ (1/6) × VDS × ID × (trv + tfi)

Example:
  VDS = 400V, ID = 100A
  tri + tfv = 80 ns, trv + tfi = 60 ns
  f_sw = 15 kHz

  E_on = (1/6) × 400 × 100 × 80n = 0.53 mJ
  E_off = (1/6) × 400 × 100 × 60n = 0.40 mJ
  P_sw = (0.53 + 0.40) mJ × 15 kHz = 14W per MOSFET
```

**Gate Drive Power Loss:**

```
P_gate = Qg × VGS_drive × f_sw × N_mosfets

Example:
  Qg = 150 nC, VGS_drive = 15V, f_sw = 15 kHz, 6 MOSFETs
  P_gate = 150n × 15 × 15k × 6 = 0.2W (negligible)
```

**Total System Loss Example:**

```
50 kW motor drive @ 400V DC:

Per MOSFET:
  P_cond = 170W
  P_sw = 14W
  Total per MOSFET = 184W

Six MOSFETs: 6 × 184 = 1104W

Other losses:
  DC link cap ESR: ~50W
  Shunt resistors (3× 11W): 33W
  Gate drivers: 5W
  Misc: 10W

Total inverter loss = 1202W
Efficiency = 50000 / (50000 + 1202) = 97.7%
```

### 7.2 Thermal Resistance and Heat Transfer

**Thermal Resistance Network:**

```
Heat flows from junction → case → heatsink → ambient

Tj = Ta + P × (Rth_JC + Rth_CS + Rth_SA)

Where:
  Tj = Junction temperature (°C)
  Ta = Ambient temperature (°C)
  P = Power dissipation (W)
  Rth_JC = Junction-to-case thermal resistance (°C/W)
  Rth_CS = Case-to-sink thermal resistance (°C/W)
  Rth_SA = Sink-to-ambient thermal resistance (°C/W)
```

**Typical Thermal Resistance Values:**

| Component | Typical Rth | Notes |
|-----------|-------------|-------|
| Rth_JC (TO-247 MOSFET) | 0.3-0.6 °C/W | From datasheet |
| Rth_CS (with TIM) | 0.1-0.5 °C/W | Depends on TIM and mounting |
| Rth_SA (heatsink) | 0.5-5 °C/W | Depends on size, airflow |
| Rth_SA (liquid cooled) | 0.05-0.2 °C/W | Much better than air |

**Junction Temperature Calculation:**

```
Maximum junction temperature: Tj_max = 150-175°C (Si), 175-200°C (SiC)

Example:
  P = 184W per MOSFET
  Ta = 65°C (hot ambient, under hood)
  Rth_JC = 0.4 °C/W (TO-247)
  Rth_CS = 0.2 °C/W (good TIM, proper torque)
  Rth_SA = 0.8 °C/W (heatsink with fan)

  Tj = 65 + 184 × (0.4 + 0.2 + 0.8) = 65 + 258 = 323°C!

  → THERMAL RUNAWAY! Need better cooling or reduce losses
```

**Derating for Reliability:**

Target maximum junction temperature for long life:
- **Automotive/Industrial**: Tj ≤ 125°C (significant margin)
- **Consumer**: Tj ≤ 150°C
- **Short bursts**: Tj ≤ 175°C (max rating)

**Reliability vs Temperature:**

```
MTBF halves for every ~10-15°C increase in junction temperature

Example:
  MTBF @ Tj=100°C: 100,000 hours
  MTBF @ Tj=125°C: 35,000 hours
  MTBF @ Tj=150°C: 12,000 hours

Keeping cool = longer life!
```

### 7.3 Heatsink Design and Capacity Calculations

**Required Heatsink Thermal Resistance:**

```
Rth_SA_required = (Tj_max - Ta) / P - Rth_JC - Rth_CS

Example (continuing from above):
  Target Tj_max = 125°C (with margin)
  Ta = 65°C
  P = 184W per MOSFET (but 6 MOSFETs on same heatsink!)
  P_total_on_heatsink = 6 × 184 = 1104W
  Rth_JC = 0.4 °C/W
  Rth_CS = 0.2 °C/W

  Rth_SA_required = (125 - 65) / 1104 - 0 = 0.054 °C/W

  Note: We don't subtract Rth_JC and Rth_CS when calculating for the
  common heatsink, only when calculating individual junction temperatures.

  More accurate:
  For worst-case MOSFET:
    Tj = Ta + Rth_SA × P_total + (Rth_JC + Rth_CS) × P_mosfet
    125 = 65 + Rth_SA × 1104 + (0.4 + 0.2) × 184
    Rth_SA = (125 - 65 - 110.4) / 1104 = -0.045 °C/W

  → Impossible with air cooling! Need liquid cooling or reduce losses.
```

**Heatsink Selection:**

**Natural Convection (No Fan):**
- Rth_SA: 1-10 °C/W
- Suitable for: <100W total power
- Pros: Silent, reliable (no moving parts)
- Cons: Large size required

**Forced Air Cooling (With Fan):**
- Rth_SA: 0.2-2 °C/W (depends on airflow)
- Suitable for: 100W - 5kW
- Pros: Compact, cost-effective
- Cons: Fan maintenance, noise, dust

**Liquid Cooling:**
- Rth_SA: 0.01-0.2 °C/W
- Suitable for: >2kW to 200+ kW
- Pros: Excellent performance, compact
- Cons: Complex, expensive, leak risk

**Heatsink Sizing Example:**

```
Required: Rth_SA = 0.5 °C/W for 200W total dissipation

Natural convection:
  Heatsink volume ≈ 1000 cm³ (very large, ~10×10×10 cm)

Forced air (1-2 m/s airflow):
  Heatsink volume ≈ 200 cm³ (5×8×5 cm)
  Fan: 40-60mm, 12VDC, ~1-2W

Liquid cooling (water, 1 L/min):
  Cold plate area ≈ 100 cm²
  Much more compact
```

**Thermal Interface Material (TIM) Selection:**

| Material | Rth_CS (mm²/W) | Cost | Notes |
|----------|----------------|------|-------|
| Dry (no TIM) | 1-5 | Free | Poor, only for testing |
| Thermal grease | 0.2-0.5 | $ | Good, common, can dry out |
| Thermal pad | 0.5-1.5 | $$ | Easy to apply, consistent |
| Phase change | 0.2-0.4 | $$$ | Excellent, one-time application |
| Solder/Brazing | 0.05-0.1 | $$$$ | Best, permanent, high reliability |

**Application Guidelines:**
- **Grease**: Thin layer (25-50 μm), spread evenly, torque to spec
- **Pads**: Pre-cut to size, moderate pressure
- **Phase Change**: Melts at ~50-60°C, fills gaps
- **Solder**: For automotive/aerospace, requires special process

### 7.4 Air Cooling vs Liquid Cooling

**Air Cooling Design:**

```
Required Airflow Calculation:

CFM = (P_total × 3.16) / (ΔT)

Where:
  P_total = Total power dissipation (W)
  ΔT = Allowed temperature rise (°C)
  CFM = Cubic feet per minute airflow

Example:
  P_total = 1000W
  ΔT = 30°C (ambient 40°C → 70°C exhaust)
  CFM = (1000 × 3.16) / 30 = 105 CFM

  Requires: 120mm fan at ~3000 RPM
```

**Fan Selection:**

| Size | Typical Airflow | Power | Noise | Use Case |
|------|-----------------|-------|-------|----------|
| 40mm | 5-15 CFM | 1-2W | High | Low power (<200W) |
| 60mm | 10-25 CFM | 1-3W | Moderate | Medium power (200-500W) |
| 80mm | 20-50 CFM | 2-5W | Moderate | Medium power (500-1000W) |
| 120mm | 40-100 CFM | 3-8W | Low | High power (>1000W) |

**Airflow Best Practices:**
- Direct airflow across heatsink fins
- Use ducting to channel air (improves effectiveness by 30-50%)
- Avoid recirculation (hot air intake)
- Filter air in dusty environments
- Monitor fan speed (detect failures)

**Liquid Cooling Design:**

```
Coolant Flow Rate:

Q (L/min) = P_total / (ρ × Cp × ΔT × 1000)

Where:
  P_total = Power dissipation (W)
  ρ = Coolant density (kg/L) ≈ 1 for water
  Cp = Specific heat (J/kg·K) ≈ 4180 for water
  ΔT = Coolant temperature rise (°C)

Example:
  P_total = 5000W
  ΔT = 10°C rise
  Q = 5000 / (1 × 4180 × 10) = 0.12 L/min

  Very small flow rate! Liquid cooling is very effective.
```

**Liquid Cooling Components:**

1. **Cold Plate**:
   - Aluminum or copper
   - Microchannel or pin-fin design
   - Direct MOSFET mounting
   - Typical Rth: 0.01-0.05 °C/W per device

2. **Pump**:
   - Brushless DC (BLDC) for reliability
   - Flow rate: 1-5 L/min typical
   - Pressure: 0.5-2 bar
   - Power: 10-50W

3. **Radiator**:
   - Air-to-liquid heat exchanger
   - With fan (like automotive radiator)
   - Size based on total heat rejection

4. **Coolant**:
   - Water/glycol mix (50/50) for automotive
   - Corrosion inhibitors essential
   - Operating range: -40°C to +120°C

**When to Use Each:**

| Cooling Method | Power Range | Pros | Cons |
|----------------|-------------|------|------|
| Natural convection | <100W | Silent, reliable | Large size |
| Forced air | 100W - 2kW | Simple, low cost | Dust, noise |
| Liquid cooling | >2kW | Compact, excellent performance | Complex, cost |
| Hybrid (liquid + air) | >5kW | Best performance | Most complex |

### 7.5 Thermal Simulation and Testing

**Thermal Simulation Tools:**

1. **ANSYS Icepak**:
   - CFD for electronics cooling
   - Accurate but expensive and complex

2. **SolidWorks Flow Simulation**:
   - Integrated with CAD
   - Good for mechanical engineers

3. **MATLAB Simscape**:
   - System-level thermal modeling
   - Fast, good for early design

4. **FloTHERM** (Mentor Graphics):
   - Industry standard for electronics
   - Component libraries

**Thermal Testing:**

**1. Thermocouple Measurements:**

```
Placement:
  - MOSFET case (under each device)
  - Heatsink surface (multiple points)
  - Air inlet and outlet
  - Ambient reference

Type K thermocouples: -40 to +200°C, ±2°C accuracy
```

**2. Thermal Camera (Infrared):**

```
Advantages:
  - See hotspots instantly
  - Non-contact measurement
  - Full board thermal map

Limitations:
  - Surface temperature only (not junction)
  - Emissivity calibration needed
  - Expensive ($2k-$20k)
```

**3. Junction Temperature Estimation:**

```
Method 1: TSEP (Temperature Sensitive Electrical Parameter)

Measure VDS_on at known current:
  VDS_on = ID × RDS_on(Tj)

Since RDS_on(Tj) has known temp coefficient:
  Tj = (RDS_on_measured / RDS_on_25C - 1) / TC + 25°C

Where TC = temperature coefficient (e.g., 0.5%/°C = 0.005/°C)

Method 2: Built-in temp diode (if available)

Some MOSFETs/modules have integrated temp sensors
```

**4. Thermal Cycling Test:**

```
Purpose: Accelerated life testing

Procedure:
  1. Cycle between Tj_min and Tj_max (e.g., 25°C to 150°C)
  2. Typical cycle: 30 min hot, 30 min cold
  3. Run 1000-10000 cycles
  4. Inspect solder joints, TIM, package integrity

Failure modes:
  - Solder joint cracking
  - Wire bond lifting
  - Package delamination
  - TIM degradation
```

**Thermal Management Checklist:**

✓ Calculate all power losses accurately
✓ Determine required heatsink thermal resistance
✓ Select appropriate cooling method (air vs liquid)
✓ Choose TIM and apply correctly
✓ Verify with thermal testing (thermocouples, IR camera)
✓ Monitor temperatures during operation
✓ Implement thermal shutdown protection
✓ Consider worst-case ambient conditions
✓ Design for target junction temperature (125°C max for long life)
✓ Test thermal cycling for reliability

---

### References for Section 7:

**Books:**
1. *"Thermal Design of Electronic Equipment"* by Ralph Remsburg
2. *"Cooling Techniques for Electronic Equipment"* by Dave S. Steinberg
3. *"Heat Sinks: Geometry, Materials, Assembly and Cost"* by Sergey Anatolyevitch Kuzmin

**Application Notes:**
1. **Infineon**: "Thermal Management in Motor Drives" (Application Note AN2016-08)
2. **Texas Instruments**: "Thermal Design Guidelines" (SLVA462)
3. **ON Semiconductor**: "Heatsink Characteristics and Selection" (AND8181/D)
4. **Infineon**: "Thermal Resistance Theory and Practice" (Application Note AN2015-10)
5. **Aavid Thermalloy**: "Heat Sink Selection Guide"

**Articles and Papers:**
1. "Thermal Management of Power Electronics for Electric Vehicles" - IEEE VPPC Conference
2. "Advanced Cooling Technologies for Automotive Power Electronics" - SAE International
3. "Junction Temperature Measurement Methods in Power Semiconductors" - PCIM Europe

**Videos:**
1. **EEVblog**: "Heatsink Selection and Thermal Management" (YouTube)
2. **Texas Instruments**: "Thermal Design for Power Modules" (YouTube)
3. **Infineon**: "How to Measure Junction Temperature" (YouTube)

**Standards:**
1. **JESD51**: JEDEC Thermal Measurement Standards
2. **IPC-2221**: PCB Thermal Management Guidelines

**Suppliers/Tools:**
1. **Heatsink manufacturers**: Aavid Thermalloy, Fischer Elektronik, Wakefield-Vette
2. **TIM suppliers**: Bergquist, Laird Technologies, Henkel
3. **Thermal cameras**: FLIR, Seek Thermal, Fluke

---

## 8. PCB Layout and Design

Proper PCB layout is critical for motor controller performance, reliability, and EMI compliance. Poor layout can cause excessive ringing, voltage overshoot, EMI, and even device failure.

### 8.1 High-Current Trace Width Calculations

**Trace Width for Current Carrying Capacity:**

The IPC-2221 standard provides guidelines for trace width based on allowable temperature rise.

```
Formula (IPC-2221):

A = (I / (k × ΔT^0.44))^(1/0.725)

Where:
  A = Cross-sectional area (mils²)
  I = Current (A)
  k = 0.048 for external traces, 0.024 for internal traces
  ΔT = Temperature rise above ambient (°C)

Width = A / (copper_thickness_oz × 1.378)

Note: 1 oz copper = 1.378 mils thick = 35 μm
```

**Practical Calculations:**

```
Example 1: 100A phase current, external trace, 1 oz copper, 20°C rise

A = (100 / (0.048 × 20^0.44))^(1/0.725)
A = (100 / (0.048 × 3.31))^1.38
A = (100 / 0.159)^1.38
A = 629^1.38
A = 7930 mils²

Width = 7930 / (1 × 1.378) = 5755 mils = 146 mm!

Clearly too wide for practical PCB!
```

**Solutions for High Current:**

1. **Multiple Layers in Parallel**:
   ```
   100A trace, 4 layers of 2 oz copper:

   Current per layer: 100A / 4 = 25A

   For 25A, 2 oz copper, 20°C rise:
   A = (25 / (0.048 × 20^0.44))^1.38 = 495 mils²
   Width = 495 / (2 × 1.378) = 180 mils = 4.5 mm per layer

   Much more reasonable!
   ```

2. **Heavy Copper PCB**:
   - 2 oz (70 μm): 2× current capacity vs 1 oz
   - 3 oz (105 μm): 3× current capacity
   - 4 oz (140 μm): 4× current capacity
   - 6 oz (210 μm): 6× current capacity (expensive, special order)

3. **Copper Bus Bars**:
   - For very high current (>200A per phase)
   - Laminated copper sheets bolted to PCB
   - Lowest resistance and inductance
   - Used in >50 kW automotive inverters

**Practical Design Guidelines:**

| Current | 1 oz Cu | 2 oz Cu | 4 oz Cu | Typical Use |
|---------|---------|---------|---------|-------------|
| 10A | 100 mil (2.5mm) | 50 mil (1.3mm) | 25 mil (0.6mm) | Gate drive power |
| 25A | 250 mil (6.4mm) | 125 mil (3.2mm) | 60 mil (1.5mm) | Low-side shunt |
| 50A | 500 mil (12.7mm) | 250 mil (6.4mm) | 125 mil (3.2mm) | Phase output |
| 100A | 1000 mil (25mm) | 500 mil (12.7mm) | 250 mil (6.4mm) | High power phase |
| 200A | Bus bar | 1000 mil (25mm) | 500 mil (12.7mm) | Very high power |

**Via Current Capacity:**

```
Standard via: 0.3mm (12 mil) drill, plated

Current capacity: ~1-2A per via

For 100A trace transition between layers:
  Need 50-100 vias! (via array)

Via array design:
  - Space vias 0.5-1.0mm apart
  - Cover trace width
  - Stitching vias every 5-10mm along trace
```

**Voltage Drop Considerations:**

```
Resistance per unit length:

R = ρ × L / A

Where:
  ρ = Resistivity of copper = 1.72×10⁻⁸ Ω·m at 20°C
  L = Length (m)
  A = Cross-sectional area (m²)

Example:
  100A trace, 50mm long, 4mm wide, 2 oz (70μm) copper

  A = 4mm × 0.07mm = 0.28 mm² = 2.8×10⁻⁷ m²
  L = 50mm = 0.05m

  R = 1.72×10⁻⁸ × 0.05 / 2.8×10⁻⁷ = 3.07 mΩ

  Voltage drop: V = I × R = 100 × 0.00307 = 0.307V
  Power loss: P = I² × R = 100² × 0.00307 = 30.7W

  → Significant loss in just 50mm of trace!
```

### 8.2 Low-Inductance Layout Techniques

**Why Low Inductance Matters:**

```
Voltage spike during switching:

ΔV = L × (di/dt)

Example:
  L = 50 nH (poor layout)
  di/dt = 3000 A/μs (fast switching)

  ΔV = 50n × 3000 A/μs = 150V spike!

  With 400V DC bus → 550V total stress on MOSFET
```

**Loop Inductance Sources:**

```
Total loop inductance:
  L_loop = L_trace + L_capacitor + L_MOSFET_package + L_via

Typical values:
  PCB trace (50mm, 0.2mm spacing): 20-40 nH
  Electrolytic capacitor (leaded): 15-30 nH
  Film capacitor (SMD): 5-10 nH
  Ceramic capacitor (0805): 1-2 nH
  MOSFET TO-247 package: 5-10 nH
  Via (0.3mm): 0.5-1 nH each
```

**Target Inductance:**

- Excellent: <20 nH total loop
- Good: 20-50 nH
- Acceptable: 50-100 nH
- Poor: >100 nH (expect significant overshoot)

**Technique 1: Minimize Loop Area**

```
Power loop: DC+ → MOSFET → DC-

Bad layout (large loop):
        DC+  ═════════════  (top layer)
         │
         │ (long distance)
         ↓
       MOSFET
         │
         │ (long distance)
         ↓
        DC-  ═════════════  (bottom layer)

Loop area: 50mm × 20mm = 1000 mm²
L ≈ 50-100 nH

Good layout (small loop):
        DC+  ═════  (top layer, directly above DC-)
         │
        MOSFET (between planes)
         │
        DC-  ═════  (bottom layer, directly below DC+)

Loop area: 10mm × 0.5mm = 5 mm²
L ≈ 10-20 nH
```

**Technique 2: Use Power Planes**

```
4-layer stackup:
  Layer 1 (Top): Signal, components
  Layer 2: DC+ power plane (solid copper)
  Layer 3: DC- power plane (solid copper)
  Layer 4 (Bottom): Signal, components

Advantages:
  - Minimum separation between DC+ and DC- (plane spacing)
  - Maximum capacitance (planes act as distributed capacitor)
  - Lowest inductance (<10 nH achievable)
  - Excellent for high power (>5kW)

DC link capacitors connect directly between layer 2 and layer 3
```

**Technique 3: Wide, Short Traces**

```
Trace inductance (approximate):

L = 0.2 × length × [ln(2 × length / (width + thickness)) + 0.5]

(L in nH, dimensions in mm)

Example:
  50mm long trace, 1mm wide, 0.035mm thick (1 oz)
  L = 0.2 × 50 × [ln(2 × 50 / 1.035) + 0.5]
  L = 10 × [ln(96.6) + 0.5]
  L = 10 × [4.57 + 0.5]
  L = 50.7 nH

  Same trace, 10mm wide:
  L = 0.2 × 50 × [ln(2 × 50 / 10.035) + 0.5]
  L = 10 × [ln(9.97) + 0.5]
  L = 10 × [2.30 + 0.5]
  L = 28 nH

  Width increased 10×, inductance reduced ~45%
```

**Technique 4: Parallel Return Path**

```
Use adjacent ground/power plane as return path:

Signal trace on top layer
↓
Ground plane immediately below (layer 2)

Return current flows in ground plane directly under signal trace
Minimizes loop area → minimizes inductance
```

**Technique 5: DC Link Capacitor Placement**

```
Critical: Place DC link capacitors as close to MOSFETs as possible

Best: <10mm from MOSFET drain/source pins
Good: 10-20mm
Poor: >20mm

Multiple smaller capacitors better than one large capacitor:
  - Place several ceramic caps (0.1-1μF) directly at each MOSFET
  - Film caps (1-10μF) nearby for medium frequency
  - Bulk electrolytic (100-1000μF) for low frequency ripple

Example:
  Each MOSFET leg (high-side + low-side):
    - 2× 1μF ceramic (X7R, 0805) within 5mm
    - 1× 10μF film cap within 15mm
    - Shared bulk caps (470μF) within 50mm
```

**Technique 6: Multi-Point Grounding**

```
For high current grounds:

Bad: Single ground return point (creates ground loops)
Good: Star ground from each phase
Best: Ground plane (distributed ground, lowest impedance)

Via stitching:
  - Place many vias connecting top ground to bottom ground
  - Spacing: 5-10mm grid
  - Purpose: Minimize ground plane impedance
```

### 8.3 Power Loop Minimization

**Identifying the Critical Power Loop:**

```
High-frequency switching loop (most critical):

DC+ cap → High-side MOSFET drain → MOSFET source →
Low-side MOSFET drain → Low-side source → DC- cap → back to DC+ cap

This loop switches at PWM frequency with high di/dt
Minimize THIS loop first!
```

**Power Loop Layout Strategy:**

```
Step 1: Place components to minimize loop

Optimal placement:
        DC+ Cap
           │
           │ (short)
           ↓
        Q1 (High-side)
           │
           ├─→ Phase output (to motor)
           │
        Q2 (Low-side)
           │
           │ (short)
           ↓
        DC- Cap

Keep loop height <20mm for good performance
```

**Step 2: Route Power Traces**

```
Priority order:
1. DC+ to high-side drain (widest, shortest)
2. High-side source to low-side drain (phase node, widest)
3. Low-side source to DC- (widest, shortest)
4. Decoupling caps across DC+ and DC-

All traces should be on same side of board when possible
Use multiple layers in parallel for very high current
```

**Step 3: Gate Drive Routing (Secondary Priority)**

```
Gate drive loop is less critical (lower current, slower edges)

But still important:
- Keep gate traces <50mm
- Route away from drain nodes (avoid coupling)
- Use dedicated gate driver ground return (Kelvin connection)
```

**Layout Checklist for Power Loop:**

✓ DC link capacitors within 10mm of MOSFETs
✓ Power loop area <100 mm²
✓ Estimated loop inductance <50 nH
✓ High-current vias properly arrayed (50+ vias for 100A)
✓ Thermal vias under MOSFETs (50-100 vias per device)
✓ No acute angles in power traces (45° minimum)
✓ Power traces not crossing signal traces
✓ Symmetrical layout for all three phases

### 8.4 Gate Drive Layout Best Practices

**Gate Drive Loop:**

```
The gate drive circuit creates its own loop:

Gate driver output → Rg → MOSFET gate → MOSFET source →
Driver ground return → back to driver output

Keep this loop small (<25mm length ideal)
```

**Critical Layout Rules:**

1. **Separate Power and Signal Grounds**:
   ```
   Bad: Common ground for gate driver and power MOSFET

   Power Ground ════════════ (high di/dt, noisy)
        │
        └── Gate Driver GND (couples noise into driver!)

   Good: Kelvin (star) ground connection

   Power Ground ═══════════ (MOSFET source, high current)
        │
        │ (single point connection)
        │
   Gate Driver GND ───────── (separate trace back to driver)
   ```

2. **Gate Trace Routing**:
   ```
   - Keep gate traces short (<50mm)
   - Route on inner layers when possible (shielded by ground planes)
   - Never cross drain traces (high dv/dt couples to gate)
   - Use ground guard traces if crossing necessary
   - Maintain constant trace width (avoid impedance changes)
   ```

3. **Gate Resistor Placement**:
   ```
   Place Rg as close to MOSFET gate as possible (<5mm)

   Driver ──→ (long trace OK) ──→ Rg ──→ (short!) ──→ Gate

   NOT:
   Driver ──→ Rg ──→ (long trace BAD) ──→ Gate

   Reason: Rg dampens ringing; must be close to gate
   ```

4. **Miller Clamp Circuit** (for SiC or high-power):
   ```
   Gate ──┬── to MOSFET gate
          │
         Rmc (5-10kΩ)
          │
        Source

   Purpose: Prevent false turn-on from dv/dt
   Place Rmc within 10mm of gate pin
   ```

5. **Bypass Capacitors for Gate Drivers**:
   ```
   Each gate driver IC needs:
   - 100nF ceramic cap within 5mm of VCC pin
   - 10μF ceramic/tantalum within 20mm
   - Connected to driver ground, NOT power ground

   Purpose: Provide peak gate current during switching
   ```

**High-Side Driver Bootstrap Layout:**

```
For bootstrap drivers (e.g., IR2110):

Bootstrap components placement:
  1. Bootstrap diode close to VCC pin (driver supply)
  2. Bootstrap capacitor between VS and VB pins (<10mm)
  3. Use ceramic cap for bootstrap (low ESL)
  4. Route bootstrap supply trace to avoid coupling

Example:
     +15V ──→ Dboot ──→ Cboot ──→ VB (floating supply)
                 │         │
                 └─────────┴──→ VS (source of high-side MOSFET)
```

**Isolated Driver Layout:**

```
For isolated drivers (magnetic or capacitive):

- Keep isolation barrier clear (no traces crossing)
- Separate grounds completely (power ground ≠ control ground)
- Bypass caps on BOTH sides of isolation
- Follow manufacturer creepage/clearance requirements
```

### 8.5 Grounding and Layer Stack-up

**Grounding Strategy:**

```
Three separate ground systems (critical for noise immunity):

1. Power Ground (PGND):
   - High-current MOSFET sources
   - DC link capacitor returns
   - Shunt resistor grounds
   - Heavy copper, low impedance

2. Analog Ground (AGND):
   - Current sense amplifiers
   - Voltage sense circuits
   - ADC reference grounds
   - Quiet, low-noise ground

3. Digital Ground (DGND):
   - Microcontroller/DSP ground
   - Gate driver control signals
   - Communication interfaces
   - Can tolerate some noise

Connection strategy: Star ground at single point (usually near MCU)
```

**Star Ground Connection:**

```
                    Single Point Ground
                           │
         ┌─────────────────┼─────────────────┐
         │                 │                 │
       PGND              AGND              DGND
    (Power GND)      (Analog GND)      (Digital GND)
         │                 │                 │
    MOSFETs,           Current           MCU, Gate
    Shunts, Caps       Sensors            Drivers
```

**Layer Stack-up Options:**

**2-Layer PCB** (Low cost, low power <5kW):
```
Layer 1 (Top): Components, signals, some power
Layer 2 (Bottom): Ground plane, some power

Pros: Lowest cost
Cons: Limited current capacity, higher EMI, poor thermal

Use for: Low-power applications (<5kW), prototypes, cost-sensitive
```

**4-Layer PCB** (Recommended for most applications):
```
Layer 1 (Top): Components, signals
Layer 2: Ground plane (PGND + AGND + DGND)
Layer 3: Power plane (DC+, DC-, phase outputs)
Layer 4 (Bottom): Signals, some components

Core thickness (L2-L3): 0.2-0.4mm (minimum for low inductance)

Pros: Good EMI, low inductance, good thermal
Cons: Moderate cost
Use for: 5-50kW motor drives, production designs
```

**6-Layer PCB** (High performance, >20kW):
```
Layer 1 (Top): Components, signals
Layer 2: Ground plane
Layer 3: DC+ power plane
Layer 4: DC- power plane
Layer 5: Ground plane
Layer 6 (Bottom): Components, signals

Pros: Lowest inductance, best EMI, excellent thermal
Cons: Highest cost
Use for: High-power (>20kW), automotive, aerospace
```

**Copper Weight Selection:**

| Layer | Function | Copper Weight | Notes |
|-------|----------|---------------|-------|
| L1 (Top) | Power traces | 2-4 oz | Heavy copper for high current |
| L2 | Ground plane | 2 oz | Good compromise |
| L3 | Power plane | 2 oz | Or 4 oz for very high power |
| L4 (Bottom) | Signals | 1-2 oz | Standard weight sufficient |

### 8.6 Component Placement Guidelines

**General Placement Strategy:**

```
1. Power Stage (MOSFETs, DC caps):
   - Center of board or one end
   - Access to heatsink mounting area
   - Keep three phases symmetrical

2. Gate Drivers:
   - Adjacent to MOSFETs
   - <20mm from MOSFET gates

3. Current Sensors:
   - Near MOSFETs (low-side shunts)
   - Or near phase outputs (inline shunts)

4. Control Section (MCU):
   - Separate area from power stage
   - 50+ mm separation if possible
   - Near connectors for programming/debug

5. Connectors:
   - Power input: One end of board
   - Motor output: Adjacent to power stage
   - Control signals: Near MCU
```

**Critical Placement Rules:**

1. **MOSFET Orientation**:
   ```
   Orient all MOSFETs same direction for:
   - Symmetrical thermal performance
   - Easier heatsink mounting
   - Consistent gate drive routing

   Typical: Drain at top, source at bottom
   ```

2. **DC Link Capacitors**:
   ```
   Placement priority:
   1st: Ceramic caps (0.1-1μF) - within 5mm of each MOSFET
   2nd: Film caps (1-10μF) - within 20mm of power stage
   3rd: Bulk electrolytics (100-1000μF) - within 50mm

   Arrange in arrays for multiple devices
   ```

3. **Current Sense Amplifiers**:
   ```
   Place amplifiers close to shunt resistors:
   - <10mm for best noise immunity
   - Route sense traces as differential pair
   - Ground plane underneath for shielding
   - Keep away from switching nodes
   ```

4. **Heat Management**:
   ```
   - Space high-power components 10+ mm apart
   - Provide thermal relief to heatsink
   - Keep temperature-sensitive components (MCU, sensors) away
   - Consider airflow direction in placement
   ```

5. **Test Points**:
   ```
   Add test points for critical signals:
   - Phase voltages (one per phase)
   - DC bus voltage
   - Gate drive signals
   - Current sense outputs
   - Ground references

   Place accessible but away from high-voltage
   ```

**Clearance Requirements (High Voltage):**

For 400V DC system:
- Conductor-to-conductor (same potential): 0.5mm minimum
- High voltage to low voltage: 3mm minimum (IPC-2221)
- High voltage to board edge: 5mm minimum
- Creepage distance: 5mm minimum (varies by standard)
- Clearance through air: 3mm minimum

---

### References for Section 8:

**Standards:**
1. **IPC-2221**: Generic Standard on Printed Board Design
2. **IPC-2152**: Standard for Determining Current Carrying Capacity in Printed Board Design
3. **IEC 61800-5-1**: Safety requirements for adjustable speed electrical power drive systems

**Application Notes:**
1. **Texas Instruments**: "PCB Layout Guidelines for Motor Drives" (SLVA959)
2. **Infineon**: "PCB Design for Motor Control Applications" (AN2016-10)
3. **ON Semiconductor**: "High Current PCB Design" (AND9156/D)
4. **STMicroelectronics**: "Motor Control PCB Layout Techniques" (AN4660)
5. **Analog Devices**: "Grounding in Mixed-Signal Systems" (MT-031)

**Books:**
1. *"High-Speed Digital Design: A Handbook of Black Magic"* by Howard Johnson - Chapters on PCB layout
2. *"PCB Design for Real-World EMI Control"* by Bruce Archambeault
3. *"Printed Circuit Board Design Techniques for EMC Compliance"* by Mark Montrose

**Videos:**
1. **Altium Academy**: "PCB Layout for Power Electronics" (YouTube series)
2. **Texas Instruments**: "Power Stage PCB Layout Best Practices" (YouTube)
3. **Rick Hartley**: "Fundamentals of PCB Layout" (YouTube - highly recommended!)

**Design Tools:**
1. **Altium Designer**: Professional PCB design software
2. **KiCad**: Open-source PCB design (free, very capable)
3. **EAGLE**: Popular for hobbyist/small projects
4. **Saturn PCB Design Toolkit**: Free trace width/impedance calculator

---

## 9. EMI/EMC Considerations

Electromagnetic Interference (EMI) and Electromagnetic Compatibility (EMC) are critical for motor controllers to meet regulatory standards and avoid interference with other systems.

### 9.1 EMI Sources in Motor Drives

**Primary EMI Sources:**

1. **Switching Transients** (Main source):
   ```
   High dv/dt and di/dt during MOSFET switching creates:
   - Conducted EMI (150 kHz - 30 MHz)
   - Radiated EMI (30 MHz - 1 GHz)

   Example:
     dv/dt = 10 V/ns (SiC MOSFET)
     Creates harmonics up to 100+ MHz
   ```

2. **PWM Fundamental and Harmonics**:
   ```
   PWM frequency: 10-20 kHz
   Harmonics extend to:
   - 3rd harmonic: 30-60 kHz
   - 5th harmonic: 50-100 kHz
   - 7th harmonic: 70-140 kHz
   - ... up to 50th harmonic or more
   ```

3. **Common-Mode Voltage**:
   ```
   Motor phase voltage relative to ground switches rapidly
   Creates common-mode current through parasitic capacitance:

   I_cm = C_parasitic × dv/dt

   Motor winding-to-ground capacitance: 100-1000 pF typical
   ```

4. **Motor Cables**:
   ```
   Long unshielded motor cables act as antennas
   Radiate EMI efficiently at high frequencies
   Cable length matters: Longer cable = more radiation
   ```

### 9.2 Conducted and Radiated Emissions

**Conducted Emissions:**

Noise propagating through wires/cables (150 kHz - 30 MHz per CISPR 25)

```
Two types:

1. Differential Mode (DM):
   - Current flows in loop (DC+ to DC-, or phase A to phase B)
   - Caused by switching current ripple
   - Lower frequency content

2. Common Mode (CM):
   - Current flows in same direction on all conductors
   - Returns through ground/chassis
   - Higher frequency content
   - More difficult to filter
```

**Measurement:**

```
LISN (Line Impedance Stabilization Network):
  - Standardized measurement setup
  - Provides defined impedance (50Ω)
  - Separates DM and CM noise
  - Frequency range: 150 kHz - 108 MHz
```

**Radiated Emissions:**

EM fields radiating into free space (30 MHz - 1 GHz per CISPR 25)

```
Main radiators:
  - Motor cables (antenna)
  - PCB traces (unintentional antenna)
  - Enclosure openings/seams

Measurement:
  - 1m or 3m distance (automotive: 1m)
  - Anechoic chamber or open-area test site (OATS)
  - Frequency: 30 MHz - 1 GHz (sometimes up to 6 GHz)
```

### 9.3 EMI Filtering and Suppression

**Input EMI Filter (DC Bus):**

```
Typical 3-stage filter:

Battery/DC ──→ [DM Filter] ──→ [CM Filter] ──→ [Bulk Cap] ──→ Inverter
             Source

Stage 1: Differential Mode Filter
  - LC filter: L (1-10 μH), C (1-10 μF film)
  - Attenuates DM noise (switching ripple)
  - Cutoff frequency: 10-50 kHz

Stage 2: Common Mode Choke
  - Two windings on common core
  - High impedance to CM noise
  - Low impedance to DM current (motor power)
  - Inductance: 10-100 μH per winding

Stage 3: Y-Capacitors (CM Filter)
  - Small caps from DC+/DC- to chassis ground
  - Typical: 1-10 nF (limited by safety standards)
  - Safety rated (Y1 or Y2 class)

Example design:
  L_dm = 4.7 μH
  C_dm = 4.7 μF (film, X2 class)
  CM choke = 47 μH
  C_Y = 2.2 nF per rail (Y2 class)
```

**Motor Cable Filtering:**

```
Options:

1. Common-Mode Choke at Motor:
   - Three phase windings on common core
   - Impedes common-mode current
   - Typical: 10-50 μH
   - Most effective solution

2. Output dv/dt Filter:
   - Small LC filter at inverter output
   - Slows down phase voltage rise time
   - Reduces bearing currents
   - L: 5-20 μH, C: 100-470 nF

3. Ferrite Beads on Cables:
   - Snap-on ferrite cores
   - Good for 10-100 MHz
   - Easy retrofit solution

4. Shielded Motor Cables:
   - 360° shield termination at both ends
   - Shield to chassis ground
   - Reduces radiated emissions
   - More expensive
```

**PCB-Level EMI Reduction:**

```
1. Spread Spectrum PWM:
   - Modulate PWM frequency ±5-10%
   - Spreads energy across frequency band
   - Reduces peak spectral components by 10-20 dB
   - Implemented in firmware/MCU

2. Soft Switching (if possible):
   - ZVS (Zero Voltage Switching)
   - ZCS (Zero Current Switching)
   - Reduces dv/dt and di/dt
   - Requires special topology or control

3. Multi-Level Inverter:
   - 3-level or 5-level instead of 2-level
   - Lower dv/dt per step
   - Reduces EMI and bearing currents
   - More complex and expensive
```

### 9.4 Shielding and Cable Design

**Enclosure Shielding:**

```
Metal enclosure requirements:
  - Aluminum or steel housing
  - All seams welded, overlapped, or gasketed
  - Openings <λ/20 at highest frequency of concern
  - Example: 1 GHz → λ = 30cm → max opening = 15mm

Ventilation openings:
  - Use honeycomb vents (not slits)
  - Metal-to-metal contact maintained
  - EMI gaskets at removable panels
```

**Cable Shielding Best Practices:**

```
1. Motor Phase Cables:
   - Use shielded twisted triples
   - 360° shield termination (not pigtail!)
   - Terminate shield at both inverter and motor
   - Ground to chassis at both ends

2. DC Bus Cables:
   - Twisted pair (DC+ and DC-)
   - Shielded if >1m length
   - Large conductors for high current
   - Keep short and close to metal chassis

3. Signal/Sensor Cables:
   - Twisted pair for each signal
   - Individual or overall shield
   - Ground shield at one end only (avoid ground loops)
   - Keep away from motor cables (>50mm separation)
```

**Shield Termination Methods:**

```
Bad: Pigtail termination (creates loop, ineffective)
      Shield ─┐
              │ (pigtail, 50mm)
              └─→ Ground

Good: 360° termination (low impedance, effective)
      Shield ═══╗
               ║ (connector with 360° shield clamp)
      Chassis ══╝

Use: Cable glands, backshells, EMI connector housings
```

### 9.5 Standards and Compliance Testing

**Automotive Standards:**

```
CISPR 25 (Component EMI):
  - Conducted emissions: 150 kHz - 108 MHz
  - Radiated emissions: 150 kHz - 2.5 GHz
  - Limits: Class 3-5 (Class 5 most stringent)
  - Test setup: 1m antenna distance

ISO 11452 (Immunity):
  - RF immunity testing
  - Frequency: 10 kHz - 18 GHz
  - Field strength: 100-200 V/m typical

SAE J1113 (Automotive EMC):
  - US standard (similar to ISO 11452)
  - Conducted and radiated immunity
```

**Industrial Standards:**

```
IEC 61800-3 (Adjustable Speed Drives):
  - Category C1: Residential (strict limits)
  - Category C2: Commercial/light industrial
  - Category C3: Industrial (relaxed limits)
  - Category C4: Industrial with restrictions

EN 55011 (Industrial Equipment):
  - Group 1: No intentional RF generation
  - Group 2: Intentional RF (e.g., induction heating)
  - Class A: Industrial environments
  - Class B: Residential (most stringent)
```

**FCC Part 15 (USA):**

```
Class A: Industrial/commercial
  - Conducted: 0.15-30 MHz
  - Radiated: 30 MHz - 1 GHz
  - Limits ~10 dB relaxed vs Class B

Class B: Residential
  - Stricter limits
  - Required for consumer products
```

**Typical Compliance Process:**

```
1. Pre-compliance Testing (in-house):
   - Near-field probes
   - Spectrum analyzer
   - Find problem areas early
   - Cost: Equipment $5k-$50k

2. Full Compliance Testing (accredited lab):
   - Conducted emissions (LISN setup)
   - Radiated emissions (chamber or OATS)
   - Immunity testing (if required)
   - Cost: $10k-$50k per test cycle

3. Iterate as Needed:
   - Fix failures (filters, shielding, layout)
   - Re-test
   - Budget 2-3 test cycles typically

4. Certification:
   - Submit test reports
   - Receive certificate
   - Maintain for product lifetime
```

**EMI Debug Tools:**

```
Essential tools:
  - Spectrum analyzer (9 kHz - 3 GHz)
  - Near-field probe set (E-field and H-field)
  - Current probe (measure cable currents)
  - LISN (for conducted emissions pre-testing)

Cost range:
  - Basic setup: $5k-$15k
  - Professional setup: $30k-$100k
  - Lab testing: $10k-$50k per submission
```

---

### References for Section 9:

**Standards Documents:**
1. **CISPR 25**: Vehicles, boats, and internal combustion engines - Radio disturbance characteristics
2. **IEC 61800-3**: Adjustable speed electrical power drive systems - EMC requirements
3. **ISO 11452**: Road vehicles - Component test methods for electrical disturbances
4. **FCC Part 15**: Radio Frequency Devices

**Books:**
1. *"Electromagnetic Compatibility Engineering"* by Henry Ott - Comprehensive EMC reference
2. *"EMI Filter Design"* by Richard Lee Ozenbaugh
3. *"Automotive EMC"* by Eur Ing Chatterton and Williams

**Application Notes:**
1. **Texas Instruments**: "EMI Considerations for Motor Drives" (SLVA838)
2. **Infineon**: "EMC Design for Motor Control Applications" (AN2017-12)
3. **Wurth Elektronik**: "EMI Filter Design Guide"
4. **Murata**: "Common Mode Choke Coils for Automotive Applications"

**Articles:**
1. "Motor Drive EMI Reduction Techniques" - IEEE Power Electronics Society
2. "Shielding and Grounding for Motor Controllers" - EDN Magazine
3. "Understanding Common-Mode Chokes" - Power Electronics Magazine

**Videos:**
1. **Keysight Technologies**: "EMI/EMC Testing Basics" (YouTube series)
2. **Clemson University Vehicular Electronics Lab**: "Automotive EMC" (YouTube)
3. **Lee Ritchey**: "EMI and Signal Integrity" (YouTube)

**Test Labs (Examples):**
1. **Automotive**: TÜV, UL, Intertek, SGS
2. **Industrial**: CSA, CE Mark testing houses
3. **FCC**: Authorized test labs (search FCC database)

---

