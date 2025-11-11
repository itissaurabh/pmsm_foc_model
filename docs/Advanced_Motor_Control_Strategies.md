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
   - 2.6 Hardware Perspective: Circuit-Level Voltage and Current Behavior

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

### 7. Sensored vs Sensorless Control
   - 7.1 Introduction and Overview
   - 7.2 Sensored Control Methods
   - 7.3 Back-EMF Based Sensorless Control
   - 7.4 High Frequency Injection Methods
   - 7.5 Hybrid and Transition Strategies
   - 7.6 Comparison and Selection Guide

### 8. Back-EMF and Overvoltage Protection
   - 8.1 Overvoltage from Motor Overspeed
   - 8.2 Motor Winding Short Circuit Protection
   - 8.3 Additional High-Voltage Scenarios
   - 8.4 Integrated Protection System Design

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

### 1.2 Control Implementation

**Field Weakening Controller Structure:**

```
The field weakening controller calculates the required id current based on:
  1. Current speed (ω)
  2. Voltage limit
  3. Current limit
  4. Torque demand

Block diagram:

Speed (ω) ──┐
            │
Torque_ref ─┼──→ [FW Controller] ──→ id_ref (negative)
            │                    └──→ iq_ref (positive)
V_limit ────┤
I_limit ────┘
```

**Algorithm 1: Simple Voltage-Based Field Weakening**

```c
// Inputs
float omega;          // Electrical speed (rad/s)
float V_limit;        // Maximum voltage (V)
float I_limit;        // Maximum current (A)
float Torque_cmd;     // Torque command (Nm)

// Motor parameters
float lambda_m;       // Flux linkage (Wb)
float Ld;            // d-axis inductance (H)
float Lq;            // q-axis inductance (H)
float P;             // Pole pairs

// Calculate characteristic current
float I_ch = lambda_m / Ld;

// Calculate base speed
float omega_base = V_limit / lambda_m;

// Determine operating region and calculate id_ref, iq_ref
float id_ref, iq_ref;

if (omega <= omega_base) {
    // Region 1: MTPA (below base speed)
    id_ref = 0.0f;  // No field weakening

    // Calculate iq from torque command
    iq_ref = (2.0f * Torque_cmd) / (3.0f * P * lambda_m);

    // Limit iq to maximum current
    if (iq_ref > I_limit) {
        iq_ref = I_limit;
    }
}
else {
    // Region 2: Field Weakening (above base speed)

    // Calculate required id for voltage constraint
    id_ref = -I_ch + (V_limit / (omega * Ld));

    // Calculate available iq from current constraint
    float id_squared = id_ref * id_ref;
    float I_limit_squared = I_limit * I_limit;

    if (id_squared < I_limit_squared) {
        iq_ref = sqrtf(I_limit_squared - id_squared);
    }
    else {
        // Too much field weakening current required
        iq_ref = 0.0f;
        id_ref = -I_limit;  // Maximum field weakening
    }

    // Scale iq based on torque command
    float max_torque = (3.0f * P * lambda_m * iq_ref) / 2.0f;
    if (Torque_cmd < max_torque) {
        iq_ref = (2.0f * Torque_cmd) / (3.0f * P * lambda_m);
    }
}

// Apply slew rate limiting for smooth transitions
id_ref = slew_rate_limit(id_ref, id_ref_prev, MAX_ID_SLEW_RATE);
iq_ref = slew_rate_limit(iq_ref, iq_ref_prev, MAX_IQ_SLEW_RATE);
```

**Algorithm 2: Voltage Feedback Field Weakening**

More robust approach that monitors actual voltage and adjusts field weakening dynamically:

```c
// Voltage monitoring
float Vd_actual = /* measured d-axis voltage */;
float Vq_actual = /* measured q-axis voltage */;
float V_actual = sqrtf(Vd_actual * Vd_actual + Vq_actual * Vq_actual);

// Voltage error
float V_error = V_limit - V_actual;

// Proportional field weakening controller
float Kp_fw = 0.1f;  // Tuning parameter

// Calculate id adjustment
float id_fw_adjustment = Kp_fw * V_error;

// Start from MTPA
id_ref = 0.0f;

// If voltage is too high, add negative id
if (V_error < 0) {
    id_ref = id_fw_adjustment;  // Will be negative
}

// Limit id
float id_min = -I_limit;
float id_max = 0.0f;
id_ref = fmaxf(id_min, fminf(id_max, id_ref));

// Calculate iq from remaining current capacity
iq_ref = sqrtf(I_limit * I_limit - id_ref * id_ref);

// Scale by torque command
float torque_scaling = Torque_cmd / Max_Torque;
iq_ref = iq_ref * torque_scaling;
```

**Algorithm 3: Look-Up Table (LUT) Method**

Pre-calculated optimal id, iq pairs for efficiency:

```c
// Pre-computed LUT during motor characterization
// Indexed by speed and torque
struct FW_LUT_Entry {
    float speed;        // rad/s
    float torque;       // Nm
    float id_optimal;   // A
    float iq_optimal;   // A
    float efficiency;   // %
};

FW_LUT_Entry fw_table[SPEED_POINTS][TORQUE_POINTS];

// Runtime lookup with interpolation
float lookup_fw_currents(float speed, float torque,
                         float* id_out, float* iq_out) {
    // Find surrounding points in table
    int speed_idx_low, speed_idx_high;
    int torque_idx_low, torque_idx_high;

    // ... find indices ...

    // Bilinear interpolation
    float id_interp = bilinear_interpolate(
        fw_table, speed, torque,
        speed_idx_low, speed_idx_high,
        torque_idx_low, torque_idx_high
    );

    float iq_interp = /* similar interpolation for iq */;

    *id_out = id_interp;
    *iq_out = iq_interp;
}
```

**Transition Logic (MTPA to FW):**

Smooth transition is critical to avoid torque ripple:

```c
// Hysteresis for transition
float omega_enter_fw = omega_base * 0.95f;  // Enter FW early
float omega_exit_fw = omega_base * 0.90f;   // Exit FW with hysteresis

static enum {MTPA_MODE, FW_MODE} control_mode = MTPA_MODE;

// State machine for smooth transition
switch (control_mode) {
    case MTPA_MODE:
        if (omega > omega_enter_fw) {
            control_mode = FW_MODE;
            // Start ramping id negative
        }
        id_ref = 0.0f;
        iq_ref = /* calculate from torque */;
        break;

    case FW_MODE:
        if (omega < omega_exit_fw) {
            control_mode = MTPA_MODE;
            // Ramp id back to zero
        }
        // Calculate field weakening id, iq
        id_ref = calculate_fw_id(omega);
        iq_ref = calculate_fw_iq(id_ref, torque_cmd);
        break;
}

// Slew rate limiting for smooth transition
float id_slew_rate = 100.0f;  // A/s (tune based on application)
id_ref = apply_slew_rate(id_ref, id_ref_prev, id_slew_rate, dt);
```

**Practical Tuning Guidelines:**

```
1. Start conservatively:
   - Enter field weakening at 90-95% of calculated base speed
   - Use slow slew rates (50-100 A/s)
   - Monitor voltage margin

2. Voltage margin:
   - Don't operate at exactly V_limit
   - Leave 5-10% margin for transients
   - Adjusted V_limit = 0.9 * Vdc / √3

3. Current margin:
   - Similarly, don't use full I_limit in FW
   - Reserve 10% for transients
   - Adjusted I_limit = 0.9 * I_rated

4. Testing procedure:
   - Start with no load
   - Gradually increase speed through base speed
   - Monitor voltage, current, torque ripple
   - Tune Kp_fw for smooth transition
   - Add load and repeat

5. Common issues:
   - Oscillation at transition → reduce gains, add filtering
   - Torque dip at transition → enter FW earlier
   - Overheating in FW → reduce maximum FW speed or current
```

---

### References for Section 1.2:

**Application Notes:**
1. **Texas Instruments**: "Software Flux Weakening for PMSM" (SPRABV1)
2. **Microchip**: "Field Weakening Implementation" (AN1162)
3. **Infineon**: "FOC Firmware Field Weakening" (AP32370)

**Papers:**
1. "Robust Flux-Weakening Control of PMSM" - IEEE Transactions on Industrial Electronics
2. "Transition Control Between MTPA and Field Weakening" - IEEE ECCE Conference

---

### 1.3 Electronics Considerations for Field Weakening

Field weakening operation imposes additional stresses on the motor controller electronics that must be carefully managed.

**Increased Electrical Stress:**

```
1. Higher Current in Field Weakening:
   - Total current magnitude remains at I_limit
   - But now id (reactive) + iq (active) = I_limit
   - No additional torque production from id component
   - id² heating losses without useful work

2. Current RMS Heating:
   I_total_rms = √(id_rms² + iq_rms²)

   Example:
     Below base speed: id = 0A, iq = 150A
       I_rms = 150A

     In field weakening: id = -100A, iq = 111A
       I_rms = √(100² + 111²) = 149A (same magnitude)
       But distribution changes!

3. Increased Conduction Loss:
   P_cond = I²_rms × RDS_on

   In FW, continuous high current → more heating
```

**Inverter Thermal Management:**

```
Field weakening operation typically means:
  - Sustained high-speed, high-current operation
  - Longer duty cycles (highway driving vs city)
  - Less cooling airflow (motor spinning fast but vehicle static)

Thermal concerns:
  1. MOSFET junction temperature
  2. DC link capacitor temperature (high RMS ripple current)
  3. Current sense resistors (continuous I²R heating)
  4. PCB copper traces (sustained high current)

Mitigation:
  - Derate maximum FW speed based on thermal limits
  - Monitor temperatures and reduce power if needed
  - Design cooling for sustained FW operation, not just peak
```

**Voltage Stress and Overvoltage:**

```
When entering/exiting field weakening rapidly:

Problem: Sudden change in motor impedance can cause voltage spikes

Example scenario:
  1. Running at high speed in FW with id = -100A
  2. Driver suddenly releases accelerator
  3. Controller tries to reduce iq rapidly
  4. Motor back-EMF hasn't changed yet (inertia)
  5. Energy stored in motor inductance must go somewhere
  6. Voltage spike: ΔV = L × (di/dt)

Mitigation:
  1. Limit di/dt (slew rate) for id and iq
  2. Adequate DC link capacitance
  3. Overvoltage protection (clamping)
  4. Controlled ramp-down from FW
```

**Controller Derating in Field Weakening:**

```c
// Temperature-based FW derating
float derate_fw_for_temperature(float T_junction, float T_capacitor) {
    float max_fw_speed_factor = 1.0f;

    // Derate based on junction temperature
    if (T_junction > 125.0f) {
        // Linear derate from 125°C to 150°C
        max_fw_speed_factor = 1.0f - (T_junction - 125.0f) / 25.0f;
    }

    // Derate based on capacitor temperature
    if (T_capacitor > 85.0f) {
        float cap_derate = 1.0f - (T_capacitor - 85.0f) / 20.0f;
        max_fw_speed_factor = fminf(max_fw_speed_factor, cap_derate);
    }

    // Clamp to reasonable range
    max_fw_speed_factor = fmaxf(0.5f, fminf(1.0f, max_fw_speed_factor));

    return max_fw_speed_factor;
}

// Apply derating
float omega_max_fw_derated = omega_max_fw * derate_fw_for_temperature(T_j, T_cap);
```

**DC Link Capacitor Stress:**

```
In field weakening:
  - Higher frequency operation
  - Increased ripple current
  - More heating in capacitor ESR

Ripple current in FW:
  I_ripple_rms ≈ 0.4 × I_motor_rms (typical)

At high speeds (2× base speed):
  - Frequency doubles
  - ESR may increase (frequency-dependent)
  - Temperature rises

Capacitor life vs. temperature:
  Every 10°C increase → ~50% reduction in lifetime

Design considerations:
  - Select capacitors rated for ripple current at FW speed
  - Monitor capacitor temperature
  - Derate FW operation if capacitor temp exceeds 85°C
```

---

### References for Section 1.3:

**Application Notes:**
1. **Infineon**: "Motor Drive Power Stage Thermal Design" (AN2016-08)
2. **Texas Instruments**: "Thermal Considerations in Motor Drives" (SLVA462)
3. **TDK**: "Aluminum Electrolytic Capacitors - Ripple Current" (Technical Note)

**Papers:**
1. "Thermal Management for Field Weakening Operation in EV Drives" - IEEE VPPC Conference
2. "DC-Link Capacitor Selection for Motor Drives" - IEEE Transactions on Power Electronics

---

### 1.4 Vehicle Towing and Field Weakening Safety

**The Towing Problem:**

When a vehicle is towed (or coasting downhill with motor unpowered), the motor acts as a generator:

```
Towing scenario:
  1. Wheels turn → motor spins
  2. Motor generates back-EMF
  3. If controller is OFF, this voltage appears across inverter
  4. Can exceed MOSFET voltage rating → FAILURE

Back-EMF at towing speeds:
  E_phase = ω × λm

Example:
  Vehicle towed at 100 km/h (62 mph)
  Motor: 4 pole pairs, λm = 0.0118 Wb
  Tire diameter: 0.65m

  Vehicle speed = 100 km/h = 27.8 m/s
  Wheel RPM = (27.8 / (π × 0.65)) × 60 = 814 RPM

  Motor RPM = Wheel RPM × Gear Ratio
            = 814 × 10 = 8,140 RPM

  Electrical frequency = 8140 × 4 / 60 = 543 Hz
  ω_elec = 2π × 543 = 3,411 rad/s

  E_phase = 3411 × 0.0118 = 40.2V (phase, RMS)
  E_line_peak = 40.2 × √2 × √3 = 98.5V

This is manageable for 400V system, but...

At 200 km/h:
  E_line_peak = 197V (still okay)

But consider:
  - Worn MOSFETs have lower voltage tolerance
  - Transient spikes during bumps
  - Uncontrolled rectification through body diodes
```

**Uncontrolled Rectification:**

```
When controller is OFF but motor spinning:

  Motor ──→ Inverter (OFF) ──→ DC Bus
           Body Diodes
           conduct

Result:
  - Back-EMF charges DC bus through body diodes
  - DC bus voltage rises
  - No controlled path for energy
  - Voltage can exceed safe limits

Worst case:
  - High-speed towing
  - Small or disconnected DC bus capacitance
  - Voltage spike during sudden deceleration
```

**Solution 1: Controlled Field Weakening During Tow**

Keep controller powered and actively weaken field:

```c
// Tow mode detection
bool detect_tow_mode() {
    bool motor_spinning = (abs(omega) > MIN_TOWING_SPEED);
    bool no_torque_command = (Torque_cmd == 0);
    bool no_throttle = (Throttle_position == 0);

    return (motor_spinning && no_torque_command && no_throttle);
}

// Tow mode field weakening
void tow_mode_field_weakening() {
    // Calculate back-EMF
    float E_backemf = omega * lambda_m;

    // Calculate required id to null the flux
    // Goal: Make net flux = 0
    float id_tow = -lambda_m / Ld;  // Maximum field weakening

    // Limit to available current
    if (fabs(id_tow) > I_LIMIT_TOW) {
        id_tow = -I_LIMIT_TOW;
    }

    // Set iq = 0 (no torque)
    float iq_tow = 0.0f;

    // Apply currents
    id_ref = id_tow;
    iq_ref = iq_tow;

    // This nullifies back-EMF and prevents voltage buildup
}
```

**Solution 2: Dynamic Braking Resistor**

Hardware solution for emergency towing:

```
          VDC+
           │
           │  ┌────┐
           ├──┤ Rdb├───┐  Dynamic Brake Resistor
           │  └────┘   │
           │          │
      [Comparator]   IGBT/MOSFET (normally OFF)
           │
         Ground

Operation:
  1. Monitor DC bus voltage
  2. If Vdc > Threshold (e.g., 450V for 400V system)
  3. Turn ON brake IGBT
  4. Dissipate energy in resistor
  5. Turn OFF when Vdc < Threshold - Hysteresis

Resistor sizing:
  P_brake = (E_backemf)² / Rdb

  Example:
    E_max = 200V (line-to-line)
    P_brake_target = 2kW

    Rdb = (200)² / 2000 = 20Ω

  Use: 20Ω, 3kW resistor (with derating)
```

**Solution 3: Mechanical Disconnect**

For vehicles that may be towed frequently:

```
Options:
  1. Manual disconnect switch (separates motor from inverter)
  2. Electromagnetic clutch (decouples motor from wheels)
  3. Freewheel mechanism in gearbox

Pros:
  - Complete isolation
  - No controller power needed
  - Safe for any towing speed

Cons:
  - Added mechanical complexity
  - Cost
  - May not be failsafe
```

**Safety Guidelines:**

```
1. Always provide towing instructions to user:
   - Maximum towing speed (if no active FW)
   - Whether controller must be ON
   - Use of neutral/disconnect

2. Implement towing mode in firmware:
   - Automatic detection
   - Active field weakening
   - Log towing events

3. Hardware protection:
   - Dynamic brake resistor for emergency
   - Overvoltage clamp (TVS diodes, varistors)
   - Monitor DC bus voltage

4. Testing:
   - Simulate towing on dynamometer
   - Test at maximum expected towing speed
   - Verify voltage stays within safe limits
   - Test with controller OFF (worst case)
```

**Towing Mode State Machine:**

```c
typedef enum {
    NORMAL_OPERATION,
    TOW_MODE_DETECTED,
    TOW_MODE_ACTIVE,
    TOW_MODE_OVERVOLTAGE,
    TOW_MODE_ERROR
} TowMode_State_t;

TowMode_State_t tow_state = NORMAL_OPERATION;

void tow_mode_state_machine() {
    switch (tow_state) {
        case NORMAL_OPERATION:
            if (detect_tow_mode()) {
                tow_state = TOW_MODE_DETECTED;
                log_event(EVENT_TOW_DETECTED);
            }
            break;

        case TOW_MODE_DETECTED:
            // Give 1 second to confirm
            if (tow_mode_confirmed()) {
                tow_state = TOW_MODE_ACTIVE;
                enable_tow_field_weakening();
            }
            else if (!detect_tow_mode()) {
                tow_state = NORMAL_OPERATION;
            }
            break;

        case TOW_MODE_ACTIVE:
            // Actively weaken field
            tow_mode_field_weakening();

            // Monitor voltage
            if (Vdc > V_OVERVOLTAGE_THRESHOLD) {
                tow_state = TOW_MODE_OVERVOLTAGE;
                engage_dynamic_brake();
            }

            // Exit condition
            if (!detect_tow_mode()) {
                tow_state = NORMAL_OPERATION;
                disable_tow_field_weakening();
            }
            break;

        case TOW_MODE_OVERVOLTAGE:
            // Emergency braking active
            if (Vdc < V_SAFE_THRESHOLD) {
                tow_state = TOW_MODE_ACTIVE;
                disengage_dynamic_brake();
            }
            break;

        case TOW_MODE_ERROR:
            // Fault condition
            shutdown_inverter();
            set_fault_code(FAULT_TOW_MODE_ERROR);
            break;
    }
}
```

---

### References for Section 1.4:

**Papers:**
1. "Over-Voltage Protection in Electric Vehicle Motor Drives" - SAE Technical Paper
2. "Towing and Push-Starting Considerations for EVs" - IEEE VPPC

**Standards:**
1. **SAE J1772**: EV Conductive Charge Coupler (includes towing considerations)
2. **ISO 6469-3**: Electric road vehicles - Safety specifications

**Application Notes:**
1. **Infineon**: "Overvoltage Protection for Motor Drives" (Application Note)
2. **Littelfuse**: "TVS Diodes for Automotive Motor Protection" (AN9768)

---

### 1.5 Practical Implementation Guidelines

**Step-by-Step Implementation:**

```
Phase 1: Characterization (1-2 weeks)
  1. Measure motor parameters:
     - Rs, Ld, Lq (impedance test)
     - λm (back-EMF test)
     - Thermal limits

  2. Calculate base speed:
     ω_base = V_limit / λm

  3. Test MTPA region first (below base speed)
     - Verify id = 0 operation
     - Tune current loops
     - Establish baseline performance

Phase 2: Basic Field Weakening (1 week)
  1. Implement Algorithm 1 (voltage-based FW)
  2. Set conservative limits:
     - Enter FW at 0.9 × ω_base
     - Slow slew rates (50 A/s)
  3. Test no-load operation through base speed
  4. Verify smooth transition

Phase 3: Optimization (2-3 weeks)
  1. Tune transition thresholds
  2. Optimize slew rates
  3. Add voltage feedback (Algorithm 2)
  4. Test with load
  5. Measure efficiency vs. speed curve

Phase 4: Safety and Protection (1-2 weeks)
  1. Implement thermal monitoring
  2. Add temperature-based derating
  3. Implement towing mode
  4. Test overvoltage protection
  5. Validate all fault conditions

Phase 5: Production Validation (2-4 weeks)
  1. Environmental testing
  2. Long-duration FW testing
  3. Thermal cycling
  4. EMC testing in FW mode
  5. Safety certification
```

**Common Pitfalls and Solutions:**

```
1. Torque dip at transition:
   Symptom: Noticeable jerk when entering FW
   Cause: Sudden change in id command
   Solution: Enter FW earlier (0.9× base speed), slower ramp

2. Oscillation in FW:
   Symptom: id, iq oscillate in FW region
   Cause: PI gains too high, voltage feedback oscillation
   Solution: Reduce Kp_fw, add low-pass filter

3. Overshoot exiting FW:
   Symptom: Torque spike when decelerating through base speed
   Cause: Rapid id return to zero
   Solution: Hysteresis, controlled ramp

4. Overheating:
   Symptom: Thermal shutdown in sustained FW
   Cause: Continuous high current, inadequate cooling
   Solution: Derate max FW speed, improve cooling

5. Voltage limit oscillation:
   Symptom: Controller bounces between MTPA and FW
   Cause: Operating right at voltage limit
   Solution: Add margin (0.9 × V_limit), hysteresis
```

**Performance Metrics:**

```
Success criteria for FW implementation:

1. Speed range extension:
   Target: 1.5-2.0× base speed achievable
   Measure: Maximum sustainable speed under load

2. Efficiency:
   Target: >90% efficiency in FW region
   Measure: Input power / Output power at various FW speeds

3. Transition smoothness:
   Target: <5% torque ripple at transition
   Measure: Torque sensor or current waveform analysis

4. Thermal performance:
   Target: Sustained FW operation without overheating
   Measure: Temperature stabilization test (30+ minutes)

5. Voltage utilization:
   Target: <95% of voltage limit (5% margin)
   Measure: Peak voltage during FW operation
```

**Commissioning Checklist:**

```
☐ Motor parameters verified
☐ Base speed calculated and validated
☐ Current limits properly set
☐ Voltage limits properly set (with margin)
☐ Slew rate limits tuned
☐ Transition thresholds optimized
☐ PI gains tuned for FW stability
☐ Temperature monitoring functional
☐ Derating curves implemented
☐ Towing mode tested
☐ Overvoltage protection verified
☐ Fault handling tested
☐ Documentation complete
☐ Safety review passed
```

---

### References for Section 1.5:

**Application Notes:**
1. **Texas Instruments**: "PMSM Field Weakening Design Guide" (SPRABQ7)
2. **STMicroelectronics**: "Motor Control Application Tuning" (UM2380)
3. **Microchip**: "Sensorless Field Weakening Tuning" (AN1299)

**Books:**
1. *"Practical Variable Speed Drives and Power Electronics"* by Malcolm Barnes - Chapter on commissioning
2. *"Electric Motor Drives: Modeling, Analysis, and Control"* by R. Krishnan - Implementation examples

---

**End of Section 1: Field Weakening Control**

---

## Section 2: Regenerative Braking Control

Regenerative braking allows electric vehicles to recover kinetic energy during deceleration by operating the motor as a generator. This section covers the physics, control strategies, and practical implementation of regen braking systems.

---

### 2.1 Physics of Regenerative Braking

#### Energy Flow and Power Conversion

During regenerative braking, the motor transitions from motoring mode to generating mode. The kinetic energy of the vehicle is converted to electrical energy and returned to the battery.

**Power Flow:**

```
Kinetic Energy → Mechanical Power → Electrical Power → Battery Energy
     (Vehicle)      (Motor Shaft)     (DC Bus)         (Storage)
```

**Mathematical Foundation:**

1. **Kinetic Energy of Vehicle:**
```
E_kinetic = (1/2) * m * v²

Where:
  m = vehicle mass (kg)
  v = vehicle velocity (m/s)
```

2. **Power Available for Regeneration:**
```
P_regen = F_brake * v = (m * a) * v

Where:
  F_brake = braking force (N)
  a = deceleration (m/s²)
  v = vehicle velocity (m/s)
```

3. **Motor Torque During Regeneration:**
```
T_regen = F_brake * r_wheel / (G * η_drivetrain)

Where:
  r_wheel = wheel radius (m)
  G = gear ratio
  η_drivetrain = drivetrain efficiency (0.90-0.95)
```

4. **Electrical Power Generated:**
```
P_elec = T_regen * ω_m = (3/2) * P * (λ_m * i_q + (L_d - L_q) * i_d * i_q)

Where:
  ω_m = mechanical angular velocity (rad/s)
  P = pole pairs
  i_d, i_q = dq-axis currents (A)
```

**Back-EMF During Regeneration:**

The motor generates voltage proportional to speed:

```
E_a = ω_e * λ_m = P * ω_m * λ_m

Where:
  ω_e = electrical angular velocity (rad/s)
  λ_m = permanent magnet flux linkage (Wb)
```

For regeneration to occur, this back-EMF must be higher than the battery voltage (after accounting for inverter voltage drop):

```
E_a > V_battery + ΔV_inverter + ΔV_cable

Minimum speed for regeneration:
ω_min = (V_battery + ΔV_losses) / λ_m
```

#### Energy Efficiency Chain

Not all kinetic energy can be recovered due to losses in the conversion chain:

```
η_total = η_mechanical × η_motor × η_inverter × η_battery

Typical values:
  η_mechanical = 0.95 (bearings, gears)
  η_motor = 0.88-0.95 (copper, iron, stray losses)
  η_inverter = 0.95-0.98 (switching, conduction)
  η_battery = 0.90-0.95 (charging efficiency)

Total: η_total = 0.75-0.85 (75-85% recovery)
```

**Loss Breakdown:**

1. **Copper Losses (I²R):**
```
P_copper = (3/2) * R_s * (i_d² + i_q²)

Where:
  R_s = stator resistance (Ω)
```

2. **Iron Losses:**
```
P_iron = K_h * f_e * B² + K_e * f_e² * B²

Where:
  K_h = hysteresis loss coefficient
  K_e = eddy current loss coefficient
  f_e = electrical frequency (Hz)
  B = flux density (T)
```

3. **Switching Losses:**
```
P_switch = f_sw * (E_on + E_off) + V_ce(sat) * I_avg

Where:
  f_sw = switching frequency (Hz)
  E_on, E_off = turn-on/off energies (J)
  V_ce(sat) = MOSFET on-state voltage (V)
```

#### Regenerative Braking Regions

**Region Analysis:**

1. **High Speed (ω > ω_base):**
   - Field weakening may be active during motoring
   - Transition to regen requires careful field control
   - Maximum regen power available

2. **Medium Speed (ω_min < ω < ω_base):**
   - Optimal regen region
   - Full torque capability
   - Best efficiency

3. **Low Speed (ω < ω_min):**
   - Back-EMF < V_battery
   - Regen not possible
   - Must transition to friction brakes

**Implementation Consideration:**

```c
// Calculate regenerative braking capability
float calculate_regen_capability(float omega_mech, float V_battery) {
    // Back-EMF calculation
    float E_backemf = POLE_PAIRS * omega_mech * FLUX_LINKAGE;

    // Required margin above battery voltage
    float V_required = V_battery + V_INVERTER_DROP + V_CABLE_DROP;

    if (E_backemf < V_required) {
        // Cannot regenerate - back-EMF too low
        return 0.0f;
    }

    // Calculate available voltage margin
    float V_margin = E_backemf - V_required;

    // Regen capability (0.0 to 1.0)
    float capability = fminf(1.0f, V_margin / V_MARGIN_NOMINAL);

    return capability;
}
```

---

### References for Section 2.1:

**Books:**
1. *"Electric Powertrain: Energy Systems, Power Electronics and Drives for Hybrid, Electric and Fuel Cell Vehicles"* by John G. Hayes and G. Abas Goodarzi - Chapter 3 (Energy Management)
2. *"Modern Electric, Hybrid Electric, and Fuel Cell Vehicles"* by Mehrdad Ehsani et al. - Chapter 5 (Regenerative Braking)
3. *"Electric and Hybrid Vehicles: Design Fundamentals"* by Iqbal Husain - Chapter 8 (Energy Storage and Management)

**Papers:**
1. Gao, Y., & Ehsani, M. (2001). "Electronic Braking System of EV and HEV—Integration of Regenerative Braking, Automatic Braking Force Control and ABS." SAE Technical Paper 2001-01-2478.
2. Bender, F. A., et al. (2013). "On the Influence of Rotational Inertia on the Energy Efficiency of Electric Vehicles." IEEE Vehicle Power and Propulsion Conference.

**Application Notes:**
1. **Texas Instruments**: "Regenerative Braking in Electric Vehicles" (SLVA672)
2. **Infineon**: "Regenerative Energy Recovery in Motor Drives" (AN2020-06)

---

### 2.2 Motor Behavior During Regeneration

#### Torque-Speed Characteristics in Generator Mode

When operating as a generator, the motor's torque-speed curve mirrors the motoring curve but in the negative torque quadrant.

**Four-Quadrant Operation:**

```
    Torque
      ↑
Q2    |    Q1
(-T,+ω)|(+T,+ω)
      |
------+------→ Speed
      |
Q3    |    Q4
(-T,-ω)|(+T,-ω)
      |
```

- **Q1**: Forward motoring (acceleration)
- **Q2**: Forward regeneration (forward motion, braking)
- **Q3**: Reverse motoring (reverse acceleration)
- **Q4**: Reverse regeneration (reverse motion, braking)

**Torque Production in Regen:**

The torque equation is the same for both motoring and generating:

```
T_e = (3/2) * P * (λ_m * i_q + (L_d - L_q) * i_d * i_q)
```

The key difference is the sign of i_q:
- **Motoring**: i_q > 0 (current in phase with back-EMF)
- **Regenerating**: i_q < 0 (current opposes back-EMF)

#### Current Vector Control During Regen

**dq-Axis Current Commands:**

```c
// Regenerative braking current control
void calculate_regen_currents(float T_regen_cmd, float omega_mech,
                               float *id_ref, float *iq_ref) {
    float omega_elec = POLE_PAIRS * omega_mech;

    // For surface-mounted PMSM (Ld ≈ Lq), use MTPA strategy
    // In regen, this is still i_d = 0 for maximum efficiency
    *id_ref = 0.0f;

    // Negative i_q for regeneration
    *iq_ref = -(2.0f * T_regen_cmd) / (3.0f * POLE_PAIRS * FLUX_LINKAGE);

    // Check if field weakening is needed
    // Even in regen, at high speed, FW may be required
    float V_available = V_DC_BUS * 0.866f;  // Max modulation index
    float V_required = sqrtf(powf(omega_elec * FLUX_LINKAGE, 2) +
                             powf(omega_elec * L_Q * (*iq_ref), 2));

    if (V_required > V_available) {
        // Apply field weakening for regen at high speed
        apply_field_weakening_regen(omega_elec, V_available, id_ref, iq_ref);
    }

    // Current limiting
    float i_total = sqrtf((*id_ref) * (*id_ref) + (*iq_ref) * (*iq_ref));
    if (i_total > I_MAX) {
        float scale = I_MAX / i_total;
        *id_ref *= scale;
        *iq_ref *= scale;
    }
}
```

#### Voltage and Current Phase Relationships

**In Motoring Mode:**
```
i_q in phase with back-EMF → Power from battery to motor
```

**In Regenerating Mode:**
```
i_q opposes back-EMF → Power from motor to battery
```

**Phase Diagram:**

```
        d-axis
          ↑
          |
          |    θ_e (rotor position)
          |   /
          |  /
          | /
----------+----------→ q-axis
          |
          |
     Back-EMF (E_a)

Motoring:   I_s leads E_a by angle (φ < 90°)
Generating: I_s lags E_a by angle (φ > 90°)

Where:
  I_s = stator current vector
  φ = power factor angle
```

#### Flux Weakening During High-Speed Regen

Just like motoring, regeneration at high speeds requires field weakening to stay within voltage limits.

**Field Weakening Regen Strategy:**

```c
void apply_field_weakening_regen(float omega_elec, float V_limit,
                                  float *id_ref, float *iq_ref) {
    // Characteristic current
    float I_ch = FLUX_LINKAGE / L_D;

    // Calculate negative i_d needed to reduce flux
    *id_ref = -I_ch + (V_limit / (omega_elec * L_D));

    // Ensure we don't exceed current limit
    float id_squared = (*id_ref) * (*id_ref);
    float I_limit_squared = I_MAX * I_MAX;

    if (id_squared < I_limit_squared) {
        // Calculate maximum allowable i_q (negative for regen)
        float iq_max = sqrtf(I_limit_squared - id_squared);

        // Use negative value for regeneration
        if (*iq_ref < -iq_max) {
            *iq_ref = -iq_max;
        }
    }
    else {
        // All current used for field weakening
        *id_ref = -I_MAX;
        *iq_ref = 0.0f;
    }
}
```

#### Transition from Motoring to Regeneration

The transition from positive torque (acceleration) to negative torque (braking) must be smooth to avoid jerky vehicle behavior.

**Smooth Transition Algorithm:**

```c
#define REGEN_TRANSITION_RATE  50.0f  // Nm/s

typedef struct {
    float torque_current;     // Current torque command
    float torque_target;      // Target torque command
    float transition_rate;    // Rate of change (Nm/s)
} RegenTransition_t;

void update_regen_transition(RegenTransition_t *trans, float dt) {
    // Calculate error
    float error = trans->torque_target - trans->torque_current;

    // Rate limit the transition
    float max_change = trans->transition_rate * dt;

    if (fabsf(error) < max_change) {
        trans->torque_current = trans->torque_target;
    }
    else if (error > 0.0f) {
        trans->torque_current += max_change;
    }
    else {
        trans->torque_current -= max_change;
    }
}

// Usage in main control loop
void motor_control_loop(float T_cmd_user, float dt) {
    static RegenTransition_t transition = {
        .torque_current = 0.0f,
        .torque_target = 0.0f,
        .transition_rate = REGEN_TRANSITION_RATE
    };

    // Update target from user command
    transition.torque_target = T_cmd_user;

    // Smooth transition
    update_regen_transition(&transition, dt);

    // Use smoothed torque for current calculation
    if (transition.torque_current >= 0.0f) {
        // Motoring mode
        calculate_motoring_currents(transition.torque_current, &id_ref, &iq_ref);
    }
    else {
        // Regeneration mode
        calculate_regen_currents(-transition.torque_current, omega_mech, &id_ref, &iq_ref);
    }
}
```

#### Thermal Considerations During Regen

During regeneration, the motor and inverter still experience losses, even though power flows back to the battery.

**Loss Distribution:**

1. **Motor Losses:**
   - Copper losses: I²R (same magnitude as motoring for same current)
   - Iron losses: May be higher due to higher flux density if FW not properly managed
   - Mechanical losses: Same as motoring

2. **Inverter Losses:**
   - Conduction losses: Same as motoring
   - Switching losses: Slightly different due to reverse current direction through body diodes

**Thermal Management Code:**

```c
// Calculate total losses during regeneration
float calculate_regen_losses(float id, float iq, float omega_elec) {
    // Copper losses
    float P_copper = 1.5f * R_STATOR * (id*id + iq*iq);

    // Iron losses (simplified Steinmetz equation)
    float f_elec = omega_elec / (2.0f * M_PI);
    float B_peak = FLUX_LINKAGE / AREA_FLUX_PATH;
    float P_iron = K_HYSTERESIS * f_elec * B_peak * B_peak +
                   K_EDDY * f_elec * f_elec * B_peak * B_peak;

    // Mechanical losses
    float P_mech = K_FRICTION * omega_elec + K_WINDAGE * omega_elec * omega_elec;

    // Inverter losses (conduction + switching)
    float I_rms = sqrtf(id*id + iq*iq);
    float P_inverter = 3.0f * (RDS_ON * I_rms * I_rms +
                              F_SWITCHING * (E_ON + E_OFF));

    return P_copper + P_iron + P_mech + P_inverter;
}
```

---

### References for Section 2.2:

**Books:**
1. *"Advanced Electric Drives: Analysis, Control, and Modeling Using MATLAB/Simulink"* by Ned Mohan and Siddharth Raju - Chapter 6 (Four-Quadrant Operation)
2. *"Vector Control and Dynamics of AC Drives"* by D.W. Novotny and T.A. Lipo - Chapter 12 (Generating Mode)

**Papers:**
1. Jung, D., et al. (2012). "Regenerative Braking Control Strategy Based on Field Oriented Control in Interior Permanent Magnet Synchronous Motor Drives." International Journal of Automotive Technology, 13(4), 603-609.
2. Patel, H., & Chandorkar, M. C. (2013). "Analysis of Regenerative Braking in PMSM Drives." IEEE PEDES Conference.

**Application Notes:**
1. **Analog Devices**: "Four-Quadrant Operation of PMSM Motors" (AN-1378)
2. **NXP Semiconductors**: "Motor Control Field Oriented Control"  (AN12444)

---

### 2.3 Regen Control Strategies

This section covers different control strategies for managing regenerative braking, from simple voltage-based control to advanced torque blending algorithms.

#### Strategy 1: Fixed Regen Current Limiting

The simplest strategy limits regenerative current to a fixed value based on battery and motor capabilities.

**Implementation:**

```c
#define REGEN_CURRENT_LIMIT  100.0f  // Maximum regen current (A)

float calculate_regen_torque_limited(float T_cmd, float omega_mech) {
    // Convert torque command to i_q
    float iq_required = -(2.0f * fabsf(T_cmd)) / (3.0f * POLE_PAIRS * FLUX_LINKAGE);

    // Limit to maximum regen current
    if (iq_required > REGEN_CURRENT_LIMIT) {
        iq_required = REGEN_CURRENT_LIMIT;
    }

    // Convert back to torque
    float T_regen_actual = -(3.0f / 2.0f) * POLE_PAIRS * FLUX_LINKAGE * iq_required;

    return T_regen_actual;
}
```

**Advantages:**
- Simple to implement
- Protects motor and battery from overcurrent
- Predictable behavior

**Disadvantages:**
- Does not account for battery SOC
- May not optimize energy recovery
- Fixed limit may be too conservative

---

#### Strategy 2: Battery SOC-Dependent Regen Control

This strategy adjusts regen power based on battery State of Charge (SOC) to prevent overcharging.

**SOC-Based Scaling:**

```c
typedef struct {
    float soc_full_limit;      // SOC above which regen is limited (e.g., 0.90)
    float soc_no_regen;        // SOC above which regen is disabled (e.g., 0.98)
    float power_max_regen;     // Maximum regen power at low SOC (W)
} RegenSOCParams_t;

float calculate_regen_power_soc_limited(float SOC, RegenSOCParams_t *params) {
    if (SOC >= params->soc_no_regen) {
        // Battery is full - no regen allowed
        return 0.0f;
    }
    else if (SOC >= params->soc_full_limit) {
        // Linearly reduce regen power as SOC increases
        float scale = (params->soc_no_regen - SOC) /
                     (params->soc_no_regen - params->soc_full_limit);
        return params->power_max_regen * scale;
    }
    else {
        // Below limit - full regen power available
        return params->power_max_regen;
    }
}

// Usage in control loop
void apply_soc_limited_regen(float T_cmd, float omega_mech, float SOC) {
    static RegenSOCParams_t soc_params = {
        .soc_full_limit = 0.90f,
        .soc_no_regen = 0.98f,
        .power_max_regen = 30000.0f  // 30 kW
    };

    // Calculate available regen power based on SOC
    float P_regen_available = calculate_regen_power_soc_limited(SOC, &soc_params);

    // Calculate maximum regen torque at current speed
    float T_regen_max = P_regen_available / fabsf(omega_mech);

    // Limit commanded torque
    float T_regen_actual = fmaxf(T_cmd, -T_regen_max);

    // Calculate dq currents
    calculate_regen_currents(-T_regen_actual, omega_mech, &id_ref, &iq_ref);
}
```

---

#### Strategy 3: Voltage-Based Regen Control

This strategy monitors DC bus voltage and reduces regen if the voltage rises too high, which can happen when the battery cannot accept all the regen power.

**DC Bus Voltage Monitoring:**

```c
#define V_DC_NOMINAL       400.0f   // Nominal DC bus voltage (V)
#define V_DC_MAX_NORMAL    430.0f   // Start reducing regen (V)
#define V_DC_MAX_CRITICAL  450.0f   // Stop all regen (V)
#define V_DC_OVERVOLTAGE   470.0f   // Trigger fault (V)

typedef struct {
    float voltage_current;         // Measured DC bus voltage
    float voltage_max_normal;      // Voltage to start reducing regen
    float voltage_max_critical;    // Voltage to stop regen
    float voltage_overvoltage;     // Fault threshold
    float regen_scale;             // Output: regen scaling factor (0-1)
    bool fault_active;             // Output: overvoltage fault flag
} RegenVoltageControl_t;

void update_regen_voltage_control(RegenVoltageControl_t *ctrl) {
    if (ctrl->voltage_current >= ctrl->voltage_overvoltage) {
        // Critical overvoltage - trigger fault
        ctrl->fault_active = true;
        ctrl->regen_scale = 0.0f;
    }
    else if (ctrl->voltage_current >= ctrl->voltage_max_critical) {
        // Above critical - no regen allowed
        ctrl->fault_active = false;
        ctrl->regen_scale = 0.0f;
    }
    else if (ctrl->voltage_current >= ctrl->voltage_max_normal) {
        // Linearly reduce regen between normal and critical
        ctrl->fault_active = false;
        float range = ctrl->voltage_max_critical - ctrl->voltage_max_normal;
        float excess = ctrl->voltage_current - ctrl->voltage_max_normal;
        ctrl->regen_scale = 1.0f - (excess / range);
    }
    else {
        // Below threshold - full regen allowed
        ctrl->fault_active = false;
        ctrl->regen_scale = 1.0f;
    }
}
```

**Integration with Torque Control:**

```c
void apply_voltage_limited_regen(float T_cmd, float V_dc) {
    static RegenVoltageControl_t voltage_ctrl = {
        .voltage_max_normal = V_DC_MAX_NORMAL,
        .voltage_max_critical = V_DC_MAX_CRITICAL,
        .voltage_overvoltage = V_DC_OVERVOLTAGE
    };

    // Update voltage control
    voltage_ctrl.voltage_current = V_dc;
    update_regen_voltage_control(&voltage_ctrl);

    // Check for fault
    if (voltage_ctrl.fault_active) {
        // Trigger fault handler
        trigger_overvoltage_fault();
        T_cmd = 0.0f;
    }
    else {
        // Scale regen torque based on voltage
        if (T_cmd < 0.0f) {  // Regen torque is negative
            T_cmd *= voltage_ctrl.regen_scale;
        }
    }

    // Calculate currents with voltage-limited torque
    calculate_regen_currents(-T_cmd, omega_mech, &id_ref, &iq_ref);
}
```

---

#### Strategy 4: Temperature-Based Regen Limiting

Regen capability should be reduced at high temperatures to prevent thermal damage.

**Multi-Source Temperature Monitoring:**

```c
typedef struct {
    float T_motor;           // Motor temperature (°C)
    float T_inverter;        // Inverter temperature (°C)
    float T_battery;         // Battery temperature (°C)
    float T_motor_limit;     // Motor temperature limit
    float T_inverter_limit;  // Inverter temperature limit
    float T_battery_limit;   // Battery temperature limit
    float regen_scale;       // Output: combined regen scaling
} RegenThermalControl_t;

void update_regen_thermal_control(RegenThermalControl_t *ctrl) {
    float scale_motor = 1.0f;
    float scale_inverter = 1.0f;
    float scale_battery = 1.0f;

    // Motor temperature derating
    if (ctrl->T_motor > ctrl->T_motor_limit) {
        float excess = ctrl->T_motor - ctrl->T_motor_limit;
        scale_motor = fmaxf(0.0f, 1.0f - (excess / 20.0f));  // Linear derate over 20°C
    }

    // Inverter temperature derating
    if (ctrl->T_inverter > ctrl->T_inverter_limit) {
        float excess = ctrl->T_inverter - ctrl->T_inverter_limit;
        scale_inverter = fmaxf(0.0f, 1.0f - (excess / 25.0f));  // Linear derate over 25°C
    }

    // Battery temperature derating
    if (ctrl->T_battery > ctrl->T_battery_limit) {
        float excess = ctrl->T_battery - ctrl->T_battery_limit;
        scale_battery = fmaxf(0.0f, 1.0f - (excess / 15.0f));  // Linear derate over 15°C
    }

    // Use the most restrictive limit
    ctrl->regen_scale = fminf(scale_motor, fminf(scale_inverter, scale_battery));
}
```

---

#### Strategy 5: Combined Multi-Factor Regen Control

The most robust strategy combines all the above factors (current, SOC, voltage, temperature) into a comprehensive regen management system.

**Master Regen Controller:**

```c
typedef struct {
    // Input parameters
    float SOC;                      // Battery state of charge (0-1)
    float V_dc;                     // DC bus voltage (V)
    float T_motor;                  // Motor temperature (°C)
    float T_inverter;               // Inverter temperature (°C)
    float T_battery;                // Battery temperature (°C)
    float omega_mech;               // Motor speed (rad/s)

    // Limits
    float I_regen_max;              // Maximum regen current (A)
    float P_regen_max;              // Maximum regen power (W)

    // Sub-controllers
    RegenSOCParams_t soc_ctrl;
    RegenVoltageControl_t voltage_ctrl;
    RegenThermalControl_t thermal_ctrl;

    // Outputs
    float regen_scale_combined;     // Combined scaling factor (0-1)
    float T_regen_max;              // Maximum allowed regen torque (Nm)
} MasterRegenControl_t;

void update_master_regen_control(MasterRegenControl_t *master) {
    // 1. Calculate SOC-based power limit
    float P_soc_limited = calculate_regen_power_soc_limited(master->SOC,
                                                            &master->soc_ctrl);

    // 2. Update voltage-based scaling
    master->voltage_ctrl.voltage_current = master->V_dc;
    update_regen_voltage_control(&master->voltage_ctrl);
    float scale_voltage = master->voltage_ctrl.regen_scale;

    // 3. Update thermal scaling
    master->thermal_ctrl.T_motor = master->T_motor;
    master->thermal_ctrl.T_inverter = master->T_inverter;
    master->thermal_ctrl.T_battery = master->T_battery;
    update_regen_thermal_control(&master->thermal_ctrl);
    float scale_thermal = master->thermal_ctrl.regen_scale;

    // 4. Combine all scaling factors (use most restrictive)
    float scale_combined = fminf(scale_voltage, scale_thermal);

    // 5. Calculate maximum regen power
    float P_max = fminf(P_soc_limited * scale_combined, master->P_regen_max);

    // 6. Convert to maximum torque at current speed
    if (fabsf(master->omega_mech) > 0.1f) {
        master->T_regen_max = P_max / fabsf(master->omega_mech);
    }
    else {
        // At very low speed, use current limit
        float iq_max = master->I_regen_max;
        master->T_regen_max = (3.0f / 2.0f) * POLE_PAIRS * FLUX_LINKAGE * iq_max;
    }

    // 7. Store combined scale for telemetry
    master->regen_scale_combined = scale_combined;
}

// Usage in main control loop
void motor_control_with_regen_management(float T_cmd_user) {
    static MasterRegenControl_t regen_master = {
        .I_regen_max = 150.0f,
        .P_regen_max = 50000.0f,  // 50 kW max
        .soc_ctrl = {
            .soc_full_limit = 0.90f,
            .soc_no_regen = 0.98f,
            .power_max_regen = 50000.0f
        },
        .voltage_ctrl = {
            .voltage_max_normal = 430.0f,
            .voltage_max_critical = 450.0f,
            .voltage_overvoltage = 470.0f
        },
        .thermal_ctrl = {
            .T_motor_limit = 120.0f,
            .T_inverter_limit = 85.0f,
            .T_battery_limit = 50.0f
        }
    };

    // Update regen master controller
    regen_master.SOC = read_battery_soc();
    regen_master.V_dc = read_dc_bus_voltage();
    regen_master.T_motor = read_motor_temperature();
    regen_master.T_inverter = read_inverter_temperature();
    regen_master.T_battery = read_battery_temperature();
    regen_master.omega_mech = read_motor_speed();

    update_master_regen_control(&regen_master);

    // Apply regen torque limit
    float T_cmd_limited = T_cmd_user;
    if (T_cmd_user < 0.0f) {  // Regen is negative torque
        T_cmd_limited = fmaxf(T_cmd_user, -regen_master.T_regen_max);
    }

    // Calculate dq currents
    if (T_cmd_limited >= 0.0f) {
        calculate_motoring_currents(T_cmd_limited, &id_ref, &iq_ref);
    }
    else {
        calculate_regen_currents(-T_cmd_limited, regen_master.omega_mech,
                                &id_ref, &iq_ref);
    }
}
```

---

### References for Section 2.3:

**Books:**
1. *"Battery Management Systems for Large Lithium-Ion Battery Packs"* by Davide Andrea - Chapter 8 (Charging and Protection)
2. *"Electric and Hybrid Vehicles: Technologies, Modeling and Control"* by Amir Khajepour et al. - Chapter 6 (Energy Management Strategies)

**Papers:**
1. Gao, Y., Chen, L., & Ehsani, M. (1999). "Investigation of the Effectiveness of Regenerative Braking for EV and HEV." SAE Technical Paper 1999-01-2910.
2. Yeo, H., & Kim, H. (2002). "Hardware-in-the-Loop Simulation of Regenerative Braking for a Hybrid Electric Vehicle." Journal of Automobile Engineering, 216(11), 855-864.

**Application Notes:**
1. **Texas Instruments**: "Battery Management System Design Considerations" (SLUA915)
2. **STMicroelectronics**: "Regenerative Braking Implementation in PMSM Drives" (AN4993)

---

### 2.4 Battery Management During Regeneration

This section addresses a critical question: **What happens to regen energy when the battery is full?**

When the battery reaches its maximum State of Charge (SOC), it cannot safely accept additional charge current. If regen energy continues to flow into the battery, it can lead to:

1. **Overvoltage**: DC bus voltage rises above safe limits
2. **Battery damage**: Overcharging reduces battery life and can cause thermal runaway
3. **System fault**: Controller shutdown due to overvoltage protection

#### Battery Charge Acceptance

**C-Rate and Charge Acceptance:**

Battery charge acceptance depends on:
- **Current SOC**: Higher SOC → Lower acceptance
- **Temperature**: Cold batteries accept less charge
- **Battery chemistry**: Different chemistries have different limits
- **Battery age**: Older batteries accept less charge

**Charge Acceptance Model:**

```c
typedef struct {
    float SOC;                  // Current state of charge (0-1)
    float T_battery;            // Battery temperature (°C)
    float I_charge_max_cell;    // Maximum cell charge current (A)
    float num_parallel;         // Number of cells in parallel
    float derating_soc;         // SOC derating factor
    float derating_temp;        // Temperature derating factor
} BatteryChargeModel_t;

float calculate_charge_acceptance(BatteryChargeModel_t *batt) {
    // 1. SOC-based derating
    if (batt->SOC < 0.80f) {
        batt->derating_soc = 1.0f;  // Full charge rate
    }
    else if (batt->SOC < 0.90f) {
        // Linear taper from 80% to 90%
        batt->derating_soc = 1.0f - ((batt->SOC - 0.80f) / 0.10f) * 0.5f;
    }
    else if (batt->SOC < 0.98f) {
        // Aggressive taper from 90% to 98%
        batt->derating_soc = 0.5f - ((batt->SOC - 0.90f) / 0.08f) * 0.5f;
    }
    else {
        // No charging above 98%
        batt->derating_soc = 0.0f;
    }

    // 2. Temperature-based derating
    if (batt->T_battery < -10.0f) {
        // Very cold - minimal charging
        batt->derating_temp = 0.1f;
    }
    else if (batt->T_battery < 0.0f) {
        // Cold - reduced charging
        batt->derating_temp = 0.1f + (batt->T_battery + 10.0f) / 10.0f * 0.4f;
    }
    else if (batt->T_battery < 15.0f) {
        // Cool - partial charging
        batt->derating_temp = 0.5f + (batt->T_battery / 15.0f) * 0.5f;
    }
    else if (batt->T_battery < 45.0f) {
        // Optimal range
        batt->derating_temp = 1.0f;
    }
    else if (batt->T_battery < 55.0f) {
        // Hot - reduce charging
        batt->derating_temp = 1.0f - ((batt->T_battery - 45.0f) / 10.0f) * 0.5f;
    }
    else {
        // Very hot - minimal charging
        batt->derating_temp = 0.5f - ((batt->T_battery - 55.0f) / 10.0f) * 0.5f;
        batt->derating_temp = fmaxf(0.0f, batt->derating_temp);
    }

    // 3. Calculate total acceptable current
    float I_max_pack = batt->I_charge_max_cell * batt->num_parallel;
    float I_acceptable = I_max_pack * batt->derating_soc * batt->derating_temp;

    return I_acceptable;
}
```

#### Managing Full Battery: Solution Strategies

When the battery is full or near full, the system must manage regen energy using one or more of these strategies:

---

**Strategy 1: Gradual Regen Reduction (Preferred)**

Smoothly reduce regen power as the battery approaches full charge.

```c
void manage_full_battery_gradual(float *T_regen_cmd, BatteryChargeModel_t *batt) {
    // Calculate how much current the battery can accept
    float I_acceptable = calculate_charge_acceptance(batt);

    // Convert to power limit
    float V_battery = read_battery_voltage();
    float P_acceptable = I_acceptable * V_battery;

    // Convert to torque limit
    float omega_mech = read_motor_speed();
    float T_regen_max = P_acceptable / fabsf(omega_mech);

    // Apply limit
    if (fabsf(*T_regen_cmd) > T_regen_max) {
        *T_regen_cmd = -T_regen_max;  // Regen is negative torque

        // Log event for user notification
        log_regen_limited_battery_full();
    }
}
```

---

**Strategy 2: Transition to Friction Brakes**

When regen is no longer available, seamlessly blend to friction brakes.

```c
typedef struct {
    float brake_force_total;       // Total desired braking force (N)
    float brake_force_regen;       // Regen braking force (N)
    float brake_force_friction;    // Friction braking force (N)
    float regen_capability;        // Regen capability (0-1)
} BrakeBlending_t;

void blend_regen_to_friction(BrakeBlending_t *blend, float SOC) {
    // Determine regen capability based on SOC
    if (SOC < 0.90f) {
        blend->regen_capability = 1.0f;
    }
    else if (SOC < 0.98f) {
        blend->regen_capability = (0.98f - SOC) / 0.08f;
    }
    else {
        blend->regen_capability = 0.0f;
    }

    // Calculate regen and friction contributions
    blend->brake_force_regen = blend->brake_force_total * blend->regen_capability;
    blend->brake_force_friction = blend->brake_force_total - blend->brake_force_regen;

    // Apply regen braking
    float T_regen = calculate_torque_from_force(blend->brake_force_regen);
    apply_regen_torque(T_regen);

    // Apply friction braking
    apply_friction_brakes(blend->brake_force_friction);
}
```

---

**Strategy 3: Dynamic Brake Resistor (Hardware Solution)**

For systems with a dynamic brake resistor (DBR), excess regen energy can be dissipated as heat.

```c
#define DBR_VOLTAGE_ENABLE    430.0f  // Enable DBR (V)
#define DBR_VOLTAGE_DISABLE   410.0f  // Disable DBR (V)
#define DBR_DUTY_MIN          0.0f
#define DBR_DUTY_MAX          1.0f

typedef struct {
    float V_dc;                 // DC bus voltage (V)
    float V_enable;             // Voltage to enable DBR
    float V_disable;            // Voltage to disable DBR (hysteresis)
    float duty_cycle;           // DBR PWM duty cycle (0-1)
    bool enabled;               // DBR active flag
} DynamicBrakeResistor_t;

void update_dynamic_brake_resistor(DynamicBrakeResistor_t *dbr) {
    // Hysteresis control
    if (dbr->V_dc > dbr->V_enable) {
        dbr->enabled = true;
    }
    else if (dbr->V_dc < dbr->V_disable) {
        dbr->enabled = false;
    }

    if (dbr->enabled) {
        // PI controller for voltage regulation
        float V_error = dbr->V_dc - dbr->V_disable;
        static float integral = 0.0f;

        float Kp = 0.01f;  // Proportional gain
        float Ki = 0.5f;   // Integral gain

        integral += V_error * DT;
        integral = fmaxf(0.0f, fminf(10.0f, integral));  // Anti-windup

        dbr->duty_cycle = Kp * V_error + Ki * integral;
        dbr->duty_cycle = fmaxf(DBR_DUTY_MIN, fminf(DBR_DUTY_MAX, dbr->duty_cycle));

        // Apply PWM to DBR
        set_dbr_pwm(dbr->duty_cycle);
    }
    else {
        dbr->duty_cycle = 0.0f;
        set_dbr_pwm(0.0f);

        // Reset integral
        static float integral = 0.0f;
        integral = 0.0f;
    }
}
```

**DBR Thermal Management:**

```c
// Monitor DBR temperature and derate if needed
float calculate_dbr_power_limit(float T_dbr, float T_max) {
    if (T_dbr < T_max - 20.0f) {
        return 1.0f;  // Full power
    }
    else if (T_dbr < T_max) {
        // Linear derate over 20°C
        return (T_max - T_dbr) / 20.0f;
    }
    else {
        return 0.0f;  // Disable DBR
    }
}
```

---

**Strategy 4: Limit Vehicle Deceleration**

If neither friction brakes nor DBR can handle the excess energy, limit the vehicle's deceleration rate.

```c
typedef struct {
    float decel_requested;      // Driver-requested deceleration (m/s²)
    float decel_max_regen;      // Max deceleration from available regen (m/s²)
    float decel_max_friction;   // Max deceleration from friction brakes (m/s²)
    float decel_actual;         // Actual applied deceleration (m/s²)
} DecelLimiting_t;

void apply_deceleration_limiting(DecelLimiting_t *decel) {
    // Total available deceleration
    float decel_available = decel->decel_max_regen + decel->decel_max_friction;

    if (decel->decel_requested <= decel_available) {
        // Can meet driver request
        decel->decel_actual = decel->decel_requested;
    }
    else {
        // Cannot meet request - limit deceleration
        decel->decel_actual = decel_available;

        // Notify driver (visual/haptic feedback)
        notify_decel_limited();
    }
}
```

---

#### Integrated Battery Management System

A complete system integrates all strategies:

```c
typedef enum {
    BATTERY_REGEN_FULL,         // Full regen available
    BATTERY_REGEN_LIMITED,      // Regen limited by SOC/temp
    BATTERY_REGEN_BLENDING,     // Blending regen + friction
    BATTERY_REGEN_FRICTION_ONLY, // Friction brakes only
    BATTERY_REGEN_DBR_ACTIVE    // DBR dissipating energy
} BatteryRegenState_t;

typedef struct {
    BatteryChargeModel_t battery;
    BrakeBlending_t blending;
    DynamicBrakeResistor_t dbr;
    DecelLimiting_t decel_limit;
    BatteryRegenState_t state;
    float regen_power_limit;    // Output: max regen power (W)
} IntegratedBatteryMgmt_t;

void update_integrated_battery_management(IntegratedBatteryMgmt_t *mgmt,
                                          float T_regen_request) {
    // 1. Calculate battery charge acceptance
    float I_acceptable = calculate_charge_acceptance(&mgmt->battery);
    float V_battery = read_battery_voltage();
    mgmt->regen_power_limit = I_acceptable * V_battery;

    // 2. Determine state
    if (mgmt->battery.SOC < 0.85f) {
        mgmt->state = BATTERY_REGEN_FULL;
    }
    else if (mgmt->battery.SOC < 0.95f) {
        mgmt->state = BATTERY_REGEN_LIMITED;
    }
    else if (mgmt->battery.SOC < 0.98f) {
        mgmt->state = BATTERY_REGEN_BLENDING;
    }
    else {
        if (mgmt->dbr.enabled) {
            mgmt->state = BATTERY_REGEN_DBR_ACTIVE;
        }
        else {
            mgmt->state = BATTERY_REGEN_FRICTION_ONLY;
        }
    }

    // 3. Execute strategy based on state
    switch (mgmt->state) {
        case BATTERY_REGEN_FULL:
            // Apply full regen request
            apply_regen_torque(T_regen_request);
            break;

        case BATTERY_REGEN_LIMITED:
            // Limit regen based on battery acceptance
            manage_full_battery_gradual(&T_regen_request, &mgmt->battery);
            apply_regen_torque(T_regen_request);
            break;

        case BATTERY_REGEN_BLENDING:
            // Blend regen and friction brakes
            blend_regen_to_friction(&mgmt->blending, mgmt->battery.SOC);
            break;

        case BATTERY_REGEN_FRICTION_ONLY:
            // Use only friction brakes
            apply_friction_brakes(mgmt->blending.brake_force_total);
            break;

        case BATTERY_REGEN_DBR_ACTIVE:
            // DBR handling excess voltage
            update_dynamic_brake_resistor(&mgmt->dbr);
            apply_friction_brakes(mgmt->blending.brake_force_total);
            break;
    }

    // 4. Monitor DC bus voltage
    float V_dc = read_dc_bus_voltage();
    mgmt->dbr.V_dc = V_dc;
    update_dynamic_brake_resistor(&mgmt->dbr);

    // 5. Log telemetry
    log_battery_regen_state(mgmt->state, mgmt->regen_power_limit);
}
```

#### Cell Balancing Considerations

When the battery is near full, cell balancing becomes important:

```c
typedef struct {
    float cell_voltage_min;     // Minimum cell voltage (V)
    float cell_voltage_max;     // Maximum cell voltage (V)
    float cell_voltage_delta;   // Delta between min and max (V)
    bool balancing_active;      // Cell balancing in progress
} CellBalancing_t;

void check_cell_balancing_impact(CellBalancing_t *cells, float *regen_limit) {
    // Calculate voltage delta
    cells->cell_voltage_delta = cells->cell_voltage_max - cells->cell_voltage_min;

    // If cells are imbalanced, reduce regen to allow balancing
    if (cells->cell_voltage_delta > 0.05f) {  // 50 mV threshold
        cells->balancing_active = true;

        // Reduce regen power to allow balancing time
        *regen_limit *= 0.5f;

        log_cell_balancing_active();
    }
    else {
        cells->balancing_active = false;
    }
}
```

---

### References for Section 2.4:

**Books:**
1. *"Battery Management Systems, Volume II: Equivalent-Circuit Methods"* by Gregory L. Plett - Chapter 6 (SOC-Dependent Charge Acceptance)
2. *"Lithium-Ion Batteries: Advanced Materials and Technologies"* by Xianxia Yuan et al. - Chapter 9 (Charging Protocols)
3. *"Battery Systems Engineering"* by Christopher D. Rahn and Chao-Yang Wang - Chapter 5 (Thermal Management During Charging)

**Papers:**
1. Chen, M., & Rincon-Mora, G. A. (2006). "Accurate Electrical Battery Model Capable of Predicting Runtime and I-V Performance." IEEE Transactions on Energy Conversion, 21(2), 504-511.
2. Plett, G. L. (2004). "Extended Kalman Filtering for Battery Management Systems of LiPB-Based HEV Battery Packs: Part 3. State and Parameter Estimation." Journal of Power Sources, 134(2), 277-292.

**Standards:**
1. **SAE J1772**: "Electric Vehicle and Plug-in Hybrid Electric Vehicle Conductive Charge Coupler" (charge acceptance)
2. **IEC 62660**: "Secondary Lithium-Ion Cells for the Propulsion of Electric Road Vehicles" (charge limits)

**Application Notes:**
1. **Texas Instruments**: "Implementing Pack-Level Charge Control" (SLVA952)
2. **Analog Devices**: "Battery Management System Design" (AN-1333)

---

### 2.5 User-Selectable Regen Levels and Percentage Control

Modern EVs offer drivers multiple regen levels, allowing them to choose between:
- **Strong regen**: Maximum energy recovery, one-pedal driving
- **Medium regen**: Balanced feel
- **Light regen**: Coast feel similar to ICE vehicles
- **Off**: No regen (for maximum coasting)

This section explains how to implement user-selectable regen settings.

#### Regen Level Mapping

**User Interface to Control Mapping:**

```c
typedef enum {
    REGEN_OFF = 0,           // No regenerative braking
    REGEN_LOW = 1,           // Light regeneration (20% max)
    REGEN_MEDIUM = 2,        // Medium regeneration (50% max)
    REGEN_HIGH = 3,          // High regeneration (80% max)
    REGEN_MAX = 4            // Maximum regeneration (100%)
} RegenLevel_t;

typedef struct {
    RegenLevel_t level;      // User-selected regen level
    float percentage;        // Regen power as percentage of max (0-1)
    float T_max_available;   // Maximum available regen torque (Nm)
    float T_regen_limited;   // Torque after applying percentage (Nm)
} RegenLevelControl_t;

void apply_regen_level(RegenLevelControl_t *ctrl) {
    // Map regen level to percentage
    switch (ctrl->level) {
        case REGEN_OFF:
            ctrl->percentage = 0.0f;
            break;
        case REGEN_LOW:
            ctrl->percentage = 0.20f;  // 20%
            break;
        case REGEN_MEDIUM:
            ctrl->percentage = 0.50f;  // 50%
            break;
        case REGEN_HIGH:
            ctrl->percentage = 0.80f;  // 80%
            break;
        case REGEN_MAX:
            ctrl->percentage = 1.0f;   // 100%
            break;
        default:
            ctrl->percentage = 0.50f;  // Default to medium
            break;
    }

    // Apply percentage to available torque
    ctrl->T_regen_limited = ctrl->T_max_available * ctrl->percentage;
}
```

#### Speed-Dependent Regen Tuning

Different regen levels can be tuned differently at various speeds for better user experience:

```c
typedef struct {
    float speed_low;         // Low speed threshold (rad/s)
    float speed_high;        // High speed threshold (rad/s)
    float scale_low_speed;   // Regen scaling at low speed
    float scale_high_speed;  // Regen scaling at high speed
} RegenSpeedTuning_t;

float calculate_speed_dependent_regen(float omega_mech, RegenSpeedTuning_t *tuning) {
    if (omega_mech < tuning->speed_low) {
        // At low speed, use low-speed scaling
        return tuning->scale_low_speed;
    }
    else if (omega_mech < tuning->speed_high) {
        // Interpolate between low and high speed
        float ratio = (omega_mech - tuning->speed_low) /
                     (tuning->speed_high - tuning->speed_low);
        return tuning->scale_low_speed +
               ratio * (tuning->scale_high_speed - tuning->scale_low_speed);
    }
    else {
        // At high speed, use high-speed scaling
        return tuning->scale_high_speed;
    }
}
```

**Example Tuning:**

```c
// Regen level definitions with speed-dependent tuning
const RegenSpeedTuning_t regen_tuning_profiles[5] = {
    // REGEN_OFF
    {
        .speed_low = 10.0f,
        .speed_high = 100.0f,
        .scale_low_speed = 0.0f,
        .scale_high_speed = 0.0f
    },
    // REGEN_LOW
    {
        .speed_low = 10.0f,
        .speed_high = 100.0f,
        .scale_low_speed = 0.10f,  // Very gentle at low speed
        .scale_high_speed = 0.25f   // Slightly more at high speed
    },
    // REGEN_MEDIUM
    {
        .speed_low = 10.0f,
        .speed_high = 100.0f,
        .scale_low_speed = 0.30f,  // Moderate at low speed
        .scale_high_speed = 0.60f   // More at high speed
    },
    // REGEN_HIGH
    {
        .speed_low = 10.0f,
        .speed_high = 100.0f,
        .scale_low_speed = 0.60f,  // Strong at low speed
        .scale_high_speed = 0.90f   // Very strong at high speed
    },
    // REGEN_MAX
    {
        .speed_low = 10.0f,
        .speed_high = 100.0f,
        .scale_low_speed = 1.0f,   // Maximum at all speeds
        .scale_high_speed = 1.0f
    }
};
```

#### Accelerator Pedal Mapping to Regen

Map accelerator pedal position to regen torque command:

```c
typedef struct {
    float pedal_position;       // Accelerator pedal (0-1, 0=released)
    float pedal_regen_start;    // Pedal position where regen starts
    float pedal_regen_max;      // Pedal position for max regen
    float regen_curve_exp;      // Exponential curve factor (1=linear, >1=progressive)
} PedalToRegenMap_t;

float map_pedal_to_regen(PedalToRegenMap_t *map, float T_regen_max) {
    // Check if pedal is in regen zone
    if (map->pedal_position > map->pedal_regen_start) {
        // Not in regen zone (pedal pressed)
        return 0.0f;
    }
    else if (map->pedal_position < map->pedal_regen_max) {
        // Maximum regen (pedal fully released or more)
        return T_regen_max;
    }
    else {
        // Proportional regen based on pedal position
        float pedal_range = map->pedal_regen_start - map->pedal_regen_max;
        float pedal_in_range = map->pedal_regen_start - map->pedal_position;
        float ratio = pedal_in_range / pedal_range;

        // Apply exponential curve for more natural feel
        ratio = powf(ratio, map->regen_curve_exp);

        return T_regen_max * ratio;
    }
}
```

**Example Configurations:**

```c
// Conservative mapping (gentle regen onset)
PedalToRegenMap_t pedal_map_conservative = {
    .pedal_regen_start = 0.15f,  // Regen starts at 15% pedal
    .pedal_regen_max = 0.0f,     // Max regen at 0% pedal
    .regen_curve_exp = 2.0f      // Quadratic curve (progressive)
};

// Aggressive mapping (one-pedal driving)
PedalToRegenMap_t pedal_map_aggressive = {
    .pedal_regen_start = 0.20f,  // Regen starts at 20% pedal
    .pedal_regen_max = 0.0f,     // Max regen at 0% pedal
    .regen_curve_exp = 1.5f      // Moderate curve
};
```

#### Paddle/Button Control for Regen Adjustment

Many EVs use steering wheel paddles or buttons to adjust regen on-the-fly:

```c
typedef struct {
    RegenLevel_t level;          // Current regen level
    RegenLevel_t level_min;      // Minimum level
    RegenLevel_t level_max;      // Maximum level
    bool paddle_plus_pressed;    // Increase regen paddle
    bool paddle_minus_pressed;   // Decrease regen paddle
} RegenPaddleControl_t;

void update_regen_paddle_control(RegenPaddleControl_t *paddle) {
    static bool paddle_plus_prev = false;
    static bool paddle_minus_prev = false;

    // Detect rising edge on increase paddle
    if (paddle->paddle_plus_pressed && !paddle_plus_prev) {
        if (paddle->level < paddle->level_max) {
            paddle->level++;
            log_regen_level_changed(paddle->level);
            provide_haptic_feedback();
        }
    }

    // Detect rising edge on decrease paddle
    if (paddle->paddle_minus_pressed && !paddle_minus_prev) {
        if (paddle->level > paddle->level_min) {
            paddle->level--;
            log_regen_level_changed(paddle->level);
            provide_haptic_feedback();
        }
    }

    // Store previous state
    paddle_plus_prev = paddle->paddle_plus_pressed;
    paddle_minus_prev = paddle->paddle_minus_pressed;
}
```

#### Adaptive Regen (Advanced Feature)

Some systems automatically adjust regen based on driving conditions:

```c
typedef struct {
    float traffic_density;       // Traffic density estimate (0-1)
    float road_grade;            // Road grade (radians, + = uphill)
    float following_distance;    // Distance to vehicle ahead (m)
    bool adaptive_enabled;       // Adaptive regen feature enabled
    float adaptive_scale;        // Output: regen scaling factor (0-1)
} AdaptiveRegen_t;

void update_adaptive_regen(AdaptiveRegen_t *adapt, RegenLevel_t base_level) {
    if (!adapt->adaptive_enabled) {
        adapt->adaptive_scale = 1.0f;
        return;
    }

    float scale = 1.0f;

    // 1. Increase regen in heavy traffic (stop-and-go)
    if (adapt->traffic_density > 0.7f) {
        scale *= 1.2f;  // 20% more regen
    }

    // 2. Reduce regen on downhill (prevent too aggressive braking)
    if (adapt->road_grade < -0.05f) {  // >5% downgrade
        scale *= 0.8f;  // 20% less regen
    }

    // 3. Increase regen when following closely
    if (adapt->following_distance < 20.0f && adapt->following_distance > 0.1f) {
        float proximity = 1.0f - (adapt->following_distance / 20.0f);
        scale *= (1.0f + 0.3f * proximity);  // Up to 30% more
    }

    // 4. Clamp scaling factor
    adapt->adaptive_scale = fmaxf(0.5f, fminf(1.5f, scale));
}
```

#### Complete Regen Level Management System

Integrate all components:

```c
typedef struct {
    // User inputs
    RegenLevel_t user_level;         // User-selected base level
    float pedal_position;            // Accelerator pedal (0-1)
    float brake_pedal;               // Brake pedal (0-1)

    // System state
    float omega_mech;                // Motor speed (rad/s)
    float T_regen_max_system;        // Max regen from battery/thermal limits

    // Configuration
    RegenLevelControl_t level_ctrl;
    PedalToRegenMap_t pedal_map;
    RegenSpeedTuning_t *speed_tuning;
    AdaptiveRegen_t adaptive;

    // Outputs
    float T_regen_command;           // Final regen torque command (Nm)
    float regen_power_actual;        // Actual regen power (W)
} RegenLevelManager_t;

void update_regen_level_manager(RegenLevelManager_t *mgr) {
    // 1. Apply user-selected regen level
    mgr->level_ctrl.level = mgr->user_level;
    mgr->level_ctrl.T_max_available = mgr->T_regen_max_system;
    apply_regen_level(&mgr->level_ctrl);

    // 2. Apply speed-dependent tuning
    float speed_scale = calculate_speed_dependent_regen(
        mgr->omega_mech,
        &mgr->speed_tuning[mgr->user_level]
    );
    mgr->level_ctrl.T_regen_limited *= speed_scale;

    // 3. Apply adaptive adjustments
    update_adaptive_regen(&mgr->adaptive, mgr->user_level);
    mgr->level_ctrl.T_regen_limited *= mgr->adaptive.adaptive_scale;

    // 4. Map pedal position to regen torque
    mgr->pedal_map.pedal_position = mgr->pedal_position;
    float T_regen_from_pedal = map_pedal_to_regen(
        &mgr->pedal_map,
        mgr->level_ctrl.T_regen_limited
    );

    // 5. Handle brake pedal override
    if (mgr->brake_pedal > 0.05f) {
        // Brake pedal pressed - use maximum regen + friction brakes
        T_regen_from_pedal = mgr->level_ctrl.T_regen_limited;
    }

    // 6. Final regen command
    mgr->T_regen_command = T_regen_from_pedal;

    // 7. Calculate actual power for telemetry
    mgr->regen_power_actual = mgr->T_regen_command * fabsf(mgr->omega_mech);
}
```

#### User Experience Considerations

**Smooth Transitions:**

When the user changes regen level, smoothly transition to avoid jerk:

```c
#define REGEN_LEVEL_TRANSITION_TIME  0.5f  // seconds

void smooth_regen_level_transition(float *T_current, float T_target, float dt) {
    float transition_rate = fabsf(T_target) / REGEN_LEVEL_TRANSITION_TIME;
    float max_change = transition_rate * dt;

    float error = T_target - *T_current;

    if (fabsf(error) < max_change) {
        *T_current = T_target;
    }
    else if (error > 0.0f) {
        *T_current += max_change;
    }
    else {
        *T_current -= max_change;
    }
}
```

**User Feedback:**

Provide clear feedback about regen status:

```c
void update_user_feedback(RegenLevelManager_t *mgr) {
    // Display current regen level on dashboard
    display_regen_level(mgr->user_level);

    // Show real-time power flow
    display_power_flow(mgr->regen_power_actual);

    // If regen is limited, notify user
    if (mgr->T_regen_command < mgr->level_ctrl.T_regen_limited * 0.8f) {
        display_warning("Regen Limited: Battery Full");
    }
}
```

---

### References for Section 2.5:

**Books:**
1. *"Automotive User Interfaces: Creating Interactive Experiences in the Car"* by Gerrit Meixner and Christoph Müller - Chapter 4 (Control Interfaces)
2. *"Electric and Hybrid Vehicles: Design Fundamentals"* by Iqbal Husain - Chapter 10 (User Interface and Control)

**Papers:**
1. Pennycott, A., et al. (2015). "The Role of Regenerative Brake Blending in EV Range and Energy Consumption Patterns." EVS28 International Electric Vehicle Symposium and Exhibition.
2. Lee, J., & Nelson, D. J. (2005). "Rotating Inertia Impact on Propulsion and Regenerative Braking for Electric Motor Driven Vehicles." IEEE Vehicle Power and Propulsion Conference.

**Industry Standards:**
1. **ISO 15622**: "Intelligent Transport Systems - Adaptive Cruise Control Systems" (discusses user-adjustable settings)
2. **SAE J2954**: "Wireless Power Transfer for Light-Duty Plug-in/Electric Vehicles and Alignment Methodology" (user interface guidelines)

**Application Notes:**
1. **Bosch**: "Regenerative Braking Systems for Electric Vehicles" (2019 Technical Paper)
2. **Tesla**: "Understanding Regenerative Braking" - Owner's Manual Section

---

### 2.6 Hardware Perspective: Circuit-Level Voltage and Current Behavior

This section provides the hardware and circuit-level understanding of regenerative braking, covering how voltage and current behave across the motor controller and their implications for component selection and system design.

#### Back-EMF Generation and Voltage Relationships

**Fundamental Voltage Relationship:**

During regeneration, the motor operates as a generator. The rotating permanent magnets induce a back-EMF in the stator windings:

```
Motor Operating as Generator:

E_backemf = K_e * ω_m

Where:
  E_backemf = back-EMF voltage (line-to-neutral, peak) (V)
  K_e = motor voltage constant (V/rad/s)
  ω_m = mechanical speed (rad/s)

For three-phase motor (line-to-line RMS):
  E_ll_rms = √3 * E_backemf / √2
```

**Key Voltage Relationships:**

```
For regen to occur:
  E_backemf > V_dc_bus > V_battery

Energy Flow:
  Motor EMF → Inverter DC Bus → Battery

Example (400V battery system):
  Motor at 3000 RPM: E_backemf = 450V (line-to-line RMS)
  DC Bus:           V_dc = 420V
  Battery:          V_batt = 380V

  ΔV available for current flow: 450V - 420V = 30V
  This voltage difference drives regen current
```

**Circuit-Level View:**

```
                    Motor (Generator Mode)
                         |
                Back-EMF: 450V
                         |
                         ↓
                    +----------+
                    | Inverter |  ← Controls current flow
                    +----------+
                         |
                   DC Bus: 420V
                         |
                    +----------+
                    |  DC Link |  ← Energy buffer
                    | Capacitor|
                    +----------+
                         |
                   Battery: 380V
                         |
                    +----------+
                    |  Battery |  ← Energy sink
                    +----------+
```

#### Inverter Current Flow During Regeneration

**Three-Phase Inverter Topology:**

```
                         +V_dc (420V)
                           |
            +-------+------+------+-------+
            |       |             |       |
          [Q1]    [Q3]           [Q5]
            |       |             |
     Phase A|  Phase B|      Phase C|
            |       |             |
          [Q2]    [Q4]           [Q6]
            |       |             |
            +-------+------+------+-------+
                           |
                         -V_dc (0V)

Where:
  Q1, Q3, Q5 = High-side MOSFETs
  Q2, Q4, Q6 = Low-side MOSFETs

Each MOSFET has an intrinsic body diode
```

**Current Flow Paths in Motoring vs Regeneration:**

```
MOTORING MODE (Power from battery to motor):
  Current flows: Battery → DC+ → MOSFETs (actively switched) → Motor phases

  Example Phase A positive current:
    Q1 ON (active): DC+ → Q1 channel → Phase A → Motor
    Q2 OFF: Body diode blocks reverse current

REGENERATION MODE (Power from motor to battery):
  Current flows: Motor phases → Body diodes or MOSFETs → DC+ → Battery

  Example Phase A negative current (motor generating):
    Q1 OFF: Current cannot flow through channel
    Q2 ON (synchronous rect): Phase A → Q2 channel → DC- → Battery

  OR (if Q2 off during commutation):
    Q2 body diode: Phase A → Q2 diode → DC- → Battery
```

**Phase Current Direction Reversal:**

During regeneration, phase currents reverse compared to motoring:

```c
// Motoring: i_q > 0, current flows from DC bus to motor
// Regen:    i_q < 0, current flows from motor to DC bus

// Example: Phase A current during regeneration
// When motor generates, induced EMF creates current in reverse direction

Motoring Phase A:    DC+ → Q1 → Phase A → Neutral
Regen Phase A:       Phase A → Q2 (diode or channel) → DC- → Battery
```

#### Synchronous Rectification vs Diode Rectification

**Two Modes of Inverter Operation During Regen:**

**1. Diode Rectification (Passive):**
- MOSFETs OFF, current flows through body diodes
- Higher losses (diode forward voltage ~0.7-1.2V)
- Simpler control, no risk of shoot-through
- Used in older or simpler controllers

```c
// Diode rectification - all MOSFETs OFF during regen
void diode_rectification_mode(void) {
    // Turn OFF all MOSFETs
    set_high_side_mosfets(OFF, OFF, OFF);
    set_low_side_mosfets(OFF, OFF, OFF);

    // Current flows through body diodes based on motor EMF
    // Energy flows to DC bus, but with higher losses
}

// Power loss calculation:
// P_loss_diode = V_f_diode * I_phase * 3 phases
// Example: 1.0V * 100A * 3 = 300W loss
```

**2. Synchronous Rectification (Active):**
- MOSFETs actively switched to conduct current
- Much lower losses (R_ds_on * I² typically < 50W)
- Requires careful control to avoid shoot-through
- Used in modern high-efficiency controllers

```c
// Synchronous rectification - actively switch MOSFETs during regen
void synchronous_rectification_mode(void) {
    // For each phase, turn ON the appropriate MOSFET when current flows

    // Example for Phase A during negative current (regen):
    if (i_phase_a < 0.0f) {
        // Current flowing from motor towards DC-
        set_low_side_mosfet_a(ON);   // Q2 ON - active conduction
        set_high_side_mosfet_a(OFF); // Q1 OFF - prevent shoot-through

        // Ensure dead-time between transitions
        delay_ns(DEAD_TIME_NS);
    }

    // Power loss calculation:
    // P_loss_mosfet = R_ds_on * I_phase² * 3 phases
    // Example: 0.005Ω * (100A)² * 3 = 150W loss
    // 50% reduction compared to diode rectification!
}
```

**Comparison Table:**

```
Parameter              Diode Rectification    Synchronous Rectification
------------------------------------------------------------------------
Conduction Loss        300-500W @ 100A       150-250W @ 100A
Efficiency             92-94%                 95-97%
Control Complexity     Simple                 Complex
Shoot-through Risk     None                   Present (requires dead-time)
Component Cost         Lower                  Higher (better MOSFETs needed)
Typical Application    Low-cost systems       High-performance EVs
```

#### DC Link Voltage Dynamics During Regeneration

**DC Link Capacitor Behavior:**

The DC link capacitor acts as an energy buffer between the motor and battery:

```
Energy Flow During Regen:

Motor → Inverter → DC Capacitor → Battery
        (fast)      (buffer)      (slow)

The capacitor voltage rises because:
1. Motor delivers power quickly (milliseconds)
2. Battery accepts power slowly (limited by C-rate)
3. Capacitor stores the difference temporarily
```

**Voltage Rise Calculation:**

```c
typedef struct {
    float V_dc_nominal;         // Nominal DC bus voltage (V)
    float V_dc_current;         // Current DC bus voltage (V)
    float C_dc_link;            // DC link capacitance (F)
    float P_regen;              // Regen power from motor (W)
    float P_to_battery;         // Power to battery (W)
    float dV_dt;                // Rate of voltage rise (V/s)
} DCLinkDynamics_t;

void calculate_dc_link_voltage_rise(DCLinkDynamics_t *dc, float dt) {
    // Net power into DC link capacitor
    float P_net = dc->P_regen - dc->P_to_battery;

    // Energy stored in capacitor: E = (1/2) * C * V²
    // Power: P = dE/dt = C * V * (dV/dt)
    // Therefore: dV/dt = P / (C * V)

    dc->dV_dt = P_net / (dc->C_dc_link * dc->V_dc_current);

    // Update voltage
    dc->V_dc_current += dc->dV_dt * dt;

    // Check for overvoltage
    if (dc->V_dc_current > V_DC_OVERVOLTAGE_LIMIT) {
        // Trigger overvoltage protection
        trigger_overvoltage_protection();

        // Immediately reduce or stop regen
        reduce_regen_power();
    }
}

// Example calculation:
// P_regen = 30kW, P_to_battery = 25kW, P_net = 5kW
// C_dc_link = 1000µF, V_dc = 400V
// dV/dt = 5000W / (0.001F * 400V) = 12,500 V/s = 12.5 V/ms
//
// In 10ms, voltage rises by 125V if battery can't accept power!
// This is why DC link capacitors must be large
```

**Capacitor Sizing for Regen:**

```c
// Design formula for DC link capacitor
float calculate_required_capacitance(float P_regen_max,
                                      float P_battery_max,
                                      float V_dc_nominal,
                                      float dV_max_allowed,
                                      float t_response) {
    // Maximum net power that capacitor must buffer
    float P_net_max = P_regen_max - P_battery_max;

    // Energy to be stored during battery response time
    float E_stored = P_net_max * t_response;

    // Energy in capacitor: E = (1/2) * C * (V_high² - V_low²)
    float V_high = V_dc_nominal + dV_max_allowed;
    float V_low = V_dc_nominal;

    float C_required = (2.0f * E_stored) / (V_high * V_high - V_low * V_low);

    return C_required;
}

// Example:
// P_regen_max = 50kW, P_battery_max = 40kW, P_net = 10kW
// V_nominal = 400V, dV_max = 30V (7.5% ripple)
// t_response = 50ms (battery response time)
//
// E_stored = 10kW * 0.05s = 500J
// C_required = (2 * 500J) / (430² - 400²) = 1000J / 24900 = 40.2mF
//
// → Use 1000µF minimum (with safety margin)
```

#### Voltage and Current Waveforms

**Three-Phase Current Waveforms During Regen:**

```c
// Visualization of phase currents during regeneration

Time-domain view:
                    Phase A Current (negative = regen)
  0A  |─────────────────────────────────────────
      |        ╱╲                    ╱╲
      |       ╱  ╲                  ╱  ╲
-50A  |      ╱    ╲                ╱    ╲
      |     ╱      ╲              ╱      ╲
-100A |____╱________╲____________╱________╲____

       Phase B leads Phase A by 120° electrical
       Phase C leads Phase A by 240° electrical

Key observations:
1. Sinusoidal current (in dq frame, DC in steady state)
2. Negative polarity compared to motoring
3. Phase with back-EMF dictates current flow direction
4. Frequency = electrical speed (pole pairs * mechanical speed)
```

**DC Bus Current Waveform:**

```c
// DC bus current during regen (seen by battery)

         I_dc_bus (current TO battery)
100A |    ___     ___     ___     ___
     |   /   \   /   \   /   \   /   \  ← Ripple from 3-phase rectification
 75A |__/     \_/     \_/     \_/     \__
     |
 50A |________________Average___________________
     |
   0 +─────────────────────────────────────────→ Time

Key characteristics:
1. Average value = regen power / DC voltage
2. Ripple frequency = 6 * electrical frequency (3-phase, 6-pulse rectification)
3. Ripple amplitude depends on motor inductance and switching
4. DC link capacitor smooths this ripple before battery
```

**DC Bus Voltage Ripple:**

```c
typedef struct {
    float V_dc_avg;              // Average DC bus voltage (V)
    float V_dc_ripple_pk_pk;     // Peak-to-peak ripple (V)
    float I_regen_avg;           // Average regen current (A)
    float I_regen_ripple;        // RMS ripple current (A)
    float ESR_capacitor;         // Capacitor ESR (Ω)
    float ripple_frequency;      // Ripple frequency (Hz)
} DCVoltageRipple_t;

void calculate_dc_voltage_ripple(DCVoltageRipple_t *ripple, float f_electrical) {
    // Ripple frequency for 3-phase rectification
    ripple->ripple_frequency = 6.0f * f_electrical;  // 6-pulse

    // Voltage ripple from capacitor impedance
    // V_ripple = I_ripple * Z_cap
    // where Z_cap = ESR + 1/(2*π*f*C)

    float X_cap = 1.0f / (2.0f * M_PI * ripple->ripple_frequency * C_DC_LINK);
    float Z_cap = sqrtf(ripple->ESR_capacitor * ripple->ESR_capacitor + X_cap * X_cap);

    ripple->V_dc_ripple_pk_pk = ripple->I_regen_ripple * Z_cap;

    // Typical values for 400V system:
    // I_regen_ripple_rms ≈ 30A, f_ripple = 1kHz (3000 RPM motor)
    // C = 1000µF, ESR = 50mΩ
    // X_cap = 1/(2π*1000*0.001) = 0.159Ω
    // Z_cap ≈ 0.167Ω
    // V_ripple_pk_pk = 30A * 0.167Ω = 5V (1.25% of 400V)
}
```

#### Power Flow and Loss Mechanisms

**Complete Power Flow During Regeneration:**

```
Motor Mechanical Power (P_mech)
          ↓
     [Motor Losses]  ← Copper loss, iron loss, friction
          ↓
Motor Electrical Power (P_motor_elec)
          ↓
  [Inverter Losses]  ← Conduction loss, switching loss
          ↓
DC Bus Power (P_dc_bus)
          ↓
   [Cable Losses]    ← I²R in cables
          ↓
Battery Power (P_battery)
          ↓
  [Battery Losses]   ← Internal resistance heating
          ↓
Stored Energy (ΔE_battery)


Total Efficiency Chain:
η_total = η_motor * η_inverter * η_cables * η_battery

Typical values during regen:
  η_motor = 0.90-0.93 (regen efficiency)
  η_inverter = 0.94-0.97 (synchronous rectification)
  η_cables = 0.98-0.99
  η_battery = 0.92-0.95 (charging efficiency)

  η_total = 0.75-0.84 (75-84% of kinetic energy recovered)
```

**Detailed Loss Calculations:**

```c
typedef struct {
    // Input
    float P_mech_in;             // Mechanical power from wheels (W)
    float omega_mech;            // Mechanical speed (rad/s)
    float i_d;                   // d-axis current (A)
    float i_q;                   // q-axis current (A)

    // Motor losses
    float P_loss_copper;         // Copper losses (W)
    float P_loss_iron;           // Iron losses (W)
    float P_loss_mechanical;     // Mechanical losses (W)

    // Inverter losses
    float P_loss_conduction;     // MOSFET conduction losses (W)
    float P_loss_switching;      // Switching losses (W)

    // System losses
    float P_loss_cable;          // Cable losses (W)
    float P_loss_battery;        // Battery internal losses (W)

    // Output
    float P_to_battery;          // Power actually stored (W)
    float efficiency_total;      // Total efficiency (%)
} RegenPowerLoss_t;

void calculate_regen_power_losses(RegenPowerLoss_t *loss) {
    // 1. Motor copper losses (I²R)
    float I_rms = sqrtf(loss->i_d * loss->i_d + loss->i_q * loss->i_q);
    loss->P_loss_copper = 1.5f * R_STATOR * I_rms * I_rms;  // 3-phase

    // 2. Motor iron losses (simplified)
    float f_elec = (POLE_PAIRS * loss->omega_mech) / (2.0f * M_PI);
    loss->P_loss_iron = K_IRON_LOSS * f_elec * f_elec;  // Proportional to f²

    // 3. Mechanical losses
    loss->P_loss_mechanical = K_FRICTION * loss->omega_mech;

    // 4. Inverter conduction losses
    // During synchronous rectification
    loss->P_loss_conduction = 3.0f * R_DS_ON * I_rms * I_rms;

    // 5. Inverter switching losses
    float f_switching = PWM_FREQUENCY;
    loss->P_loss_switching = 6.0f * f_switching * (E_ON + E_OFF) * (I_rms / I_RATED);

    // 6. Cable losses
    float I_dc_bus = loss->P_mech_in / V_DC_BUS;
    loss->P_loss_cable = R_CABLE * I_dc_bus * I_dc_bus;

    // 7. Battery internal losses
    float I_battery = I_dc_bus;
    loss->P_loss_battery = R_BATTERY_INTERNAL * I_battery * I_battery;

    // Calculate power to battery
    float P_motor_elec = loss->P_mech_in - loss->P_loss_copper -
                         loss->P_loss_iron - loss->P_loss_mechanical;
    float P_dc_bus = P_motor_elec - loss->P_loss_conduction -
                     loss->P_loss_switching;
    float P_to_battery_terminals = P_dc_bus - loss->P_loss_cable;
    loss->P_to_battery = P_to_battery_terminals - loss->P_loss_battery;

    // Total efficiency
    loss->efficiency_total = (loss->P_to_battery / loss->P_mech_in) * 100.0f;
}

// Example calculation for 30kW regen:
// P_mech_in = 30kW
// Motor losses: 1.5kW (copper) + 0.5kW (iron) + 0.2kW (mech) = 2.2kW
// P_motor_elec = 27.8kW (92.7% motor efficiency)
// Inverter losses: 0.6kW (conduction) + 0.3kW (switching) = 0.9kW
// P_dc_bus = 26.9kW (96.8% inverter efficiency)
// Cable losses: 0.2kW
// P_to_battery_terminals = 26.7kW
// Battery losses: 0.8kW
// P_to_battery = 25.9kW (86.3% total efficiency)
```

#### Hardware Stress and Implications

**Component Stress During Regeneration:**

**1. MOSFET/IGBT Stress:**

```c
// Junction temperature rise during regen
typedef struct {
    float I_rms_regen;           // RMS current during regen (A)
    float R_ds_on;               // MOSFET on-resistance (Ω)
    float R_th_jc;               // Thermal resistance junction-case (°C/W)
    float R_th_ca;               // Thermal resistance case-ambient (°C/W)
    float T_ambient;             // Ambient temperature (°C)
    float T_junction;            // Calculated junction temperature (°C)
} MOSFETStressRegen_t;

void calculate_mosfet_stress_regen(MOSFETStressRegen_t *mosfet) {
    // Conduction loss per MOSFET (assuming synchronous rectification)
    float P_cond = mosfet->R_ds_on * mosfet->I_rms_regen * mosfet->I_rms_regen;

    // Add switching loss (approximately)
    float P_switch = estimate_switching_loss(mosfet->I_rms_regen);
    float P_total = P_cond + P_switch;

    // Junction temperature rise
    float R_th_total = mosfet->R_th_jc + mosfet->R_th_ca;
    float delta_T = P_total * R_th_total;
    mosfet->T_junction = mosfet->T_ambient + delta_T;

    // Check against limits
    if (mosfet->T_junction > T_JUNCTION_MAX) {
        // Reduce regen power to protect MOSFETs
        reduce_regen_power();
        log_warning("MOSFET temperature limit during regen");
    }
}

// Key insight: Regen can stress inverter as much as motoring!
// Even though power flows "backwards", losses are similar
```

**2. DC Link Capacitor Stress:**

```c
typedef struct {
    float I_ripple_rms;          // RMS ripple current (A)
    float I_ripple_rated;        // Rated ripple current (A)
    float V_dc_peak;             // Peak DC voltage (V)
    float V_rated;               // Rated voltage (V)
    float T_capacitor;           // Capacitor temperature (°C)
    float stress_factor;         // Stress factor (0-1)
} CapacitorStressRegen_t;

void calculate_capacitor_stress_regen(CapacitorStressRegen_t *cap) {
    // Current stress
    float current_stress = cap->I_ripple_rms / cap->I_ripple_rated;

    // Voltage stress
    float voltage_stress = cap->V_dc_peak / cap->V_rated;

    // Temperature stress (Arrhenius relationship)
    float T_rated = 105.0f;  // °C
    float temp_stress = powf(2.0f, (cap->T_capacitor - T_rated) / 10.0f);

    // Combined stress factor
    cap->stress_factor = current_stress * voltage_stress * temp_stress;

    // Lifetime impact
    // L_actual = L_rated / stress_factor
    // If stress_factor = 2.0, lifetime halved

    if (cap->stress_factor > 1.5f) {
        log_warning("High capacitor stress during regen: %.2f", cap->stress_factor);
    }
}

// Critical insight: Regen causes voltage spikes on DC link
// Must ensure capacitor voltage rating has adequate margin
// Typical: 450V cap for 400V system (12.5% margin minimum)
```

**3. Battery Pack Stress:**

```c
typedef struct {
    float I_charge_current;      // Charging current (A)
    float I_charge_max;          // Maximum continuous charge current (A)
    float C_rate;                // Charge C-rate
    float V_cell_max;            // Maximum cell voltage (V)
    float T_cell;                // Cell temperature (°C)
    bool stress_warning;         // Stress warning flag
} BatteryStressRegen_t;

void calculate_battery_stress_regen(BatteryStressRegen_t *batt) {
    // Calculate C-rate
    batt->C_rate = batt->I_charge_current / BATTERY_CAPACITY_AH;

    // Check against maximum continuous C-rate
    // Most EV batteries: 1C to 2C charge rate max
    if (batt->C_rate > 1.5f) {
        batt->stress_warning = true;
        log_warning("High battery charge rate during regen: %.2f C", batt->C_rate);

        // Reduce regen to protect battery
        reduce_regen_power();
    }

    // Check cell voltage
    // During regen, cell voltage rises: V_cell = V_ocv + I*R_internal
    float V_cell_during_charge = calculate_cell_voltage(batt->I_charge_current);
    if (V_cell_during_charge > batt->V_cell_max) {
        // Approaching overvoltage - must reduce regen
        reduce_regen_power();
        log_warning("Cell voltage approaching limit during regen");
    }

    // Temperature check
    if (batt->T_cell < 0.0f) {
        // Cold charging is harmful - severely limit regen
        float temp_factor = fmaxf(0.0f, batt->T_cell / 10.0f);
        limit_regen_power(temp_factor);
    }
}
```

#### Practical Design Implications

**1. Inverter Design for Regeneration:**

```c
// Key design parameters for regen-capable inverter
typedef struct {
    // MOSFET selection
    float V_ds_rating;           // Drain-source voltage rating (V)
    float I_d_continuous;        // Continuous drain current (A)
    float R_ds_on;               // On-resistance (Ω)

    // DC link design
    float C_dc_link;             // DC link capacitance (F)
    float V_cap_rating;          // Capacitor voltage rating (V)
    float I_ripple_rating;       // Capacitor ripple current rating (A)

    // Protection circuits
    float V_overvoltage_trip;    // Overvoltage protection threshold (V)
    float I_overcurrent_trip;    // Overcurrent protection threshold (A)
    bool dynamic_brake_resistor; // DBR present for voltage clamping

    // Thermal design
    float R_th_heatsink;         // Heatsink thermal resistance (°C/W)
    float airflow_cfm;           // Cooling airflow (CFM)
} InverterDesignRegen_t;

void design_inverter_for_regen(InverterDesignRegen_t *design,
                                float P_regen_max,
                                float V_battery_max) {
    // 1. MOSFET voltage rating
    // Must handle: V_battery + V_regen_spike + safety margin
    float V_regen_spike = 50.0f;  // Expected voltage rise during regen
    float safety_margin = 1.25f;   // 25% margin
    design->V_ds_rating = (V_battery_max + V_regen_spike) * safety_margin;
    // Example: (400V + 50V) * 1.25 = 562.5V → Use 650V MOSFETs

    // 2. MOSFET current rating
    float I_regen_peak = P_regen_max / V_battery_max;
    design->I_d_continuous = I_regen_peak * 1.5f;  // 50% margin
    // Example: 50kW / 400V * 1.5 = 188A → Use 200A MOSFETs

    // 3. DC link capacitance
    design->C_dc_link = calculate_required_capacitance(
        P_regen_max,
        P_BATTERY_MAX,
        V_battery_max,
        30.0f,  // Max 30V ripple allowed
        0.05f   // 50ms battery response time
    );

    // 4. Overvoltage protection
    design->V_overvoltage_trip = V_battery_max * 1.15f;  // 15% above nominal
    // Must trip before MOSFET V_ds_max

    // 5. Dynamic brake resistor sizing (if used)
    if (design->dynamic_brake_resistor) {
        // DBR must dissipate full regen power if battery full
        float R_dbr = (V_battery_max * V_battery_max) / P_regen_max;
        float P_dbr_rating = P_regen_max * 1.2f;  // 20% margin

        log_info("DBR requirements: %.1f Ω, %.1f kW rating",
                 R_dbr, P_dbr_rating / 1000.0f);
    }
}
```

**2. System-Level Design Considerations:**

```
Design Checklist for Regen-Capable System:
═══════════════════════════════════════════════════════════

☐ MOSFETs rated for peak regen voltage (V_batt + spike + margin)
☐ Body diodes rated for continuous regen current
☐ DC link capacitor sized for regen power buffering
☐ Capacitor ESR low enough for acceptable ripple
☐ Overvoltage protection set correctly (before MOSFET damage)
☐ Battery BMS can communicate regen limits to motor controller
☐ Dynamic brake resistor included (for high-power systems)
☐ Gate drive optimized for synchronous rectification
☐ Dead-time adjusted to prevent shoot-through during regen
☐ Thermal management adequate for continuous regen
☐ Current sensors have bipolar range (+ and - currents)
☐ DC link precharge circuit handles reverse current
☐ Software includes all regen limiting strategies
☐ Fault detection for regen-specific failures
☐ Testing performed with full battery (worst case)
```

#### Measurement and Monitoring

**Critical Parameters to Monitor:**

```c
typedef struct {
    // Voltage monitoring
    float V_dc_bus;              // DC bus voltage (V)
    float V_dc_min;              // Minimum during regen (V)
    float V_dc_max;              // Maximum during regen (V)
    float V_battery;             // Battery voltage (V)

    // Current monitoring
    float I_phase_a;             // Phase A current (A)
    float I_phase_b;             // Phase B current (A)
    float I_phase_c;             // Phase C current (A)
    float I_dc_bus;              // DC bus current (A)
    float I_battery;             // Battery current (A)

    // Power flow
    float P_motor;               // Motor electrical power (W)
    float P_dc_bus;              // DC bus power (W)
    float P_battery;             // Battery power (W)

    // Efficiency metrics
    float efficiency_motor;      // Motor efficiency (%)
    float efficiency_inverter;   // Inverter efficiency (%)
    float efficiency_total;      // Total regen efficiency (%)

    // Diagnostic flags
    bool regen_active;           // Regen currently active
    bool voltage_limiting;       // Voltage limiting active
    bool current_limiting;       // Current limiting active
    bool temperature_limiting;   // Temperature limiting active
} RegenMonitoring_t;

void update_regen_monitoring(RegenMonitoring_t *mon) {
    // Real-time monitoring during regen

    // Voltage check
    if (mon->V_dc_bus > mon->V_dc_max) {
        mon->V_dc_max = mon->V_dc_bus;
    }

    // Power flow verification
    // In regen: P_motor < 0, I_battery < 0 (charging)
    if (mon->P_motor < 0.0f && mon->I_battery < 0.0f) {
        mon->regen_active = true;

        // Calculate efficiencies
        mon->efficiency_motor = (mon->P_dc_bus / mon->P_motor) * 100.0f;
        mon->efficiency_inverter = (mon->P_battery / mon->P_dc_bus) * 100.0f;
        mon->efficiency_total = (mon->P_battery / mon->P_motor) * 100.0f;
    }
    else {
        mon->regen_active = false;
    }

    // Log anomalies
    if (mon->regen_active && mon->efficiency_total < 70.0f) {
        log_warning("Low regen efficiency: %.1f%%", mon->efficiency_total);
    }
}
```

---

### References for Section 2.6:

**Books:**
1. *"Power Electronics: Converters, Applications, and Design"* by Ned Mohan et al. - Chapter 8 (DC-AC Inverters and Rectifiers)
2. *"Advanced Electric Drive Vehicles"* by Ali Emadi - Chapter 4 (Power Electronics for Electric Drives)
3. *"Pulse Width Modulated DC-AC Converters"* by Dorin O. Neacsu - Chapter 6 (Regenerative Operation)

**Papers:**
1. Takahashi, I., & Noguchi, T. (1986). "A New Quick-Response and High-Efficiency Control Strategy of an Induction Motor." IEEE Transactions on Industry Applications, IA-22(5), 820-827.
2. Pitel, I. J., & Talukdar, S. N. (1982). "Characterization of Programmed-Waveform Pulsewidth Modulation." IEEE Transactions on Industry Applications, IA-18(6), 707-715.

**Application Notes:**
1. **Infineon**: "Understanding Power Losses in MOSFETs" (AN2015-07)
2. **Texas Instruments**: "Synchronous Rectification in Motor Drives" (SLVA662)
3. **ON Semiconductor**: "IGBT or MOSFET: Choose Wisely for Inverter Applications" (AND9083/D)
4. **Vishay**: "DC-Link Capacitor Selection for Motor Drive Applications" (Application Note)
5. **STMicroelectronics**: "Dead-Time Compensation in Field Oriented Control" (AN4863)

**Standards:**
1. **IEC 61800-5-1**: "Adjustable Speed Electrical Power Drive Systems - Part 5-1: Safety Requirements - Electrical, Thermal and Energy"
2. **UL 1741**: "Inverters, Converters, Controllers and Interconnection System Equipment for Use with Distributed Energy Resources"

---

**End of Section 2: Regenerative Braking Control**

---

## Section 3: Braking Strategy and Blending

Modern electric vehicles must intelligently coordinate regenerative and friction braking to maximize energy recovery while ensuring safe, predictable, and comfortable braking performance. This section covers the algorithmic strategies for brake blending and the factors that determine when to use electrical vs mechanical braking.

---

### 3.1 Braking System Architecture and Requirements

#### System Components

A complete EV braking system consists of:

1. **Regenerative Braking System**
   - Motor/generator
   - Inverter
   - Battery pack
   - Energy management controller

2. **Friction Braking System**
   - Hydraulic brake system
   - Brake calipers and pads
   - Brake master cylinder
   - Electronic brake actuator (for blending)

3. **Brake Blending Controller**
   - Sensor inputs (pedal, speed, SOC, etc.)
   - Blending algorithm
   - Actuator outputs (motor torque, hydraulic pressure)

4. **Safety Systems**
   - Anti-lock Braking System (ABS)
   - Electronic Stability Control (ESC)
   - Brake-by-Wire redundancy

#### Functional Requirements

**1. Energy Recovery:**
- Maximize regenerative braking usage to recover energy
- Typical target: 70-80% of normal braking events using regen only
- Maximize overall system efficiency

**2. Deceleration Performance:**
```c
// Typical deceleration requirements
#define DECEL_LIGHT_BRAKING    -2.0f  // m/s² (normal braking)
#define DECEL_MODERATE_BRAKING -4.0f  // m/s² (firm braking)
#define DECEL_EMERGENCY_BRAKING -9.0f // m/s² (emergency stop)
#define DECEL_MAX_ACHIEVABLE   -10.0f // m/s² (with ABS)
```

**3. Brake Feel and Consistency:**
- Consistent pedal feel regardless of regen availability
- Linear relationship between pedal travel and deceleration
- No sudden transitions or "grabbiness"
- Target variation: < ±5% deceleration for same pedal position

**4. Response Time:**
```c
// Response time requirements
#define BRAKE_RESPONSE_TIME_MAX     150   // ms (from pedal to initial response)
#define REGEN_TO_FRICTION_BLEND    100   // ms (transition time)
#define EMERGENCY_BRAKE_RESPONSE    50    // ms (emergency mode)
```

**5. Safety and Redundancy:**
- Friction brakes must always be available
- System must fail-safe to friction brakes
- ABS integration for wheel slip control
- Independent braking on all four wheels

#### Braking Power Budget

Calculate total braking power available from each source:

```c
typedef struct {
    float P_regen_max;          // Max regen power available (W)
    float P_friction_max;       // Max friction brake power (W)
    float vehicle_mass;         // Vehicle mass (kg)
    float vehicle_speed;        // Current speed (m/s)
    float decel_required;       // Required deceleration (m/s²)
} BrakingPowerBudget_t;

void calculate_braking_power_budget(BrakingPowerBudget_t *budget) {
    // Total power to dissipate
    float P_total_required = budget->vehicle_mass *
                            fabsf(budget->decel_required) *
                            budget->vehicle_speed;

    // Check if regen alone is sufficient
    if (P_total_required <= budget->P_regen_max) {
        // Regen-only braking
        budget->P_regen_max = P_total_required;
        budget->P_friction_max = 0.0f;
    }
    else {
        // Blended braking needed
        // Use max regen, supplement with friction
        float P_friction_needed = P_total_required - budget->P_regen_max;
        budget->P_friction_max = P_friction_needed;
    }
}
```

#### Brake System States

```c
typedef enum {
    BRAKE_IDLE,                 // No braking
    BRAKE_REGEN_ONLY,           // Regen braking only
    BRAKE_BLENDED,              // Regen + friction blending
    BRAKE_FRICTION_ONLY,        // Friction brakes only
    BRAKE_EMERGENCY,            // Emergency braking mode
    BRAKE_ABS_ACTIVE,           // ABS engaged
    BRAKE_FAULT                 // System fault, friction only
} BrakeSystemState_t;

typedef struct {
    BrakeSystemState_t state;
    BrakeSystemState_t prev_state;
    float time_in_state;        // Time in current state (s)
    bool transition_pending;    // State transition in progress
} BrakeStateMachine_t;
```

---

### References for Section 3.1:

**Books:**
1. *"Brake Design and Safety"* by Rudolf Limpert - Chapter 12 (Electric and Hybrid Vehicle Braking)
2. *"Automotive Control Systems"* by Uwe Kiencke and Lars Nielsen - Chapter 9 (Brake Control)

**Standards:**
1. **FMVSS 135**: "Light Vehicle Brake Systems" (US Federal Motor Vehicle Safety Standards)
2. **ECE R13**: "Uniform Provisions Concerning the Approval of Vehicles with Regard to Braking"
3. **ISO 26262**: "Road Vehicles - Functional Safety" (brake system safety requirements)

**Papers:**
1. Ko, J., et al. (2015). "Development of Brake System and Regenerative Braking Cooperative Control Algorithm for Automatic-Transmission-Based Hybrid Electric Vehicles." IEEE Transactions on Vehicular Technology, 64(2), 431-440.

---

### 3.2 Algorithmic Strategies for Brake Blending

This section presents different algorithmic approaches for coordinating regenerative and friction braking.

#### Strategy 1: Sequential Blending (Simple)

Use regen first, then add friction brakes when regen capacity is exceeded.

```c
typedef struct {
    float decel_target;         // Target deceleration (m/s²)
    float decel_regen_max;      // Max decel from regen (m/s²)
    float decel_actual;         // Actual applied deceleration (m/s²)
    float force_regen;          // Regen braking force (N)
    float force_friction;       // Friction braking force (N)
} SequentialBlending_t;

void sequential_blending_strategy(SequentialBlending_t *blend, float vehicle_mass) {
    // Convert target deceleration to force
    float force_total = vehicle_mass * fabsf(blend->decel_target);

    // 1. Apply regen up to its maximum
    float force_regen_max = vehicle_mass * blend->decel_regen_max;

    if (force_total <= force_regen_max) {
        // Regen-only braking sufficient
        blend->force_regen = force_total;
        blend->force_friction = 0.0f;
    }
    else {
        // Regen at max, add friction for remainder
        blend->force_regen = force_regen_max;
        blend->force_friction = force_total - force_regen_max;
    }

    // Calculate actual deceleration
    blend->decel_actual = -(blend->force_regen + blend->force_friction) / vehicle_mass;
}
```

**Advantages:**
- Simple to implement
- Maximizes energy recovery
- Clear priority: regen first

**Disadvantages:**
- Transition point when friction engages can be felt by driver
- Not optimal for brake feel
- Step change in brake characteristics

---

#### Strategy 2: Proportional Blending

Maintain a constant ratio between regen and friction throughout the braking range.

```c
typedef struct {
    float regen_proportion;     // Proportion of braking from regen (0-1)
    float decel_target;         // Target deceleration (m/s²)
    float force_regen;          // Regen braking force (N)
    float force_friction;       // Friction braking force (N)
} ProportionalBlending_t;

void proportional_blending_strategy(ProportionalBlending_t *blend,
                                     float vehicle_mass,
                                     float regen_capability) {
    // Total braking force needed
    float force_total = vehicle_mass * fabsf(blend->decel_target);

    // Determine proportion based on regen capability
    // regen_capability = 0.0 to 1.0 (e.g., based on SOC, temp, speed)
    blend->regen_proportion = regen_capability;

    // Apply proportion
    blend->force_regen = force_total * blend->regen_proportion;
    blend->force_friction = force_total * (1.0f - blend->regen_proportion);
}
```

**Advantages:**
- Smooth brake feel
- No transition points
- Consistent pedal feel as regen capability changes

**Disadvantages:**
- Less energy recovery than sequential
- Friction brakes engaged even for light braking
- More friction brake wear

---

#### Strategy 3: Adaptive Blending (Recommended)

Intelligently blend based on multiple factors: deceleration level, speed, SOC, temperature, and driver preference.

```c
typedef struct {
    // Inputs
    float decel_target;         // Target deceleration (m/s²)
    float decel_rate;           // Rate of change of deceleration (m/s³)
    float vehicle_speed;        // Vehicle speed (m/s)
    float SOC;                  // Battery SOC (0-1)
    float T_motor;              // Motor temperature (°C)
    float T_battery;            // Battery temperature (°C)
    RegenLevel_t regen_level;   // User-selected regen level

    // System capabilities
    float decel_regen_max;      // Max decel from regen (m/s²)
    float decel_friction_max;   // Max decel from friction (m/s²)

    // Outputs
    float force_regen;          // Regen braking force (N)
    float force_friction;       // Friction braking force (N)
    float blend_factor;         // Regen proportion (0-1)
} AdaptiveBlending_t;

void adaptive_blending_strategy(AdaptiveBlending_t *blend, float vehicle_mass) {
    // Calculate total force needed
    float force_total = vehicle_mass * fabsf(blend->decel_target);

    // 1. Determine regen capability based on multiple factors
    float regen_capability = 1.0f;

    // Factor 1: SOC-based limiting
    if (blend->SOC > 0.95f) {
        regen_capability *= (0.98f - blend->SOC) / 0.03f;  // Linear taper
    }

    // Factor 2: Speed-based limiting (low speed)
    if (blend->vehicle_speed < 5.0f) {  // Below 5 m/s (18 km/h)
        regen_capability *= (blend->vehicle_speed / 5.0f);
    }

    // Factor 3: Temperature-based limiting
    if (blend->T_motor > 120.0f) {
        regen_capability *= fmaxf(0.5f, (150.0f - blend->T_motor) / 30.0f);
    }

    // Factor 4: User preference (regen level)
    // Already captured in decel_regen_max

    // 2. Categorize braking intensity
    float decel_magnitude = fabsf(blend->decel_target);

    if (decel_magnitude < 2.0f) {
        // *** LIGHT BRAKING (< 2 m/s²) ***
        // Use regen-only if possible
        float force_regen_available = vehicle_mass * blend->decel_regen_max *
                                      regen_capability;

        if (force_total <= force_regen_available) {
            // Regen-only
            blend->force_regen = force_total;
            blend->force_friction = 0.0f;
            blend->blend_factor = 1.0f;
        }
        else {
            // Light blending
            blend->force_regen = force_regen_available;
            blend->force_friction = force_total - force_regen_available;
            blend->blend_factor = force_regen_available / force_total;
        }
    }
    else if (decel_magnitude < 5.0f) {
        // *** MODERATE BRAKING (2-5 m/s²) ***
        // Blend to maintain consistent feel
        float force_regen_available = vehicle_mass * blend->decel_regen_max *
                                      regen_capability;

        // Use 80% regen / 20% friction for smooth feel
        float target_regen_proportion = 0.80f;

        blend->force_regen = fminf(force_total * target_regen_proportion,
                                  force_regen_available);
        blend->force_friction = force_total - blend->force_regen;
        blend->blend_factor = blend->force_regen / force_total;
    }
    else {
        // *** HARD BRAKING (> 5 m/s²) or EMERGENCY ***
        // Prioritize deceleration, use all available braking
        float force_regen_available = vehicle_mass * blend->decel_regen_max *
                                      regen_capability;

        blend->force_regen = force_regen_available;
        blend->force_friction = force_total - force_regen_available;

        // Ensure we don't exceed friction brake capacity
        float force_friction_max = vehicle_mass * blend->decel_friction_max;
        if (blend->force_friction > force_friction_max) {
            blend->force_friction = force_friction_max;
        }

        blend->blend_factor = blend->force_regen / (blend->force_regen + blend->force_friction);

        // Detect emergency braking (very high decel rate)
        if (fabsf(blend->decel_rate) > 10.0f) {  // m/s³
            // Emergency: immediately apply maximum friction
            blend->force_friction = force_friction_max;
        }
    }
}
```

---

#### Strategy 4: Predictive Blending

Use vehicle sensors and driver behavior to predict braking needs and preemptively adjust blending strategy.

```c
typedef struct {
    float distance_to_obstacle;     // Distance to vehicle/obstacle ahead (m)
    float relative_velocity;        // Closing velocity (m/s)
    float brake_pedal_velocity;     // Rate of pedal application (1/s)
    float time_to_collision;        // Calculated TTC (s)
    bool prediction_active;         // Predictive mode active
    float predicted_decel;          // Predicted deceleration need (m/s²)
} PredictiveBlending_t;

void predictive_blending_strategy(PredictiveBlending_t *pred) {
    // 1. Calculate time to collision
    if (pred->distance_to_obstacle > 0.1f && pred->relative_velocity > 0.1f) {
        pred->time_to_collision = pred->distance_to_obstacle / pred->relative_velocity;
    }
    else {
        pred->time_to_collision = 999.0f;  // No imminent collision
    }

    // 2. Analyze brake pedal application rate
    // Fast pedal application → likely emergency
    bool emergency_intent = (pred->brake_pedal_velocity > 5.0f);  // Fast application

    // 3. Predict required deceleration
    if (pred->time_to_collision < 3.0f || emergency_intent) {
        // Predict aggressive braking needed
        pred->prediction_active = true;

        // Estimate required deceleration to stop in time
        // Using kinematic equation: v² = v₀² + 2*a*d
        float current_speed = get_vehicle_speed();
        float stopping_distance = pred->distance_to_obstacle - 2.0f;  // 2m safety margin

        if (stopping_distance > 0.1f) {
            pred->predicted_decel = (current_speed * current_speed) /
                                   (2.0f * stopping_distance);
        }
        else {
            pred->predicted_decel = 10.0f;  // Maximum
        }

        // Preemptively allocate more to friction brakes
        // This reduces regen proportion to ensure immediate response
    }
    else {
        pred->prediction_active = false;
        pred->predicted_decel = 0.0f;
    }
}
```

---

#### Strategy 5: Wheel-Individual Blending (Advanced)

For vehicles with individual wheel control, blend regen and friction per wheel for optimal performance and stability.

```c
typedef struct {
    float wheel_speed[4];           // Wheel speeds (rad/s): FL, FR, RL, RR
    float wheel_slip[4];            // Wheel slip ratios (0-1)
    float force_regen[4];           // Regen force per wheel (N)
    float force_friction[4];        // Friction force per wheel (N)
    float axle_torque_limit_front;  // Front axle torque limit (Nm)
    float axle_torque_limit_rear;   // Rear axle torque limit (Nm)
    bool abs_active[4];             // ABS active per wheel
} WheelIndividualBlending_t;

void wheel_individual_blending(WheelIndividualBlending_t *wib,
                                float total_force_required,
                                float vehicle_mass) {
    // 1. Calculate target force distribution
    // Typical: 60% front, 40% rear for FWD vehicle
    float force_front = total_force_required * 0.60f;
    float force_rear = total_force_required * 0.40f;

    // 2. Allocate regen to driven axle (rear in RWD example)
    float max_regen_force = (wib->axle_torque_limit_rear / WHEEL_RADIUS);

    // 3. Distribute regen on rear axle
    if (force_rear <= max_regen_force) {
        // Rear axle can handle all rear braking with regen
        wib->force_regen[2] = force_rear / 2.0f;  // Rear left
        wib->force_regen[3] = force_rear / 2.0f;  // Rear right
        wib->force_friction[2] = 0.0f;
        wib->force_friction[3] = 0.0f;
    }
    else {
        // Regen at max, add friction
        wib->force_regen[2] = max_regen_force / 2.0f;
        wib->force_regen[3] = max_regen_force / 2.0f;
        float friction_rear = force_rear - max_regen_force;
        wib->force_friction[2] = friction_rear / 2.0f;
        wib->force_friction[3] = friction_rear / 2.0f;
    }

    // 4. Front axle uses friction only (no regen)
    wib->force_regen[0] = 0.0f;  // Front left
    wib->force_regen[1] = 0.0f;  // Front right
    wib->force_friction[0] = force_front / 2.0f;
    wib->force_friction[1] = force_front / 2.0f;

    // 5. Check for wheel slip and adjust with ABS
    for (int i = 0; i < 4; i++) {
        if (wib->wheel_slip[i] > 0.15f) {  // >15% slip
            // ABS intervention - reduce braking on this wheel
            wib->abs_active[i] = true;

            // Reduce regen immediately
            wib->force_regen[i] *= 0.5f;

            // Modulate friction (ABS)
            wib->force_friction[i] *= (1.0f - wib->wheel_slip[i]);
        }
        else {
            wib->abs_active[i] = false;
        }
    }
}
```

---

### References for Section 3.2:

**Books:**
1. *"Vehicle Dynamics and Control"* by Rajesh Rajamani - Chapter 5 (Brake Control Systems)
2. *"Electric and Hybrid Vehicles: Technologies, Modeling and Control"* by Amir Khajepour et al. - Chapter 7 (Brake Blending Strategies)

**Papers:**
1. Kim, J., & Park, Y. (2018). "Novel Regenerative Braking Cooperative Control Method for Front-Wheel-Drive Hybrid Electric Vehicles Based on Adaptive Regenerative Brake Torque Optimization." IEEE Access, 6, 63033-63049.
2. Zhang, J., et al. (2020). "Regenerative Braking Control Strategy for Electric Vehicles Based on Optimization of Switched Reluctance Generator Drive." IEEE/ASME Transactions on Mechatronics, 25(1), 279-288.

**Application Notes:**
1. **Bosch**: "Electro-Hydraulic Brake Systems for Hybrid and Electric Vehicles" (Technical White Paper)
2. **Continental**: "Integrated Brake Control for Electric Vehicles" (2021)

---

### 3.3 Brake Feel and User Experience

Creating natural, confidence-inspiring brake feel is critical for user acceptance. This section covers the factors that influence brake feel and how to optimize the user experience.

#### Brake Pedal Feel Requirements

**Key Characteristics:**

1. **Linearity**: Proportional relationship between pedal travel/force and deceleration
2. **Progressiveness**: Gradual increase in braking force
3. **Consistency**: Same pedal position → same deceleration
4. **Feedback**: Driver can feel what the brakes are doing

**Pedal Travel and Force Curves:**

```c
typedef struct {
    float pedal_position;       // Pedal position (0-1)
    float pedal_force;          // Applied force (N)
    float decel_target;         // Target deceleration (m/s²)
    float pedal_feel_curve_exp; // Exponential curve factor
} BrakePedalFeel_t;

float map_pedal_to_deceleration(BrakePedalFeel_t *pedal) {
    // Option 1: Linear mapping
    // decel = pedal_position * MAX_DECEL

    // Option 2: Progressive mapping (preferred)
    // Provides fine control at light braking, more aggressive at high pedal
    float normalized = pedal->pedal_position;

    // Apply curve (exponential for progressiveness)
    float curve_factor = pedal->pedal_feel_curve_exp;  // Typically 1.5-2.0
    normalized = powf(normalized, curve_factor);

    // Map to deceleration range
    float decel_range = DECEL_MAX_ACHIEVABLE - DECEL_LIGHT_BRAKING;
    pedal->decel_target = -(DECEL_LIGHT_BRAKING + normalized * decel_range);

    return pedal->decel_target;
}
```

**Pedal Feel Compensation:**

When regen availability changes (e.g., battery full), the system must compensate to maintain consistent pedal feel:

```c
typedef struct {
    float decel_target;             // Target deceleration (m/s²)
    float decel_actual;             // Actual measured deceleration (m/s²)
    float decel_error;              // Error between target and actual
    float friction_compensation;    // Additional friction force needed (N)
} PedalFeelCompensation_t;

void compensate_pedal_feel(PedalFeelCompensation_t *comp, float dt) {
    // Calculate deceleration error
    comp->decel_error = comp->decel_target - comp->decel_actual;

    // PI controller for compensation
    static float integral = 0.0f;
    float Kp = 500.0f;  // Proportional gain (N/(m/s²))
    float Ki = 100.0f;  // Integral gain (N/(m/s²*s))

    integral += comp->decel_error * dt;
    integral = fmaxf(-1000.0f, fminf(1000.0f, integral));  // Anti-windup

    // Calculate additional friction force needed
    comp->friction_compensation = Kp * comp->decel_error + Ki * integral;

    // Clamp to reasonable limits
    comp->friction_compensation = fmaxf(0.0f, fminf(5000.0f, comp->friction_compensation));
}
```

#### Blending Transparency

The driver should not notice transitions between regen and friction braking.

**Smooth Transition Algorithm:**

```c
#define BLEND_TRANSITION_TIME  0.3f  // seconds

typedef struct {
    float force_regen_current;      // Current regen force (N)
    float force_regen_target;       // Target regen force (N)
    float force_friction_current;   // Current friction force (N)
    float force_friction_target;    // Target friction force (N)
    float transition_rate;          // Max rate of change (N/s)
} BlendTransition_t;

void smooth_blend_transition(BlendTransition_t *trans, float dt) {
    // Calculate maximum change per timestep
    float max_change = trans->transition_rate * dt;

    // Smooth regen force transition
    float regen_error = trans->force_regen_target - trans->force_regen_current;
    if (fabsf(regen_error) < max_change) {
        trans->force_regen_current = trans->force_regen_target;
    }
    else {
        trans->force_regen_current += (regen_error > 0.0f) ? max_change : -max_change;
    }

    // Smooth friction force transition
    float friction_error = trans->force_friction_target - trans->force_friction_current;
    if (fabsf(friction_error) < max_change) {
        trans->force_friction_current = trans->force_friction_target;
    }
    else {
        trans->force_friction_current += (friction_error > 0.0f) ? max_change : -max_change;
    }

    // Ensure total force remains constant during transition
    // This is key to transparency
    float force_total_target = trans->force_regen_target + trans->force_friction_target;
    float force_total_current = trans->force_regen_current + trans->force_friction_current;

    if (fabsf(force_total_current - force_total_target) > 10.0f) {
        // Adjust friction to maintain total force
        trans->force_friction_current = force_total_target - trans->force_regen_current;
    }
}
```

#### Brake Noise, Vibration, and Harshness (NVH)

Regen braking is quieter and smoother than friction braking, which can affect user perception:

```c
typedef struct {
    float regen_proportion;         // Proportion of regen (0-1)
    float nvh_score;                // NVH quality score (0-1, higher=better)
    bool low_speed_creep;           // Enable creep at low speed
    float creep_torque;             // Creep torque (Nm)
} BrakeNVH_t;

void optimize_brake_nvh(BrakeNVH_t *nvh, float vehicle_speed) {
    // 1. At low speeds, reduce regen to avoid NVH issues
    if (vehicle_speed < 2.0f) {  // Below 2 m/s (7.2 km/h)
        // Blend to friction for smoother stop
        nvh->regen_proportion *= (vehicle_speed / 2.0f);

        // Enable creep torque for natural feel (like auto transmission)
        if (vehicle_speed < 0.5f && nvh->low_speed_creep) {
            nvh->creep_torque = 20.0f;  // Nm
        }
    }

    // 2. Calculate NVH quality score
    // More regen = better NVH (quieter, smoother)
    nvh->nvh_score = 0.6f + 0.4f * nvh->regen_proportion;
}
```

#### Driver Confidence and Training

```c
typedef struct {
    bool first_drive;               // First time driver
    uint32_t brake_events;          // Number of brake events
    float avg_regen_usage;          // Average regen usage (0-1)
    bool coaching_enabled;          // Enable coaching mode
} BrakeCoaching_t;

void brake_coaching_system(BrakeCoaching_t *coaching) {
    // Track driver behavior
    coaching->brake_events++;

    // After initial learning period, provide feedback
    if (coaching->brake_events > 100 && coaching->coaching_enabled) {
        // Analyze usage patterns
        if (coaching->avg_regen_usage < 0.5f) {
            // Driver not using regen effectively
            display_message("Tip: Release throttle earlier to maximize energy recovery");
        }
    }
}
```

---

### References for Section 3.3:

**Books:**
1. *"Automotive Ergonomics: Driver-Vehicle Interaction"* by Heiner Bubb et al. - Chapter 8 (Pedal Design and Feel)
2. *"Vehicle Handling Dynamics"* by Masato Abe - Chapter 11 (Brake Feel Engineering)

**Papers:**
1. Heydari, S., et al. (2016). "Evaluation of Regenerative Braking Effect on Brake Pedal Feel in Electric Vehicles." SAE International Journal of Alternative Powertrains, 5(1), 160-171.
2. Wei, Z., et al. (2017). "Modeling and Testing of Electro-Mechanical Brake System for Vehicle Control Research." IFAC-PapersOnLine, 50(1), 13137-13142.

**Standards:**
1. **ISO 2575**: "Road Vehicles - Symbols for Controls, Indicators and Tell-Tales"
2. **SAE J2954**: User interface guidelines for electric vehicle features

---

### 3.4 Safety and Fault Tolerance

Brake systems are safety-critical. The blending controller must handle faults gracefully and always ensure the vehicle can stop safely.

#### Fault Detection and Management

```c
typedef enum {
    BRAKE_FAULT_NONE = 0,
    BRAKE_FAULT_REGEN_SENSOR,       // Regen sensor failure
    BRAKE_FAULT_REGEN_ACTUATOR,     // Cannot control motor
    BRAKE_FAULT_FRICTION_SENSOR,    // Friction brake sensor failure
    BRAKE_FAULT_FRICTION_ACTUATOR,  // Hydraulic actuator failure
    BRAKE_FAULT_PEDAL_SENSOR,       // Pedal sensor failure
    BRAKE_FAULT_COMMUNICATION,      // CAN communication loss
    BRAKE_FAULT_POWER_LOSS          // Loss of electrical power
} BrakeFaultType_t;

typedef struct {
    BrakeFaultType_t active_fault;
    bool fault_detected;
    uint32_t fault_timestamp;
    bool safe_mode_active;
    float degraded_capability;      // Braking capability (0-1)
} BrakeFaultManager_t;

void handle_brake_fault(BrakeFaultManager_t *fault) {
    switch (fault->active_fault) {
        case BRAKE_FAULT_REGEN_SENSOR:
        case BRAKE_FAULT_REGEN_ACTUATOR:
            // Regen unavailable - use friction only
            disable_regen_braking();
            set_friction_brake_mode(FRICTION_MODE_FULL);
            fault->degraded_capability = 1.0f;  // Full capability
            fault->safe_mode_active = true;
            display_warning("Regenerative braking unavailable");
            break;

        case BRAKE_FAULT_FRICTION_SENSOR:
            // Friction sensor fault - use regen primary, friction backup
            set_regen_mode(REGEN_MODE_PRIMARY);
            enable_friction_brake_backup();
            fault->degraded_capability = 0.8f;  // Reduced capability
            fault->safe_mode_active = true;
            display_warning("Brake system degraded - service required");
            break;

        case BRAKE_FAULT_FRICTION_ACTUATOR:
            // Critical fault - friction actuator failed
            // Use regen only but limit vehicle speed
            disable_friction_braking();
            set_regen_mode(REGEN_MODE_ONLY);
            limit_vehicle_speed(50.0f);  // km/h
            fault->degraded_capability = 0.5f;  // Significantly reduced
            fault->safe_mode_active = true;
            display_critical_warning("BRAKE FAULT - REDUCE SPEED");
            break;

        case BRAKE_FAULT_PEDAL_SENSOR:
            // Pedal sensor fault - use redundant sensor or limp mode
            if (pedal_sensor_redundant_available()) {
                switch_to_redundant_pedal_sensor();
                fault->degraded_capability = 1.0f;
                fault->safe_mode_active = false;
            }
            else {
                // Limp mode - fixed braking strategy
                enable_limp_mode_braking();
                fault->degraded_capability = 0.6f;
                fault->safe_mode_active = true;
                display_critical_warning("BRAKE PEDAL FAULT - SERVICE IMMEDIATELY");
            }
            break;

        case BRAKE_FAULT_COMMUNICATION:
            // CAN communication loss
            // Revert to local control, disable coordinated functions
            set_brake_mode(BRAKE_MODE_LOCAL_CONTROL);
            disable_abs();
            disable_stability_control();
            fault->degraded_capability = 0.9f;
            fault->safe_mode_active = true;
            display_warning("Communication fault - advanced features unavailable");
            break;

        case BRAKE_FAULT_POWER_LOSS:
            // Loss of 12V power
            // Hydraulic brakes must function mechanically
            ensure_mechanical_brake_linkage();
            disable_all_electronic_braking();
            fault->degraded_capability = 0.7f;  // Manual braking only
            fault->safe_mode_active = true;
            display_critical_warning("POWER LOSS - MECHANICAL BRAKES ONLY");
            break;

        case BRAKE_FAULT_NONE:
        default:
            fault->safe_mode_active = false;
            fault->degraded_capability = 1.0f;
            break;
    }
}
```

#### Redundancy and Fail-Safe Design

```c
typedef struct {
    bool pedal_sensor_primary_ok;
    bool pedal_sensor_secondary_ok;
    bool friction_circuit_1_ok;
    bool friction_circuit_2_ok;
    bool regen_available;
    bool abs_available;
    uint8_t safety_level;           // 0=Critical, 1=Degraded, 2=Normal
} BrakeRedundancy_t;

void evaluate_brake_safety_level(BrakeRedundancy_t *redundancy) {
    // Evaluate system health
    bool pedal_ok = redundancy->pedal_sensor_primary_ok ||
                   redundancy->pedal_sensor_secondary_ok;
    bool friction_ok = redundancy->friction_circuit_1_ok &&
                      redundancy->friction_circuit_2_ok;

    // Determine safety level
    if (pedal_ok && friction_ok) {
        if (redundancy->regen_available && redundancy->abs_available) {
            redundancy->safety_level = 2;  // Normal operation
        }
        else {
            redundancy->safety_level = 1;  // Degraded (friction OK, regen/ABS out)
        }
    }
    else if (pedal_ok && (redundancy->friction_circuit_1_ok ||
                          redundancy->friction_circuit_2_ok)) {
        redundancy->safety_level = 1;  // Degraded (one friction circuit out)
        display_warning("Brake circuit fault - service required");
    }
    else {
        redundancy->safety_level = 0;  // Critical fault
        display_critical_warning("CRITICAL BRAKE FAULT - STOP VEHICLE SAFELY");
        trigger_hazard_lights();
    }
}
```

#### Integration with ABS/ESC

```c
typedef struct {
    bool abs_active;
    bool esc_active;
    float wheel_slip[4];            // Per-wheel slip ratio (0-1)
    float yaw_rate_error;           // Yaw rate error (rad/s)
    float regen_force_limit;        // Regen force limit from ABS/ESC (N)
} ABSESCIntegration_t;

void integrate_abs_esc_with_regen(ABSESCIntegration_t *abs_esc) {
    // 1. Check if ABS is active on any wheel
    abs_esc->abs_active = false;
    for (int i = 0; i < 4; i++) {
        if (abs_esc->wheel_slip[i] > 0.15f) {  // >15% slip
            abs_esc->abs_active = true;
            break;
        }
    }

    // 2. If ABS active, reduce/eliminate regen
    if (abs_esc->abs_active) {
        // Regen can interfere with ABS wheel speed control
        // Reduce regen proportionally to worst wheel slip
        float max_slip = 0.0f;
        for (int i = 0; i < 4; i++) {
            if (abs_esc->wheel_slip[i] > max_slip) {
                max_slip = abs_esc->wheel_slip[i];
            }
        }

        // Scale regen based on slip
        float regen_scale = fmaxf(0.0f, 1.0f - (max_slip / 0.20f));
        abs_esc->regen_force_limit *= regen_scale;

        log_event("ABS active - regen reduced to %.1f%%", regen_scale * 100.0f);
    }

    // 3. Check ESC status
    if (fabsf(abs_esc->yaw_rate_error) > 0.1f) {  // Significant yaw error
        abs_esc->esc_active = true;

        // ESC needs full authority over individual wheel braking
        // Disable regen to allow ESC full control
        abs_esc->regen_force_limit = 0.0f;

        log_event("ESC active - regen disabled");
    }
    else {
        abs_esc->esc_active = false;
    }
}
```

#### Emergency Braking Assist

```c
typedef struct {
    float brake_pedal_force;        // Pedal force (N)
    float brake_pedal_velocity;     // Pedal application rate (N/s)
    float decel_current;            // Current deceleration (m/s²)
    bool emergency_detected;        // Emergency braking detected
    float boost_factor;             // Brake boost factor (1.0-2.0)
} EmergencyBrakeAssist_t;

void emergency_brake_assist(EmergencyBrakeAssist_t *eba) {
    // Detect emergency braking intent
    // High pedal force + fast application = emergency
    if (eba->brake_pedal_force > 300.0f &&          // >300 N
        eba->brake_pedal_velocity > 2000.0f) {      // >2000 N/s

        eba->emergency_detected = true;

        // Apply maximum available braking
        // Boost factor increases braking beyond normal pedal mapping
        eba->boost_factor = 1.5f;

        // Immediately apply max friction + max regen
        apply_maximum_braking();

        // Activate hazard lights
        activate_hazard_lights();

        log_event("Emergency braking assist activated");
    }
    else if (eba->emergency_detected &&
             eba->brake_pedal_force < 100.0f) {
        // Emergency over - return to normal
        eba->emergency_detected = false;
        eba->boost_factor = 1.0f;

        log_event("Emergency braking assist deactivated");
    }
}
```

---

### References for Section 3.4:

**Standards:**
1. **ISO 26262**: "Road Vehicles - Functional Safety" (ASIL D for brake systems)
2. **FMVSS 135**: "Light Vehicle Brake Systems" (redundancy requirements)
3. **ECE R13-H**: "Approval of Passenger Cars with Regard to Braking" (Annex 13: Electric Regenerative Braking)

**Books:**
1. *"Safety-Critical Systems Handbook"* by David J. Smith and Kenneth G.L. Simpson - Chapter 15 (Automotive Safety)
2. *"Automotive Software Engineering"* by Jörg Schäuffele and Thomas Zurawka - Chapter 12 (Fail-Safe Design)

**Papers:**
1. Savitski, D., et al. (2016). "Wheel Slip Control for Electric Vehicles with In-Wheel Motors." Vehicle System Dynamics, 54(10), 1366-1389.
2. Haus, B., et al. (2015). "Fail-Operational Automotive Brake by Wire System." IEEE Intelligent Vehicles Symposium.

**Application Notes:**
1. **Bosch**: "Functional Safety for Brake-by-Wire Systems" (Technical Paper)
2. **Continental**: "Redundancy Concepts for Electric Brake Systems" (2020 White Paper)

---

**End of Section 3: Braking Strategy and Blending**

---

## Section 4: Hill Hold Control

Hill hold (also called hill start assist) prevents the vehicle from rolling backwards when stopped on an incline. This feature is essential for driver confidence and safety, especially in stop-and-go traffic on hills.

---

### 4.1 Hill Hold Theory and Requirements

#### Physical Principles

When a vehicle is stopped on a hill, gravity creates a rolling force that must be counteracted:

```
Rolling Force Analysis:

F_roll = m * g * sin(θ)

Where:
  m = vehicle mass (kg)
  g = gravitational acceleration (9.81 m/s²)
  θ = road grade angle (radians)

For typical grades:
  5% grade (2.86°):   F_roll = 0.050 * m * g = 490 N for 1000 kg vehicle
  10% grade (5.71°):  F_roll = 0.100 * m * g = 981 N for 1000 kg vehicle
  20% grade (11.31°): F_roll = 0.196 * m * g = 1922 N for 1000 kg vehicle
```

**Torque Required at Wheels:**

```
T_hold = F_roll * r_wheel / η_drivetrain

Where:
  r_wheel = wheel radius (m)
  η_drivetrain = drivetrain efficiency (0.90-0.95)

Example (10% grade, 1000 kg, 0.3m wheel radius):
  T_hold = 981 N * 0.3 m / 0.95 = 310 Nm at wheels
  T_motor = T_hold / gear_ratio
```

#### Functional Requirements

**1. Activation Conditions:**
- Vehicle at complete stop (speed < 0.1 m/s)
- Road grade exceeds threshold (typically > 3%)
- Driver foot off accelerator and brake
- System enabled by driver (may be user-selectable)

**2. Hold Duration:**
- Typical: 2-3 seconds after brake release
- Maximum: 3-5 seconds (regulatory requirements)
- Release conditions: accelerator pressed OR timeout

**3. Roll-Back Prevention:**
- Maximum allowed roll-back: < 10 cm
- Roll-back velocity: < 0.2 m/s

**4. Smooth Release:**
- Seamless transition to drive torque
- No jerk or abrupt release
- Driver should not notice activation/deactivation

#### Grade Detection

```c
typedef struct {
    float accel_longitudinal;   // Longitudinal accelerometer (m/s²)
    float accel_lateral;        // Lateral accelerometer (m/s²)
    float accel_vertical;       // Vertical accelerometer (m/s²)
    float vehicle_speed;        // Vehicle speed (m/s)
    float grade_angle;          // Calculated grade angle (radians)
    float grade_percent;        // Grade as percentage
    bool grade_valid;           // Grade measurement valid
} GradeDetection_t;

void calculate_road_grade(GradeDetection_t *grade) {
    // Vehicle must be stationary for accurate grade measurement
    if (grade->vehicle_speed > 0.5f) {  // Moving
        grade->grade_valid = false;
        return;
    }

    // Calculate grade from accelerometer
    // When stationary, longitudinal accel = g * sin(θ)
    // and vertical accel = g * cos(θ)

    float g_total = sqrtf(grade->accel_longitudinal * grade->accel_longitudinal +
                         grade->accel_vertical * grade->accel_vertical);

    // Calculate angle
    grade->grade_angle = atan2f(grade->accel_longitudinal, grade->accel_vertical);

    // Convert to percentage
    grade->grade_percent = tanf(grade->grade_angle) * 100.0f;

    // Validate measurement
    if (fabsf(g_total - 9.81f) < 0.5f) {  // Within reasonable range
        grade->grade_valid = true;
    }
    else {
        grade->grade_valid = false;
    }

    // Apply low-pass filter to reduce noise
    static float grade_filtered = 0.0f;
    float alpha = 0.1f;  // Filter coefficient
    grade_filtered = alpha * grade->grade_percent + (1.0f - alpha) * grade_filtered;
    grade->grade_percent = grade_filtered;
}
```

#### Direction Detection

Hill hold must determine if the vehicle is facing uphill or downhill:

```c
typedef enum {
    HILL_DIRECTION_NONE = 0,    // Flat or insignificant grade
    HILL_DIRECTION_UPHILL,      // Vehicle facing uphill
    HILL_DIRECTION_DOWNHILL     // Vehicle facing downhill
} HillDirection_t;

typedef struct {
    float grade_angle;          // Road grade (radians, + = uphill)
    HillDirection_t direction;
    float grade_threshold;      // Minimum grade for activation (radians)
} HillDirectionDetection_t;

void detect_hill_direction(HillDirectionDetection_t *dir) {
    if (fabsf(dir->grade_angle) < dir->grade_threshold) {
        // Grade too small - no hill hold needed
        dir->direction = HILL_DIRECTION_NONE;
    }
    else if (dir->grade_angle > 0.0f) {
        // Positive grade - vehicle facing uphill
        dir->direction = HILL_DIRECTION_UPHILL;
    }
    else {
        // Negative grade - vehicle facing downhill
        dir->direction = HILL_DIRECTION_DOWNHILL;
    }
}
```

---

### References for Section 4.1:

**Books:**
1. *"Automotive Control Systems"* by Uwe Kiencke and Lars Nielsen - Chapter 10 (Driver Assistance Systems)
2. *"Vehicle Dynamics: Theory and Application"* by Reza N. Jazar - Chapter 2 (Longitudinal Dynamics on Grades)

**Standards:**
1. **ECE R13**: "Uniform Provisions Concerning the Approval of Vehicles with Regard to Braking" (Annex 8: Hill Holder Performance)
2. **ISO 2575**: "Road Vehicles - Symbols for Controls, Indicators and Tell-Tales" (Hill Hold Indicator)

**Papers:**
1. Sugai, M., et al. (2003). "Hill-Start Assist Control for AT Vehicles." JSAE Review, 24(2), 205-209.

---

### 4.2 Hill Hold Implementation Strategies

This section presents three strategies for implementing hill hold control.

#### Strategy 1: Friction Brake Hold (Hydraulic)

The most common approach uses the vehicle's hydraulic brake system to maintain brake pressure after the driver releases the pedal.

```c
typedef struct {
    bool active;                // Hill hold currently active
    float brake_pressure_hold;  // Brake pressure to maintain (bar)
    float grade_angle;          // Current grade (radians)
    float hold_time_remaining;  // Time remaining (s)
    float hold_duration_max;    // Maximum hold time (s)
} HillHoldFrictionBrake_t;

void hill_hold_friction_brake_update(HillHoldFrictionBrake_t *hh, float dt) {
    // 1. Calculate required brake pressure
    float vehicle_mass = get_vehicle_mass();
    float force_required = vehicle_mass * 9.81f * sinf(hh->grade_angle);

    // Convert force to brake pressure (simplified)
    // Pressure = Force / (4 wheels * piston area * friction coefficient)
    float brake_piston_area = 0.001f;  // m² per wheel
    float friction_coeff = 0.4f;       // Brake pad friction
    hh->brake_pressure_hold = force_required /
                              (4.0f * brake_piston_area * friction_coeff * 1e5f);  // bar

    // Clamp to reasonable range
    hh->brake_pressure_hold = fmaxf(5.0f, fminf(100.0f, hh->brake_pressure_hold));

    // 2. Activate hold if conditions met
    if (!hh->active) {
        // Check activation conditions
        if (is_vehicle_stopped() &&
            is_brake_pedal_released() &&
            fabsf(hh->grade_angle) > 0.03f &&  // > ~3% grade
            is_gear_engaged()) {

            // Activate hill hold
            hh->active = true;
            hh->hold_time_remaining = hh->hold_duration_max;

            // Command brake system to maintain pressure
            set_brake_pressure(hh->brake_pressure_hold);

            log_event("Hill hold activated on %.1f%% grade",
                     tanf(hh->grade_angle) * 100.0f);
        }
    }

    // 3. Update active hold
    if (hh->active) {
        // Decrement timer
        hh->hold_time_remaining -= dt;

        // Check release conditions
        if (is_accelerator_pressed() ||
            hh->hold_time_remaining <= 0.0f) {

            // Release hill hold
            hh->active = false;
            release_brake_pressure();

            log_event("Hill hold released");
        }
    }
}
```

**Advantages:**
- Uses existing hydraulic brake hardware
- Reliable and fail-safe
- Independent of drivetrain

**Disadvantages:**
- Requires electrohydraulic brake system
- Cannot be used with simple mechanical brakes
- Brake wear during hold

---

#### Strategy 2: Motor Torque Hold (Electric)

Use the electric motor to generate holding torque. This is unique to EVs and provides additional capabilities.

```c
typedef struct {
    bool active;                // Hill hold active
    float motor_torque_hold;    // Motor torque to maintain (Nm)
    float grade_angle;          // Current grade (radians)
    float hold_time_remaining;  // Time remaining (s)
    bool motor_available;       // Motor can provide hold torque
} HillHoldMotorTorque_t;

void hill_hold_motor_torque_update(HillHoldMotorTorque_t *hh, float dt) {
    // 1. Calculate required motor torque
    float vehicle_mass = get_vehicle_mass();
    float wheel_radius = get_wheel_radius();
    float gear_ratio = get_gear_ratio();
    float force_required = vehicle_mass * 9.81f * sinf(hh->grade_angle);

    // Torque at motor shaft
    hh->motor_torque_hold = (force_required * wheel_radius) / gear_ratio;

    // 2. Check if motor can provide this torque at zero speed
    // Many motors have reduced torque at zero speed due to cooling
    float motor_torque_max_static = get_motor_max_torque_static();

    if (fabsf(hh->motor_torque_hold) > motor_torque_max_static) {
        hh->motor_available = false;
        return;  // Cannot use motor hold
    }
    else {
        hh->motor_available = true;
    }

    // 3. Activation
    if (!hh->active) {
        if (is_vehicle_stopped() &&
            is_brake_pedal_released() &&
            fabsf(hh->grade_angle) > 0.03f &&
            hh->motor_available) {

            // Activate motor hold
            hh->active = true;
            hh->hold_time_remaining = 3.0f;  // seconds

            // Command motor torque
            // Positive torque for uphill, negative for downhill
            set_motor_torque_command(hh->motor_torque_hold);

            log_event("Motor hill hold activated");
        }
    }

    // 4. Update
    if (hh->active) {
        hh->hold_time_remaining -= dt;

        // Update torque based on vehicle movement
        float vehicle_speed = get_vehicle_speed();
        if (fabsf(vehicle_speed) > 0.1f) {
            // Vehicle moving - increase torque if rolling back
            if ((hh->grade_angle > 0.0f && vehicle_speed < 0.0f) ||
                (hh->grade_angle < 0.0f && vehicle_speed > 0.0f)) {
                // Rolling backwards - increase torque
                hh->motor_torque_hold *= 1.1f;
            }
        }

        // Release conditions
        if (is_accelerator_pressed() || hh->hold_time_remaining <= 0.0f) {
            hh->active = false;
            log_event("Motor hill hold released");
        }
    }
}
```

**Advantages:**
- No brake wear
- Can provide smooth transition to drive torque
- No additional hardware needed

**Disadvantages:**
- Motor thermal limits at zero speed
- May not handle steep grades
- Safety concern if motor fails

---

#### Strategy 3: Hybrid Hold (Motor + Friction)

Combine motor and friction brakes for optimal performance.

```c
typedef struct {
    bool active;
    float motor_torque;         // Motor contribution (Nm)
    float brake_pressure;       // Brake contribution (bar)
    float grade_angle;          // Grade (radians)
    float hold_time;            // Time in hold (s)
    float motor_thermal_limit;  // Motor thermal limit (0-1)
} HillHoldHybrid_t;

void hill_hold_hybrid_update(HillHoldHybrid_t *hh, float dt) {
    if (!hh->active) {
        // Activation logic (same as before)
        if (is_vehicle_stopped() && is_brake_pedal_released() &&
            fabsf(hh->grade_angle) > 0.03f) {

            hh->active = true;
            hh->hold_time = 0.0f;
        }
    }

    if (hh->active) {
        hh->hold_time += dt;

        // Calculate total required force
        float vehicle_mass = get_vehicle_mass();
        float force_total = vehicle_mass * 9.81f * fabsf(sinf(hh->grade_angle));

        // Determine motor contribution based on thermal state
        float motor_torque_available = get_motor_max_torque_static() *
                                       hh->motor_thermal_limit;

        float force_motor_max = (motor_torque_available * get_gear_ratio()) /
                                get_wheel_radius();

        // Strategy: Use motor first, supplement with friction
        if (force_motor_max >= force_total) {
            // Motor can handle entire hold
            hh->motor_torque = (force_total * get_wheel_radius()) /
                               get_gear_ratio();
            hh->brake_pressure = 0.0f;
        }
        else {
            // Use max motor, supplement with brakes
            hh->motor_torque = motor_torque_available;
            float force_remaining = force_total - force_motor_max;
            hh->brake_pressure = calculate_brake_pressure(force_remaining);
        }

        // After 2 seconds, transition fully to friction brakes
        // This prevents motor overheating
        if (hh->hold_time > 2.0f) {
            float transition_factor = fminf(1.0f, (hh->hold_time - 2.0f) / 1.0f);
            hh->motor_torque *= (1.0f - transition_factor);
            hh->brake_pressure = calculate_brake_pressure(force_total);
        }

        // Apply commands
        set_motor_torque_command(hh->motor_torque);
        set_brake_pressure(hh->brake_pressure);

        // Release conditions
        if (is_accelerator_pressed() || hh->hold_time > 5.0f) {
            hh->active = false;
            release_brake_pressure();
            set_motor_torque_command(0.0f);
            log_event("Hybrid hill hold released");
        }
    }
}
```

**Smooth Transition to Drive Torque:**

Critical for good user experience:

```c
typedef struct {
    float torque_hold;          // Holding torque (Nm)
    float torque_drive;         // Desired drive torque (Nm)
    float torque_command;       // Actual commanded torque (Nm)
    float blend_time;           // Blend duration (s)
    float blend_progress;       // Blend progress (0-1)
    bool blending;              // Currently blending
} HillHoldTransition_t;

void hill_hold_transition_to_drive(HillHoldTransition_t *trans, float dt) {
    if (!trans->blending) {
        // Start blend when accelerator pressed
        if (is_accelerator_pressed()) {
            trans->blending = true;
            trans->blend_progress = 0.0f;
            trans->blend_time = 0.5f;  // 0.5 second blend

            // Get drive torque request from accelerator
            trans->torque_drive = get_accelerator_torque_request();
        }
    }

    if (trans->blending) {
        // Update blend progress
        trans->blend_progress += dt / trans->blend_time;

        if (trans->blend_progress >= 1.0f) {
            // Blend complete
            trans->blending = false;
            trans->torque_command = trans->torque_drive;
        }
        else {
            // Smooth S-curve blend for natural feel
            float t = trans->blend_progress;
            float s_curve = t * t * (3.0f - 2.0f * t);  // Smoothstep

            // Blend from hold torque to drive torque
            trans->torque_command = trans->torque_hold * (1.0f - s_curve) +
                                   trans->torque_drive * s_curve;

            // Ensure minimum torque to prevent rollback
            float torque_min = trans->torque_hold * 0.9f;
            trans->torque_command = fmaxf(trans->torque_command, torque_min);
        }

        // Apply command
        set_motor_torque_command(trans->torque_command);
    }
}
```

---

### References for Section 4.2:

**Papers:**
1. Kim, S., et al. (2012). "Development of Hill-Start Assist Control System for Electric Vehicles." International Journal of Automotive Technology, 13(4), 627-635.
2. Huang, M., & Liu, X. (2014). "Hill Start Assistance Control for Electric Vehicles with Automatic Transmission." SAE Technical Paper 2014-01-1798.

**Application Notes:**
1. **Bosch**: "Hill Hold Control for Electric and Hybrid Vehicles" (2018 Technical Paper)
2. **Continental**: "Electronic Parking Brake with Hill Hold Function" (Application Guide)

---

### 4.3 Integration with Braking and Traction Systems

#### Integration with Regenerative Braking

Hill hold must be coordinated with regenerative braking to avoid conflicts.

```c
typedef struct {
    bool hill_hold_active;
    bool regen_available;
    float grade_angle;
    float motor_torque_hold;
    float motor_torque_regen;
    float motor_torque_command;
} HillHoldRegenIntegration_t;

void integrate_hill_hold_with_regen(HillHoldRegenIntegration_t *integ) {
    // Regen may interfere with hill hold, especially on uphill
    if (integ->hill_hold_active) {
        if (integ->grade_angle > 0.0f) {
            // Uphill - need positive motor torque to hold
            // Regen would produce negative torque - disable it
            disable_regenerative_braking();

            integ->motor_torque_command = integ->motor_torque_hold;
        }
        else {
            // Downhill - need negative motor torque to hold
            // Regen naturally produces negative torque - can use it
            // But ensure it's sufficient
            float torque_regen_max = get_max_regen_torque();

            if (fabsf(torque_regen_max) >= fabsf(integ->motor_torque_hold)) {
                // Regen can handle hold
                integ->motor_torque_command = integ->motor_torque_hold;
                enable_regenerative_braking();
            }
            else {
                // Regen insufficient - add friction brakes
                integ->motor_torque_command = -torque_regen_max;
                float force_remaining = calculate_remaining_hold_force(
                    integ->motor_torque_hold,
                    integ->motor_torque_command
                );
                set_brake_pressure(calculate_brake_pressure(force_remaining));
            }
        }
    }
}
```

#### Integration with Traction Control

```c
typedef struct {
    bool hill_hold_active;
    bool traction_control_active;
    float wheel_slip[4];
    float motor_torque_hold;
    float motor_torque_drive;
} HillHoldTractionIntegration_t;

void integrate_hill_hold_with_traction_control(HillHoldTractionIntegration_t *integ) {
    // During hill start, wheel slip may occur
    if (integ->hill_hold_active) {
        // Monitor wheel slip
        float max_wheel_slip = 0.0f;
        for (int i = 0; i < 4; i++) {
            if (integ->wheel_slip[i] > max_wheel_slip) {
                max_wheel_slip = integ->wheel_slip[i];
            }
        }

        // If excessive slip during hill start, reduce torque
        if (max_wheel_slip > 0.10f) {  // >10% slip
            integ->traction_control_active = true;

            // Reduce drive torque
            float slip_factor = 1.0f - (max_wheel_slip / 0.20f);
            slip_factor = fmaxf(0.5f, fminf(1.0f, slip_factor));

            integ->motor_torque_drive *= slip_factor;

            // Ensure we maintain minimum hold torque
            if (integ->motor_torque_drive < integ->motor_torque_hold) {
                integ->motor_torque_drive = integ->motor_torque_hold;
            }

            log_event("Traction control active during hill start");
        }
        else {
            integ->traction_control_active = false;
        }
    }
}
```

#### User Interface and Feedback

```c
typedef struct {
    bool hill_hold_available;
    bool hill_hold_active;
    bool user_enabled;          // User preference setting
    float grade_percent;
    float time_remaining;
} HillHoldUserInterface_t;

void update_hill_hold_user_interface(HillHoldUserInterface_t *ui) {
    // 1. Display hill hold status on instrument cluster
    if (ui->hill_hold_active) {
        display_hill_hold_icon(true);
        display_message("Hill Hold Active");

        // Show countdown timer in last 2 seconds
        if (ui->time_remaining < 2.0f) {
            display_countdown(ui->time_remaining);
        }
    }
    else {
        display_hill_hold_icon(false);
    }

    // 2. Provide haptic feedback
    if (ui->hill_hold_active) {
        // Gentle pulse when first activated
        provide_haptic_pulse(HAPTIC_GENTLE);
    }

    // 3. User settings menu
    if (user_in_settings_menu()) {
        display_setting("Hill Hold Assist", ui->user_enabled ? "ON" : "OFF");

        if (user_toggled_setting()) {
            ui->user_enabled = !ui->user_enabled;
            save_user_preference("hill_hold_enabled", ui->user_enabled);
        }
    }

    // 4. Grade display (optional, for driver information)
    if (ui->grade_percent > 3.0f) {
        display_grade_indicator(ui->grade_percent);
    }
}
```

---

### References for Section 4.3:

**Books:**
1. *"Electric and Hybrid Vehicles: Design Fundamentals"* by Iqbal Husain - Chapter 11 (Integration of Subsystems)

**Standards:**
1. **SAE J2807**: "Performance Requirements for Determining Tow-Vehicle Gross Combination Weight Rating and Trailer Weight Rating" (includes hill start requirements)

**Papers:**
1. Park, J., et al. (2018). "Integrated Control of the Differential Braking, the Suspension System, and the Active Roll Bar for Improvement of the Roll Stability." IMechE Part D: Journal of Automobile Engineering, 232(13), 1762-1779.

**Application Notes:**
1. **ZF**: "Hill Holder Control Systems" (Technical Documentation)

---

**End of Section 4: Hill Hold Control**

---

## Section 5: Temperature-Based Derating

Temperature management is critical for protecting both the motor and controller from thermal damage while maintaining maximum performance. This section covers thermal modeling and derating strategies.

---

### 5.1 Thermal Modeling and Monitoring

#### Temperature Sources and Sensors

**Key Temperature Measurement Points:**

1. **Motor Temperatures:**
   - Stator winding temperature (most critical)
   - Rotor temperature (magnets)
   - Bearing temperature
   - Housing/case temperature

2. **Inverter Temperatures:**
   - MOSFET/IGBT junction temperature
   - Gate driver temperature
   - DC bus capacitor temperature
   - Heatsink temperature

3. **Battery Temperatures:**
   - Cell temperatures (min, max, average)
   - BMS board temperature
   - Coolant temperature (if liquid cooled)

**Sensor Types:**

```c
typedef enum {
    TEMP_SENSOR_NTC,            // Negative Temperature Coefficient thermistor
    TEMP_SENSOR_PTC,            // Positive Temperature Coefficient
    TEMP_SENSOR_THERMOCOUPLE,   // K-type, J-type, etc.
    TEMP_SENSOR_RTD,            // Resistance Temperature Detector (PT100, PT1000)
    TEMP_SENSOR_SEMICONDUCTOR   // Integrated IC sensor
} TempSensorType_t;

typedef struct {
    TempSensorType_t type;
    float resistance;           // Current resistance (Ω)
    float voltage;              // Measured voltage (V)
    float temperature;          // Calculated temperature (°C)
    bool valid;                 // Reading valid
    float calibration_offset;   // Calibration offset (°C)
} TemperatureSensor_t;
```

#### NTC Thermistor Temperature Calculation

Most common sensor for motor and inverter monitoring:

```c
// Steinhart-Hart equation for NTC thermistor
float calculate_temperature_ntc(TemperatureSensor_t *sensor,
                                 float R_ref,
                                 float T_ref,
                                 float Beta) {
    // R_ref = reference resistance at T_ref (typically 10kΩ at 25°C)
    // Beta = Beta coefficient (typically 3950 for automotive NTCs)

    float R = sensor->resistance;

    // Steinhart-Hart equation (simplified Beta formula)
    float T_kelvin = 1.0f / ((1.0f / (T_ref + 273.15f)) +
                             (1.0f / Beta) * logf(R / R_ref));

    sensor->temperature = T_kelvin - 273.15f + sensor->calibration_offset;

    // Validate range
    if (sensor->temperature >= -40.0f && sensor->temperature <= 200.0f) {
        sensor->valid = true;
    }
    else {
        sensor->valid = false;
    }

    return sensor->temperature;
}
```

#### Junction Temperature Estimation

MOSFET junction temperature is not directly measurable but can be estimated:

```c
typedef struct {
    float T_heatsink;           // Heatsink temperature (°C)
    float P_dissipation;        // Power dissipation (W)
    float R_junction_case;      // Thermal resistance junction-to-case (°C/W)
    float R_case_heatsink;      // Thermal resistance case-to-heatsink (°C/W)
    float T_junction_estimated; // Estimated junction temp (°C)
} JunctionTempEstimation_t;

void estimate_junction_temperature(JunctionTempEstimation_t *junc) {
    // Thermal model: T_j = T_hs + P * (R_jc + R_ch)
    float R_total = junc->R_junction_case + junc->R_case_heatsink;
    junc->T_junction_estimated = junc->T_heatsink + junc->P_dissipation * R_total;
}
```

#### Thermal Time Constants

Different components have different thermal time constants:

```
Component          Time Constant    Implication
---------------------------------------------------------
MOSFET Junction    1-10 ms          Fast response to load changes
Motor Winding      10-60 s          Moderate response
Motor Housing      5-15 min         Slow thermal mass
Battery Cell       2-10 min         Moderate to slow
DC Capacitor       30-120 s         Moderate response
```

**Thermal Model with Time Constants:**

```c
typedef struct {
    float T_current;            // Current temperature (°C)
    float T_ambient;            // Ambient temperature (°C)
    float P_dissipation;        // Power dissipation (W)
    float thermal_resistance;   // Thermal resistance (°C/W)
    float thermal_capacitance;  // Thermal capacitance (J/°C)
    float time_constant;        // Thermal time constant (s)
} ThermalModel_t;

void update_thermal_model(ThermalModel_t *model, float dt) {
    // Calculate steady-state temperature rise
    float delta_T_ss = model->P_dissipation * model->thermal_resistance;
    float T_final = model->T_ambient + delta_T_ss;

    // First-order thermal model
    // dT/dt = (T_final - T_current) / tau
    float tau = model->time_constant;
    float alpha = dt / (tau + dt);  // Discrete-time coefficient

    model->T_current = model->T_current + alpha * (T_final - model->T_current);
}
```

---

### References for Section 5.1:

**Books:**
1. *"Thermal Design of Electronic Equipment"* by Ralph Remsburg - Chapter 3 (Thermal Resistance)
2. *"Thermal Management of Electric Vehicle Battery Systems"* by Ibrahim Dincer et al. - Chapter 4 (Thermal Modeling)

**Application Notes:**
1. **Infineon**: "Thermal Equivalent Circuit Models" (AN2008-03)
2. **Texas Instruments**: "Temperature Sensing with NTC Thermistors" (SLVA473)

---

### 5.2 Motor Temperature Derating

#### Motor Thermal Limits

Motor insulation classes define maximum allowable winding temperatures:

```
Insulation Class    Max Temperature    Typical Application
---------------------------------------------------------------
Class B             130°C              Industrial motors
Class F             155°C              Automotive motors (common)
Class H             180°C              High-performance EV motors
Class N (200)       200°C              Racing/performance applications
```

**Thermal Margin Strategy:**

```c
typedef struct {
    float T_winding;            // Measured winding temperature (°C)
    float T_limit_continuous;   // Continuous operating limit (°C)
    float T_limit_peak;         // Peak operating limit (°C)
    float T_warning;            // Warning threshold (°C)
    float T_derate_start;       // Start derating (°C)
    float torque_derate_factor; // Output: torque derate (0-1)
} MotorThermalDerate_t;

void calculate_motor_thermal_derate(MotorThermalDerate_t *derate) {
    // Example limits for Class F motor:
    derate->T_limit_continuous = 155.0f;
    derate->T_limit_peak = 180.0f;        // Short duration only
    derate->T_warning = 140.0f;
    derate->T_derate_start = 130.0f;

    if (derate->T_winding < derate->T_derate_start) {
        // Below derating threshold - full torque
        derate->torque_derate_factor = 1.0f;
    }
    else if (derate->T_winding < derate->T_limit_continuous) {
        // Linear derate between start and continuous limit
        float temp_range = derate->T_limit_continuous - derate->T_derate_start;
        float temp_excess = derate->T_winding - derate->T_derate_start;
        derate->torque_derate_factor = 1.0f - (temp_excess / temp_range) * 0.5f;
    }
    else if (derate->T_winding < derate->T_limit_peak) {
        // Heavy derate between continuous and peak
        float temp_range = derate->T_limit_peak - derate->T_limit_continuous;
        float temp_excess = derate->T_winding - derate->T_limit_continuous;
        derate->torque_derate_factor = 0.5f - (temp_excess / temp_range) * 0.4f;
    }
    else {
        // At or above peak limit - minimum torque only
        derate->torque_derate_factor = 0.1f;
    }

    // Clamp to valid range
    derate->torque_derate_factor = fmaxf(0.0f, fminf(1.0f,
                                         derate->torque_derate_factor));
}
```

#### Magnet Demagnetization Protection

Permanent magnets can be permanently damaged by high temperatures:

```c
typedef struct {
    float T_rotor_estimated;    // Estimated rotor temperature (°C)
    float T_magnet_limit;       // Magnet demagnetization temp (°C)
    float speed_limit_factor;   // Speed limit multiplier (0-1)
    bool protection_active;
} MagnetProtection_t;

void protect_magnet_from_demagnetization(MagnetProtection_t *prot) {
    // NdFeB magnets: demagnetization risk above 150-180°C
    // Ferrite magnets: lower limit (~100-120°C)

    prot->T_magnet_limit = 150.0f;  // Conservative for NdFeB

    if (prot->T_rotor_estimated > prot->T_magnet_limit - 20.0f) {
        // Approaching limit - reduce speed to reduce rotor heating
        prot->protection_active = true;

        float temp_margin = prot->T_magnet_limit - prot->T_rotor_estimated;
        prot->speed_limit_factor = temp_margin / 20.0f;
        prot->speed_limit_factor = fmaxf(0.3f, fminf(1.0f,
                                         prot->speed_limit_factor));
    }
    else {
        prot->protection_active = false;
        prot->speed_limit_factor = 1.0f;
    }
}
```

#### Continuous vs Peak Power Curves

Motors have different torque capability based on duration:

```c
typedef struct {
    float torque_continuous;    // Continuous torque rating (Nm)
    float torque_peak_30s;      // 30-second peak torque (Nm)
    float torque_peak_10s;      // 10-second peak torque (Nm)
    float torque_peak_3s;       // 3-second peak torque (Nm)
    float time_at_peak;         // Time in peak region (s)
    float cooldown_time;        // Required cooldown time (s)
    float torque_available;     // Output: available torque (Nm)
} MotorTorqueDuration_t;

void calculate_duration_limited_torque(MotorTorqueDuration_t *dur, float dt) {
    // Update time counter
    float torque_requested = get_torque_request();

    if (fabsf(torque_requested) > dur->torque_continuous) {
        // Operating in peak region
        dur->time_at_peak += dt;
    }
    else {
        // Operating in continuous region - cooldown
        dur->time_at_peak -= dt * 0.5f;  // Cooldown at 50% rate
        dur->time_at_peak = fmaxf(0.0f, dur->time_at_peak);
    }

    // Determine available torque based on time at peak
    if (dur->time_at_peak < 3.0f) {
        // 0-3 seconds: peak torque available
        dur->torque_available = dur->torque_peak_3s;
    }
    else if (dur->time_at_peak < 10.0f) {
        // 3-10 seconds: interpolate
        float t = (dur->time_at_peak - 3.0f) / 7.0f;
        dur->torque_available = dur->torque_peak_3s * (1.0f - t) +
                                dur->torque_peak_10s * t;
    }
    else if (dur->time_at_peak < 30.0f) {
        // 10-30 seconds: interpolate
        float t = (dur->time_at_peak - 10.0f) / 20.0f;
        dur->torque_available = dur->torque_peak_10s * (1.0f - t) +
                                dur->torque_peak_30s * t;
    }
    else {
        // > 30 seconds: continuous only
        dur->torque_available = dur->torque_continuous;
    }
}
```

---

### References for Section 5.2:

**Books:**
1. *"Electric Motor Handbook"* by H. Wayne Beaty and James L. Kirtley - Chapter 15 (Motor Thermal Protection)

**Standards:**
1. **IEC 60034-1**: "Rotating Electrical Machines - Part 1: Rating and Performance" (insulation classes)
2. **NEMA MG 1**: "Motors and Generators" (thermal protection)

**Papers:**
1. Staton, D., et al. (2005). "Thermal Analysis of Electric Motors and Generators." IEEE Industry Applications Magazine, 11(4), 19-25.

---

### 5.3 Inverter Temperature Derating

#### Power Semiconductor Thermal Limits

**Typical Junction Temperature Limits:**
- Silicon MOSFETs/IGBTs: 150-175°C
- SiC MOSFETs: 175-200°C
- GaN FETs: 150-175°C

```c
typedef struct {
    float T_junction;           // Junction temperature (°C)
    float T_junction_max;       // Maximum junction temp (°C)
    float T_derate_start;       // Start derating (°C)
    float current_derate_factor; // Output: current limit (0-1)
    float switching_freq_factor; // Output: switching freq limit (0-1)
} InverterThermalDerate_t;

void calculate_inverter_thermal_derate(InverterThermalDerate_t *derate) {
    derate->T_junction_max = 150.0f;     // Silicon MOSFET limit
    derate->T_derate_start = 120.0f;     // Start derating at 120°C

    if (derate->T_junction < derate->T_derate_start) {
        // Full capability
        derate->current_derate_factor = 1.0f;
        derate->switching_freq_factor = 1.0f;
    }
    else if (derate->T_junction < derate->T_junction_max) {
        // Linear derate
        float temp_range = derate->T_junction_max - derate->T_derate_start;
        float temp_excess = derate->T_junction - derate->T_derate_start;
        float derate_ratio = temp_excess / temp_range;

        // Reduce current capability
        derate->current_derate_factor = 1.0f - derate_ratio * 0.6f;  // Down to 40%

        // Reduce switching frequency to reduce switching losses
        derate->switching_freq_factor = 1.0f - derate_ratio * 0.5f;  // Down to 50%
    }
    else {
        // At limit - minimum operation
        derate->current_derate_factor = 0.3f;
        derate->switching_freq_factor = 0.5f;
    }
}
```

#### DC Bus Capacitor Temperature Management

Electrolytic capacitors are temperature-sensitive:

```c
typedef struct {
    float T_capacitor;          // Capacitor temperature (°C)
    float T_rated;              // Rated temperature (typically 85°C or 105°C)
    float ripple_current_derate; // Output: ripple current limit (0-1)
    float lifetime_factor;      // Lifetime multiplier at current temp
} CapacitorThermalManagement_t;

void manage_capacitor_temperature(CapacitorThermalManagement_t *cap) {
    cap->T_rated = 105.0f;  // High-temp automotive cap

    if (cap->T_capacitor < cap->T_rated - 20.0f) {
        // Well below rating - full capability
        cap->ripple_current_derate = 1.0f;
        cap->lifetime_factor = 4.0f;  // Double life for every 10°C below rating
    }
    else if (cap->T_capacitor < cap->T_rated) {
        // Approaching rating - some derate
        float temp_excess = cap->T_capacitor - (cap->T_rated - 20.0f);
        cap->ripple_current_derate = 1.0f - (temp_excess / 20.0f) * 0.3f;

        // Lifetime calculation (Arrhenius equation approximation)
        // Life halves for every 10°C increase
        float delta_T = cap->T_capacitor - (cap->T_rated - 20.0f);
        cap->lifetime_factor = powf(2.0f, -delta_T / 10.0f);
    }
    else {
        // Above rating - significant derate
        float temp_excess = cap->T_capacitor - cap->T_rated;
        cap->ripple_current_derate = 0.7f - (temp_excess / 20.0f) * 0.5f;
        cap->ripple_current_derate = fmaxf(0.2f, cap->ripple_current_derate);

        cap->lifetime_factor = powf(2.0f, -20.0f / 10.0f);  // Much reduced life
    }
}
```

---

### References for Section 5.3:

**Application Notes:**
1. **Infineon**: "Thermal Management of Power Semiconductors" (AN2015-10)
2. **ON Semiconductor**: "MOSFET & IGBT Gate Drive Design Guide" (AND9093/D)
3. **Nichicon**: "Aluminum Electrolytic Capacitors: Life Expectancy" (CAT.8101E)

---

### 5.4 Integrated Thermal Management System

#### Master Thermal Controller

Coordinates all thermal management functions:

```c
typedef struct {
    // Temperature inputs
    float T_motor_winding;
    float T_motor_housing;
    float T_inverter_junction;
    float T_inverter_heatsink;
    float T_battery_max;
    float T_battery_min;
    float T_dc_capacitor;
    float T_ambient;

    // Thermal models
    MotorThermalDerate_t motor_derate;
    InverterThermalDerate_t inverter_derate;
    CapacitorThermalManagement_t capacitor_mgmt;

    // System limits
    float torque_limit_thermal;      // Torque limit from thermal (Nm)
    float current_limit_thermal;     // Current limit from thermal (A)
    float power_limit_thermal;       // Power limit from thermal (W)
    float speed_limit_thermal;       // Speed limit from thermal (rad/s)

    // Cooling control
    float fan_duty_cycle;            // Cooling fan PWM (0-1)
    float pump_duty_cycle;           // Coolant pump PWM (0-1)

    // Status
    bool thermal_warning;
    bool thermal_fault;
    bool cooling_active;
} MasterThermalController_t;

void update_master_thermal_controller(MasterThermalController_t *thermal) {
    // 1. Update individual thermal models
    thermal->motor_derate.T_winding = thermal->T_motor_winding;
    calculate_motor_thermal_derate(&thermal->motor_derate);

    thermal->inverter_derate.T_junction = thermal->T_inverter_junction;
    calculate_inverter_thermal_derate(&thermal->inverter_derate);

    thermal->capacitor_mgmt.T_capacitor = thermal->T_dc_capacitor;
    manage_capacitor_temperature(&thermal->capacitor_mgmt);

    // 2. Calculate combined limits (use most restrictive)
    thermal->torque_limit_thermal = get_base_torque_rating() *
                                    thermal->motor_derate.torque_derate_factor;

    thermal->current_limit_thermal = get_base_current_rating() *
                                     thermal->inverter_derate.current_derate_factor;

    thermal->power_limit_thermal = thermal->torque_limit_thermal *
                                   get_motor_speed();

    // 3. Cooling control
    update_cooling_system(thermal);

    // 4. Status and warnings
    if (thermal->T_motor_winding > 140.0f ||
        thermal->T_inverter_junction > 135.0f ||
        thermal->T_battery_max > 50.0f) {
        thermal->thermal_warning = true;
    }
    else {
        thermal->thermal_warning = false;
    }

    if (thermal->T_motor_winding > 160.0f ||
        thermal->T_inverter_junction > 155.0f ||
        thermal->T_battery_max > 60.0f) {
        thermal->thermal_fault = true;
        // Trigger system shutdown
        trigger_thermal_shutdown();
    }
}

void update_cooling_system(MasterThermalController_t *thermal) {
    // Calculate cooling demand from each subsystem
    float cooling_demand_motor = 0.0f;
    float cooling_demand_inverter = 0.0f;
    float cooling_demand_battery = 0.0f;

    // Motor cooling demand
    if (thermal->T_motor_winding > 100.0f) {
        cooling_demand_motor = (thermal->T_motor_winding - 100.0f) / 40.0f;
    }

    // Inverter cooling demand
    if (thermal->T_inverter_junction > 100.0f) {
        cooling_demand_inverter = (thermal->T_inverter_junction - 100.0f) / 40.0f;
    }

    // Battery cooling demand
    if (thermal->T_battery_max > 35.0f) {
        cooling_demand_battery = (thermal->T_battery_max - 35.0f) / 20.0f;
    }

    // Combined cooling demand
    float cooling_demand = fmaxf(cooling_demand_motor,
                           fmaxf(cooling_demand_inverter, cooling_demand_battery));
    cooling_demand = fmaxf(0.0f, fminf(1.0f, cooling_demand));

    // Control fan
    if (cooling_demand > 0.1f) {
        thermal->cooling_active = true;
        thermal->fan_duty_cycle = 0.3f + cooling_demand * 0.7f;  // 30-100%
    }
    else {
        thermal->cooling_active = false;
        thermal->fan_duty_cycle = 0.0f;
    }

    // Control pump (if liquid cooled)
    if (cooling_demand > 0.2f) {
        thermal->pump_duty_cycle = 0.5f + cooling_demand * 0.5f;  // 50-100%
    }
    else {
        thermal->pump_duty_cycle = 0.0f;
    }

    // Apply PWM to cooling hardware
    set_fan_pwm(thermal->fan_duty_cycle);
    set_pump_pwm(thermal->pump_duty_cycle);
}
```

#### Predictive Thermal Management

Anticipate thermal issues before they occur:

```c
typedef struct {
    float T_current;
    float T_predicted_60s;      // Temperature in 60 seconds
    float power_current;
    ThermalModel_t thermal_model;
    bool preemptive_derate;
} PredictiveThermalMgmt_t;

void predict_thermal_behavior(PredictiveThermalMgmt_t *pred, float dt) {
    // Update thermal model with current power
    pred->thermal_model.P_dissipation = pred->power_current;
    pred->thermal_model.T_current = pred->T_current;

    // Simulate forward 60 seconds
    ThermalModel_t model_copy = pred->thermal_model;
    for (int i = 0; i < 60; i++) {
        update_thermal_model(&model_copy, 1.0f);  // 1 second steps
    }
    pred->T_predicted_60s = model_copy.T_current;

    // If predicted temp will exceed limits, start derating now
    if (pred->T_predicted_60s > 130.0f) {
        pred->preemptive_derate = true;
        // Reduce power before we hit the limit
        float derate_factor = 1.0f - ((pred->T_predicted_60s - 130.0f) / 20.0f);
        apply_preemptive_power_limit(derate_factor);
    }
    else {
        pred->preemptive_derate = false;
    }
}
```

---

### References for Section 5.4:

**Books:**
1. *"Electric Vehicle Technology Explained"* by James Larminie and John Lowry - Chapter 5 (Electric Motors and Controllers)
2. *"Thermal Management of Electric Vehicle Battery Systems"* by Ibrahim Dincer - Chapter 7 (Integrated Thermal Systems)

**Papers:**
1. Finesso, R., et al. (2016). "Thermal Management System for Hybrid Electric Vehicles Including a Rotating Thermal Storage Device." Applied Thermal Engineering, 98, 190-201.
2. Kim, D., et al. (2019). "Integrated Thermal Management for Electric Vehicle Powertrains." IEEE Transactions on Vehicular Technology, 68(12), 11476-11486.

**Application Notes:**
1. **Bosch**: "Thermal Management in Electric Vehicles" (2020 Technical White Paper)

---

**End of Section 5: Temperature-Based Derating**

---

## Section 6: CAN Communication and Monitoring

Controller Area Network (CAN) communication enables monitoring, debugging, and integration with other vehicle systems. This section covers essential CAN messages and parameters.

---

### 6.1 Essential CAN Parameters

#### Motor Control Status Messages

**CAN Message 1: Motor Status (ID: 0x200, 10ms period)**

```c
typedef struct __attribute__((packed)) {
    uint16_t motor_speed_rpm;       // Motor speed (RPM) [0-10000]
    int16_t motor_torque_Nm_x10;    // Torque * 10 (0.1 Nm resolution)
    uint16_t dc_voltage_V_x10;      // DC bus voltage * 10 (0.1V resolution)
    uint16_t motor_current_A_x10;   // Motor current * 10 (0.1A resolution)
} CAN_MotorStatus_t;

void send_motor_status_can(CAN_MotorStatus_t *msg) {
    uint8_t data[8];

    // Pack data into CAN frame
    data[0] = (msg->motor_speed_rpm >> 8) & 0xFF;
    data[1] = msg->motor_speed_rpm & 0xFF;
    data[2] = (msg->motor_torque_Nm_x10 >> 8) & 0xFF;
    data[3] = msg->motor_torque_Nm_x10 & 0xFF;
    data[4] = (msg->dc_voltage_V_x10 >> 8) & 0xFF;
    data[5] = msg->dc_voltage_V_x10 & 0xFF;
    data[6] = (msg->motor_current_A_x10 >> 8) & 0xFF;
    data[7] = msg->motor_current_A_x10 & 0xFF;

    can_transmit(0x200, data, 8);
}
```

**CAN Message 2: Motor Temperatures (ID: 0x201, 100ms period)**

```c
typedef struct __attribute__((packed)) {
    int16_t T_winding_C_x10;        // Winding temp * 10 (0.1°C resolution)
    int16_t T_inverter_C_x10;       // Inverter temp * 10 (0.1°C resolution)
    int16_t T_housing_C_x10;        // Housing temp * 10 (0.1°C resolution)
    uint8_t thermal_derate_pct;     // Thermal derating (0-100%)
    uint8_t reserved;
} CAN_MotorTemperatures_t;
```

**CAN Message 3: Motor Currents (ID: 0x202, 10ms period)**

```c
typedef struct __attribute__((packed)) {
    int16_t i_d_A_x10;              // d-axis current * 10 (0.1A resolution)
    int16_t i_q_A_x10;              // q-axis current * 10 (0.1A resolution)
    int16_t i_phase_a_A_x10;        // Phase A current * 10
    int16_t i_phase_b_A_x10;        // Phase B current * 10
} CAN_MotorCurrents_t;
```

#### Battery and Power Messages

**CAN Message 4: Battery Status (ID: 0x210, 100ms period)**

```c
typedef struct __attribute__((packed)) {
    uint16_t battery_voltage_V_x10; // Battery voltage * 10 (0.1V)
    int16_t battery_current_A_x10;  // Battery current * 10 (+ = discharge)
    uint16_t battery_soc_pct_x10;   // SOC * 10 (0.1% resolution)
    int16_t battery_power_W;        // Battery power (W)
} CAN_BatteryStatus_t;
```

**CAN Message 5: Power Flow (ID: 0x211, 100ms period)**

```c
typedef struct __attribute__((packed)) {
    int16_t regen_power_W;          // Regen power (W, negative = regen)
    uint16_t regen_energy_Wh;       // Cumulative regen energy (Wh)
    uint16_t motor_power_W;         // Motor mechanical power (W)
    uint16_t efficiency_pct;        // System efficiency (0-100%)
} CAN_PowerFlow_t;
```

#### Control and Command Messages

**CAN Message 6: Control Mode (ID: 0x220, 20ms period)**

```c
typedef enum {
    CONTROL_MODE_IDLE = 0,
    CONTROL_MODE_TORQUE,
    CONTROL_MODE_SPEED,
    CONTROL_MODE_REGEN,
    CONTROL_MODE_FAULT
} ControlMode_t;

typedef struct __attribute__((packed)) {
    uint8_t control_mode;           // Current control mode
    int16_t torque_command_Nm_x10;  // Commanded torque * 10
    int16_t speed_command_rpm;      // Commanded speed (RPM)
    uint8_t throttle_position_pct;  // Throttle position (0-100%)
    uint8_t brake_position_pct;     // Brake position (0-100%)
    uint8_t regen_level;            // Regen level (0-4)
    uint8_t status_flags;           // Status bits
} CAN_ControlMode_t;
```

---

### References for Section 6.1:

**Standards:**
1. **ISO 11898**: "Road Vehicles - Controller Area Network (CAN)"
2. **SAE J1939**: "Serial Control and Communications Heavy Duty Vehicle Network"

---

### 6.2 Debug and Diagnostic Messages

#### Field Weakening Debug (ID: 0x230, 50ms period)

```c
typedef struct __attribute__((packed)) {
    uint16_t motor_speed_rpm;
    uint16_t base_speed_rpm;
    int16_t id_ref_A_x10;           // d-axis current reference
    int16_t iq_ref_A_x10;           // q-axis current reference
    uint8_t fw_active;              // Field weakening active flag
    uint8_t fw_region;              // FW region (0=MTPA, 1=FW, 2=MTPV)
} CAN_FieldWeakeningDebug_t;
```

#### Brake Blending Debug (ID: 0x231, 50ms period)

```c
typedef struct __attribute__((packed)) {
    int16_t brake_force_total_N;    // Total braking force (N)
    int16_t brake_force_regen_N;    // Regen braking force (N)
    int16_t brake_force_friction_N; // Friction braking force (N)
    uint8_t blend_factor_pct;       // Regen proportion (0-100%)
    uint8_t brake_system_state;     // Brake system state
    uint16_t decel_actual_mmss;     // Actual deceleration (mm/s²)
} CAN_BrakeBlendingDebug_t;
```

#### Hill Hold Debug (ID: 0x232, 100ms period)

```c
typedef struct __attribute__((packed)) {
    int16_t grade_angle_deg_x100;   // Grade angle * 100 (0.01° resolution)
    int16_t grade_percent_x10;      // Grade * 10 (0.1%)
    uint8_t hill_hold_active;       // Hill hold active flag
    int16_t hold_torque_Nm_x10;     // Holding torque * 10
    uint16_t hold_time_remaining_ms; // Time remaining (ms)
    uint8_t hill_direction;         // 0=none, 1=uphill, 2=downhill
} CAN_HillHoldDebug_t;
```

---

### References for Section 6.2:

**Application Notes:**
1. **Vector**: "CAN Database and Message Development" (Application Guide)
2. **Kvaser**: "CAN Protocol Tutorial" (Technical Documentation)

---

### 6.3 Performance Monitoring

#### Energy Efficiency Tracking

```c
typedef struct {
    uint32_t energy_from_battery_Wh; // Energy consumed from battery
    uint32_t energy_to_motor_Wh;     // Energy delivered to motor
    uint32_t energy_regen_Wh;        // Energy recovered via regen
    float efficiency_drive_pct;      // Drive efficiency (%)
    float efficiency_regen_pct;      // Regen efficiency (%)
    uint32_t distance_traveled_m;    // Distance traveled (m)
    float energy_per_km_Whpkm;       // Energy consumption (Wh/km)
} EnergyMonitoring_t;

void update_energy_monitoring(EnergyMonitoring_t *energy, float dt) {
    // Read instantaneous power
    float P_battery = read_battery_power();    // Watts
    float P_motor = read_motor_power();        // Watts
    float vehicle_speed = read_vehicle_speed(); // m/s

    // Integrate energy (Power * time)
    if (P_battery > 0.0f) {
        // Discharging
        energy->energy_from_battery_Wh += (P_battery * dt) / 3600.0f;
    }
    else {
        // Charging (regen)
        energy->energy_regen_Wh += (-P_battery * dt) / 3600.0f;
    }

    if (P_motor > 0.0f) {
        energy->energy_to_motor_Wh += (P_motor * dt) / 3600.0f;
    }

    // Distance traveled
    energy->distance_traveled_m += vehicle_speed * dt;

    // Calculate efficiencies
    if (energy->energy_from_battery_Wh > 0.1f) {
        energy->efficiency_drive_pct = (energy->energy_to_motor_Wh /
                                        energy->energy_from_battery_Wh) * 100.0f;
    }

    // Energy consumption per km
    if (energy->distance_traveled_m > 100.0f) {  // At least 100m traveled
        float distance_km = energy->distance_traveled_m / 1000.0f;
        float net_energy = energy->energy_from_battery_Wh - energy->energy_regen_Wh;
        energy->energy_per_km_Whpkm = net_energy / distance_km;
    }
}

// CAN Message 7: Energy Monitoring (ID: 0x240, 1000ms period)
typedef struct __attribute__((packed)) {
    uint32_t energy_from_battery_Wh;
    uint32_t energy_regen_Wh;
    uint16_t efficiency_drive_pct;
    uint16_t energy_per_km_Whpkm;
} CAN_EnergyMonitoring_t;
```

#### Performance Metrics

```c
// CAN Message 8: Performance Metrics (ID: 0x241, 1000ms period)
typedef struct __attribute__((packed)) {
    uint16_t max_motor_speed_rpm;    // Peak speed recorded
    int16_t max_motor_torque_Nm_x10; // Peak torque recorded
    uint16_t max_motor_power_kW;     // Peak power recorded
    uint32_t motor_operating_hours;  // Total operating hours
    uint32_t distance_total_km;      // Odometer (km)
    uint16_t regen_events_count;     // Number of regen events
    uint16_t avg_regen_power_W;      // Average regen power
} CAN_PerformanceMetrics_t;
```

---

### References for Section 6.3:

**Standards:**
1. **ISO 15118**: "Road Vehicles - Vehicle to Grid Communication Interface"

**Papers:**
1. Yilmaz, M., & Krein, P. T. (2013). "Review of Battery Charger Topologies, Charging Power Levels, and Infrastructure for Plug-In Electric and Hybrid Vehicles." IEEE Transactions on Power Electronics, 28(5), 2151-2169.

---

### 6.4 Fault Reporting and Diagnostics

#### Fault Code System

```c
typedef enum {
    FAULT_NONE = 0x0000,

    // Motor faults (0x01xx)
    FAULT_MOTOR_OVERSPEED = 0x0101,
    FAULT_MOTOR_OVERCURRENT = 0x0102,
    FAULT_MOTOR_OVERTEMP = 0x0103,
    FAULT_MOTOR_STALL = 0x0104,

    // Inverter faults (0x02xx)
    FAULT_INVERTER_OVERVOLTAGE = 0x0201,
    FAULT_INVERTER_UNDERVOLTAGE = 0x0202,
    FAULT_INVERTER_OVERCURRENT = 0x0203,
    FAULT_INVERTER_OVERTEMP = 0x0204,
    FAULT_INVERTER_DESATURATION = 0x0205,

    // Battery faults (0x03xx)
    FAULT_BATTERY_OVERVOLTAGE = 0x0301,
    FAULT_BATTERY_UNDERVOLTAGE = 0x0302,
    FAULT_BATTERY_OVERCURRENT = 0x0303,
    FAULT_BATTERY_OVERTEMP = 0x0304,
    FAULT_BATTERY_COMM_LOSS = 0x0305,

    // Sensor faults (0x04xx)
    FAULT_SENSOR_TEMP = 0x0401,
    FAULT_SENSOR_CURRENT = 0x0402,
    FAULT_SENSOR_VOLTAGE = 0x0403,
    FAULT_SENSOR_POSITION = 0x0404,

    // System faults (0x05xx)
    FAULT_SYSTEM_WATCHDOG = 0x0501,
    FAULT_SYSTEM_CAN_TIMEOUT = 0x0502,
    FAULT_SYSTEM_EEPROM = 0x0503
} FaultCode_t;

typedef struct {
    FaultCode_t code;
    uint32_t timestamp_ms;
    uint8_t severity;           // 0=info, 1=warning, 2=error, 3=critical
    float fault_value;          // Value that triggered fault
    bool active;                // Fault currently active
    uint16_t occurrence_count;  // Number of times fault occurred
} Fault_t;

// CAN Message 9: Active Faults (ID: 0x250, 100ms period)
typedef struct __attribute__((packed)) {
    uint16_t fault_code_1;
    uint16_t fault_code_2;
    uint16_t fault_code_3;
    uint16_t fault_code_4;
} CAN_ActiveFaults_t;

// CAN Message 10: Fault Details (ID: 0x251, on-demand)
typedef struct __attribute__((packed)) {
    uint16_t fault_code;
    uint32_t timestamp_ms;
    uint8_t severity;
    uint16_t fault_value_x10;
    uint8_t occurrence_count;
} CAN_FaultDetails_t;
```

#### Fault Logging and History

```c
#define FAULT_LOG_SIZE 32

typedef struct {
    Fault_t fault_log[FAULT_LOG_SIZE];
    uint8_t fault_log_head;
    uint8_t fault_log_count;
    uint32_t total_faults;
} FaultLogger_t;

void log_fault(FaultLogger_t *logger, Fault_t *fault) {
    // Add to circular buffer
    logger->fault_log[logger->fault_log_head] = *fault;
    logger->fault_log_head = (logger->fault_log_head + 1) % FAULT_LOG_SIZE;

    if (logger->fault_log_count < FAULT_LOG_SIZE) {
        logger->fault_log_count++;
    }

    logger->total_faults++;

    // Send CAN message
    CAN_FaultDetails_t msg;
    msg.fault_code = fault->code;
    msg.timestamp_ms = fault->timestamp_ms;
    msg.severity = fault->severity;
    msg.fault_value_x10 = (uint16_t)(fault->fault_value * 10.0f);
    msg.occurrence_count = fault->occurrence_count;

    can_transmit(0x251, (uint8_t*)&msg, sizeof(msg));

    // Log to non-volatile memory
    save_fault_to_eeprom(fault);
}
```

#### Diagnostic Trouble Codes (DTCs)

```c
// CAN Message 11: DTC Summary (ID: 0x252, 1000ms period)
typedef struct __attribute__((packed)) {
    uint8_t dtc_count_total;        // Total DTCs stored
    uint8_t dtc_count_active;       // Currently active DTCs
    uint8_t dtc_count_critical;     // Critical DTCs
    uint8_t system_health_pct;      // Overall system health (0-100%)
    uint32_t uptime_hours;          // System uptime (hours)
} CAN_DTCSummary_t;
```

---

### References for Section 6.4:

**Standards:**
1. **ISO 14229**: "Road Vehicles - Unified Diagnostic Services (UDS)"
2. **ISO 15765**: "Road Vehicles - Diagnostic on Controller Area Network (CAN)"
3. **SAE J2012**: "Diagnostic Trouble Code Definitions"

**Books:**
1. *"Automotive Diagnostic Systems"* by Keith McCord - Chapter 4 (CAN Diagnostics)

**Application Notes:**
1. **Vector**: "Unified Diagnostic Services (UDS) Implementation Guide"

---

**End of Section 6: CAN Communication and Monitoring**

---

## Section 7: Sensored vs Sensorless Control

One of the most fundamental design decisions in PMSM motor control is whether to use position sensors (Hall sensors, encoders, resolvers) or implement sensorless control algorithms that estimate rotor position from electrical measurements. This section provides a comprehensive comparison of both approaches, detailed sensorless methods, and guidance on selecting the right approach for your application.

---

### 7.1 Introduction and Overview

#### The Position Sensing Challenge

Field Oriented Control requires accurate knowledge of the rotor electrical position (θ_e) to perform the Park and inverse Park transformations. The position can be obtained through:

1. **Sensored Control**: Direct measurement using physical sensors
2. **Sensorless Control**: Estimation from motor electrical quantities (voltages, currents)

**Fundamental Trade-off:**
```
Sensored Control:
  ✓ Simple, robust, predictable performance
  ✗ Higher cost, sensor wiring, reliability concerns

Sensorless Control:
  ✓ Lower cost, no sensor wiring, higher reliability (fewer parts)
  ✗ Complex algorithms, startup challenges, parameter sensitivity
```

#### Why Sensorless Control?

**Cost Reduction:**
- Eliminates sensor cost ($5-$200 depending on type)
- No sensor cables or connectors
- Reduced wiring complexity
- Simplified mechanical design

**Reliability:**
- Fewer components to fail
- No sensor misalignment issues
- No cable damage concerns
- Better for harsh environments (vibration, temperature, moisture)

**Compactness:**
- Smaller motor packaging
- Easier integration in space-constrained applications

**When Sensorless Makes Sense:**
- Consumer appliances (fans, pumps, compressors)
- Cost-sensitive applications
- High-volume production
- Applications with limited low-speed requirements
- Harsh environments where sensors are problematic

**When Sensors Are Better:**
- Safety-critical applications (automotive, aerospace)
- Precision position control (robotics, CNC)
- High torque at zero/low speed required
- Redundancy requirements (ISO 26262, functional safety)
- Applications where sensor cost is negligible compared to system cost

#### Sensorless Control Categories

Modern sensorless control methods fall into two main categories based on the speed range:

**1. Back-EMF Based Methods (Medium to High Speed)**
- Operate above ~10-15% of rated speed
- Use motor back-EMF for position estimation
- Lower computational burden
- Cannot start motor from standstill

**2. High Frequency Injection (HFI) Methods (Zero to Low Speed)**
- Work from standstill to ~20-30% of rated speed
- Exploit magnetic saliency
- Higher computational cost
- Can produce audible noise

**3. Hybrid Methods (Full Speed Range)**
- HFI at low speed, back-EMF at high speed
- Smooth transition between methods
- Best of both worlds

---

### References for Section 7.1:

**Books:**
1. *"Control of Electric Machine Drive Systems"* by Seung-Ki Sul - Chapter 10: Sensorless Control
2. *"Sensorless Vector and Direct Torque Control"* by Peter Vas
3. *"Advanced Electric Drives"* by Ned Mohan - Chapter 18: Sensorless Control

**Papers:**
1. Holtz, J. (2002). "Sensorless Control of Induction Motor Drives." *Proceedings of the IEEE*, 90(8), 1359-1394.
2. Lorenz, R. D. (2006). "The Technology of Sensorless Control." *IEEE Industry Applications Magazine*

**Application Notes:**
1. **Texas Instruments**: "Sensorless Field Oriented Control of 3-Phase PMSMs" (SPRABQ7)
2. **Microchip**: AN1078 - "Sensorless Field Oriented Control of PMSM Motors"
3. **STMicroelectronics**: "Sensorless FOC for PMSM" (AN4680)

---

### 7.2 Sensored Control Methods

Before diving into sensorless methods, let's briefly review sensored control to establish the baseline for comparison.

#### 7.2.1 Hall Effect Sensors

**Principles:**
- 3 Hall sensors spaced 120° electrically
- Provide 6 discrete position states per electrical cycle
- Low resolution (60° electrical per state)

**Advantages:**
- Very low cost ($2-5)
- Simple interface (3 digital signals)
- Robust to EMI
- Works from zero speed

**Disadvantages:**
- Low resolution → torque ripple
- Requires interpolation for smooth control
- Sensitive to mounting accuracy
- Limited to ~120°C typically

**Performance Characteristics:**
```
Position accuracy: ±30° electrical (without interpolation)
Update rate: Varies with speed (6 updates per e-revolution)
Cost: $2-5
Interface: 3 x GPIO
Latency: ~1 μs
Temperature range: -40°C to 120°C (typical)
```

**Typical Implementation:**
```c
// Hall sensor reading (3 digital inputs)
uint8_t read_hall_sensors(void) {
    uint8_t hall = 0;
    hall |= (HAL_GPIO_ReadPin(HALL_U_PORT, HALL_U_PIN) << 2);
    hall |= (HAL_GPIO_ReadPin(HALL_V_PORT, HALL_V_PIN) << 1);
    hall |= (HAL_GPIO_ReadPin(HALL_W_PORT, HALL_W_PIN) << 0);
    return hall & 0x07;  // 0-7
}

// Hall state to electrical angle lookup
const float hall_to_angle[8] = {
    0.0f,      // Invalid state 0
    210.0f,    // State 1: 210°
    330.0f,    // State 2: 330°
    270.0f,    // State 3: 270°
    30.0f,     // State 4: 30°
    0.0f,      // State 5: 0° (or 360°)
    90.0f,     // State 6: 90°
    0.0f       // Invalid state 7
};

float get_electrical_angle_from_hall(void) {
    uint8_t hall_state = read_hall_sensors();
    if (hall_state == 0 || hall_state == 7) {
        // Invalid hall state - use last known angle
        return last_angle;
    }
    return hall_to_angle[hall_state] * (PI / 180.0f);
}
```

#### 7.2.2 Incremental Encoders

**Principles:**
- Optical or magnetic quadrature encoder
- Provides A/B signals + optional index (Z)
- High resolution (100-10,000 PPR typical)

**Advantages:**
- High position accuracy (<0.1° electrical)
- Smooth torque with fine resolution
- Excellent speed estimation
- Works from zero speed

**Disadvantages:**
- Moderate cost ($10-100)
- Requires quadrature decoding hardware
- Relative position only (needs homing for absolute position)
- Sensitive to vibration (optical types)

**Performance Characteristics:**
```
Position accuracy: 360° / (4 × PPR) electrical
Update rate: Continuous (quadrature)
Cost: $10-100
Interface: 2 x quadrature + 1 x index (optional)
Latency: <1 μs
Temperature range: -40°C to 85°C (typical)
```

**Typical Implementation:**
```c
// STM32 encoder interface using Timer in Encoder Mode
void encoder_init(void) {
    // Configure TIM2 in encoder mode
    TIM2->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;  // Encoder mode 3
    TIM2->ARR = 0xFFFF;
    TIM2->CNT = 0;
    TIM2->CR1 = TIM_CR1_CEN;
}

// Read position (handled by hardware)
int32_t read_encoder_position(void) {
    return (int32_t)TIM2->CNT;
}

// Convert to electrical angle
float get_electrical_angle_from_encoder(int32_t pole_pairs) {
    int32_t mech_counts = read_encoder_position();
    float counts_per_e_rev = ENCODER_PPR * 4 / pole_pairs;  // PPR × 4 (quadrature)
    float elec_angle = (mech_counts % (int32_t)counts_per_e_rev) / counts_per_e_rev * 2.0f * PI;
    return elec_angle;
}
```

#### 7.2.3 Resolvers

**Principles:**
- Rotary transformer providing sine/cosine analog outputs
- Requires Resolver-to-Digital Converter (RDC) IC
- Absolute position within one revolution

**Advantages:**
- Extremely robust (automotive-grade)
- Wide temperature range (-55°C to 200°C)
- Immune to vibration, dust, moisture
- High accuracy (0.05° typical)
- Inherent redundancy (sine + cosine)

**Disadvantages:**
- Highest cost ($50-200)
- Complex excitation and decoding circuitry
- Requires RDC chip ($5-20)
- Larger size

**Performance Characteristics:**
```
Position accuracy: ±0.05° electrical (12-bit RDC)
Update rate: 1-10 kHz (RDC dependent)
Cost: $50-200
Interface: Sine/Cosine analog + excitation
Latency: 100-500 μs (RDC conversion)
Temperature range: -55°C to 200°C
```

**Typical Implementation:**
```c
// Using AD2S1210 Resolver-to-Digital Converter
typedef struct {
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *sample_port;
    uint16_t sample_pin;
    float angle_offset;
} Resolver_t;

uint16_t read_resolver_angle(Resolver_t *res) {
    uint8_t tx_data[2] = {0xFF, 0xFF};  // Dummy bytes
    uint8_t rx_data[2];

    // Assert SAMPLE line
    HAL_GPIO_WritePin(res->sample_port, res->sample_pin, GPIO_PIN_RESET);
    delay_us(1);
    HAL_GPIO_WritePin(res->sample_port, res->sample_pin, GPIO_PIN_SET);

    // Read 16-bit angle via SPI
    HAL_SPI_TransmitReceive(res->spi, tx_data, rx_data, 2, 100);

    uint16_t raw_angle = (rx_data[0] << 8) | rx_data[1];
    return raw_angle;  // 0-65535 for 0-360°
}

float get_electrical_angle_from_resolver(Resolver_t *res, uint8_t pole_pairs) {
    uint16_t raw = read_resolver_angle(res);
    float mech_angle = (raw / 65536.0f) * 2.0f * PI;
    float elec_angle = fmodf(mech_angle * pole_pairs + res->angle_offset, 2.0f * PI);
    return elec_angle;
}
```

#### 7.2.4 Sensored Control Summary

**Decision Matrix for Sensors:**

| Application | Recommended Sensor | Rationale |
|-------------|-------------------|-----------|
| Low-cost appliances | Hall sensors | Adequate performance, lowest cost |
| Industrial servo | Encoder (optical) | High precision, smooth operation |
| EV traction | Resolver | Harsh environment, safety, redundancy |
| E-bikes, scooters | Hall or sensorless | Cost optimization |
| Robotics | Encoder (high-res) | Position accuracy critical |
| Aerospace | Resolver (redundant) | Reliability, temperature, vibration |
| HVAC fans | Sensorless | Cost, no low-speed torque needed |

---

### References for Section 7.2:

**Books:**
1. *"Resolver and Encoder Conversion Handbook"* by Analog Devices
2. *"Rotary Encoder Handbook"* by Dynapar

**Application Notes:**
1. **Analog Devices**: "A Resolver-to-Digital Converter for UAV Applications" (AN-1333)
2. **Allegro**: "Hall-Effect IC Applications Guide"
3. **TI**: "Position Sensors for Motor Control Applications" (SLYT527)

**Standards:**
1. **IEC 61326**: Electrical equipment for measurement, control - EMC requirements
2. **MIL-PRF-39016**: Resolvers and Synchros (military specification)

---

### 7.3 Back-EMF Based Sensorless Control

Back-EMF based sensorless methods are the most widely used approach for medium to high-speed operation. They estimate rotor position by observing the motor's back-electromotive force (back-EMF), which is directly proportional to rotor speed.

#### 7.3.1 Fundamental Principle

When a PMSM rotates, the permanent magnets induce voltages in the stator windings called back-EMF:

```
E_backemf = K_e × ω_m

Where:
  E_backemf = back-EMF voltage (V)
  K_e = motor voltage constant (V/rad/s)
  ω_m = mechanical speed (rad/s)
```

**Key Insight:** The back-EMF is perpendicular to the rotor flux (90° electrical ahead). By measuring the back-EMF, we can deduce the rotor position.

**The Problem at Low Speed:**
```
At low speeds:
  ω_m → 0
  E_backemf → 0
  Signal-to-noise ratio becomes very poor
  Position estimation fails below ~10-15% of rated speed
```

#### 7.3.2 Sliding Mode Observer (SMO)

The Sliding Mode Observer is one of the most popular back-EMF estimation methods due to its robustness and relatively simple implementation.

**Theory:**

The SMO uses a switching function to force estimated currents to track measured currents. The switching signal contains information about the back-EMF, which is extracted through filtering.

**Mathematical Model:**

```
Motor voltage equations in αβ frame:

v_α = R_s × i_α + L_s × (di_α/dt) + e_α
v_β = R_s × i_β + L_s × (di_β/dt) + e_β

Where e_α, e_β are the back-EMF components.

Observer equations:

di_α_est/dt = (v_α - R_s × i_α_est - e_α_est) / L_s
di_β_est/dt = (v_β - R_s × i_β_est - e_β_est) / L_s

Sliding mode function:

e_α_est = K_smo × sign(i_α - i_α_est)
e_β_est = K_smo × sign(i_β - i_β_est)

Position extraction:

θ_e = atan2(e_β_est, e_α_est)
```

**C Implementation:**

```c
// Sliding Mode Observer Structure
typedef struct {
    // Motor parameters
    float R_s;              // Stator resistance (Ω)
    float L_s;              // Stator inductance (H)
    float K_e;              // Back-EMF constant (V/rad/s)

    // Observer gains
    float K_smo;            // SMO gain
    float K_filter;         // Low-pass filter coefficient

    // Estimated currents (αβ frame)
    float i_alpha_est;
    float i_beta_est;

    // Estimated back-EMF (αβ frame)
    float e_alpha_est;
    float e_beta_est;

    // Filtered back-EMF (for position extraction)
    float e_alpha_filt;
    float e_beta_filt;

    // Estimated position and speed
    float theta_est;
    float omega_est;

    // Sample time
    float T_s;
} SMO_t;

void smo_init(SMO_t *smo, float R_s, float L_s, float K_e, float T_s) {
    smo->R_s = R_s;
    smo->L_s = L_s;
    smo->K_e = K_e;
    smo->T_s = T_s;

    // Tuning parameters
    smo->K_smo = 0.5f;      // Adjust based on motor and speed
    smo->K_filter = 0.05f;  // Low-pass filter coefficient

    // Initialize states
    smo->i_alpha_est = 0.0f;
    smo->i_beta_est = 0.0f;
    smo->e_alpha_filt = 0.0f;
    smo->e_beta_filt = 0.0f;
    smo->theta_est = 0.0f;
    smo->omega_est = 0.0f;
}

void smo_update(SMO_t *smo, float v_alpha, float v_beta, float i_alpha, float i_beta) {
    // Current estimation error
    float i_alpha_err = i_alpha - smo->i_alpha_est;
    float i_beta_err = i_beta - smo->i_beta_est;

    // Sliding mode switching function (sign function with hysteresis to reduce chattering)
    float sign_alpha = (i_alpha_err > 0.01f) ? 1.0f : ((i_alpha_err < -0.01f) ? -1.0f : 0.0f);
    float sign_beta = (i_beta_err > 0.01f) ? 1.0f : ((i_beta_err < -0.01f) ? -1.0f : 0.0f);

    // Estimated back-EMF (raw)
    smo->e_alpha_est = smo->K_smo * sign_alpha;
    smo->e_beta_est = smo->K_smo * sign_beta;

    // Low-pass filter for back-EMF (reduce chattering)
    smo->e_alpha_filt += smo->K_filter * (smo->e_alpha_est - smo->e_alpha_filt);
    smo->e_beta_filt += smo->K_filter * (smo->e_beta_est - smo->e_beta_filt);

    // Current observer update (Euler integration)
    float di_alpha = (v_alpha - smo->R_s * smo->i_alpha_est - smo->e_alpha_est) / smo->L_s;
    float di_beta = (v_beta - smo->R_s * smo->i_beta_est - smo->e_beta_est) / smo->L_s;

    smo->i_alpha_est += di_alpha * smo->T_s;
    smo->i_beta_est += di_beta * smo->T_s;

    // Position estimation from filtered back-EMF
    smo->theta_est = atan2f(smo->e_beta_filt, smo->e_alpha_filt);

    // Normalize to [0, 2π]
    if (smo->theta_est < 0.0f) {
        smo->theta_est += 2.0f * PI;
    }

    // Speed estimation (from back-EMF magnitude)
    float e_mag = sqrtf(smo->e_alpha_filt * smo->e_alpha_filt +
                        smo->e_beta_filt * smo->e_beta_filt);
    smo->omega_est = e_mag / smo->K_e;
}

// Get estimated position
float smo_get_position(SMO_t *smo) {
    return smo->theta_est;
}

// Get estimated speed
float smo_get_speed(SMO_t *smo) {
    return smo->omega_est;
}
```

**Advantages of SMO:**
- Robust to parameter variations
- Simple implementation
- Works well at medium to high speeds
- Good dynamic response

**Disadvantages:**
- Chattering in switching function (mitigated with filtering)
- Filtering introduces phase delay
- Fails at very low speeds (< 10% rated)
- Sensitive to measurement noise at low speeds

#### 7.3.3 Phase-Locked Loop (PLL) Based Observer

PLL-based observers are another popular approach that uses a feedback loop to track the rotor position.

**Principle:**

A PLL locks onto the phase of the back-EMF vector, continuously adjusting the estimated position to minimize the error.

**Block Diagram:**
```
         e_αβ (measured)
              │
              ↓
    ┌─────────────────┐
    │  Back-EMF       │
    │  Calculation    │
    └─────────────────┘
              │
              ↓
    ┌─────────────────┐      θ_error
    │  Phase Detector │───────────→  ┌──────┐
    │  (cross product)│              │  PI  │
    └─────────────────┘              │ Ctrl │
              ↑                      └──────┘
              │                          │
              │                          ↓ ω_est
              │                      ┌──────┐
              │                      │ ∫ dt │
              │                      └──────┘
              │                          │
              │                          ↓ θ_est
              └──────────────────────────┘
```

**C Implementation:**

```c
// PLL-based Observer Structure
typedef struct {
    // Motor parameters
    float K_e;              // Back-EMF constant
    float pole_pairs;       // Number of pole pairs

    // PLL gains
    float Kp_pll;           // Proportional gain
    float Ki_pll;           // Integral gain

    // PLL states
    float theta_est;        // Estimated position
    float omega_est;        // Estimated speed
    float integral;         // PI controller integral

    // Sample time
    float T_s;
} PLL_Observer_t;

void pll_init(PLL_Observer_t *pll, float K_e, float pole_pairs, float T_s) {
    pll->K_e = K_e;
    pll->pole_pairs = pole_pairs;
    pll->T_s = T_s;

    // Tuning (adjust based on motor dynamics)
    pll->Kp_pll = 500.0f;   // Proportional gain
    pll->Ki_pll = 10000.0f; // Integral gain

    pll->theta_est = 0.0f;
    pll->omega_est = 0.0f;
    pll->integral = 0.0f;
}

void pll_update(PLL_Observer_t *pll, float v_alpha, float v_beta,
                float i_alpha, float i_beta, float R_s, float L_s) {

    // Estimate back-EMF from motor model
    // e = v - R×i - L×(di/dt)
    // Simplified: e ≈ v - R×i (assuming di/dt is small)
    float e_alpha = v_alpha - R_s * i_alpha;
    float e_beta = v_beta - R_s * i_beta;

    // Transform estimated back-EMF to rotating frame using estimated position
    float cos_theta = cosf(pll->theta_est);
    float sin_theta = sinf(pll->theta_est);

    float e_d = e_alpha * cos_theta + e_beta * sin_theta;
    float e_q = -e_alpha * sin_theta + e_beta * cos_theta;

    // Phase error (e_d should be zero when locked)
    // Error signal: cross product of estimated and actual back-EMF
    float phase_error = e_d;  // Simplification: error proportional to e_d

    // PI controller
    pll->integral += phase_error * pll->T_s;

    // Anti-windup
    if (pll->integral > 1000.0f) pll->integral = 1000.0f;
    if (pll->integral < -1000.0f) pll->integral = -1000.0f;

    // Estimated speed (mechanical)
    pll->omega_est = pll->Kp_pll * phase_error + pll->Ki_pll * pll->integral;

    // Integrate speed to get position
    pll->theta_est += pll->omega_est * pll->T_s;

    // Normalize angle
    while (pll->theta_est > 2.0f * PI) pll->theta_est -= 2.0f * PI;
    while (pll->theta_est < 0.0f) pll->theta_est += 2.0f * PI;
}

float pll_get_position(PLL_Observer_t *pll) {
    return pll->theta_est;
}

float pll_get_speed(PLL_Observer_t *pll) {
    return pll->omega_est;
}
```

#### 7.3.4 Flux Linkage Observer

This method integrates the stator voltage equations to estimate flux linkage, from which position is extracted.

**Voltage Equations:**

```
ψ_α = ∫(v_α - R_s × i_α) dt
ψ_β = ∫(v_β - R_s × i_β) dt

θ_e = atan2(ψ_β, ψ_α)
```

**Challenges:**
- **DC drift**: Pure integration accumulates DC offset errors
- **Initial condition**: Unknown flux at startup

**Solution:** Use compensated integrator or high-pass filter

```c
// Flux Linkage Observer with DC Drift Compensation
typedef struct {
    float R_s;
    float psi_alpha;
    float psi_beta;
    float theta_est;
    float omega_est;
    float T_s;
    float drift_comp;  // Drift compensation factor
} FluxObserver_t;

void flux_observer_update(FluxObserver_t *obs, float v_alpha, float v_beta,
                          float i_alpha, float i_beta) {

    // Back-EMF estimation
    float e_alpha = v_alpha - obs->R_s * i_alpha;
    float e_beta = v_beta - obs->R_s * i_beta;

    // Integration with drift compensation
    obs->psi_alpha += (e_alpha - obs->drift_comp * obs->psi_alpha) * obs->T_s;
    obs->psi_beta += (e_beta - obs->drift_comp * obs->psi_beta) * obs->T_s;

    // Position from flux
    obs->theta_est = atan2f(obs->psi_beta, obs->psi_alpha);

    // Speed from flux magnitude change
    float psi_mag = sqrtf(obs->psi_alpha * obs->psi_alpha +
                          obs->psi_beta * obs->psi_beta);
    obs->omega_est = psi_mag / K_FLUX_LINKAGE;  // Motor-specific constant
}
```

#### 7.3.5 Back-EMF Method Comparison

| Method | Complexity | Robustness | Min Speed | Accuracy | CPU Load |
|--------|-----------|------------|-----------|----------|----------|
| **Sliding Mode Observer** | Medium | High | 10-15% | Good | Low-Medium |
| **PLL Observer** | Medium | Medium | 10-15% | Very Good | Medium |
| **Flux Linkage** | Low | Low (drift) | 15-20% | Medium | Low |
| **Extended Kalman Filter** | High | Very High | 10-15% | Excellent | High |

#### 7.3.6 Startup Strategy for Back-EMF Methods

Since back-EMF methods don't work at zero speed, special startup strategies are needed:

**Method 1: Open-Loop V/f Ramp**
```c
void startup_open_loop_ramp(void) {
    float angle = 0.0f;
    float omega = STARTUP_OMEGA_MIN;  // Start at minimum speed
    float accel = STARTUP_ACCEL;      // rad/s²

    while (omega < OMEGA_TRANSITION) {  // Until back-EMF is strong enough
        // Open-loop position
        angle += omega * T_s;
        if (angle > 2.0f * PI) angle -= 2.0f * PI;

        // Calculate voltage magnitude (V/f control)
        float v_mag = V_F_RATIO * omega;

        // Apply voltage
        foc_set_voltage(v_mag, 0.0f, angle);  // id=0, vq based on V/f

        // Ramp up speed
        omega += accel * T_s;

        delay_ms(T_s * 1000);
    }

    // Transition to sensorless observer
    enable_sensorless_observer();
}
```

**Method 2: I-f Startup with Alignment**
```c
void startup_with_alignment(void) {
    // Step 1: Align rotor to known position
    float align_current = 2.0f;  // Alignment current (A)
    foc_set_current(align_current, 0.0f, 0.0f);  // id, iq, angle=0
    delay_ms(500);  // Wait for alignment

    // Step 2: Ramp speed in open loop
    float angle = 0.0f;
    float omega = 0.0f;
    while (omega < OMEGA_TRANSITION) {
        angle += omega * T_s;
        foc_set_current(0.0f, TORQUE_CURRENT, angle);
        omega += STARTUP_ACCEL * T_s;
        delay_ms(T_s * 1000);
    }

    // Step 3: Hand over to sensorless
    enable_sensorless_observer();
}
```

---

### References for Section 7.3:

**Books:**
1. *"Sensorless Vector and Direct Torque Control"* by Peter Vas - Chapter 3: Back-EMF Based Methods
2. *"Control of Electric Machine Drive Systems"* by Seung-Ki Sul - Chapter 10.2: Observer-Based Sensorless Control

**Papers:**
1. Morimoto, S., et al. (2002). "Sensorless Control Strategy for Salient-Pole PMSM Based on Extended EMF in Rotating Reference Frame." *IEEE Trans. on Industry Applications*
2. Chen, Z., et al. (2003). "A Sliding Mode Observer for Sensorless Control of PMSM." *IEEE Power Electronics Specialists Conference*
3. Bolognani, S., et al. (1999). "Extended Kalman Filter Tuning in Sensorless PMSM Drives." *IEEE Trans. on Industry Applications*

**Application Notes:**
1. **Microchip**: AN1078 - "Sensorless FOC for PMSM" (includes SMO implementation)
2. **STMicroelectronics**: AN4680 - "Sensorless FOC for PMSM using State Observer PLL"
3. **Texas Instruments**: SPRABQ2 - "Sensorless FOC with Back-EMF Observer"
4. **Infineon**: AP32370 - "Sensorless Field Oriented Control"

**Websites:**
1. **SimpleFOC**: Open-source library with back-EMF observer implementations
2. **VESC Project**: Open-source motor controller with robust sensorless algorithms

---

### 7.4 High Frequency Injection (HFI) Methods

High Frequency Injection is a powerful sensorless control technique that works from zero speed to low/medium speeds by exploiting the **magnetic saliency** of the motor. Unlike back-EMF methods, HFI doesn't require rotor motion, making it ideal for startup and low-speed operation.

#### 7.4.1 Fundamental Principle: Magnetic Saliency

**What is Magnetic Saliency?**

Magnetic saliency means that the motor's inductance varies with rotor position. This is naturally present in:
- **IPM motors** (Interior Permanent Magnet): Inherently salient due to magnet placement
- **SPM motors** (Surface Permanent Magnet): Minimal saliency, but some exists due to stator slotting

**Inductance Variation:**

```
For IPM motors:

L_d ≠ L_q  (d-axis inductance ≠ q-axis inductance)

Typically: L_q > L_d (by 20-50%)

This creates position-dependent reluctance.
```

**HFI Core Idea:**

1. Inject a high-frequency (HF) voltage signal (e.g., 500 Hz - 2 kHz)
2. Due to saliency, the resulting HF current response varies with rotor position
3. Extract position information from the HF current by signal processing
4. Works even at zero speed!

**Key Advantage:** Position information is embedded in the electromagnetic structure, not dependent on motion (back-EMF).

**Key Disadvantage:**
- Requires saliency (doesn't work well on non-salient SPM motors)
- Audible noise from HF injection
- Higher computational load
- Acoustic emissions may be problematic in some applications

#### 7.4.2 Rotating High Frequency Injection

This is the classic HFI method where a rotating voltage vector is injected in the stationary (αβ) frame.

**Theory:**

Inject a rotating HF voltage:
```
v_αβ_hf = V_hf × cos(ω_hf × t + θ_hf_inj)

Where:
  V_hf = HF voltage amplitude (typ. 5-10% of rated voltage)
  ω_hf = HF injection frequency (typ. 500-2000 Hz)
  θ_hf_inj = injected HF angle
```

The resulting HF current contains the rotor position:
```
i_αβ_hf ≈ (I_hf_avg + I_hf_sal × cos(2(θ_hf_inj - θ_rotor)))

Where:
  θ_rotor = actual rotor position (what we want to estimate)
  I_hf_sal = saliency-dependent current amplitude
```

The current modulation at **twice** the position error frequency allows extraction of rotor position.

**C Implementation:**

```c
// Rotating HFI Structure
typedef struct {
    // HF injection parameters
    float V_hf;             // HF voltage amplitude (V)
    float omega_hf;         // HF frequency (rad/s), typically 500-2000 Hz
    float theta_hf_inj;     // Injected HF angle

    // Demodulation
    float theta_est;        // Estimated rotor position
    float omega_est;        // Estimated speed
    float position_error;   // Position tracking error

    // Tracking observer (PLL for position)
    float Kp_track;         // Proportional gain
    float Ki_track;         // Integral gain
    float integral;         // Integrator state

    // Sample time
    float T_s;

    // Band-pass filter states for HF current extraction
    float i_alpha_hf;
    float i_beta_hf;
} RotatingHFI_t;

void rotating_hfi_init(RotatingHFI_t *hfi, float T_s) {
    hfi->V_hf = 2.0f;           // 2V HF injection
    hfi->omega_hf = 2.0f * PI * 1000.0f;  // 1000 Hz
    hfi->theta_hf_inj = 0.0f;
    hfi->theta_est = 0.0f;
    hfi->omega_est = 0.0f;
    hfi->T_s = T_s;

    // Tuning
    hfi->Kp_track = 100.0f;
    hfi->Ki_track = 2000.0f;
    hfi->integral = 0.0f;
}

void rotating_hfi_inject(RotatingHFI_t *hfi, float *v_alpha_out, float *v_beta_out) {
    // Generate rotating HF voltage vector
    *v_alpha_out += hfi->V_hf * cosf(hfi->theta_hf_inj);
    *v_beta_out += hfi->V_hf * sinf(hfi->theta_hf_inj);

    // Update injection angle
    hfi->theta_hf_inj += hfi->omega_hf * hfi->T_s;
    if (hfi->theta_hf_inj > 2.0f * PI) hfi->theta_hf_inj -= 2.0f * PI;
}

void rotating_hfi_process(RotatingHFI_t *hfi, float i_alpha, float i_beta) {
    // Step 1: Extract HF current component using band-pass filter
    // (Simplified - in practice, use proper BPF or demodulation)
    // Here we assume i_alpha, i_beta have been high-pass filtered

    hfi->i_alpha_hf = i_alpha;  // Assume already filtered
    hfi->i_beta_hf = i_beta;

    // Step 2: Demodulate to extract position error
    // Transform HF current to estimated rotor frame
    float cos_est = cosf(hfi->theta_est);
    float sin_est = sinf(hfi->theta_est);

    float i_d_hf = hfi->i_alpha_hf * cos_est + hfi->i_beta_hf * sin_est;
    float i_q_hf = -hfi->i_alpha_hf * sin_est + hfi->i_beta_hf * cos_est;

    // Step 3: Extract position error from demodulated current
    // Position error is proportional to i_q_hf (simplified)
    hfi->position_error = i_q_hf;

    // Step 4: Track position using PLL
    hfi->integral += hfi->position_error * hfi->T_s;

    // Anti-windup
    if (hfi->integral > 10.0f) hfi->integral = 10.0f;
    if (hfi->integral < -10.0f) hfi->integral = -10.0f;

    // Estimated speed
    hfi->omega_est = hfi->Kp_track * hfi->position_error + hfi->Ki_track * hfi->integral;

    // Update position estimate
    hfi->theta_est += hfi->omega_est * hfi->T_s;

    // Normalize
    while (hfi->theta_est > 2.0f * PI) hfi->theta_est -= 2.0f * PI;
    while (hfi->theta_est < 0.0f) hfi->theta_est += 2.0f * PI;
}

float rotating_hfi_get_position(RotatingHFI_t *hfi) {
    return hfi->theta_est;
}
```

**Advantages:**
- Works at zero speed
- Good accuracy
- Relatively simple demodulation

**Disadvantages:**
- Audible noise (1-2 kHz tone)
- Interferes with fundamental current control
- Not suitable for non-salient motors

#### 7.4.3 Pulsating High Frequency Injection

Instead of a rotating vector, inject a pulsating (alternating) voltage along a single axis.

**Theory:**

Inject pulsating voltage in estimated d-axis:
```
v_d_hf = V_hf × cos(ω_hf × t)
v_q_hf = 0

Where injection is in estimated dq frame.
```

Due to position error, a HF current appears in q-axis:
```
i_q_hf ∝ sin(2 × position_error) × cos(ω_hf × t)

Extract position error from i_q_hf demodulation.
```

**C Implementation:**

```c
// Pulsating HFI Structure
typedef struct {
    float V_hf;             // HF voltage amplitude
    float omega_hf;         // HF frequency (rad/s)
    float hf_phase;         // Current HF phase

    float theta_est;        // Estimated position
    float omega_est;        // Estimated speed

    // Demodulation
    float i_q_hf_filt;      // Filtered q-axis HF current
    float position_error;   // Position error signal

    // Tracking PLL
    float Kp_track;
    float Ki_track;
    float integral;

    float T_s;
} PulsatingHFI_t;

void pulsating_hfi_init(PulsatingHFI_t *hfi, float T_s) {
    hfi->V_hf = 2.0f;
    hfi->omega_hf = 2.0f * PI * 1000.0f;  // 1 kHz
    hfi->hf_phase = 0.0f;
    hfi->theta_est = 0.0f;
    hfi->omega_est = 0.0f;
    hfi->T_s = T_s;

    hfi->Kp_track = 100.0f;
    hfi->Ki_track = 2000.0f;
    hfi->integral = 0.0f;
}

void pulsating_hfi_inject(PulsatingHFI_t *hfi, float *v_d_out, float *v_q_out) {
    // Inject pulsating voltage in d-axis only
    float hf_carrier = cosf(hfi->hf_phase);
    *v_d_out += hfi->V_hf * hf_carrier;
    *v_q_out += 0.0f;  // No q-axis injection

    // Update HF phase
    hfi->hf_phase += hfi->omega_hf * hfi->T_s;
    if (hfi->hf_phase > 2.0f * PI) hfi->hf_phase -= 2.0f * PI;
}

void pulsating_hfi_process(PulsatingHFI_t *hfi, float i_d, float i_q) {
    // Step 1: Extract HF component from q-axis current
    // (Requires band-pass filter centered at omega_hf - not shown)
    float i_q_hf = i_q;  // Assume i_q contains only HF component after filtering

    // Step 2: Demodulate using HF carrier
    float hf_carrier = cosf(hfi->hf_phase);
    float demod_signal = i_q_hf * hf_carrier;

    // Step 3: Low-pass filter demodulated signal
    float alpha_lpf = 0.1f;  // LPF coefficient
    hfi->i_q_hf_filt += alpha_lpf * (demod_signal - hfi->i_q_hf_filt);

    // Step 4: Position error is proportional to filtered signal
    hfi->position_error = hfi->i_q_hf_filt;

    // Step 5: Track position with PLL
    hfi->integral += hfi->position_error * hfi->T_s;

    // Anti-windup
    if (hfi->integral > 10.0f) hfi->integral = 10.0f;
    if (hfi->integral < -10.0f) hfi->integral = -10.0f;

    hfi->omega_est = hfi->Kp_track * hfi->position_error + hfi->Ki_track * hfi->integral;
    hfi->theta_est += hfi->omega_est * hfi->T_s;

    // Normalize
    while (hfi->theta_est > 2.0f * PI) hfi->theta_est -= 2.0f * PI;
    while (hfi->theta_est < 0.0f) hfi->theta_est += 2.0f * PI;
}
```

**Advantages over Rotating HFI:**
- Less interference with fundamental control (injected in d-axis only)
- Slightly lower acoustic noise
- Easier to implement filtering

**Disadvantages:**
- Similar audible noise issues
- Requires good saliency
- Polarity detection needed (N-S ambiguity)

#### 7.4.4 Square Wave Injection

Instead of sinusoidal injection, use square wave voltage pulses.

**Theory:**

Inject short voltage pulses and measure current response:
```
Apply: v_d = +V_pulse for Δt
Measure: Δi_d

Apply: v_d = -V_pulse for Δt
Measure: Δi_d

Current response depends on inductance, which varies with position.
```

**Advantages:**
- Can inject at PWM frequency (no additional HF needed)
- Lower acoustic noise (random/spread spectrum)
- Simpler implementation

**Disadvantages:**
- Requires fast ADC sampling
- More sensitive to measurement noise
- Requires careful timing with PWM

**Simplified Concept:**
```c
// Square wave injection at PWM updates
void square_wave_hfi_process(float i_d_sample1, float i_d_sample2, float *theta_est) {
    // Sample current before and after voltage pulse
    float delta_i_d = i_d_sample2 - i_d_sample1;

    // Delta current is inversely proportional to inductance
    // L_d varies with position for salient motors
    // Extract position from inductance variation

    // (Implementation requires advanced signal processing)
    // This is a simplified representation
}
```

#### 7.4.5 Saliency Requirements for HFI

**HFI works best when:**

```
Saliency ratio ξ = L_q / L_d

Excellent (IPM):  ξ = 1.5 - 3.0
Good:             ξ = 1.2 - 1.5
Marginal (SPM):   ξ = 1.05 - 1.2
Poor (SPM):       ξ < 1.05
```

**For SPM motors with low saliency:**
- Use higher HF voltage
- More sophisticated signal processing
- May not be reliable
- Consider back-EMF methods only

**For IPM motors:**
- Natural saliency makes HFI very effective
- Can work reliably down to zero speed
- Excellent choice for traction applications

#### 7.4.6 Acoustic Noise Mitigation

HFI produces audible noise due to magnetostrictive forces at HF.

**Mitigation Strategies:**

1. **Frequency Selection:**
   ```c
   // Choose HF above human hearing (> 16 kHz)
   // Trade-off: Higher frequency = harder to filter, more switching losses
   float omega_hf_low_noise = 2.0f * PI * 18000.0f;  // 18 kHz
   ```

2. **Random Frequency Modulation:**
   ```c
   // Spread spectrum to reduce tonal noise
   void hfi_random_freq_modulation(RotatingHFI_t *hfi) {
       float freq_variation = ((rand() % 200) - 100) * 2.0f * PI;  // ±100 Hz
       hfi->omega_hf = 2.0f * PI * 1000.0f + freq_variation;
   }
   ```

3. **Amplitude Modulation:**
   ```c
   // Reduce HF amplitude at higher speeds (back-EMF becomes available)
   void hfi_amplitude_scaling(RotatingHFI_t *hfi, float motor_speed) {
       if (motor_speed < 100.0f) {
           hfi->V_hf = 2.0f;  // Full HF voltage at low speed
       } else if (motor_speed < 300.0f) {
           // Ramp down linearly
           hfi->V_hf = 2.0f * (1.0f - (motor_speed - 100.0f) / 200.0f);
       } else {
           hfi->V_hf = 0.0f;  // Disable HFI at higher speeds
       }
   }
   ```

#### 7.4.7 HFI Implementation Challenges

**Challenge 1: Signal Processing**
- Need band-pass filters for HF current extraction
- Demodulation requires trigonometric calculations
- Phase delay from filtering affects performance

**Challenge 2: Computational Load**
- HFI adds 20-40% CPU overhead
- Need fast MCU with FPU (STM32F4, C2000)
- Optimize using lookup tables for trig functions

**Challenge 3: Parameter Sensitivity**
- Requires accurate L_d, L_q values
- Temperature variation affects inductance
- Saturation changes saliency

**Challenge 4: Polarity Detection**
- HFI can't distinguish N from S pole (180° ambiguity)
- Need initial polarity detection routine
- Magnetic polarity test at startup

**Polarity Detection Code:**
```c
// Initial polarity detection for HFI
float detect_magnetic_polarity(void) {
    // Apply positive d-axis current
    foc_set_current(2.0f, 0.0f, 0.0f);  // id=2A, iq=0, angle=0
    delay_ms(100);

    // Apply negative d-axis current
    foc_set_current(-2.0f, 0.0f, 0.0f);
    delay_ms(100);

    // Measure voltage or current response
    // If response is symmetric: rotor aligned with d-axis
    // If asymmetric: rotor is at d-axis + 180°

    // (Detailed implementation requires careful measurement)

    return 0.0f;  // or PI depending on detected polarity
}
```

#### 7.4.8 When to Use HFI

**✅ Use HFI When:**
- IPM motor with good saliency (ξ > 1.3)
- Zero/low speed torque required
- Startup from standstill needed
- Cost savings from eliminating sensors important
- Application can tolerate slight acoustic noise

**❌ Avoid HFI When:**
- SPM motor with low saliency (ξ < 1.1)
- Quiet operation is critical (e.g., HVAC, medical)
- Simple control is preferred
- MCU has limited computational power
- High-speed operation only (use back-EMF instead)

---

### References for Section 7.4:

**Books:**
1. *"Sensorless Vector and Direct Torque Control"* by Peter Vas - Chapter 4: High Frequency Injection
2. *"Control of Electric Machine Drive Systems"* by Seung-Ki Sul - Chapter 10.3: Signal Injection Methods

**Papers (Foundational):**
1. Holtz, J. (2006). "Initial Rotor Polarity Detection and Sensorless Control of PM Synchronous Machines." *IEEE Industrial Electronics Conference*
2. Jansen, P. L., & Lorenz, R. D. (1995). "Transducerless Position and Velocity Estimation in Induction and Salient AC Machines." *IEEE Trans. on Industry Applications*, 31(2), 240-247.
3. Corley, M. J., & Lorenz, R. D. (1998). "Rotor Position and Velocity Estimation for a Salient-Pole Permanent Magnet Synchronous Machine at Standstill and High Speeds." *IEEE Trans. on Industry Applications*, 34(4), 784-789.

**Papers (Advanced/Recent):**
1. Kim, S., et al. (2011). "Acoustic Noise Reduction of Pulsating Torque Using Switching Frequency Modulation in PMSM Sensorless Drives." *IEEE Trans. on Industrial Electronics*
2. Wang, G., et al. (2014). "Position Sensorless Permanent Magnet Synchronous Machine Drives—A Review." *IEEE Trans. on Industrial Electronics*, 67(7), 5830-5842.

**Application Notes:**
1. **Texas Instruments**: "Sensorless Control with HFI for IPM Motors" (Application Report)
2. **Infineon**: "High Frequency Injection for Sensorless PMSM Control" (AN2018-15)
3. **STMicroelectronics**: "HFI-Based Sensorless Control" (Technical Note)

**Implementation References:**
1. **VESC Project** (GitHub): vedderb/bldc - Includes HFI implementation
2. **ODrive** (GitHub): odriverobotics/ODrive - Sensorless control with HFI

---

### 7.5 Hybrid Sensorless Methods and Transition Strategies

For full-speed-range sensorless control, combining HFI at low speeds with back-EMF observers at high speeds provides the best of both worlds. The key challenge is achieving smooth, stable transitions between the two methods.

#### 7.5.1 Why Hybrid Methods?

**Speed Range Coverage:**

```
Speed Range          | Recommended Method
---------------------|--------------------
0 - 10% rated speed  | HFI (back-EMF too weak)
10% - 30% rated      | Transition zone (both methods)
30% - 100% rated     | Back-EMF observer (HFI unnecessary, noisy)
Above base speed     | Back-EMF observer only
```

**Benefits:**
- HFI provides reliable startup from standstill
- Back-EMF observer reduces acoustic noise at higher speeds
- Lower computational load at cruising speeds
- Better overall efficiency (no HF losses at high speed)

#### 7.5.2 Transition Region Design

**Transition Speed Selection:**

```c
// Transition thresholds
#define SPEED_HFI_ONLY          50.0f   // Below this: HFI only (rad/s)
#define SPEED_TRANSITION_START  100.0f  // Start blending
#define SPEED_TRANSITION_END    200.0f  // Above this: Back-EMF only
#define SPEED_HYSTERESIS        20.0f   // Hysteresis to prevent chattering
```

**Transition Strategies:**

**1. Hard Switching (Simple but can cause disturbance):**
```c
typedef enum {
    SENSORLESS_MODE_HFI,
    SENSORLESS_MODE_BEMF,
    SENSORLESS_MODE_TRANSITION
} SensorlessMode_t;

SensorlessMode_t select_sensorless_mode(float speed_abs) {
    static SensorlessMode_t current_mode = SENSORLESS_MODE_HFI;

    if (current_mode == SENSORLESS_MODE_HFI) {
        if (speed_abs > SPEED_TRANSITION_END + SPEED_HYSTERESIS) {
            current_mode = SENSORLESS_MODE_BEMF;
        }
    } else if (current_mode == SENSORLESS_MODE_BEMF) {
        if (speed_abs < SPEED_TRANSITION_START - SPEED_HYSTERESIS) {
            current_mode = SENSORLESS_MODE_HFI;
        }
    }

    return current_mode;
}
```

**2. Soft Blending (Smoother, recommended):**
```c
typedef struct {
    // Observers
    RotatingHFI_t hfi;
    SMO_t smo;

    // Blending
    float blend_factor;     // 0 = HFI only, 1 = back-EMF only
    float theta_blended;    // Blended position estimate
    float omega_blended;    // Blended speed estimate

    // Transition parameters
    float speed_trans_start;
    float speed_trans_end;
} HybridObserver_t;

void hybrid_observer_init(HybridObserver_t *hyb, float T_s) {
    rotating_hfi_init(&hyb->hfi, T_s);
    smo_init(&hyb->smo, R_S, L_S, K_E, T_s);

    hyb->blend_factor = 0.0f;  // Start with HFI
    hyb->speed_trans_start = 100.0f;  // rad/s
    hyb->speed_trans_end = 200.0f;    // rad/s
}

void hybrid_observer_update(HybridObserver_t *hyb, float v_alpha, float v_beta,
                            float i_alpha, float i_beta, float speed_abs) {

    // Run both observers
    rotating_hfi_process(&hyb->hfi, i_alpha, i_beta);
    smo_update(&hyb->smo, v_alpha, v_beta, i_alpha, i_beta);

    // Calculate blend factor based on speed
    if (speed_abs < hyb->speed_trans_start) {
        hyb->blend_factor = 0.0f;  // HFI only
    } else if (speed_abs > hyb->speed_trans_end) {
        hyb->blend_factor = 1.0f;  // Back-EMF only
    } else {
        // Linear blending in transition region
        hyb->blend_factor = (speed_abs - hyb->speed_trans_start) /
                           (hyb->speed_trans_end - hyb->speed_trans_start);
    }

    // Blend position estimates
    float theta_hfi = rotating_hfi_get_position(&hyb->hfi);
    float theta_smo = smo_get_position(&hyb->smo);

    // Handle angle wrap-around for smooth blending
    float theta_diff = theta_smo - theta_hfi;
    if (theta_diff > PI) theta_diff -= 2.0f * PI;
    if (theta_diff < -PI) theta_diff += 2.0f * PI;

    hyb->theta_blended = theta_hfi + hyb->blend_factor * theta_diff;

    // Normalize
    while (hyb->theta_blended > 2.0f * PI) hyb->theta_blended -= 2.0f * PI;
    while (hyb->theta_blended < 0.0f) hyb->theta_blended += 2.0f * PI;

    // Blend speed estimates
    float omega_hfi = rotating_hfi_get_speed(&hyb->hfi);
    float omega_smo = smo_get_speed(&hyb->smo);
    hyb->omega_blended = omega_hfi * (1.0f - hyb->blend_factor) +
                        omega_smo * hyb->blend_factor;
}

float hybrid_observer_get_position(HybridObserver_t *hyb) {
    return hyb->theta_blended;
}

float hybrid_observer_get_speed(HybridObserver_t *hyb) {
    return hyb->omega_blended;
}

// Control HFI injection based on blend factor
void hybrid_observer_injection_control(HybridObserver_t *hyb) {
    // Gradually reduce HFI amplitude as back-EMF takes over
    hyb->hfi.V_hf = V_HF_NOMINAL * (1.0f - hyb->blend_factor);
}
```

#### 7.5.3 PLL-Based Position Tracking for Stability

To ensure smooth handover, use a Phase-Locked Loop that can track either observer:

```c
typedef struct {
    float theta_tracked;
    float omega_tracked;
    float Kp_pll;
    float Ki_pll;
    float integral;
    float T_s;
} TrackingPLL_t;

void tracking_pll_init(TrackingPLL_t *pll, float T_s) {
    pll->theta_tracked = 0.0f;
    pll->omega_tracked = 0.0f;
    pll->Kp_pll = 200.0f;
    pll->Ki_pll = 5000.0f;
    pll->integral = 0.0f;
    pll->T_s = T_s;
}

void tracking_pll_update(TrackingPLL_t *pll, float theta_observed) {
    // Calculate phase error
    float phase_error = theta_observed - pll->theta_tracked;

    // Wrap error to [-π, π]
    while (phase_error > PI) phase_error -= 2.0f * PI;
    while (phase_error < -PI) phase_error += 2.0f * PI;

    // PI controller
    pll->integral += phase_error * pll->T_s;

    // Anti-windup
    if (pll->integral > 100.0f) pll->integral = 100.0f;
    if (pll->integral < -100.0f) pll->integral = -100.0f;

    // Speed output
    pll->omega_tracked = pll->Kp_pll * phase_error + pll->Ki_pll * pll->integral;

    // Integrate to get position
    pll->theta_tracked += pll->omega_tracked * pll->T_s;

    // Normalize
    while (pll->theta_tracked > 2.0f * PI) pll->theta_tracked -= 2.0f * PI;
    while (pll->theta_tracked < 0.0f) pll->theta_tracked += 2.0f * PI;
}
```

#### 7.5.4 Complete Hybrid Sensorless Startup Sequence

```c
typedef enum {
    STARTUP_INIT,
    STARTUP_POLARITY_DETECT,
    STARTUP_HFI_STABILIZE,
    STARTUP_OPEN_LOOP_RAMP,
    STARTUP_HFI_ONLY,
    STARTUP_TRANSITION,
    RUNNING_BEMF_ONLY
} StartupState_t;

void hybrid_sensorless_startup_state_machine(void) {
    static StartupState_t state = STARTUP_INIT;
    static uint32_t timer = 0;

    switch (state) {
        case STARTUP_INIT:
            // Initialize all observers
            hybrid_observer_init(&hybrid_obs, T_S);
            timer = 0;
            state = STARTUP_POLARITY_DETECT;
            break;

        case STARTUP_POLARITY_DETECT:
            // Detect magnetic polarity for HFI
            float polarity = detect_magnetic_polarity();
            hybrid_obs.hfi.theta_est = polarity;
            timer = 0;
            state = STARTUP_HFI_STABILIZE;
            break;

        case STARTUP_HFI_STABILIZE:
            // Run HFI at standstill to stabilize estimate
            hybrid_observer_update(&hybrid_obs, v_alpha, v_beta, i_alpha, i_beta, 0.0f);
            timer++;

            if (timer > 500) {  // 500 ms stabilization
                state = STARTUP_OPEN_LOOP_RAMP;
                timer = 0;
            }
            break;

        case STARTUP_OPEN_LOOP_RAMP:
            // Open-loop ramp with HFI position tracking
            // ... (ramp motor speed gradually)

            if (motor_speed > SPEED_HFI_RELIABLE) {
                state = STARTUP_HFI_ONLY;
            }
            break;

        case STARTUP_HFI_ONLY:
            // Run with HFI only below transition speed
            hybrid_observer_update(&hybrid_obs, v_alpha, v_beta, i_alpha, i_beta, motor_speed);

            if (motor_speed > SPEED_TRANSITION_START) {
                state = STARTUP_TRANSITION;
            }
            break;

        case STARTUP_TRANSITION:
            // Blending region - both observers running
            hybrid_observer_update(&hybrid_obs, v_alpha, v_beta, i_alpha, i_beta, motor_speed);

            if (motor_speed > SPEED_TRANSITION_END) {
                state = RUNNING_BEMF_ONLY;
            }
            break;

        case RUNNING_BEMF_ONLY:
            // Back-EMF observer only
            hybrid_observer_update(&hybrid_obs, v_alpha, v_beta, i_alpha, i_beta, motor_speed);

            // Can transition back if speed drops
            if (motor_speed < SPEED_TRANSITION_START - SPEED_HYSTERESIS) {
                state = STARTUP_HFI_ONLY;
            }
            break;
    }
}
```

#### 7.5.5 Transition Challenges and Solutions

**Challenge 1: Observer Disagreement**

During transition, HFI and back-EMF may give slightly different position estimates.

**Solution:** PLL tracking + slow blend rate
```c
// Use low-pass filtered blend factor for smoother transition
float blend_alpha = 0.01f;  // Slow blend rate
blend_factor_filt += blend_alpha * (blend_factor_target - blend_factor_filt);
```

**Challenge 2: Noise in Transition Region**

Back-EMF is still weak, HFI is being reduced.

**Solution:** Maximize overlap, keep both observers active
```c
// Run both observers in transition, even if one is dominant
// This provides backup if one fails
```

**Challenge 3: Load Disturbances**

Sudden load changes during transition can cause instability.

**Solution:** Increase PLL bandwidth temporarily during transition
```c
if (in_transition_region) {
    tracking_pll.Kp_pll *= 2.0f;  // More aggressive tracking
    tracking_pll.Ki_pll *= 1.5f;
}
```

---

### References for Section 7.5:

**Papers:**
1. Kim, H., et al. (2011). "A New Hybrid Method for Rotor Position Estimation of IPMSM Using HFI and Back-EMF." *IEEE Energy Conversion Congress and Exposition*
2. Yoon, Y., et al. (2013). "Hybrid Observer with Smooth Transition for Sensorless PMSM Drives." *IEEE Trans. on Industrial Electronics*, 60(7), 2789-2797.
3. Liu, J., & Zhu, Z. Q. (2014). "Sensorless Control Strategy by Square-Wave Injection Into Stationary Reference Frame for PMSM." *IEEE Trans. on Industrial Electronics*, 61(9), 4672-4682.

**Application Notes:**
1. **Infineon**: "Hybrid Sensorless Control for PMSM" (Application Note)
2. **Texas Instruments**: "Transitioning Between Sensorless Methods" (Application Report)

---

### 7.6 Comparison and Selection Guide

This section provides a comprehensive decision framework for choosing between sensored and sensorless control, and selecting the appropriate sensorless method.

#### 7.6.1 Complete Method Comparison Matrix

| Criterion | Hall Sensors | Encoder | Resolver | Back-EMF | HFI | Hybrid |
|-----------|--------------|---------|----------|----------|-----|--------|
| **Cost** | $ | $$ | $$$$ | Free | Free | Free |
| **Zero Speed** | ✓ | ✓ | ✓ | ✗ | ✓ | ✓ |
| **High Speed** | ✓ | ✓ | ✓ | ✓✓ | ✗ | ✓✓ |
| **Accuracy** | Poor | Excellent | Excellent | Good | Good | Good |
| **Reliability** | Good | Good (mag) | Excellent | Good | Good | Good |
| **CPU Load** | Minimal | Minimal | Low | Medium | High | High |
| **Acoustic Noise** | None | None | None | None | Present | Low-Med |
| **SPM Motors** | ✓ | ✓ | ✓ | ✓ | Limited | Limited |
| **IPM Motors** | ✓ | ✓ | ✓ | ✓ | ✓✓ | ✓✓ |
| **Calibration** | Required | Optional | Required | Yes | Yes | Yes |
| **Startup** | Instant | Instant | Instant | Ramp | Instant | Instant |

#### 7.6.2 Application-Specific Recommendations

**Electric Vehicle Traction Motors:**
```
✓ Primary: Resolver (safety-critical)
✓ Backup/Alternative: Hybrid sensorless (HFI + back-EMF)
✗ Not recommended: Hall only, encoder (vibration)

Rationale:
- Resolver: Extreme reliability, temperature, vibration resistance
- Hybrid sensorless: Cost reduction, full speed range
- IPM motors have natural saliency for HFI
```

**E-bikes / E-scooters:**
```
✓ Primary: Hall sensors
✓ Alternative: Hybrid sensorless (cost-optimized)
✗ Not recommended: Encoder, resolver (overkill)

Rationale:
- Hall: Good enough performance, low cost
- Sensorless: Further cost reduction for high volume
```

**Industrial Servo / CNC:**
```
✓ Primary: High-resolution encoder (absolute or incremental + index)
✓ Alternative: Resolver (harsh environment)
✗ Not recommended: Sensorless (precision requirement)

Rationale:
- Encoder: Precision position control essential
- Absolute feedback required for safety
```

**HVAC Fans / Pumps:**
```
✓ Primary: Sensorless back-EMF
✗ Not recommended: Any sensor (cost), HFI (noise)

Rationale:
- No low-speed operation required
- Cost-critical application
- Startup ramp is acceptable
```

**Robotics / Drones:**
```
✓ Primary: High-res encoder (magnetic preferred)
✓ Alternative: Hybrid sensorless (weight-constrained)
✗ Not recommended: Resolver (too heavy)

Rationale:
- Precision control needed
- Weight is critical
- Magnetic encoder good vibration resistance
```

**Home Appliances (Washer, Dryer):**
```
✓ Primary: Hall sensors or sensorless back-EMF
✗ Not recommended: High-res sensors (cost)

Rationale:
- Low-cost imperative
- Adequate performance from simple sensors
```

#### 7.6.3 Decision Tree for Sensorless Method Selection

```
START: Want sensorless control
    │
    ├─ Motor Type?
    │  ├─ SPM (low saliency)
    │  │  └─ Use: Back-EMF observer only
    │  │     └─ Startup: Open-loop ramp
    │  │
    │  └─ IPM (high saliency)
    │     └─ Zero-speed torque needed?
    │        ├─ Yes → Hybrid (HFI + back-EMF)
    │        └─ No → Back-EMF only
    │
    ├─ Acoustic noise acceptable?
    │  ├─ Yes → Can use HFI
    │  └─ No → Back-EMF only, avoid HFI
    │
    ├─ MCU capability?
    │  ├─ Low-end → Back-EMF only (lower CPU)
    │  └─ High-end → Can use HFI or hybrid
    │
    └─ Speed range?
       ├─ 0-100% → Hybrid method required
       ├─ 10-100% → Back-EMF sufficient
       └─ High speed only → Back-EMF only
```

#### 7.6.4 Implementation Complexity Ranking

From simplest to most complex:

1. **Hall sensors** - GPIO pins, lookup table
2. **Back-EMF observer (basic SMO)** - Medium complexity, single observer
3. **Encoder** - Hardware timer in quadrature mode
4. **Back-EMF observer (PLL-based)** - Higher complexity, better performance
5. **Pulsating HFI** - Signal processing, demodulation
6. **Rotating HFI** - More complex demodulation
7. **Hybrid sensorless** - Two observers + blending logic
8. **Resolver** - RDC chip interface, calibration

#### 7.6.5 Cost-Performance Trade-offs

| Solution | Hardware Cost | Development Time | Performance | Best For |
|----------|--------------|------------------|-------------|----------|
| Hall | $ | 1 week | Adequate | Low-cost, moderate performance |
| Back-EMF | $0 | 4-6 weeks | Good | High volume, cost-critical |
| HFI | $0 | 8-12 weeks | Very Good | Zero-speed capable, IPM |
| Hybrid | $0 | 12-16 weeks | Excellent | Premium sensorless |
| Encoder | $$-$$$ | 2 weeks | Excellent | Precision control |
| Resolver | $$$$-$$$$$ | 4 weeks | Outstanding | Safety-critical |

#### 7.6.6 Final Recommendation Summary

**For most applications:**
- **Consumer products**: Back-EMF sensorless or Hall sensors
- **Industrial**: Encoder (incremental or absolute)
- **Automotive**: Resolver (safety) or hybrid sensorless (cost)
- **High-performance**: Encoder + sensorless backup
- **Cost-critical**: Back-EMF sensorless only

**Key Takeaway:** The "best" solution depends entirely on application requirements, cost constraints, and performance needs. There is no one-size-fits-all answer.

---

### References for Section 7.6:

**Standards:**
1. **ISO 26262**: Functional Safety for Automotive (sensor redundancy requirements)
2. **IEC 61800-5-1**: Safety requirements for adjustable speed drives

**Papers:**
1. Boldea, I., et al. (2007). "Automotive Electric Propulsion Systems With Reduced or No Permanent Magnets: An Overview." *IEEE Trans. on Industrial Electronics*, 61(10), 5696-5711.
2. Wang, G., et al. (2020). "Sensorless PMSM Drives—A Survey Covering the Past Decade." *CES Trans. on Electrical Machines and Systems*, 4(4), 249-264.

**Books:**
1. *"Design of Brushless Permanent-Magnet Machines"* by J.R. Hendershot and T.J.E. Miller - Chapter 12: Sensorless Control

---

**End of Section 7: Sensored vs Sensorless Control**

---

## Section 8: Back-EMF and Overvoltage Protection

Protecting the motor controller from excessive voltages generated by the motor is critical for system reliability and safety. This section covers protection strategies for overspeed conditions, winding faults, and other high-voltage scenarios that can damage power electronics or pose safety risks.

---

### 8.1 Overvoltage from Motor Overspeed

When a PMSM rotates faster than the nominal speed—especially with the controller turned off or disabled—the motor acts as a generator, producing back-EMF that can exceed the battery voltage and damage the controller.

#### 8.1.1 Physics of Back-EMF Overvoltage

**Back-EMF Generation:**

```
E_backemf = K_e × ω_m × p

Where:
  E_backemf = Line-to-line back-EMF voltage (V)
  K_e = Motor voltage constant (V/rad/s electrical)
  ω_m = Mechanical speed (rad/s)
  p = Number of pole pairs

Example:
  K_e = 0.05 V/rad/s (electrical)
  Pole pairs = 4
  Nominal speed = 3000 RPM (314 rad/s mechanical)

  E_backemf = 0.05 × 314 × 4 = 62.8 V (at nominal speed)

If motor overspeeds to 6000 RPM:
  E_backemf = 0.05 × 628 × 4 = 125.6 V (doubled!)
```

**The Danger:**

```
Scenario: Vehicle rolling downhill with controller OFF
  1. Motor spins faster than nominal speed
  2. Back-EMF exceeds battery voltage (e.g., 125V > 48V nominal)
  3. Current flows backwards through body diodes of MOSFETs
  4. DC link voltage rises rapidly
  5. If uncontrolled: DC link capacitor overvoltage → explosion/failure
```

**Critical Speed Calculation:**

```c
// Calculate critical overspeed threshold
float calculate_critical_speed(float V_battery, float K_e, uint8_t pole_pairs) {
    // Speed at which back-EMF equals battery voltage
    float omega_critical = V_battery / (K_e * pole_pairs);  // rad/s mechanical

    // Add safety margin (typically 80% of critical)
    float omega_max_safe = omega_critical * 0.8f;

    return omega_max_safe;
}

// Example:
// V_battery = 48V, K_e = 0.05, pole_pairs = 4
// omega_critical = 48 / (0.05 × 4) = 240 rad/s = 2292 RPM
// omega_max_safe = 192 rad/s = 1834 RPM
```

#### 8.1.2 Hardware Protection: Overvoltage Clamp Circuits

**Method 1: Zener Diode + Crowbar Circuit**

```
Hardware configuration:

    VDC+ ──┬──────────────────┬──
           │                  │
         [Zener]           [SCR/Thyristor]
           │                  │
           ├──[R_gate]────────┤
           │                  │
    VDC- ──┴──────────────────┴──

Operation:
  1. When V_dc > V_zener (e.g., 60V for 48V system)
  2. Zener conducts, triggering SCR gate
  3. SCR shorts DC link through brake resistor
  4. Energy dissipated as heat in resistor
```

**Design Calculations:**

```c
// Brake resistor sizing
typedef struct {
    float V_clamp;          // Clamping voltage (V)
    float P_max_regen;      // Maximum regen power (W)
    float duty_cycle;       // Expected duty cycle (0-1)
    float R_brake;          // Brake resistor value (Ω)
    float P_brake_avg;      // Average power dissipation (W)
    float P_brake_peak;     // Peak power dissipation (W)
} BrakeResistorCalc_t;

void calculate_brake_resistor(BrakeResistorCalc_t *br) {
    // Resistance: R = V² / P
    br->R_brake = (br->V_clamp * br->V_clamp) / br->P_max_regen;

    // Peak power
    br->P_brake_peak = (br->V_clamp * br->V_clamp) / br->R_brake;

    // Average power (with duty cycle)
    br->P_brake_avg = br->P_brake_peak * br->duty_cycle;

    printf("Brake Resistor Design:\n");
    printf("  Resistance: %.2f Ω\n", br->R_brake);
    printf("  Peak Power: %.1f W\n", br->P_brake_peak);
    printf("  Avg Power: %.1f W (at %.0f%% duty)\n",
           br->P_brake_avg, br->duty_cycle * 100);
}

// Example for 48V, 500W system:
BrakeResistorCalc_t br = {
    .V_clamp = 60.0f,      // Clamp at 60V
    .P_max_regen = 500.0f,  // 500W max regen
    .duty_cycle = 0.1f      // 10% duty (worst case)
};
calculate_brake_resistor(&br);
// Output: R = 7.2Ω, Peak = 500W, Avg = 50W
```

**Method 2: Active Brake Chopper with IGBT/MOSFET**

```c
// Active brake chopper control
typedef struct {
    float V_dc_threshold;   // Start braking above this voltage (V)
    float V_dc_hysteresis;  // Hysteresis band (V)
    bool brake_active;      // Brake chopper state
    float duty_cycle;       // PWM duty cycle for brake
    uint32_t pwm_freq;      // Brake chopper PWM frequency (Hz)
} BrakeChopper_t;

void brake_chopper_control(BrakeChopper_t *bc, float V_dc_measured) {
    // Hysteresis control
    if (V_dc_measured > (bc->V_dc_threshold + bc->V_dc_hysteresis)) {
        bc->brake_active = true;

        // Calculate duty cycle based on overvoltage amount
        float V_error = V_dc_measured - bc->V_dc_threshold;
        bc->duty_cycle = fminf(V_error / bc->V_dc_hysteresis, 1.0f);

    } else if (V_dc_measured < bc->V_dc_threshold) {
        bc->brake_active = false;
        bc->duty_cycle = 0.0f;
    }

    // Apply PWM to brake IGBT/MOSFET
    if (bc->brake_active) {
        set_brake_chopper_pwm(bc->duty_cycle);
    } else {
        set_brake_chopper_pwm(0.0f);
    }
}

// Initialize brake chopper for 48V system
BrakeChopper_t brake_chopper = {
    .V_dc_threshold = 58.0f,   // Start braking at 58V
    .V_dc_hysteresis = 2.0f,   // 2V hysteresis band
    .pwm_freq = 20000,         // 20 kHz PWM
    .brake_active = false,
    .duty_cycle = 0.0f
};

// Call in main control loop (e.g., every 100 µs)
void control_loop_1khz(void) {
    float V_dc = read_dc_link_voltage();
    brake_chopper_control(&brake_chopper, V_dc);
}
```

**Method 3: Transient Voltage Suppressor (TVS) Diodes**

```
TVS diode selection:

For 48V nominal system:
  - V_breakdown: 60-65V (standoff voltage)
  - V_clamp: 75-80V (at rated current)
  - P_peak: 1500W minimum (for transients)
  - I_peak: 20A minimum

Example: Littelfuse P6KE62A
  - V_breakdown: 62V
  - V_clamp: 78V @ 10A
  - P_peak: 600W (pulsed)
```

**Hardware Protection Comparison:**

| Method | Response Time | Cost | Energy Handling | Complexity |
|--------|--------------|------|-----------------|------------|
| TVS Diodes | <1 ns | $ | Low (transients only) | Very Low |
| Zener + Crowbar | <1 µs | $$ | Medium | Low |
| Active Brake Chopper | <100 µs | $$$ | High (continuous) | Medium |
| Controller-based regen | <1 ms | $0 | Highest (to battery) | High |

#### 8.1.3 Software Detection and Mitigation

**Overvoltage Detection:**

```c
// Overvoltage monitoring and response
typedef struct {
    // Thresholds
    float V_dc_warning;         // Warning level (V)
    float V_dc_critical;        // Critical shutdown level (V)
    float V_dc_nominal;         // Nominal voltage (V)

    // State
    uint32_t overvoltage_count; // Consecutive overvoltage samples
    uint32_t count_threshold;   // Samples before action
    bool fault_active;

    // Response
    float torque_limit_factor;  // Reduce torque request (0-1)
} OvervoltageProtection_t;

void overvoltage_monitor_update(OvervoltageProtection_t *ovp, float V_dc) {
    // Check voltage levels
    if (V_dc > ovp->V_dc_critical) {
        // CRITICAL: Immediate shutdown
        ovp->fault_active = true;
        emergency_shutdown();
        log_fault(FAULT_OVERVOLTAGE_CRITICAL, V_dc);

    } else if (V_dc > ovp->V_dc_warning) {
        // WARNING: Increment counter
        ovp->overvoltage_count++;

        if (ovp->overvoltage_count > ovp->count_threshold) {
            // Sustained overvoltage - take action

            // Option 1: Enable active braking
            enable_active_braking();

            // Option 2: Reduce motor torque (limit acceleration)
            float V_excess = V_dc - ovp->V_dc_warning;
            ovp->torque_limit_factor = 1.0f - (V_excess /
                (ovp->V_dc_critical - ovp->V_dc_warning));
            ovp->torque_limit_factor = fmaxf(ovp->torque_limit_factor, 0.0f);

            log_warning(WARNING_OVERVOLTAGE, V_dc);
        }

    } else {
        // Normal operation
        ovp->overvoltage_count = 0;
        ovp->torque_limit_factor = 1.0f;
    }
}

// Initialize for 48V system
OvervoltageProtection_t ovp = {
    .V_dc_nominal = 48.0f,
    .V_dc_warning = 58.0f,
    .V_dc_critical = 65.0f,
    .count_threshold = 10,  // 10 consecutive samples
    .overvoltage_count = 0,
    .fault_active = false,
    .torque_limit_factor = 1.0f
};
```

**Controller-Disabled Overspeed Protection:**

This is the critical scenario: motor controller is OFF but motor is spinning (rolling downhill).

**Solution 1: Keep Controller Partially Active**

```c
// Minimal active mode for overvoltage protection
typedef enum {
    CONTROLLER_OFF,           // Completely off (dangerous!)
    CONTROLLER_SLEEP,         // Sleep with wakeup on overvoltage
    CONTROLLER_OVERVOLTAGE_GUARD  // Active monitoring only
} ControllerPowerMode_t;

void set_controller_power_mode(ControllerPowerMode_t mode) {
    switch (mode) {
        case CONTROLLER_OFF:
            // Disable all MOSFETs
            disable_all_mosfets();
            // Disable PWM timers
            disable_pwm_timers();
            // Power down MCU (DANGER: no protection!)
            enter_deep_sleep();
            break;

        case CONTROLLER_SLEEP:
            // Disable MOSFETs
            disable_all_mosfets();
            // Disable PWM
            disable_pwm_timers();
            // Keep ADC active for voltage monitoring
            enable_adc_with_interrupt();
            // Configure ADC interrupt on V_dc > threshold
            configure_adc_threshold_interrupt(V_DC_WAKEUP_THRESHOLD);
            // Enter low-power sleep
            enter_sleep_mode();
            break;

        case CONTROLLER_OVERVOLTAGE_GUARD:
            // Disable normal motor control
            disable_foc_control();
            disable_all_mosfets();
            // Keep monitoring active
            enable_voltage_monitoring();
            enable_brake_chopper();
            // Minimal power consumption
            reduce_cpu_frequency();
            break;
    }
}

// ADC interrupt handler (wakes from sleep on overvoltage)
void ADC_IRQHandler(void) {
    float V_dc = read_dc_link_voltage_fast();

    if (V_dc > V_DC_PROTECTION_THRESHOLD) {
        // Wake up and activate brake chopper
        exit_sleep_mode();
        enable_brake_chopper();
        set_controller_power_mode(CONTROLLER_OVERVOLTAGE_GUARD);
    }
}
```

**Solution 2: Mechanical Brake Engagement**

```c
// Engage mechanical brake if overspeed detected while controller off
typedef struct {
    bool brake_engaged;
    float speed_threshold;      // Engage brake above this speed
    uint32_t engagement_delay;  // Delay before engaging (ms)
    uint32_t timer;
} MechanicalBrakeProtection_t;

void mechanical_brake_overspeed_protection(MechanicalBrakeProtection_t *mbp,
                                          float motor_speed,
                                          bool controller_enabled) {
    if (!controller_enabled && (motor_speed > mbp->speed_threshold)) {
        // Controller is OFF but motor is spinning too fast
        mbp->timer++;

        if (mbp->timer > mbp->engagement_delay) {
            // Engage mechanical brake
            engage_friction_brake(BRAKE_PRESSURE_MEDIUM);
            mbp->brake_engaged = true;
            log_event(EVENT_OVERSPEED_BRAKE_ENGAGED);
        }
    } else {
        mbp->timer = 0;
        if (mbp->brake_engaged && controller_enabled) {
            // Release brake when controller is re-enabled
            release_friction_brake();
            mbp->brake_engaged = false;
        }
    }
}
```

**Solution 3: Regenerative Braking Auto-Activation**

```c
// Automatically enable regen if overvoltage detected
void auto_regen_overvoltage_protection(float V_dc, float motor_speed) {
    static bool auto_regen_active = false;

    if (V_dc > V_DC_AUTO_REGEN_THRESHOLD) {
        if (!auto_regen_active) {
            // Enable FOC in regen mode
            enable_foc_control();

            // Set negative torque command (braking)
            float braking_torque = -calculate_safe_braking_torque(motor_speed, V_dc);
            set_torque_command(braking_torque);

            // Activate battery charging if safe
            if (battery_can_accept_charge()) {
                enable_battery_charging();
            } else {
                // Battery full - use brake resistor
                enable_brake_chopper();
            }

            auto_regen_active = true;
            log_event(EVENT_AUTO_REGEN_ACTIVATED);
        }

        // Dynamically adjust braking torque to maintain voltage
        float V_error = V_dc - V_DC_TARGET;
        float torque_adjustment = V_error * REGEN_GAIN;
        adjust_braking_torque(torque_adjustment);

    } else if (V_dc < V_DC_AUTO_REGEN_RELEASE) {
        if (auto_regen_active) {
            // Release auto-regen
            set_torque_command(0.0f);
            disable_foc_control();
            auto_regen_active = false;
        }
    }
}
```

#### 8.1.4 Recommended Protection Strategy

**Multi-Layer Protection (Defense in Depth):**

```c
// Integrated overvoltage protection system
void overvoltage_protection_system(float V_dc, float motor_speed,
                                  bool controller_enabled) {
    // Layer 1: Hardware TVS diodes (always active)
    // - No software needed, instant protection

    // Layer 2: Active brake chopper (hardware-controlled)
    brake_chopper_control(&brake_chopper, V_dc);

    // Layer 3: Software overvoltage monitoring
    overvoltage_monitor_update(&ovp, V_dc);

    // Layer 4: Auto-regenerative braking
    if (controller_enabled) {
        auto_regen_overvoltage_protection(V_dc, motor_speed);
    }

    // Layer 5: Mechanical brake backup (last resort)
    mechanical_brake_overspeed_protection(&mbp, motor_speed, controller_enabled);

    // Layer 6: Battery charge management
    if (V_dc > V_DC_WARNING && battery_soc > 95.0f) {
        // Battery nearly full - route energy to brake resistor
        disable_battery_charging();
        force_brake_resistor_mode();
    }
}
```

**Design Checklist:**

```
✓ Hardware Protection:
  ☐ TVS diodes rated for 1.5× nominal voltage
  ☐ Brake resistor sized for worst-case power
  ☐ Brake chopper circuit tested under load
  ☐ DC link capacitor voltage rating > 1.2× max expected

✓ Software Protection:
  ☐ Overvoltage threshold < hardware damage limit
  ☐ Multi-level warning system (warning, critical, shutdown)
  ☐ Auto-regen implemented and tested
  ☐ Fault logging for diagnostics

✓ Testing:
  ☐ Downhill coast test (controller OFF)
  ☐ Sudden regen with full battery
  ☐ Maximum overspeed scenario
  ☐ Brake resistor thermal testing
```

---

### References for Section 8.1:

**Books:**
1. *"Power Electronics for Motor Drives"* by R. Krishnan - Chapter 9: Protection Circuits
2. *"Electric Vehicle Technology Explained"* by Larminie & Lowry - Chapter 8: Safety Systems

**Application Notes:**
1. **Infineon**: "Brake Chopper Design for Motor Drives" (AN2017-12)
2. **Texas Instruments**: "Overvoltage Protection in Motor Control" (SLVA856)
3. **ON Semiconductor**: "TVS Diode Selection Guide" (AND8231/D)
4. **Littelfuse**: "Transient Voltage Suppression" (Application Guide)

**Standards:**
1. **IEC 61800-5-1**: Adjustable Speed Drives - Safety Requirements (Overvoltage Protection)
2. **UL 2231**: "Personnel Protection Systems for Electric Vehicle Supply Circuits"

---

### 8.2 Motor Winding Short Circuit Protection

Motor winding short circuits are catastrophic faults that can destroy the motor controller within milliseconds if not detected and isolated immediately. This section covers detection methods and protection strategies.

#### 8.2.1 Types of Winding Faults

**1. Phase-to-Phase Short:**
```
Scenario: Two motor phases short together (e.g., Phase A to Phase B)

Consequences:
  - Extremely high current (limited only by cable/winding resistance)
  - Typically 10-50× rated current
  - MOSFET destruction in <1 ms
  - Potential fire hazard
```

**2. Phase-to-Ground Short:**
```
Scenario: One motor phase shorts to motor chassis/ground

Consequences:
  - High current through ground path
  - Controller damage
  - Shock hazard if chassis not properly grounded
  - May trigger ground fault protection
```

**3. Turn-to-Turn Short (Internal):**
```
Scenario: Adjacent winding turns short within same phase

Consequences:
  - Localized heating in winding
  - Gradual insulation degradation
  - Eventually leads to phase-to-phase short
  - Harder to detect externally
```

#### 8.2.2 Detection Methods

**Method 1: Hardware Overcurrent Detection (Fastest)**

```c
// Hardware comparator-based overcurrent shutdown
// Typical response time: <1 µs

Hardware Setup:
  - Current shunt in each phase
  - Fast comparator (LM339, LM393, or built-in)
  - Direct connection to PWM disable (hardware shutdown)

Example circuit (per phase):
  V_sense (from shunt) ──> [Comparator+] ──> Fault_Output ──> PWM_Disable
                            [Comparator-] ←── V_threshold

Threshold setting:
  I_fault = V_threshold / (R_shunt × Gain)

  Example:
    R_shunt = 0.001Ω (1mΩ)
    Amplifier gain = 50
    V_threshold = 3.3V
    I_fault = 3.3 / (0.001 × 50) = 66A
```

```c
// Software configuration of hardware comparator
typedef struct {
    float I_fault_threshold;    // Fault current level (A)
    float R_shunt;              // Shunt resistance (Ω)
    float amplifier_gain;       // Current sense amplifier gain
    float V_ref;                // Reference voltage for comparator (V)
} HardwareOCProtection_t;

void configure_hardware_overcurrent_protection(HardwareOCProtection_t *hw) {
    // Calculate required comparator threshold voltage
    hw->V_ref = hw->I_fault_threshold * hw->R_shunt * hw->amplifier_gain;

    // Set DAC output for comparator reference
    set_dac_voltage(hw->V_ref);

    // Enable hardware overcurrent shutdown
    // (Typically done through timer break input)
    TIM1->BDTR |= TIM_BDTR_AOE;  // Automatic output enable
    TIM1->BDTR |= TIM_BDTR_BKE;  // Break enable

    printf("Hardware OC Protection configured:\n");
    printf("  Fault threshold: %.1f A\n", hw->I_fault_threshold);
    printf("  Comparator V_ref: %.3f V\n", hw->V_ref);
}
```

**Method 2: Software Current Monitoring**

```c
// Fast software overcurrent detection
// Typical response time: 10-100 µs (depends on control loop rate)

typedef struct {
    float I_rated;              // Motor rated current (A)
    float I_peak_allowed;       // Peak current limit (A)
    float I_fault;              // Fault shutdown current (A)
    uint32_t fault_count;       // Consecutive fault samples
    uint32_t fault_threshold;   // Samples before shutdown
    bool fault_active;
} SoftwareOCProtection_t;

void software_overcurrent_monitor(SoftwareOCProtection_t *sw,
                                  float i_a, float i_b, float i_c) {
    // Calculate magnitude
    float i_max = fmaxf(fabsf(i_a), fmaxf(fabsf(i_b), fabsf(i_c)));

    if (i_max > sw->I_fault) {
        // CRITICAL: Immediate shutdown
        sw->fault_count++;

        if (sw->fault_count > sw->fault_threshold) {
            sw->fault_active = true;
            emergency_shutdown();
            disable_all_pwm();
            log_fault(FAULT_OVERCURRENT_SHUTDOWN, i_max);
        }

    } else if (i_max > sw->I_peak_allowed) {
        // WARNING: Current exceeds peak but not fault level
        sw->fault_count++;
        limit_current_reference(sw->I_peak_allowed);
        log_warning(WARNING_OVERCURRENT, i_max);

    } else {
        // Normal operation
        sw->fault_count = 0;
    }
}

// Initialize for typical motor
SoftwareOCProtection_t sw_oc = {
    .I_rated = 10.0f,           // 10A rated
    .I_peak_allowed = 30.0f,    // 3× rated peak
    .I_fault = 50.0f,           // 5× rated fault level
    .fault_threshold = 3,       // 3 consecutive samples
    .fault_count = 0,
    .fault_active = false
};
```

**Method 3: Phase Current Imbalance Detection**

Motor winding shorts often cause significant current imbalance between phases:

```c
// Detect winding fault by phase current imbalance
typedef struct {
    float imbalance_threshold;  // Max allowed imbalance (% of average)
    uint32_t imbalance_count;
    uint32_t count_threshold;
    bool winding_fault_detected;
} WindingFaultDetection_t;

void detect_winding_fault_by_imbalance(WindingFaultDetection_t *wfd,
                                       float i_a, float i_b, float i_c) {
    // Calculate average current magnitude
    float i_avg = (fabsf(i_a) + fabsf(i_b) + fabsf(i_c)) / 3.0f;

    if (i_avg < 1.0f) {
        // Skip check at very low currents (noise dominated)
        return;
    }

    // Calculate maximum deviation from average
    float dev_a = fabsf(fabsf(i_a) - i_avg);
    float dev_b = fabsf(fabsf(i_b) - i_avg);
    float dev_c = fabsf(fabsf(i_c) - i_avg);
    float max_dev = fmaxf(dev_a, fmaxf(dev_b, dev_c));

    // Calculate imbalance percentage
    float imbalance_pct = (max_dev / i_avg) * 100.0f;

    if (imbalance_pct > wfd->imbalance_threshold) {
        wfd->imbalance_count++;

        if (wfd->imbalance_count > wfd->count_threshold) {
            wfd->winding_fault_detected = true;
            log_fault(FAULT_WINDING_IMBALANCE, imbalance_pct);
            initiate_safe_shutdown();
        }
    } else {
        wfd->imbalance_count = 0;
    }
}

// Typical initialization
WindingFaultDetection_t winding_fault = {
    .imbalance_threshold = 25.0f,  // 25% imbalance triggers fault
    .count_threshold = 100,        // 100 consecutive samples (10ms @ 10kHz)
    .imbalance_count = 0,
    .winding_fault_detected = false
};
```

#### 8.2.3 Protection Response Strategies

**Immediate Actions (within 1 ms):**

```c
void emergency_winding_fault_response(void) {
    // Step 1: Disable all PWM outputs (CRITICAL - do this FIRST)
    TIM1->BDTR &= ~TIM_BDTR_MOE;  // Main output disable
    __HAL_TIM_MOE_DISABLE(&htim1);

    // Step 2: Set all low-side MOSFETs ON (short motor phases together)
    // This provides a current path and prevents back-EMF voltage spikes
    HAL_GPIO_WritePin(LS_A_PORT, LS_A_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LS_B_PORT, LS_B_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LS_C_PORT, LS_C_PIN, GPIO_PIN_SET);

    // Step 3: Disable high-side MOSFETs
    HAL_GPIO_WritePin(HS_A_PORT, HS_A_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(HS_B_PORT, HS_B_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(HS_C_PORT, HS_C_PIN, GPIO_PIN_RESET);

    // Step 4: Activate brake resistor if available
    activate_brake_resistor();

    // Step 5: Log fault for diagnostics
    log_critical_fault(FAULT_WINDING_SHORT_CIRCUIT);
}
```

**Diagnostic Mode (after fault):**

```c
// Attempt to identify which phase is shorted
typedef struct {
    bool phase_a_fault;
    bool phase_b_fault;
    bool phase_c_fault;
    float test_voltage;      // Low voltage for testing (V)
    float test_duration_ms;  // Test pulse duration
} WindingDiagnostics_t;

void diagnose_winding_fault(WindingDiagnostics_t *diag) {
    // IMPORTANT: Only run this in safe, controlled environment
    // NOT during normal operation!

    // Test Phase A
    apply_low_voltage_pulse(PHASE_A, diag->test_voltage, diag->test_duration_ms);
    float i_a_response = measure_current(PHASE_A);
    diag->phase_a_fault = (i_a_response > EXPECTED_CURRENT_THRESHOLD);

    // Test Phase B
    apply_low_voltage_pulse(PHASE_B, diag->test_voltage, diag->test_duration_ms);
    float i_b_response = measure_current(PHASE_B);
    diag->phase_b_fault = (i_b_response > EXPECTED_CURRENT_THRESHOLD);

    // Test Phase C
    apply_low_voltage_pulse(PHASE_C, diag->test_voltage, diag->test_duration_ms);
    float i_c_response = measure_current(PHASE_C);
    diag->phase_c_fault = (i_c_response > EXPECTED_CURRENT_THRESHOLD);

    // Report results
    if (diag->phase_a_fault || diag->phase_b_fault || diag->phase_c_fault) {
        printf("Winding fault detected:\n");
        if (diag->phase_a_fault) printf("  Phase A: FAULT\n");
        if (diag->phase_b_fault) printf("  Phase B: FAULT\n");
        if (diag->phase_c_fault) printf("  Phase C: FAULT\n");
    }
}
```

**Safe Shutdown Sequence:**

```c
void initiate_safe_shutdown_winding_fault(void) {
    typedef enum {
        SHUTDOWN_DISABLE_PWM,
        SHUTDOWN_SHORT_PHASES,
        SHUTDOWN_ENGAGE_BRAKE,
        SHUTDOWN_WAIT_MOTOR_STOP,
        SHUTDOWN_OPEN_CONTACTORS,
        SHUTDOWN_COMPLETE
    } ShutdownState_t;

    static ShutdownState_t state = SHUTDOWN_DISABLE_PWM;
    static uint32_t timer = 0;

    switch (state) {
        case SHUTDOWN_DISABLE_PWM:
            disable_all_pwm();
            state = SHUTDOWN_SHORT_PHASES;
            break;

        case SHUTDOWN_SHORT_PHASES:
            // Short motor phases through low-side MOSFETs
            // Provides dynamic braking and current path
            enable_all_low_side_mosfets();
            timer = 0;
            state = SHUTDOWN_ENGAGE_BRAKE;
            break;

        case SHUTDOWN_ENGAGE_BRAKE:
            // Engage mechanical brake gradually
            apply_brake_gradually(100);  // 100% in 1 second
            state = SHUTDOWN_WAIT_MOTOR_STOP;
            timer = 0;
            break;

        case SHUTDOWN_WAIT_MOTOR_STOP:
            timer++;
            if (get_motor_speed() < 10.0f || timer > 5000) {  // 5 seconds max
                state = SHUTDOWN_OPEN_CONTACTORS;
            }
            break;

        case SHUTDOWN_OPEN_CONTACTORS:
            // Open main power contactors to isolate battery
            open_main_contactor();
            // Disable low-side MOSFETs
            disable_all_mosfets();
            state = SHUTDOWN_COMPLETE;
            break;

        case SHUTDOWN_COMPLETE:
            // System is safe, await manual reset
            set_fault_led(LED_ON_SOLID);
            break;
    }
}
```

#### 8.2.4 Prevention and Early Detection

**Insulation Resistance Testing:**

```c
// Perform insulation resistance test at startup (offline)
typedef struct {
    float test_voltage;       // DC test voltage (V)
    float min_resistance;     // Minimum acceptable resistance (MΩ)
    bool test_passed;
} InsulationTest_t;

bool perform_insulation_test(InsulationTest_t *test) {
    // Disable all MOSFETs
    disable_all_mosfets();

    // Apply test voltage between phases and ground
    float R_phase_a_gnd = measure_resistance_to_ground(PHASE_A, test->test_voltage);
    float R_phase_b_gnd = measure_resistance_to_ground(PHASE_B, test->test_voltage);
    float R_phase_c_gnd = measure_resistance_to_ground(PHASE_C, test->test_voltage);

    float R_min = fminf(R_phase_a_gnd, fminf(R_phase_b_gnd, R_phase_c_gnd));

    test->test_passed = (R_min > test->min_resistance * 1e6f);  // Convert MΩ to Ω

    if (!test->test_passed) {
        log_fault(FAULT_INSULATION_FAILURE, R_min / 1e6f);
    }

    return test->test_passed;
}

// Run at startup
Insulation Test_t insulation_test = {
    .test_voltage = 12.0f,      // 12V DC test
    .min_resistance = 1.0f,     // 1 MΩ minimum
    .test_passed = false
};

if (!perform_insulation_test(&insulation_test)) {
    // Do NOT enable motor controller
    enter_fault_state(FAULT_INSULATION_FAILURE);
}
```

**Winding Temperature Monitoring:**

High winding temperature can indicate developing shorts:

```c
// Monitor winding temperature for early fault detection
void monitor_winding_temperature(float T_winding) {
    const float T_WARNING = 120.0f;   // Warning at 120°C
    const float T_CRITICAL = 150.0f;  // Critical at 150°C
    const float T_DERATE_START = 100.0f;

    if (T_winding > T_CRITICAL) {
        log_fault(FAULT_WINDING_OVERTEMP, T_winding);
        emergency_shutdown();

    } else if (T_winding > T_WARNING) {
        // Derate power to reduce heating
        float derate_factor = 1.0f - ((T_winding - T_DERATE_START) /
                                     (T_CRITICAL - T_DERATE_START));
        derate_factor = fmaxf(derate_factor, 0.5f);  // Minimum 50% power
        apply_thermal_derating(derate_factor);
        log_warning(WARNING_WINDING_OVERTEMP, T_winding);
    }
}
```

---

### References for Section 8.2:

**Books:**
1. *"Electric Motor Repair"* by Robert Rosenberg - Chapter 12: Winding Failures
2. *"Condition Monitoring of Rotating Electrical Machines"* by Tavner et al.

**Papers:**
1. Bellini, A., et al. (2008). "Detection of Generalized-Roughness Bearing Fault by Spectral-Kurtosis Energy of Vibration or Current Signals." *IEEE Trans. on Industrial Electronics*
2. Nandi, S., et al. (2005). "Condition Monitoring and Fault Diagnosis of Electrical Motors—A Review." *IEEE Trans. on Energy Conversion*

**Application Notes:**
1. **Texas Instruments**: "Overcurrent Protection in Motor Drives" (SLVA959)
2. **Infineon**: "Gate Driver with Integrated Protection" (Application Note)
3. **STMicroelectronics**: "Short Circuit Protection Strategies" (AN4678)

**Standards:**
1. **IEC 60034-27**: Rotating Electrical Machines - Off-line Partial Discharge Measurements
2. **IEEE 43**: Recommended Practice for Testing Insulation Resistance

---

### 8.3 Additional High-Voltage Protection Scenarios

Beyond overspeed and winding faults, several other scenarios can generate dangerous voltages.

#### 8.3.1 Sudden Mechanical Deceleration

**Scenario:**
Vehicle collision or sudden wheel lockup causes rapid motor deceleration while controller is commanding torque.

**Physics:**
```
P_regen = τ_motor × ω_motor

If ω drops rapidly (e.g., wheel hits obstacle):
  - Motor still producing torque (inertia)
  - Rapid deceleration = high regen power
  - Power has nowhere to go → voltage spike
```

**Protection:**

```c
// Detect sudden deceleration and reduce torque
typedef struct {
    float decel_threshold;      // Max allowed decel (rad/s²)
    float last_speed;
    float T_s;                  // Sample time (s)
    bool sudden_stop_detected;
} SuddenStopProtection_t;

void detect_sudden_deceleration(SuddenStopProtection_t *ssp, float current_speed) {
    float decel = (ssp->last_speed - current_speed) / ssp->T_s;

    if (decel > ssp->decel_threshold) {
        ssp->sudden_stop_detected = true;

        // Immediately reduce torque command to zero
        set_torque_command(0.0f);

        // Short motor phases for dynamic braking
        enable_dynamic_braking();

        log_event(EVENT_SUDDEN_DECELERATION);
    }

    ssp->last_speed = current_speed;
}
```

#### 8.3.2 DC Link Capacitor Failure

**Scenario:**
DC link capacitor fails open or loses capacitance.

**Consequences:**
```
Reduced capacitance → reduced filtering
  → Higher voltage ripple
  → Potential overvoltage during regen transients
```

**Detection:**

```c
// Monitor DC link ripple to detect capacitor degradation
typedef struct {
    float V_dc_min_sample;
    float V_dc_max_sample;
    float ripple_threshold;     // Max allowed ripple (V)
    uint32_t sample_count;
    uint32_t samples_per_window;
    bool capacitor_degraded;
} CapacitorHealthMonitor_t;

void monitor_capacitor_health(CapacitorHealthMonitor_t *chm, float V_dc) {
    // Track min/max within sampling window
    if (V_dc < chm->V_dc_min_sample) chm->V_dc_min_sample = V_dc;
    if (V_dc > chm->V_dc_max_sample) chm->V_dc_max_sample = V_dc;

    chm->sample_count++;

    if (chm->sample_count >= chm->samples_per_window) {
        // Calculate ripple
        float ripple = chm->V_dc_max_sample - chm->V_dc_min_sample;

        if (ripple > chm->ripple_threshold) {
            chm->capacitor_degraded = true;
            log_warning(WARNING_CAPACITOR_DEGRADED, ripple);
            // Limit power to reduce ripple
            limit_motor_power(0.7f);  // 70% power limit
        }

        // Reset for next window
        chm->V_dc_min_sample = 1000.0f;
        chm->V_dc_max_sample = 0.0f;
        chm->sample_count = 0;
    }
}
```

#### 8.3.3 Battery Disconnect During Operation

**Scenario:**
Battery contactor opens while motor is running (could be due to BMS fault, loose connection, etc.)

**Consequences:**
```
With motor spinning:
  - Back-EMF continues to generate voltage
  - No battery to sink current
  - DC link voltage rises rapidly
  - Potentially catastrophic overvoltage
```

**Protection:**

```c
// Detect battery disconnect and take emergency action
void detect_battery_disconnect(float V_battery, float I_battery) {
    static uint32_t disconnect_count = 0;
    const uint32_t DISCONNECT_THRESHOLD = 5;  // 5 consecutive samples

    // Check if battery voltage is missing but DC link has voltage
    bool disconnect_suspected = (V_battery < 10.0f) && (read_dc_link_voltage() > 20.0f);

    if (disconnect_suspected) {
        disconnect_count++;

        if (disconnect_count > DISCONNECT_THRESHOLD) {
            // CRITICAL: Battery is disconnected
            log_critical_fault(FAULT_BATTERY_DISCONNECT);

            // Emergency response
            disable_all_pwm();
            enable_all_low_side_mosfets();  // Short motor phases
            activate_brake_resistor();      // Dissipate energy
            engage_mechanical_brake();      // Stop vehicle

            // Do NOT re-enable until manually cleared
            enter_lockout_state();
        }
    } else {
        disconnect_count = 0;
    }
}
```

---

### References for Section 8.3:

**Application Notes:**
1. **Texas Instruments**: "DC Link Capacitor Selection for Motor Drives" (SLVA569)
2. **Kemet**: "Aluminum Electrolytic Capacitor Application Guide"

---

### 8.4 Integrated Protection System Design

A robust motor controller requires all protection mechanisms working together.

**Complete Protection Architecture:**

```c
// Master protection coordinator
typedef struct {
    // Protection modules
    OvervoltageProtection_t overvoltage;
    SoftwareOCProtection_t overcurrent;
    WindingFaultDetection_t winding_fault;
    SuddenStopProtection_t sudden_stop;
    CapacitorHealthMonitor_t cap_health;

    // System state
    bool system_ok;
    uint32_t active_faults;
    uint32_t fault_history[16];
    uint8_t fault_index;

} ProtectionSystem_t;

void protection_system_update(ProtectionSystem_t *ps, SystemMeasurements_t *meas) {
    // Run all protection checks
    overvoltage_monitor_update(&ps->overvoltage, meas->V_dc);
    software_overcurrent_monitor(&ps->overcurrent, meas->i_a, meas->i_b, meas->i_c);
    detect_winding_fault_by_imbalance(&ps->winding_fault, meas->i_a, meas->i_b, meas->i_c);
    detect_sudden_deceleration(&ps->sudden_stop, meas->motor_speed);
    monitor_capacitor_health(&ps->cap_health, meas->V_dc);
    detect_battery_disconnect(meas->V_battery, meas->I_battery);

    // Aggregate fault status
    ps->system_ok = !(ps->overvoltage.fault_active ||
                      ps->overcurrent.fault_active ||
                      ps->winding_fault.winding_fault_detected ||
                      ps->sudden_stop.sudden_stop_detected);

    if (!ps->system_ok) {
        // Record fault in history
        ps->fault_history[ps->fault_index] = ps->active_faults;
        ps->fault_index = (ps->fault_index + 1) % 16;
    }
}
```

**Design Checklist:**

```
✓ Hardware Protection (must-have):
  ☐ Hardware overcurrent comparators on all phases
  ☐ TVS diodes on DC link
  ☐ Brake chopper or brake resistor
  ☐ Gate driver desaturation detection
  ☐ Thermal shutdown (MOSFETs, motor)

✓ Software Protection (recommended):
  ☐ Multi-level overvoltage monitoring
  ☐ Phase current imbalance detection
  ☐ Sudden deceleration detection
  ☐ Battery disconnect detection
  ☐ Capacitor health monitoring

✓ Safe Shutdown (critical):
  ☐ Defined shutdown sequence
  ☐ Fault logging to non-volatile memory
  ☐ Manual reset required after critical fault
  ☐ Diagnostic mode for fault identification

✓ Testing (verification):
  ☐ Overspeed test (simulate downhill)
  ☐ Short circuit injection test (controlled)
  ☐ Battery disconnect test
  ☐ Thermal runaway test
  ☐ Brake resistor thermal test
```

---

### References for Section 8.4:

**Standards:**
1. **ISO 26262**: Functional Safety for Automotive Electronics
2. **IEC 61508**: Functional Safety of Electrical/Electronic Systems
3. **UL 2202**: Electric Vehicle Charging Equipment

**Books:**
1. *"Functional Safety for Road Vehicles"* by Schäuffele & Zurawka

---

**End of Section 8: Back-EMF and Overvoltage Protection**

---

## Conclusion

This document has covered advanced motor control strategies for electric vehicles, including:

1. **Field Weakening Control**: Mathematics, implementation, and safety considerations
2. **Regenerative Braking**: Physics, battery management, hardware perspectives, and user-selectable levels
3. **Braking Strategy and Blending**: Algorithmic approaches for coordinating regen and friction braking
4. **Hill Hold Control**: Theory, implementation strategies, and system integration
5. **Temperature-Based Derating**: Thermal management for motors and inverters
6. **CAN Communication**: Essential parameters, debugging, and fault reporting
7. **Sensored vs Sensorless Control**: Complete comparison, back-EMF methods, high frequency injection, and hybrid approaches
8. **Back-EMF and Overvoltage Protection**: Protection from overspeed, winding faults, and high-voltage scenarios

Each section provides:
- Theoretical foundation with mathematical derivations
- Practical C code implementations
- Comprehensive references to books, papers, standards, and application notes

This knowledge base enables the development of production-ready motor control systems for electric vehicles.

---

