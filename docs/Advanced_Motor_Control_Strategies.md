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

## Conclusion

This document has covered advanced motor control strategies for electric vehicles, including:

1. **Field Weakening Control**: Mathematics, implementation, and safety considerations
2. **Regenerative Braking**: Physics, battery management, and user-selectable levels
3. **Braking Strategy and Blending**: Algorithmic approaches for coordinating regen and friction braking
4. **Hill Hold Control**: Theory, implementation strategies, and system integration
5. **Temperature-Based Derating**: Thermal management for motors and inverters
6. **CAN Communication**: Essential parameters, debugging, and fault reporting

Each section provides:
- Theoretical foundation with mathematical derivations
- Practical C code implementations
- Comprehensive references to books, papers, standards, and application notes

This knowledge base enables the development of production-ready motor control systems for electric vehicles.

---

