# Simulink Model Building Guide
## PMSM FOC with Hall Sensors

This document provides step-by-step instructions for building the complete Simulink model for PMSM motor control with Field Oriented Control (FOC) and Hall effect sensors.

---

## Table of Contents
1. [Prerequisites](#prerequisites)
2. [Model Overview](#model-overview)
3. [Step-by-Step Building Instructions](#step-by-step-building-instructions)
4. [Subsystem Details](#subsystem-details)
5. [Configuration and Settings](#configuration-and-settings)
6. [Running the Simulation](#running-the-simulation)
7. [Troubleshooting](#troubleshooting)

---

## Prerequisites

### Software Requirements
- MATLAB R2019b or later (recommended R2021a+)
- Simulink
- Simscape Electrical (for motor modeling - optional but recommended)

### Before Starting
1. Run the parameter initialization script:
   ```matlab
   cd scripts
   motor_parameters
   ```
2. Add the scripts folder to MATLAB path:
   ```matlab
   addpath('scripts')
   ```

---

## Model Overview

### Top-Level Architecture

The complete model consists of the following main blocks:

```
┌──────────────────────────────────────────────────────────────┐
│                    PMSM FOC Control System                    │
├──────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌────────────┐    ┌─────────────┐    ┌──────────────┐     │
│  │   Speed    │───▶│   Current   │───▶│ Coordinate   │     │
│  │ Controller │    │ Controllers │    │ Transforms   │     │
│  │  (Outer)   │    │  (Inner)    │    │ & SVPWM      │     │
│  └────────────┘    └─────────────┘    └──────────────┘     │
│        ▲                  ▲                    │             │
│        │                  │                    ▼             │
│  ┌────────────┐    ┌─────────────┐    ┌──────────────┐     │
│  │   Speed    │    │   Current   │    │  Inverter    │     │
│  │ Estimation │◀───│ Measurement │◀───│  & Motor     │     │
│  │(Hall Sens.)│    │ (Clarke+Park│    │   Model      │     │
│  └────────────┘    └─────────────┘    └──────────────┘     │
│        ▲                                       │             │
│        │                                       │             │
│  ┌────────────┐                               │             │
│  │   Hall     │◀──────────────────────────────┘             │
│  │  Sensors   │                                              │
│  └────────────┘                                              │
│                                                               │
└──────────────────────────────────────────────────────────────┘
```

---

## Step-by-Step Building Instructions

### Step 1: Create New Simulink Model

1. Open MATLAB
2. Type `simulink` in command window or click "Simulink" button
3. Click "Blank Model"
4. Save as `PMSM_FOC_Hall_Model.slx` in the `models` folder

### Step 2: Configure Model Settings

1. Go to **Model Settings** (Ctrl+E or click gear icon)
2. **Solver Settings:**
   - Type: `Fixed-step`
   - Solver: `ode4 (Runge-Kutta)`
   - Fixed-step size: `1e-5` (10 µs)
   - Stop time: `2.0` seconds

3. **Data Import/Export:**
   - Check "Time" and "Output"
   - Check "Log Dataset data to file" (optional)

4. **Optimization:**
   - Check "Block reduction"
   - Check "Signal storage reuse"

### Step 3: Build Speed Reference Input

**Purpose:** Generate the desired speed reference signal

**Steps:**
1. Add a **Signal Builder** or **Step** block from Simulink/Sources
2. For Step block configuration:
   - Step time: `0.2`
   - Initial value: `0`
   - Final value: `1000` (RPM)
3. Add a **Gain** block to convert RPM to rad/s
   - Gain: `2*pi/60`
4. Add a **Saturation** block to limit speed reference
   - Upper limit: `motor.omega_rated`
   - Lower limit: `0`
5. Label the output signal: `Speed_Reference [rad/s]`

### Step 4: Build Speed Controller Subsystem

**Purpose:** Outer loop PI controller for speed regulation

**Steps:**
1. Add a **Subsystem** block and name it `Speed_Controller`
2. Double-click to enter the subsystem
3. Add input ports: `Speed_Ref`, `Speed_Measured`
4. Add a **Sum** block for error calculation
   - Signs: `+-`
   - Connect Speed_Ref to + and Speed_Measured to -
5. Add **MATLAB Function** block for PI controller:
   - Name: `Speed_PI`
   - Code:
   ```matlab
   function [iq_ref, integrator] = Speed_PI(error, integrator_in)
       % Persistent state not needed as integrator is input/output
       persistent first_run;
       if isempty(first_run)
           first_run = false;
       end

       % Load parameters from base workspace
       Kp = evalin('base', 'speed_ctrl.Kp');
       Ki = evalin('base', 'speed_ctrl.Ki');
       Ts = evalin('base', 'sim.Ts_speed');
       iq_max = evalin('base', 'speed_ctrl.output_max');
       iq_min = evalin('base', 'speed_ctrl.output_min');

       % PI controller with anti-windup
       P_term = Kp * error;
       I_term = integrator_in + Ki * Ts * error;

       % Calculate output
       iq_ref_unsat = P_term + I_term;

       % Saturate
       iq_ref = max(min(iq_ref_unsat, iq_max), iq_min);

       % Anti-windup: only update integrator if not saturated
       if iq_ref == iq_ref_unsat
           integrator = I_term;
       else
           integrator = integrator_in;
       end
   end
   ```
6. Add **Unit Delay** block for integrator state
   - Initial condition: `0`
   - Sample time: `sim.Ts_speed`
   - Connect: integrator output → Unit Delay → integrator input
7. Add output port: `iq_Reference`
8. Return to top level

### Step 5: Build Current Controller Subsystem

**Purpose:** Inner loop PI controllers for id and iq regulation

**Steps:**
1. Add a **Subsystem** block and name it `Current_Controllers`
2. Add input ports: `id_ref`, `iq_ref`, `id_meas`, `iq_meas`, `theta_e`
3. Add two error calculation blocks:
   - **Sum** for id error: `id_ref - id_meas`
   - **Sum** for iq error: `iq_ref - iq_meas`
4. Add **MATLAB Function** for d-axis PI:
   ```matlab
   function [vd, integrator_d] = Current_PI_d(error_d, integrator_d_in)
       Kp = evalin('base', 'current_ctrl.Kp_d');
       Ki = evalin('base', 'current_ctrl.Ki_d');
       Ts = evalin('base', 'sim.Ts_control');
       vd_max = evalin('base', 'current_ctrl.output_max');
       vd_min = evalin('base', 'current_ctrl.output_min');

       % PI control
       P_term = Kp * error_d;
       I_term = integrator_d_in + Ki * Ts * error_d;
       vd_unsat = P_term + I_term;

       % Saturate
       vd = max(min(vd_unsat, vd_max), vd_min);

       % Anti-windup
       if vd == vd_unsat
           integrator_d = I_term;
       else
           integrator_d = integrator_d_in;
       end
   end
   ```
5. Add similar MATLAB Function for q-axis PI (replace `_d` with `_q`)
6. Add **Unit Delay** blocks for both integrator states
7. Add output ports: `vd_ref`, `vq_ref`

### Step 6: Build Coordinate Transformation Subsystems

#### 6.1 Clarke Transform (abc → αβ)

1. Add **Subsystem** named `Clarke_Transform`
2. Input ports: `ia`, `ib`, `ic`
3. Add **MATLAB Function**:
   ```matlab
   function [i_alpha, i_beta] = Clarke(ia, ib, ic)
       i_alpha = (2/3) * (ia - 0.5*ib - 0.5*ic);
       i_beta = (2/3) * (sqrt(3)/2) * (ib - ic);
   end
   ```
4. Output ports: `i_alpha`, `i_beta`

#### 6.2 Park Transform (αβ → dq)

1. Add **Subsystem** named `Park_Transform`
2. Input ports: `i_alpha`, `i_beta`, `theta_e`
3. Add **MATLAB Function**:
   ```matlab
   function [id, iq] = Park(i_alpha, i_beta, theta_e)
       cos_theta = cos(theta_e);
       sin_theta = sin(theta_e);
       id = cos_theta * i_alpha + sin_theta * i_beta;
       iq = -sin_theta * i_alpha + cos_theta * i_beta;
   end
   ```
4. Output ports: `id`, `iq`

#### 6.3 Inverse Park Transform (dq → αβ)

1. Add **Subsystem** named `Inverse_Park_Transform`
2. Input ports: `vd`, `vq`, `theta_e`
3. Add **MATLAB Function**:
   ```matlab
   function [v_alpha, v_beta] = Inverse_Park(vd, vq, theta_e)
       cos_theta = cos(theta_e);
       sin_theta = sin(theta_e);
       v_alpha = vd * cos_theta - vq * sin_theta;
       v_beta = vd * sin_theta + vq * cos_theta;
   end
   ```
4. Output ports: `v_alpha`, `v_beta`

#### 6.4 Inverse Clarke Transform (αβ → abc)

1. Add **Subsystem** named `Inverse_Clarke_Transform`
2. Input ports: `v_alpha`, `v_beta`
3. Add **MATLAB Function**:
   ```matlab
   function [va, vb, vc] = Inverse_Clarke(v_alpha, v_beta)
       va = v_alpha;
       vb = -0.5*v_alpha + (sqrt(3)/2)*v_beta;
       vc = -0.5*v_alpha - (sqrt(3)/2)*v_beta;
   end
   ```
4. Output ports: `va`, `vb`, `vc`

### Step 7: Build SVPWM Subsystem

**Purpose:** Generate three-phase PWM duty cycles

**Steps:**
1. Add **Subsystem** named `SVPWM`
2. Input ports: `v_alpha`, `v_beta`, `Vdc`
3. Add **MATLAB Function** using the svpwm function from scripts
4. Output ports: `Ta`, `Tb`, `Tc`

### Step 8: Build PMSM Motor Model

You have two options:

#### Option A: Using Simscape Electrical (Recommended)

1. Add **PMSM** block from Simscape Electrical library
2. Configure parameters using workspace variables:
   - Rs, Ld, Lq, lambda_m, P, J, B
3. Add **Controlled Voltage Source** blocks for three phases
4. Connect voltage inputs to SVPWM outputs (multiply by Vdc)
5. Add **Current Sensor** blocks to measure phase currents

#### Option B: Using MATLAB Function (Mathematical Model)

1. Add **MATLAB Function** named `PMSM_Model`
2. Implement the motor equations:
   ```matlab
   function [ia, ib, ic, omega_m, theta_e] = PMSM_Model(va, vb, vc, TL)
       % Declare persistent states
       persistent ia_state ib_state ic_state omega_state theta_state

       % Initialize on first run
       if isempty(ia_state)
           ia_state = 0; ib_state = 0; ic_state = 0;
           omega_state = 0; theta_state = 0;
       end

       % Load parameters
       Rs = evalin('base', 'motor.Rs');
       Ls = evalin('base', 'motor.Ls');
       lambda_m = evalin('base', 'motor.lambda_m');
       P = evalin('base', 'motor.P');
       J = evalin('base', 'motor.J');
       B = evalin('base', 'motor.B');
       Ts = evalin('base', 'sim.Ts');

       % Calculate electrical position
       theta_e = P * theta_state;

       % Calculate back-EMF
       omega_e = P * omega_state;
       ea = -lambda_m * omega_e * sin(theta_e);
       eb = -lambda_m * omega_e * sin(theta_e - 2*pi/3);
       ec = -lambda_m * omega_e * sin(theta_e + 2*pi/3);

       % Current dynamics: dia/dt = (va - Rs*ia - ea) / Ls
       dia_dt = (va - Rs*ia_state - ea) / Ls;
       dib_dt = (vb - Rs*ib_state - eb) / Ls;
       dic_dt = (vc - Rs*ic_state - ec) / Ls;

       % Integrate currents (Euler method)
       ia = ia_state + dia_dt * Ts;
       ib = ib_state + dib_dt * Ts;
       ic = ic_state + dic_dt * Ts;

       % Calculate electromagnetic torque
       Te = 1.5 * P * lambda_m * (ia*sin(theta_e) + ib*sin(theta_e - 2*pi/3) + ic*sin(theta_e + 2*pi/3));

       % Mechanical dynamics
       domega_dt = (Te - TL - B*omega_state) / J;
       omega_m = omega_state + domega_dt * Ts;
       theta_m = theta_state + omega_state * Ts;

       % Update states
       ia_state = ia; ib_state = ib; ic_state = ic;
       omega_state = omega_m; theta_state = theta_m;
   end
   ```

### Step 9: Build Hall Sensor Model

**Purpose:** Simulate Hall effect sensors

**Steps:**
1. Add **MATLAB Function** named `Hall_Sensors`
2. Input: `theta_e` (electrical position)
3. Code:
   ```matlab
   function [Ha, Hb, Hc, theta_est] = Hall_Sensors(theta_e)
       % Normalize angle to [0, 2*pi]
       theta_e = mod(theta_e, 2*pi);
       theta_deg = rad2deg(theta_e);

       % Determine sector and Hall states
       if theta_deg >= 0 && theta_deg < 60
           Ha = 1; Hb = 0; Hc = 1;
           theta_est = deg2rad(30);
       elseif theta_deg >= 60 && theta_deg < 120
           Ha = 1; Hb = 0; Hc = 0;
           theta_est = deg2rad(90);
       elseif theta_deg >= 120 && theta_deg < 180
           Ha = 1; Hb = 1; Hc = 0;
           theta_est = deg2rad(150);
       elseif theta_deg >= 180 && theta_deg < 240
           Ha = 0; Hb = 1; Hc = 0;
           theta_est = deg2rad(210);
       elseif theta_deg >= 240 && theta_deg < 300
           Ha = 0; Hb = 1; Hc = 1;
           theta_est = deg2rad(270);
       else
           Ha = 0; Hb = 0; Hc = 1;
           theta_est = deg2rad(330);
       end
   end
   ```
4. Outputs: `Ha`, `Hb`, `Hc`, `theta_estimated`

### Step 10: Build Speed Estimation

**Purpose:** Estimate speed from Hall sensor transitions

**Steps:**
1. Add **MATLAB Function** named `Speed_Estimator`
2. Use **Detect Rise Positive** blocks to detect Hall transitions
3. Measure time between transitions
4. Calculate speed based on 60° electrical angle per transition

### Step 11: Add Load Torque

**Purpose:** Simulate external load on motor

**Steps:**
1. Add **Step** or **Signal Builder** block
2. Configure for load torque profile:
   - Step time: `1.0` s
   - Initial: `0` Nm
   - Final: `load.torque_step_value` Nm
3. Label: `Load_Torque`

### Step 12: Add Scopes and Visualization

**Purpose:** Monitor simulation results

**Add the following Scope blocks:**

1. **Speed Scope:**
   - Inputs: Speed reference, Speed measured
   - Layout: 1 plot, 2 signals

2. **Current Scope (abc):**
   - Inputs: ia, ib, ic
   - Layout: 3 plots stacked

3. **Current Scope (dq):**
   - Inputs: id, iq, id_ref, iq_ref
   - Layout: 2 plots stacked

4. **Hall Sensor Scope:**
   - Inputs: Ha, Hb, Hc
   - Layout: 3 plots stacked

5. **Torque Scope:**
   - Inputs: Electromagnetic torque, Load torque
   - Layout: 1 plot, 2 signals

6. **Position Scope:**
   - Inputs: Actual theta_e, Estimated theta_e
   - Layout: 1 plot, 2 signals

### Step 13: Connect All Blocks

**Main Signal Flow:**

```
Speed_Ref → Speed_Controller → iq_ref
                ↑                  ↓
            Speed_Meas      Current_Controllers
                               (id_ref=0, iq_ref, id_meas, iq_meas)
                                   ↓
                            vd_ref, vq_ref
                                   ↓
                          Inverse_Park (+ theta_e)
                                   ↓
                            v_alpha, v_beta
                                   ↓
                          Inverse_Clarke
                                   ↓
                              va, vb, vc
                                   ↓
                                SVPWM
                                   ↓
                              Ta, Tb, Tc
                                   ↓
                            PMSM Motor Model
                                   ↓
                          ia, ib, ic, omega, theta
                                   ↓
                     ┌──────────────┴──────────────┐
                     ↓                             ↓
              Clarke Transform              Hall Sensors
                     ↓                             ↓
              Park Transform               theta_estimated
              (+ theta_e)                         ↓
                     ↓                      Speed Estimation
              id_meas, iq_meas                    ↓
                                            Speed_Measured
```

---

## Subsystem Details

### Critical Connections

1. **Feedback loops must be properly closed:**
   - Speed feedback: Motor speed → Speed estimator → Speed controller
   - Current feedback: Motor currents → Clarke → Park → Current controllers

2. **Position synchronization:**
   - Use Hall sensor estimated position (`theta_est`) for Park/Inverse Park
   - This simulates real-world operation where exact position is unknown

3. **Sample times:**
   - Current loop: `sim.Ts_control` (100 µs)
   - Speed loop: `sim.Ts_speed` (1 ms)
   - PWM: `sim.Ts` (10 µs)

---

## Configuration and Settings

### Signal Properties

Set signal data types for efficiency:
- Currents: `single` or `double`
- Angles: `single`
- Hall states: `boolean`
- Duty cycles: `single` (0 to 1 range)

### Algebraic Loop Resolution

If you encounter algebraic loops:
1. Add **Unit Delay** blocks to break loops
2. Use very small delay (one sample time)
3. Check initial conditions are realistic

---

## Running the Simulation

### Pre-Simulation Checklist

- [ ] Run `motor_parameters.m` script
- [ ] All parameters loaded in workspace
- [ ] Model saved
- [ ] Solver settings correct
- [ ] All blocks properly connected

### Running the Sim

1. Click **Run** button (or Ctrl+T)
2. Monitor simulation progress
3. Check for warnings/errors
4. Simulation should complete in 10-30 seconds

### Post-Simulation Analysis

1. **Open Scopes** to view results
2. **Check Speed Response:**
   - Rise time
   - Overshoot
   - Settling time
   - Steady-state error

3. **Check Current Response:**
   - id should be near 0
   - iq should follow torque command
   - Three-phase currents should be balanced

4. **Check Hall Sensors:**
   - Should show 6 distinct states per electrical revolution
   - Transitions should be clean

### Expected Results

**Good FOC Performance:**
- Speed follows reference with <5% overshoot
- Settling time <0.2 seconds
- Steady-state error <1%
- id ≈ 0 (±0.1A variation)
- iq smooth, follows reference
- Balanced three-phase currents

---

## Troubleshooting

### Common Issues

| Problem | Possible Cause | Solution |
|---------|---------------|----------|
| Simulation very slow | Step size too small | Increase to 1e-5 or 1e-4 |
| Motor doesn't start | Wrong Hall mapping | Check Hall sensor model |
| High current oscillations | Controller gains too high | Reduce Kp and Ki |
| Speed oscillations | Speed controller gains wrong | Retune speed loop |
| Algebraic loop error | Feedback loop issue | Add Unit Delay |
| Parameter not found | Didn't run initialization | Run motor_parameters.m |
| NaN or Inf values | Divide by zero or instability | Check calculations and limits |

### Debugging Tips

1. **Start Simple:**
   - Test motor model alone (open-loop)
   - Add controllers one at a time

2. **Use Data Inspector:**
   - Tools → Data Inspector
   - Log all signals
   - Plot and analyze

3. **Check Signals:**
   - Add Display blocks for key values
   - Use Scope blocks liberally
   - Check signal dimensions

4. **Verify Parameters:**
   - Type variable names in command window
   - Check values are reasonable

---

## Advanced Features (Optional)

### Add Overcurrent Protection

Add **Saturation** blocks after current controllers to limit current commands.

### Add Dead-Time Compensation

Modify SVPWM to account for inverter dead-time.

### Add Sensorless Startup

Implement open-loop startup sequence before engaging FOC.

### Add Field Weakening

Implement automatic id injection at high speeds.

---

## Conclusion

You now have a complete Simulink model for PMSM FOC with Hall sensors! The model includes:
- Speed and current control loops
- Coordinate transformations
- SVPWM generation
- Motor model
- Hall sensor simulation
- Position estimation
- Comprehensive monitoring

**Next Steps:**
1. Run the simulation and verify operation
2. Experiment with different load conditions
3. Tune controller parameters for your specific application
4. Add additional features as needed

**For questions or issues, refer to:**
- PMSM_FOC_Theory_and_Implementation.md
- README.md
- MATLAB/Simulink documentation

---

**Happy Simulating!**
