%% PI CONTROLLER FOR FOC
% This file contains PI controller implementations for Field Oriented Control
%
% Controllers included:
% - Discrete PI controller with anti-windup
% - Current PI controller (d-axis and q-axis)
% - Speed PI controller
% - Utilities for controller tuning and testing
%
% Author: PMSM FOC Project
% Date: 2025-11-07

%% DISCRETE PI CONTROLLER WITH ANTI-WINDUP

function [output, integrator_state] = pi_controller(error, integrator_state, Kp, Ki, Ts, output_min, output_max)
% PI_CONTROLLER - Discrete PI controller with anti-windup
%
% Implements a digital PI controller using the backward Euler method for
% integration. Includes anti-windup protection using integrator clamping.
%
% CONTROL LAW:
%   Proportional term: P(k) = Kp * error(k)
%   Integral term:     I(k) = I(k-1) + Ki * Ts * error(k)
%   Output:            u(k) = P(k) + I(k)
%
% ANTI-WINDUP:
%   The integrator is clamped to prevent windup when the output saturates.
%   This improves transient response when coming out of saturation.
%
% INPUTS:
%   error             - Control error (reference - measurement)
%   integrator_state  - Previous integrator value (persistent state)
%   Kp                - Proportional gain
%   Ki                - Integral gain
%   Ts                - Sample time [s]
%   output_min        - Minimum output limit
%   output_max        - Maximum output limit
%
% OUTPUTS:
%   output            - Controller output (saturated)
%   integrator_state  - Updated integrator value (for next iteration)
%
% EXAMPLE:
%   persistent integrator;
%   if isempty(integrator)
%       integrator = 0;
%   end
%   [u, integrator] = pi_controller(error, integrator, 0.1, 10, 0.0001, -5, 5);
%
% NOTE:
%   The integrator_state must be persistent or stored between calls

    % Calculate proportional term
    P_term = Kp * error;

    % Update integral term (backward Euler integration)
    I_term = integrator_state + Ki * Ts * error;

    % Calculate unsaturated output
    output_unsaturated = P_term + I_term;

    % Apply output saturation
    output = max(min(output_unsaturated, output_max), output_min);

    % Anti-windup: Only update integrator if not saturated
    % This prevents integrator windup
    if output == output_unsaturated
        % Output not saturated, accept the integrator update
        integrator_state = I_term;
    else
        % Output saturated, hold integrator at current value
        integrator_state = integrator_state;
    end

    % Alternative anti-windup method (back-calculation):
    % Calculate the difference between saturated and unsaturated output
    % saturation_error = output - output_unsaturated;
    % integrator_state = I_term + Kb * saturation_error;
    % where Kb is the back-calculation gain (typically Kb = Ki)

end


%% PARALLEL PI CONTROLLER (ALTERNATIVE IMPLEMENTATION)

function [output, integrator_state] = pi_controller_parallel(error, integrator_state, Kp, Ki, Ts, output_min, output_max, integrator_min, integrator_max)
% PI_CONTROLLER_PARALLEL - PI controller with separate integrator limits
%
% This version allows independent limits for the integrator and output,
% providing more flexible anti-windup control.
%
% INPUTS:
%   error             - Control error
%   integrator_state  - Previous integrator value
%   Kp                - Proportional gain
%   Ki                - Integral gain
%   Ts                - Sample time [s]
%   output_min        - Minimum output limit
%   output_max        - Maximum output limit
%   integrator_min    - Minimum integrator limit
%   integrator_max    - Maximum integrator limit
%
% OUTPUTS:
%   output            - Controller output (saturated)
%   integrator_state  - Updated integrator value

    % Update integral term with clamping
    integrator_state = integrator_state + Ki * Ts * error;
    integrator_state = max(min(integrator_state, integrator_max), integrator_min);

    % Calculate proportional term
    P_term = Kp * error;

    % Calculate output
    output = P_term + integrator_state;

    % Apply output saturation
    output = max(min(output, output_max), output_min);

end


%% CURRENT PI CONTROLLER

function [vd_ref, vq_ref, integrator_d, integrator_q] = current_pi_controller(id_ref, iq_ref, id_meas, iq_meas, integrator_d, integrator_q, params)
% CURRENT_PI_CONTROLLER - Dual PI controllers for d-axis and q-axis currents
%
% Implements two independent PI controllers for id and iq control in the
% dq reference frame. This is the inner loop of the FOC cascade control.
%
% INPUTS:
%   id_ref        - d-axis current reference [A] (typically 0 for surface PMSM)
%   iq_ref        - q-axis current reference [A] (torque command)
%   id_meas       - Measured d-axis current [A]
%   iq_meas       - Measured q-axis current [A]
%   integrator_d  - d-axis integrator state
%   integrator_q  - q-axis integrator state
%   params        - Structure with controller parameters:
%                   .Kp_d, .Ki_d    - d-axis gains
%                   .Kp_q, .Ki_q    - q-axis gains
%                   .Ts             - Sample time [s]
%                   .vd_min, .vd_max - d-axis voltage limits [V]
%                   .vq_min, .vq_max - q-axis voltage limits [V]
%
% OUTPUTS:
%   vd_ref        - d-axis voltage reference [V]
%   vq_ref        - q-axis voltage reference [V]
%   integrator_d  - Updated d-axis integrator state
%   integrator_q  - Updated q-axis integrator state
%
% EXAMPLE:
%   persistent int_d int_q;
%   if isempty(int_d), int_d = 0; int_q = 0; end
%   [vd, vq, int_d, int_q] = current_pi_controller(0, 2, id, iq, int_d, int_q, params);

    % Calculate current errors
    error_d = id_ref - id_meas;
    error_q = iq_ref - iq_meas;

    % d-axis PI controller
    [vd_ref, integrator_d] = pi_controller(error_d, integrator_d, ...
        params.Kp_d, params.Ki_d, params.Ts, ...
        params.vd_min, params.vd_max);

    % q-axis PI controller
    [vq_ref, integrator_q] = pi_controller(error_q, integrator_q, ...
        params.Kp_q, params.Ki_q, params.Ts, ...
        params.vq_min, params.vq_max);

end


%% SPEED PI CONTROLLER

function [iq_ref, integrator_state] = speed_pi_controller(speed_ref, speed_meas, integrator_state, params)
% SPEED_PI_CONTROLLER - PI controller for motor speed control
%
% Implements the outer loop speed controller in the FOC cascade control.
% The output is the q-axis current reference (torque command).
%
% INPUTS:
%   speed_ref        - Speed reference [rad/s] or [RPM] (must match speed_meas units)
%   speed_meas       - Measured speed [rad/s] or [RPM]
%   integrator_state - Speed integrator state
%   params           - Structure with controller parameters:
%                      .Kp            - Proportional gain
%                      .Ki            - Integral gain
%                      .Ts            - Sample time [s]
%                      .iq_min, .iq_max - Current limits [A]
%
% OUTPUTS:
%   iq_ref           - q-axis current reference [A]
%   integrator_state - Updated integrator state
%
% EXAMPLE:
%   persistent int_speed;
%   if isempty(int_speed), int_speed = 0; end
%   [iq_ref, int_speed] = speed_pi_controller(1000, 950, int_speed, params);

    % Calculate speed error
    error_speed = speed_ref - speed_meas;

    % Speed PI controller
    [iq_ref, integrator_state] = pi_controller(error_speed, integrator_state, ...
        params.Kp, params.Ki, params.Ts, ...
        params.iq_min, params.iq_max);

end


%% FEEDFORWARD COMPENSATION (OPTIONAL ENHANCEMENT)

function [vd_ff, vq_ff] = current_feedforward(id, iq, omega_e, motor_params)
% CURRENT_FEEDFORWARD - Feedforward compensation for current control
%
% Adds feedforward terms to improve current control dynamic response.
% This compensates for cross-coupling and back-EMF effects.
%
% FEEDFORWARD TERMS:
%   vd_ff = -omega_e * Lq * iq             (cross-coupling term)
%   vq_ff = omega_e * Ld * id + omega_e * lambda_m  (cross-coupling + back-EMF)
%
% INPUTS:
%   id           - d-axis current [A]
%   iq           - q-axis current [A]
%   omega_e      - Electrical angular velocity [rad/s]
%   motor_params - Structure with motor parameters:
%                  .Ld, .Lq     - Inductances [H]
%                  .lambda_m    - Flux linkage [Wb]
%
% OUTPUTS:
%   vd_ff - d-axis feedforward voltage [V]
%   vq_ff - q-axis feedforward voltage [V]
%
% USAGE:
%   Add these feedforward terms to the PI controller outputs:
%   vd_total = vd_pi + vd_ff;
%   vq_total = vq_pi + vq_ff;

    % Cross-coupling compensation
    vd_ff = -omega_e * motor_params.Lq * iq;
    vq_ff = omega_e * motor_params.Ld * id;

    % Back-EMF compensation (only for q-axis)
    vq_ff = vq_ff + omega_e * motor_params.lambda_m;

end


%% CONTROLLER TUNING UTILITIES

function gains = tune_current_controller(motor_params, desired_bandwidth_hz)
% TUNE_CURRENT_CONTROLLER - Calculate current controller gains
%
% Uses the technical optimum method to calculate PI gains for current control.
%
% INPUTS:
%   motor_params          - Structure with .Rs, .Ls
%   desired_bandwidth_hz  - Desired closed-loop bandwidth [Hz]
%
% OUTPUTS:
%   gains - Structure with .Kp and .Ki

    omega_cc = 2*pi * desired_bandwidth_hz;
    gains.Kp = motor_params.Ls * omega_cc;
    gains.Ki = motor_params.Rs * omega_cc;

    fprintf('Current Controller Tuning:\n');
    fprintf('  Bandwidth: %.1f Hz (%.1f rad/s)\n', desired_bandwidth_hz, omega_cc);
    fprintf('  Kp = %.5f\n', gains.Kp);
    fprintf('  Ki = %.5f\n', gains.Ki);

end


function gains = tune_speed_controller(motor_params, desired_bandwidth_hz, damping_ratio)
% TUNE_SPEED_CONTROLLER - Calculate speed controller gains
%
% Uses the symmetrical optimum method to calculate PI gains for speed control.
%
% INPUTS:
%   motor_params          - Structure with .J, .B, .Kt
%   desired_bandwidth_hz  - Desired closed-loop bandwidth [Hz]
%   damping_ratio         - Desired damping ratio (typically 0.707)
%
% OUTPUTS:
%   gains - Structure with .Kp and .Ki

    if nargin < 3
        damping_ratio = 0.707;  % Default to critically damped
    end

    omega_sc = 2*pi * desired_bandwidth_hz;

    % Simplified tuning based on mechanical parameters
    gains.Kp = 2 * damping_ratio * motor_params.J * omega_sc / motor_params.Kt;
    gains.Ki = motor_params.J * omega_sc^2 / motor_params.Kt;

    fprintf('Speed Controller Tuning:\n');
    fprintf('  Bandwidth: %.1f Hz (%.1f rad/s)\n', desired_bandwidth_hz, omega_sc);
    fprintf('  Damping: %.3f\n', damping_ratio);
    fprintf('  Kp = %.5f\n', gains.Kp);
    fprintf('  Ki = %.5f\n', gains.Ki);

end


%% STEP RESPONSE SIMULATION

function plot_pi_step_response(Kp, Ki, Ts, reference, disturbance_time, simulation_time)
% PLOT_PI_STEP_RESPONSE - Simulate and plot PI controller step response
%
% Simulates a simple first-order plant with PI controller to visualize
% the controller performance.
%
% INPUTS:
%   Kp                - Proportional gain
%   Ki                - Integral gain
%   Ts                - Sample time [s]
%   reference         - Reference value
%   disturbance_time  - Time of load disturbance [s]
%   simulation_time   - Total simulation time [s]

    % Plant parameters (simple first-order system)
    tau = 0.01;  % Time constant
    K_plant = 1; % Gain

    % Simulation setup
    time = 0:Ts:simulation_time;
    n_steps = length(time);

    % Initialize arrays
    output = zeros(1, n_steps);
    error = zeros(1, n_steps);
    control = zeros(1, n_steps);
    ref = zeros(1, n_steps);
    disturbance = zeros(1, n_steps);

    % Set reference and disturbance
    ref(:) = reference;
    disturbance(time > disturbance_time) = -0.3 * reference;

    % Initialize controller state
    integrator = 0;

    % Simulation loop
    for k = 2:n_steps
        % Calculate error
        error(k) = ref(k) - output(k-1);

        % PI controller
        [control(k), integrator] = pi_controller(error(k), integrator, ...
            Kp, Ki, Ts, -10, 10);

        % Simple plant model (first-order system with disturbance)
        output(k) = output(k-1) + (Ts/tau) * (K_plant * control(k) - output(k-1)) + disturbance(k);
    end

    % Plot results
    figure('Name', 'PI Controller Step Response', 'NumberTitle', 'off');

    subplot(3,1,1);
    plot(time, ref, 'r--', 'LineWidth', 1.5); hold on;
    plot(time, output, 'b', 'LineWidth', 2);
    grid on;
    xlabel('Time [s]');
    ylabel('Output');
    title('Step Response');
    legend('Reference', 'Output', 'Location', 'best');

    subplot(3,1,2);
    plot(time, error, 'r', 'LineWidth', 2);
    grid on;
    xlabel('Time [s]');
    ylabel('Error');
    title('Tracking Error');

    subplot(3,1,3);
    plot(time, control, 'b', 'LineWidth', 2);
    grid on;
    xlabel('Time [s]');
    ylabel('Control Signal');
    title('Controller Output');

    % Calculate performance metrics
    settling_index = find(abs(error) < 0.05*reference, 1, 'first');
    if ~isempty(settling_index)
        settling_time = time(settling_index);
        fprintf('\nPerformance Metrics:\n');
        fprintf('  Settling Time (5%%): %.4f s\n', settling_time);
    end

    % Overshoot
    max_output = max(output);
    overshoot = (max_output - reference) / reference * 100;
    fprintf('  Overshoot: %.2f%%\n', overshoot);

    % Steady-state error
    ss_error = abs(mean(error(end-100:end)));
    fprintf('  Steady-State Error: %.6f\n', ss_error);

end


%% VALIDATION FUNCTION

function validate_pi_controller()
% VALIDATE_PI_CONTROLLER - Test the PI controller implementation
%
% Runs validation tests on the PI controller to ensure correct operation.

    fprintf('\n========================================\n');
    fprintf('PI Controller Validation\n');
    fprintf('========================================\n\n');

    % Test parameters
    Kp = 0.1;
    Ki = 10;
    Ts = 0.0001;
    output_min = -5;
    output_max = 5;

    % Test 1: Zero error should give zero output (initially)
    fprintf('Test 1: Zero error response\n');
    error = 0;
    integrator = 0;
    [output, integrator_new] = pi_controller(error, integrator, Kp, Ki, Ts, output_min, output_max);
    fprintf('  Error=%.2f, Output=%.4f (expected: 0.0000) %s\n\n', ...
        error, output, ternary(abs(output) < 1e-10, '✓ PASS', '✗ FAIL'));

    % Test 2: Positive error should give positive output
    fprintf('Test 2: Positive error response\n');
    error = 1.0;
    integrator = 0;
    [output, integrator_new] = pi_controller(error, integrator, Kp, Ki, Ts, output_min, output_max);
    fprintf('  Error=%.2f, Output=%.4f (expected: >0) %s\n\n', ...
        error, output, ternary(output > 0, '✓ PASS', '✗ FAIL'));

    % Test 3: Saturation test
    fprintf('Test 3: Output saturation\n');
    error = 100.0;  % Very large error
    integrator = 0;
    [output, integrator_new] = pi_controller(error, integrator, Kp, Ki, Ts, output_min, output_max);
    fprintf('  Error=%.2f, Output=%.4f (expected: %.2f) %s\n\n', ...
        error, output, output_max, ternary(abs(output - output_max) < 1e-6, '✓ PASS', '✗ FAIL'));

    % Test 4: Integrator accumulation
    fprintf('Test 4: Integrator accumulation\n');
    error = 1.0;
    integrator = 0;
    for i = 1:10
        [output, integrator] = pi_controller(error, integrator, Kp, Ki, Ts, output_min, output_max);
    end
    fprintf('  After 10 iterations with constant error=%.2f\n', error);
    fprintf('  Output=%.4f (should be increasing due to integration) %s\n\n', ...
        output, ternary(output > Kp*error, '✓ PASS', '✗ FAIL'));

    fprintf('========================================\n\n');

    % Visual test - step response
    fprintf('Generating step response plot...\n');
    plot_pi_step_response(0.5, 50, 0.0001, 1.0, 0.5, 1.0);

end

function result = ternary(condition, true_val, false_val)
    if condition
        result = true_val;
    else
        result = false_val;
    end
end
