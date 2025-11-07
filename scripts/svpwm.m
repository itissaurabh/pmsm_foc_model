%% SPACE VECTOR PULSE WIDTH MODULATION (SVPWM)
% This file contains SVPWM implementation for three-phase inverter control
%
% SVPWM is an advanced PWM technique that provides:
% - Better DC bus utilization (15% more voltage compared to SPWM)
% - Lower harmonic distortion
% - More efficient operation
%
% Author: PMSM FOC Project
% Date: 2025-11-07

function [Ta, Tb, Tc, sector] = svpwm(v_alpha, v_beta, Vdc, Ts)
% SVPWM - Space Vector Pulse Width Modulation
%
% Generates three-phase PWM duty cycles using space vector modulation.
% This is the most efficient PWM technique for three-phase inverters.
%
% THEORY:
%   The voltage space vector is decomposed into two adjacent basic vectors
%   and a zero vector. The duty cycles are calculated to synthesize the
%   desired voltage vector on average over one PWM period.
%
%   The voltage plane is divided into 6 sectors (60° each).
%   Each sector uses two active vectors and zero vectors.
%
% INPUTS:
%   v_alpha - α-axis voltage reference [V]
%   v_beta  - β-axis voltage reference [V]
%   Vdc     - DC-link voltage [V]
%   Ts      - PWM switching period [s]
%
% OUTPUTS:
%   Ta, Tb, Tc - Duty cycles for phases A, B, C (0 to 1)
%   sector     - Current sector number (1 to 6)
%
% EXAMPLE:
%   [Ta, Tb, Tc, sector] = svpwm(10, 5, 28, 0.0001);
%
% NOTE:
%   - Maximum modulation index is ~0.577 for linear operation
%   - Above this, overmodulation occurs (handled gracefully)
%   - Duty cycles are normalized to [0, 1] range

    % Calculate magnitude and angle of voltage vector
    V_ref = sqrt(v_alpha^2 + v_beta^2);
    theta = atan2(v_beta, v_alpha);

    % Normalize angle to [0, 2π]
    if theta < 0
        theta = theta + 2*pi;
    end

    % Determine sector (1 to 6)
    sector = floor(theta / (pi/3)) + 1;
    if sector > 6
        sector = 6;
    end

    % Calculate angle within sector [0, π/3]
    theta_sector = theta - (sector - 1) * (pi/3);

    % Maximum voltage that can be achieved with SVPWM
    V_max = Vdc / sqrt(3);

    % Check for overmodulation and limit if necessary
    if V_ref > V_max
        V_ref = V_max;
        % warning('SVPWM: Voltage reference exceeds maximum, limiting to %.2f V', V_max);
    end

    % Modulation index (0 to 1 for linear region, up to 1.15 for overmodulation)
    m = V_ref / V_max;

    % Calculate duty cycle for adjacent vectors
    T1 = m * Ts * sin(pi/3 - theta_sector) / sin(pi/3);
    T2 = m * Ts * sin(theta_sector) / sin(pi/3);
    T0 = Ts - T1 - T2;  % Zero vector time

    % Ensure times are non-negative (numerical safety)
    T1 = max(T1, 0);
    T2 = max(T2, 0);
    T0 = max(T0, 0);

    % Calculate duty cycles for each phase based on sector
    % Using centered PWM (zero vectors split equally: T0/2 at start and end)
    switch sector
        case 1  % Sector 1: 0° to 60°
            % Active vectors: V1(100) and V2(110)
            Ta = (T1 + T2 + T0/2) / Ts;
            Tb = (T2 + T0/2) / Ts;
            Tc = T0/2 / Ts;

        case 2  % Sector 2: 60° to 120°
            % Active vectors: V2(110) and V3(010)
            Ta = (T1 + T0/2) / Ts;
            Tb = (T1 + T2 + T0/2) / Ts;
            Tc = T0/2 / Ts;

        case 3  % Sector 3: 120° to 180°
            % Active vectors: V3(010) and V4(011)
            Ta = T0/2 / Ts;
            Tb = (T1 + T2 + T0/2) / Ts;
            Tc = (T2 + T0/2) / Ts;

        case 4  % Sector 4: 180° to 240°
            % Active vectors: V4(011) and V5(001)
            Ta = T0/2 / Ts;
            Tb = (T1 + T0/2) / Ts;
            Tc = (T1 + T2 + T0/2) / Ts;

        case 5  % Sector 5: 240° to 300°
            % Active vectors: V5(001) and V6(101)
            Ta = (T2 + T0/2) / Ts;
            Tb = T0/2 / Ts;
            Tc = (T1 + T2 + T0/2) / Ts;

        case 6  % Sector 6: 300° to 360°
            % Active vectors: V6(101) and V1(100)
            Ta = (T1 + T2 + T0/2) / Ts;
            Tb = T0/2 / Ts;
            Tc = (T1 + T0/2) / Ts;

        otherwise
            % Should never happen, but provide safe fallback
            Ta = 0.5;
            Tb = 0.5;
            Tc = 0.5;
    end

    % Ensure duty cycles are within [0, 1]
    Ta = max(min(Ta, 1), 0);
    Tb = max(min(Tb, 1), 0);
    Tc = max(min(Tc, 1), 0);

end


function [Va, Vb, Vc] = duty_to_voltage(Ta, Tb, Tc, Vdc)
% DUTY_TO_VOLTAGE - Convert duty cycles to average phase voltages
%
% Calculates the average phase voltages from PWM duty cycles.
% This is useful for simulation and verification.
%
% INPUTS:
%   Ta, Tb, Tc - Duty cycles (0 to 1)
%   Vdc        - DC-link voltage [V]
%
% OUTPUTS:
%   Va, Vb, Vc - Average phase voltages [V]
%
% EXAMPLE:
%   [Va, Vb, Vc] = duty_to_voltage(0.6, 0.5, 0.4, 28);

    % Phase-to-neutral voltages (considering the neutral point)
    Va = Vdc * (Ta - (Ta + Tb + Tc)/3);
    Vb = Vdc * (Tb - (Ta + Tb + Tc)/3);
    Vc = Vdc * (Tc - (Ta + Tb + Tc)/3);

end


function [v_alpha_recon, v_beta_recon] = reconstruct_voltage(Ta, Tb, Tc, Vdc)
% RECONSTRUCT_VOLTAGE - Reconstruct α-β voltages from duty cycles
%
% Converts duty cycles back to α-β reference frame voltages.
% Useful for verification and closed-loop simulation.
%
% INPUTS:
%   Ta, Tb, Tc - Duty cycles (0 to 1)
%   Vdc        - DC-link voltage [V]
%
% OUTPUTS:
%   v_alpha_recon - Reconstructed α-axis voltage [V]
%   v_beta_recon  - Reconstructed β-axis voltage [V]

    % Convert duty cycles to phase voltages
    [Va, Vb, Vc] = duty_to_voltage(Ta, Tb, Tc, Vdc);

    % Apply Clarke transformation
    v_alpha_recon = (2/3) * (Va - 0.5*Vb - 0.5*Vc);
    v_beta_recon = (2/3) * (sqrt(3)/2) * (Vb - Vc);

end


function plot_svpwm_hexagon()
% PLOT_SVPWM_HEXAGON - Visualize SVPWM space vector hexagon
%
% Creates a plot showing the voltage space vectors and the hexagonal
% boundary that defines the maximum achievable voltages.
%
% EXAMPLE:
%   plot_svpwm_hexagon();

    % DC-link voltage for visualization
    Vdc = 28;
    V_max = Vdc / sqrt(3);

    % Define the 6 active voltage vectors (corners of hexagon)
    % Vector angles: 0°, 60°, 120°, 180°, 240°, 300°
    angles = [0, 60, 120, 180, 240, 300] * pi/180;
    V_vectors = (2/3) * Vdc * [cos(angles); sin(angles)];

    % Close the hexagon
    V_vectors = [V_vectors, V_vectors(:,1)];

    % Create figure
    figure('Name', 'SVPWM Space Vector Hexagon', 'NumberTitle', 'off');
    hold on;

    % Plot hexagon
    plot(V_vectors(1,:), V_vectors(2,:), 'b-', 'LineWidth', 2);

    % Plot the 6 active vectors
    for i = 1:6
        quiver(0, 0, V_vectors(1,i), V_vectors(2,i), 0, ...
            'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
        text(V_vectors(1,i)*1.1, V_vectors(2,i)*1.1, ...
            sprintf('V%d', i), 'FontSize', 12, 'FontWeight', 'bold');
    end

    % Plot maximum circle (inscribed circle)
    theta_circle = linspace(0, 2*pi, 100);
    circle_x = V_max * cos(theta_circle);
    circle_y = V_max * sin(theta_circle);
    plot(circle_x, circle_y, 'g--', 'LineWidth', 1.5);

    % Plot sector boundaries
    for i = 1:6
        angle = (i-1) * pi/3;
        plot([0, V_max*1.2*cos(angle)], [0, V_max*1.2*sin(angle)], ...
            'k--', 'LineWidth', 0.5);
        % Label sectors
        label_angle = (i-0.5) * pi/3;
        text(V_max*0.5*cos(label_angle), V_max*0.5*sin(label_angle), ...
            sprintf('S%d', i), 'FontSize', 10, 'Color', 'blue', ...
            'HorizontalAlignment', 'center');
    end

    % Plot zero vector at origin
    plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    text(0, -1, 'V0, V7', 'FontSize', 10, 'HorizontalAlignment', 'center');

    % Formatting
    grid on;
    axis equal;
    xlabel('V_{\alpha} [V]');
    ylabel('V_{\beta} [V]');
    title(sprintf('SVPWM Space Vector Diagram (Vdc = %.0f V)', Vdc));
    legend('Hexagon Boundary', 'Active Vectors', 'Max Linear Circle', ...
        'Location', 'best');

    % Add text annotations
    text(0, V_max*1.4, sprintf('V_{max} = %.2f V', V_max), ...
        'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold');
    text(0, V_max*1.3, sprintf('Modulation Index = %.2f', V_max/(Vdc/2)), ...
        'HorizontalAlignment', 'center', 'FontSize', 10);

    hold off;

end


function plot_svpwm_duty_cycles(v_alpha_range, Vdc)
% PLOT_SVPWM_DUTY_CYCLES - Plot duty cycles for rotating voltage vector
%
% Visualizes how the three-phase duty cycles vary as the voltage vector
% rotates through one complete revolution.
%
% INPUTS:
%   v_alpha_range - Array of v_alpha values (one complete rotation)
%   Vdc           - DC-link voltage [V]
%
% EXAMPLE:
%   theta = linspace(0, 2*pi, 360);
%   V_ref = 10;  % 10V magnitude
%   v_alpha = V_ref * cos(theta);
%   v_beta = V_ref * sin(theta);
%   plot_svpwm_duty_cycles_rotation(v_alpha, v_beta, 28);

    if nargin < 2
        Vdc = 28;
    end

    % Generate one complete rotation
    theta = linspace(0, 2*pi, 360);
    V_ref = 10;  % 10V reference magnitude
    v_alpha = V_ref * cos(theta);
    v_beta = V_ref * sin(theta);

    % PWM period
    Ts = 0.0001;  % 100 µs (10 kHz)

    % Calculate duty cycles for each angle
    n_points = length(theta);
    Ta_array = zeros(1, n_points);
    Tb_array = zeros(1, n_points);
    Tc_array = zeros(1, n_points);
    sector_array = zeros(1, n_points);

    for i = 1:n_points
        [Ta_array(i), Tb_array(i), Tc_array(i), sector_array(i)] = ...
            svpwm(v_alpha(i), v_beta(i), Vdc, Ts);
    end

    % Create plots
    figure('Name', 'SVPWM Duty Cycles', 'NumberTitle', 'off');

    % Plot 1: Duty cycles vs angle
    subplot(3,1,1);
    plot(rad2deg(theta), Ta_array, 'r', 'LineWidth', 2); hold on;
    plot(rad2deg(theta), Tb_array, 'g', 'LineWidth', 2);
    plot(rad2deg(theta), Tc_array, 'b', 'LineWidth', 2);
    grid on;
    xlabel('Electrical Angle [degrees]');
    ylabel('Duty Cycle');
    title('SVPWM Duty Cycles vs Rotor Position');
    legend('Ta (Phase A)', 'Tb (Phase B)', 'Tc (Phase C)', 'Location', 'best');
    ylim([0, 1]);

    % Plot 2: Sector number
    subplot(3,1,2);
    plot(rad2deg(theta), sector_array, 'k', 'LineWidth', 2);
    grid on;
    xlabel('Electrical Angle [degrees]');
    ylabel('Sector Number');
    title('Active Sector vs Rotor Position');
    ylim([0, 7]);
    yticks(1:6);

    % Plot 3: Reconstructed voltages
    subplot(3,1,3);
    v_alpha_recon = zeros(1, n_points);
    v_beta_recon = zeros(1, n_points);
    for i = 1:n_points
        [v_alpha_recon(i), v_beta_recon(i)] = ...
            reconstruct_voltage(Ta_array(i), Tb_array(i), Tc_array(i), Vdc);
    end
    plot(rad2deg(theta), v_alpha, 'r--', 'LineWidth', 1); hold on;
    plot(rad2deg(theta), v_alpha_recon, 'r', 'LineWidth', 2);
    plot(rad2deg(theta), v_beta, 'b--', 'LineWidth', 1);
    plot(rad2deg(theta), v_beta_recon, 'b', 'LineWidth', 2);
    grid on;
    xlabel('Electrical Angle [degrees]');
    ylabel('Voltage [V]');
    title('Reference vs Reconstructed Voltages');
    legend('V_{\alpha} ref', 'V_{\alpha} recon', 'V_{\beta} ref', 'V_{\beta} recon', ...
        'Location', 'best');

end


function validate_svpwm()
% VALIDATE_SVPWM - Test the SVPWM implementation
%
% Runs validation tests to ensure SVPWM is working correctly.

    fprintf('\n========================================\n');
    fprintf('SVPWM Validation\n');
    fprintf('========================================\n\n');

    Vdc = 28;
    Ts = 0.0001;

    % Test 1: Zero voltage
    fprintf('Test 1: Zero voltage reference\n');
    [Ta, Tb, Tc, sector] = svpwm(0, 0, Vdc, Ts);
    fprintf('  v_alpha=0, v_beta=0\n');
    fprintf('  Ta=%.4f, Tb=%.4f, Tc=%.4f, Sector=%d\n', Ta, Tb, Tc, sector);
    fprintf('  Expected: All duty cycles ≈ 0.5 (50%%) %s\n\n', ...
        ternary(abs(Ta-0.5)<0.01 && abs(Tb-0.5)<0.01 && abs(Tc-0.5)<0.01, '✓ PASS', '✗ FAIL'));

    % Test 2: Sector 1 (0 degrees)
    fprintf('Test 2: Voltage in Sector 1 (θ = 0°)\n');
    v_alpha = 10; v_beta = 0;
    [Ta, Tb, Tc, sector] = svpwm(v_alpha, v_beta, Vdc, Ts);
    fprintf('  v_alpha=%.2f, v_beta=%.2f\n', v_alpha, v_beta);
    fprintf('  Ta=%.4f, Tb=%.4f, Tc=%.4f, Sector=%d\n', Ta, Tb, Tc, sector);
    fprintf('  Expected: Sector=1, Ta>Tb>Tc %s\n\n', ...
        ternary(sector==1 && Ta>Tb && Tb>Tc, '✓ PASS', '✗ FAIL'));

    % Test 3: Sector 3 (120 degrees)
    fprintf('Test 3: Voltage in Sector 3 (θ = 120°)\n');
    v_alpha = -5; v_beta = 8.66;
    [Ta, Tb, Tc, sector] = svpwm(v_alpha, v_beta, Vdc, Ts);
    fprintf('  v_alpha=%.2f, v_beta=%.2f\n', v_alpha, v_beta);
    fprintf('  Ta=%.4f, Tb=%.4f, Tc=%.4f, Sector=%d\n', Ta, Tb, Tc, sector);
    fprintf('  Expected: Sector=3, Tb>Tc>Ta %s\n\n', ...
        ternary(sector==3 && Tb>Tc && Tc>Ta, '✓ PASS', '✗ FAIL'));

    % Test 4: Voltage reconstruction accuracy
    fprintf('Test 4: Voltage reconstruction accuracy\n');
    v_alpha = 8; v_beta = 6;
    [Ta, Tb, Tc, sector] = svpwm(v_alpha, v_beta, Vdc, Ts);
    [v_alpha_recon, v_beta_recon] = reconstruct_voltage(Ta, Tb, Tc, Vdc);
    error_alpha = abs(v_alpha - v_alpha_recon);
    error_beta = abs(v_beta - v_beta_recon);
    fprintf('  Original:      v_alpha=%.4f, v_beta=%.4f\n', v_alpha, v_beta);
    fprintf('  Reconstructed: v_alpha=%.4f, v_beta=%.4f\n', v_alpha_recon, v_beta_recon);
    fprintf('  Error:         Δα=%.6f, Δβ=%.6f %s\n\n', error_alpha, error_beta, ...
        ternary(error_alpha<0.01 && error_beta<0.01, '✓ PASS', '✗ FAIL'));

    % Test 5: Maximum voltage
    fprintf('Test 5: Maximum voltage (linear region limit)\n');
    V_max = Vdc / sqrt(3);
    [Ta, Tb, Tc, sector] = svpwm(V_max, 0, Vdc, Ts);
    fprintf('  V_max = %.4f V\n', V_max);
    fprintf('  Ta=%.4f, Tb=%.4f, Tc=%.4f\n', Ta, Tb, Tc);
    fprintf('  All duty cycles should be in [0,1] %s\n\n', ...
        ternary(Ta>=0 && Ta<=1 && Tb>=0 && Tb<=1 && Tc>=0 && Tc<=1, '✓ PASS', '✗ FAIL'));

    fprintf('========================================\n\n');

    % Generate visualization plots
    fprintf('Generating SVPWM visualization plots...\n');
    plot_svpwm_hexagon();
    plot_svpwm_duty_cycles([], Vdc);

end

function result = ternary(condition, true_val, false_val)
    if condition
        result = true_val;
    else
        result = false_val;
    end
end
