function [Ha, Hb, Hc, sector, theta_est] = hall_sensor_model(theta_e, offset_angle)
% HALL_SENSOR_MODEL - Simulates 3-phase Hall effect sensors for PMSM
%
% This function simulates the behavior of three Hall effect sensors placed
% 120 electrical degrees apart. Each sensor outputs a digital HIGH (1) or
% LOW (0) signal based on the rotor's electrical position.
%
% INPUTS:
%   theta_e       - Electrical rotor position [rad] (0 to 2*pi)
%   offset_angle  - Hall sensor offset angle [rad] (default: 0)
%
% OUTPUTS:
%   Ha            - Hall sensor A output (0 or 1)
%   Hb            - Hall sensor B output (0 or 1)
%   Hc            - Hall sensor C output (0 or 1)
%   sector        - Current sector number (1 to 6)
%   theta_est     - Estimated electrical position [rad]
%
% HALL SENSOR TRUTH TABLE:
% Sector | Angle Range    | Ha | Hb | Hc | Binary
% -------|----------------|----|----|----|---------
%   1    | 0° to 60°      | 1  | 0  | 1  | 101 (5)
%   2    | 60° to 120°    | 1  | 0  | 0  | 100 (4)
%   3    | 120° to 180°   | 1  | 1  | 0  | 110 (6)
%   4    | 180° to 240°   | 0  | 1  | 0  | 010 (2)
%   5    | 240° to 300°   | 0  | 1  | 1  | 011 (3)
%   6    | 300° to 360°   | 0  | 0  | 1  | 001 (1)
%
% EXAMPLE:
%   theta_e = pi/4;  % 45 degrees
%   [Ha, Hb, Hc, sector, theta_est] = hall_sensor_model(theta_e, 0);
%   % Returns: Ha=1, Hb=0, Hc=1, sector=1, theta_est=0 (sector 1 center)
%
% Author: PMSM FOC Project
% Date: 2025-11-07

    % Handle default offset angle
    if nargin < 2
        offset_angle = 0;
    end

    % Normalize theta_e to [0, 2*pi]
    theta_e = mod(theta_e + offset_angle, 2*pi);

    % Convert to degrees for easier understanding
    theta_deg = rad2deg(theta_e);

    % Determine sector based on electrical angle
    % Each sector spans 60 electrical degrees
    if theta_deg >= 0 && theta_deg < 60
        sector = 1;
        Ha = 1; Hb = 0; Hc = 1;
        theta_est = deg2rad(30);  % Center of sector 1
    elseif theta_deg >= 60 && theta_deg < 120
        sector = 2;
        Ha = 1; Hb = 0; Hc = 0;
        theta_est = deg2rad(90);  % Center of sector 2
    elseif theta_deg >= 120 && theta_deg < 180
        sector = 3;
        Ha = 1; Hb = 1; Hc = 0;
        theta_est = deg2rad(150); % Center of sector 3
    elseif theta_deg >= 180 && theta_deg < 240
        sector = 4;
        Ha = 0; Hb = 1; Hc = 0;
        theta_est = deg2rad(210); % Center of sector 4
    elseif theta_deg >= 240 && theta_deg < 300
        sector = 5;
        Ha = 0; Hb = 1; Hc = 1;
        theta_est = deg2rad(270); % Center of sector 5
    else  % 300 to 360
        sector = 6;
        Ha = 0; Hb = 0; Hc = 1;
        theta_est = deg2rad(330); % Center of sector 6
    end

end


function [theta_est, sector] = hall_decode(Ha, Hb, Hc)
% HALL_DECODE - Decodes Hall sensor states to electrical position
%
% This function takes the three Hall sensor digital outputs and decodes
% them to estimate the electrical rotor position. The position estimate
% has 60-degree resolution.
%
% INPUTS:
%   Ha, Hb, Hc  - Hall sensor outputs (0 or 1)
%
% OUTPUTS:
%   theta_est   - Estimated electrical position [rad]
%   sector      - Current sector number (1 to 6)
%
% EXAMPLE:
%   [theta_est, sector] = hall_decode(1, 0, 1);
%   % Returns: theta_est = 0.524 rad (30 deg), sector = 1

    % Combine Hall states into a single number (0-7)
    hall_state = 4*Ha + 2*Hb + Hc;

    % Decode Hall state to sector and position
    switch hall_state
        case 5  % 101 - Sector 1
            sector = 1;
            theta_est = deg2rad(30);
        case 4  % 100 - Sector 2
            sector = 2;
            theta_est = deg2rad(90);
        case 6  % 110 - Sector 3
            sector = 3;
            theta_est = deg2rad(150);
        case 2  % 010 - Sector 4
            sector = 4;
            theta_est = deg2rad(210);
        case 3  % 011 - Sector 5
            sector = 5;
            theta_est = deg2rad(270);
        case 1  % 001 - Sector 6
            sector = 6;
            theta_est = deg2rad(330);
        otherwise  % Invalid Hall state (0 or 7)
            sector = 0;
            theta_est = 0;
            warning('Invalid Hall sensor state: %d', hall_state);
    end

end


function [speed_est] = hall_speed_estimator(sector_current, sector_previous, time_elapsed)
% HALL_SPEED_ESTIMATOR - Estimates motor speed from Hall sensor transitions
%
% This function estimates the electrical angular velocity by measuring the
% time between Hall sensor state changes. Each state change represents a
% 60-degree rotation.
%
% INPUTS:
%   sector_current   - Current Hall sector (1-6)
%   sector_previous  - Previous Hall sector (1-6)
%   time_elapsed     - Time since last sector change [s]
%
% OUTPUTS:
%   speed_est        - Estimated electrical angular velocity [rad/s]
%
% EXAMPLE:
%   speed_est = hall_speed_estimator(2, 1, 0.001);
%   % Returns speed estimate based on 1ms sector transition time

    % Each sector transition represents 60 electrical degrees
    sector_angle = pi/3;  % 60 degrees in radians

    % Check for valid sector change
    sector_diff = sector_current - sector_previous;

    % Handle wraparound (sector 6 to sector 1)
    if sector_diff == -5
        sector_diff = 1;
    elseif sector_diff == 5
        sector_diff = -1;
    end

    % Calculate speed (positive for forward, negative for reverse)
    if time_elapsed > 0 && abs(sector_diff) == 1
        speed_est = sector_diff * sector_angle / time_elapsed;
    else
        speed_est = 0;  % No movement or invalid transition
    end

    % Limit maximum estimated speed (prevent division by very small time)
    max_speed = 10000;  % rad/s (very high limit for safety)
    speed_est = max(min(speed_est, max_speed), -max_speed);

end


function [Ha_noisy, Hb_noisy, Hc_noisy] = hall_add_noise(Ha, Hb, Hc, noise_prob)
% HALL_ADD_NOISE - Adds random noise to Hall sensor signals
%
% Simulates real-world Hall sensor noise and glitches by randomly flipping
% sensor states with a given probability.
%
% INPUTS:
%   Ha, Hb, Hc   - Clean Hall sensor outputs (0 or 1)
%   noise_prob   - Probability of bit flip (0 to 1, typical: 0.001)
%
% OUTPUTS:
%   Ha_noisy, Hb_noisy, Hc_noisy - Noisy Hall sensor outputs
%
% EXAMPLE:
%   [Ha_n, Hb_n, Hc_n] = hall_add_noise(1, 0, 1, 0.01);
%   % Returns Hall states with 1% chance of bit flip per sensor

    if nargin < 4
        noise_prob = 0.001;  % Default 0.1% noise
    end

    % Add noise to each Hall sensor independently
    if rand() < noise_prob
        Ha_noisy = ~Ha;
    else
        Ha_noisy = Ha;
    end

    if rand() < noise_prob
        Hb_noisy = ~Hb;
    else
        Hb_noisy = Hb;
    end

    if rand() < noise_prob
        Hc_noisy = ~Hc;
    else
        Hc_noisy = Hc;
    end

end


function plot_hall_sensors(theta_range)
% PLOT_HALL_SENSORS - Plots Hall sensor outputs vs rotor position
%
% Generates a visualization showing how the three Hall sensor outputs
% change as the rotor rotates through one complete electrical revolution.
%
% INPUT:
%   theta_range - Optional angle range [rad] (default: 0 to 2*pi)
%
% EXAMPLE:
%   plot_hall_sensors();  % Plot one complete electrical revolution

    if nargin < 1
        theta_range = linspace(0, 2*pi, 1000);
    end

    % Initialize arrays
    n_points = length(theta_range);
    Ha_array = zeros(1, n_points);
    Hb_array = zeros(1, n_points);
    Hc_array = zeros(1, n_points);
    sector_array = zeros(1, n_points);

    % Calculate Hall states for each angle
    for i = 1:n_points
        [Ha_array(i), Hb_array(i), Hc_array(i), sector_array(i)] = ...
            hall_sensor_model(theta_range(i), 0);
    end

    % Create figure
    figure('Name', 'Hall Sensor Signals', 'NumberTitle', 'off');

    % Plot Hall sensor outputs
    subplot(2,1,1);
    hold on;
    plot(rad2deg(theta_range), Ha_array + 0.2, 'r', 'LineWidth', 2);
    plot(rad2deg(theta_range), Hb_array, 'g', 'LineWidth', 2);
    plot(rad2deg(theta_range), Hc_array - 0.2, 'b', 'LineWidth', 2);
    hold off;
    grid on;
    xlabel('Electrical Angle [degrees]');
    ylabel('Hall Sensor Output');
    title('Hall Sensor Signals vs Rotor Position');
    legend('Hall A', 'Hall B', 'Hall C', 'Location', 'best');
    ylim([-0.5, 1.5]);

    % Plot sector number
    subplot(2,1,2);
    plot(rad2deg(theta_range), sector_array, 'k', 'LineWidth', 2);
    grid on;
    xlabel('Electrical Angle [degrees]');
    ylabel('Sector Number');
    title('Hall Sensor Sector vs Rotor Position');
    ylim([0, 7]);
    yticks(1:6);

end
