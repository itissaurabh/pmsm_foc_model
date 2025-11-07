%% COORDINATE TRANSFORMATIONS FOR FOC
% This file contains all coordinate transformation functions used in
% Field Oriented Control (FOC) of PMSM motors
%
% Transformations included:
% - Clarke Transform (abc → αβ)
% - Inverse Clarke Transform (αβ → abc)
% - Park Transform (αβ → dq)
% - Inverse Park Transform (dq → αβ)
%
% Author: PMSM FOC Project
% Date: 2025-11-07

%% CLARKE TRANSFORMATION (abc → αβ)

function [alpha, beta] = clarke_transform(a, b, c)
% CLARKE_TRANSFORM - Transforms three-phase quantities to two-phase stationary frame
%
% The Clarke transformation (also known as αβ0 transformation) converts
% three-phase balanced quantities (a, b, c) into a two-phase orthogonal
% stationary reference frame (α, β).
%
% This transformation is power-invariant (amplitude is scaled by 2/3).
%
% MATHEMATICAL FORMULA:
%   α = (2/3) * (a - b/2 - c/2)
%   β = (2/3) * (√3/2) * (b - c)
%
% Or in matrix form:
%   [α]   (2/3) * [  1    -1/2     -1/2   ] [a]
%   [β] =        [  0   √3/2    -√3/2   ] [b]
%                                           [c]
%
% INPUTS:
%   a, b, c - Three-phase quantities (currents or voltages)
%
% OUTPUTS:
%   alpha - α-axis component (aligned with phase a)
%   beta  - β-axis component (90° ahead of α)
%
% EXAMPLE:
%   % For balanced three-phase currents
%   ia = 1; ib = -0.5; ic = -0.5;
%   [i_alpha, i_beta] = clarke_transform(ia, ib, ic);

    % Power-invariant Clarke transformation
    alpha = (2/3) * (a - 0.5*b - 0.5*c);
    beta = (2/3) * (sqrt(3)/2) * (b - c);

end


%% INVERSE CLARKE TRANSFORMATION (αβ → abc)

function [a, b, c] = inverse_clarke_transform(alpha, beta)
% INVERSE_CLARKE_TRANSFORM - Transforms two-phase stationary frame to three-phase
%
% The inverse Clarke transformation converts quantities from the two-phase
% orthogonal stationary reference frame (α, β) back to three-phase
% quantities (a, b, c).
%
% MATHEMATICAL FORMULA:
%   a =  α
%   b = -α/2 + (√3/2)*β
%   c = -α/2 - (√3/2)*β
%
% Or in matrix form:
%   [a]   [    1         0    ] [α]
%   [b] = [ -1/2      √3/2    ] [β]
%   [c]   [ -1/2     -√3/2    ]
%
% INPUTS:
%   alpha - α-axis component
%   beta  - β-axis component
%
% OUTPUTS:
%   a, b, c - Three-phase quantities
%
% EXAMPLE:
%   [va, vb, vc] = inverse_clarke_transform(v_alpha, v_beta);

    % Inverse Clarke transformation
    a = alpha;
    b = -0.5*alpha + (sqrt(3)/2)*beta;
    c = -0.5*alpha - (sqrt(3)/2)*beta;

end


%% PARK TRANSFORMATION (αβ → dq)

function [d, q] = park_transform(alpha, beta, theta)
% PARK_TRANSFORM - Transforms stationary frame to rotating frame
%
% The Park transformation (also known as dq0 transformation) converts
% quantities from the two-phase stationary reference frame (α, β) to a
% two-phase rotating reference frame (d, q) that is synchronized with
% the rotor position.
%
% This transformation "freezes" the AC quantities into DC quantities in
% the rotating frame, enabling simpler PI controller design.
%
% MATHEMATICAL FORMULA:
%   d =  α*cos(θ) + β*sin(θ)
%   q = -α*sin(θ) + β*cos(θ)
%
% Or in matrix form:
%   [d]   [ cos(θ)   sin(θ)] [α]
%   [q] = [-sin(θ)   cos(θ)] [β]
%
% INPUTS:
%   alpha - α-axis component (stationary frame)
%   beta  - β-axis component (stationary frame)
%   theta - Electrical rotor position [rad]
%
% OUTPUTS:
%   d - d-axis component (direct axis, aligned with rotor flux)
%   q - q-axis component (quadrature axis, 90° ahead of d-axis)
%
% EXAMPLE:
%   theta_e = pi/4;  % 45 degrees electrical
%   [id, iq] = park_transform(i_alpha, i_beta, theta_e);
%
% NOTE:
%   - d-axis is aligned with the rotor flux (permanent magnets)
%   - q-axis is perpendicular to d-axis and produces torque
%   - For FOC with id=0 control, we only control the q-axis current

    % Calculate sine and cosine once for efficiency
    cos_theta = cos(theta);
    sin_theta = sin(theta);

    % Park transformation
    d = alpha * cos_theta + beta * sin_theta;
    q = -alpha * sin_theta + beta * cos_theta;

end


%% INVERSE PARK TRANSFORMATION (dq → αβ)

function [alpha, beta] = inverse_park_transform(d, q, theta)
% INVERSE_PARK_TRANSFORM - Transforms rotating frame to stationary frame
%
% The inverse Park transformation converts quantities from the two-phase
% rotating reference frame (d, q) back to the two-phase stationary
% reference frame (α, β).
%
% This is used to convert the voltage commands from the current controllers
% (which operate in the dq frame) back to the stationary frame for PWM
% generation.
%
% MATHEMATICAL FORMULA:
%   α = d*cos(θ) - q*sin(θ)
%   β = d*sin(θ) + q*cos(θ)
%
% Or in matrix form:
%   [α]   [cos(θ)  -sin(θ)] [d]
%   [β] = [sin(θ)   cos(θ)] [q]
%
% INPUTS:
%   d     - d-axis component (rotating frame)
%   q     - q-axis component (rotating frame)
%   theta - Electrical rotor position [rad]
%
% OUTPUTS:
%   alpha - α-axis component (stationary frame)
%   beta  - β-axis component (stationary frame)
%
% EXAMPLE:
%   [v_alpha, v_beta] = inverse_park_transform(vd, vq, theta_e);

    % Calculate sine and cosine once for efficiency
    cos_theta = cos(theta);
    sin_theta = sin(theta);

    % Inverse Park transformation
    alpha = d * cos_theta - q * sin_theta;
    beta = d * sin_theta + q * cos_theta;

end


%% COMPLETE FORWARD TRANSFORMATION (abc → dq)

function [d, q] = abc_to_dq(a, b, c, theta)
% ABC_TO_DQ - Complete transformation from three-phase to dq frame
%
% This is a convenience function that combines Clarke and Park
% transformations in one step: abc → αβ → dq
%
% INPUTS:
%   a, b, c - Three-phase quantities
%   theta   - Electrical rotor position [rad]
%
% OUTPUTS:
%   d, q - dq-axis components
%
% EXAMPLE:
%   [id, iq] = abc_to_dq(ia, ib, ic, theta_e);

    % Step 1: Clarke transformation (abc → αβ)
    [alpha, beta] = clarke_transform(a, b, c);

    % Step 2: Park transformation (αβ → dq)
    [d, q] = park_transform(alpha, beta, theta);

end


%% COMPLETE INVERSE TRANSFORMATION (dq → abc)

function [a, b, c] = dq_to_abc(d, q, theta)
% DQ_TO_ABC - Complete transformation from dq frame to three-phase
%
% This is a convenience function that combines inverse Park and inverse
% Clarke transformations in one step: dq → αβ → abc
%
% INPUTS:
%   d, q  - dq-axis components
%   theta - Electrical rotor position [rad]
%
% OUTPUTS:
%   a, b, c - Three-phase quantities
%
% EXAMPLE:
%   [va, vb, vc] = dq_to_abc(vd, vq, theta_e);

    % Step 1: Inverse Park transformation (dq → αβ)
    [alpha, beta] = inverse_park_transform(d, q, theta);

    % Step 2: Inverse Clarke transformation (αβ → abc)
    [a, b, c] = inverse_clarke_transform(alpha, beta);

end


%% VISUALIZATION FUNCTION

function plot_transformations()
% PLOT_TRANSFORMATIONS - Visualize the coordinate transformations
%
% This function creates plots to help understand how the Clarke and Park
% transformations convert three-phase quantities to different reference frames.
%
% EXAMPLE:
%   plot_transformations();

    % Time vector for one electrical cycle
    t = linspace(0, 2*pi, 360);

    % Create balanced three-phase currents (assuming 1A peak)
    ia = cos(t);
    ib = cos(t - 2*pi/3);
    ic = cos(t - 4*pi/3);

    % Initialize arrays for transformed quantities
    i_alpha = zeros(size(t));
    i_beta = zeros(size(t));
    id = zeros(size(t));
    iq = zeros(size(t));

    % Perform transformations for each time point
    for i = 1:length(t)
        % Clarke transformation
        [i_alpha(i), i_beta(i)] = clarke_transform(ia(i), ib(i), ic(i));

        % Park transformation (assuming theta = t for this example)
        [id(i), iq(i)] = park_transform(i_alpha(i), i_beta(i), t(i));
    end

    % Create visualization
    figure('Name', 'Coordinate Transformations', 'Position', [100 100 1200 800]);

    % Plot 1: Three-phase currents (abc)
    subplot(2,2,1);
    plot(rad2deg(t), ia, 'r', 'LineWidth', 2); hold on;
    plot(rad2deg(t), ib, 'g', 'LineWidth', 2);
    plot(rad2deg(t), ic, 'b', 'LineWidth', 2);
    grid on;
    xlabel('Electrical Angle [degrees]');
    ylabel('Current [A]');
    title('Three-Phase Currents (abc frame)');
    legend('ia', 'ib', 'ic', 'Location', 'best');

    % Plot 2: Two-phase stationary frame (αβ)
    subplot(2,2,2);
    plot(rad2deg(t), i_alpha, 'r', 'LineWidth', 2); hold on;
    plot(rad2deg(t), i_beta, 'b', 'LineWidth', 2);
    grid on;
    xlabel('Electrical Angle [degrees]');
    ylabel('Current [A]');
    title('Stationary Frame (αβ frame) - After Clarke');
    legend('iα', 'iβ', 'Location', 'best');

    % Plot 3: Two-phase rotating frame (dq)
    subplot(2,2,3);
    plot(rad2deg(t), id, 'r', 'LineWidth', 2); hold on;
    plot(rad2deg(t), iq, 'b', 'LineWidth', 2);
    grid on;
    xlabel('Electrical Angle [degrees]');
    ylabel('Current [A]');
    title('Rotating Frame (dq frame) - After Park');
    legend('id', 'iq', 'Location', 'best');

    % Plot 4: Space vector plot (αβ plane)
    subplot(2,2,4);
    plot(i_alpha, i_beta, 'b', 'LineWidth', 2);
    grid on;
    xlabel('α-axis [A]');
    ylabel('β-axis [A]');
    title('Space Vector Plot (αβ plane)');
    axis equal;

    % Add annotations
    sgtitle('FOC Coordinate Transformations Visualization');

end


%% VALIDATION FUNCTION

function validate_transformations()
% VALIDATE_TRANSFORMATIONS - Test the transformation functions
%
% This function validates that the forward and inverse transformations
% are correct by checking round-trip conversions.
%
% EXAMPLE:
%   validate_transformations();

    fprintf('\n========================================\n');
    fprintf('Coordinate Transformation Validation\n');
    fprintf('========================================\n\n');

    % Test case 1: Clarke transformation
    fprintf('Test 1: Clarke Transformation (abc → αβ → abc)\n');
    ia = 1.0; ib = -0.5; ic = -0.5;  % Balanced three-phase
    [i_alpha, i_beta] = clarke_transform(ia, ib, ic);
    [ia_back, ib_back, ic_back] = inverse_clarke_transform(i_alpha, i_beta);

    fprintf('  Input:  ia=%.3f, ib=%.3f, ic=%.3f\n', ia, ib, ic);
    fprintf('  αβ:     α=%.3f, β=%.3f\n', i_alpha, i_beta);
    fprintf('  Output: ia=%.3f, ib=%.3f, ic=%.3f\n', ia_back, ib_back, ic_back);

    error1 = max([abs(ia-ia_back), abs(ib-ib_back), abs(ic-ic_back)]);
    fprintf('  Max Error: %.6f %s\n\n', error1, ternary(error1 < 1e-10, '✓ PASS', '✗ FAIL'));

    % Test case 2: Park transformation
    fprintf('Test 2: Park Transformation (αβ → dq → αβ)\n');
    theta = pi/4;  % 45 degrees
    alpha = 1.0; beta = 0.5;
    [id, iq] = park_transform(alpha, beta, theta);
    [alpha_back, beta_back] = inverse_park_transform(id, iq, theta);

    fprintf('  Input:  α=%.3f, β=%.3f, θ=%.3f rad\n', alpha, beta, theta);
    fprintf('  dq:     d=%.3f, q=%.3f\n', id, iq);
    fprintf('  Output: α=%.3f, β=%.3f\n', alpha_back, beta_back);

    error2 = max([abs(alpha-alpha_back), abs(beta-beta_back)]);
    fprintf('  Max Error: %.6f %s\n\n', error2, ternary(error2 < 1e-10, '✓ PASS', '✗ FAIL'));

    % Test case 3: Complete transformation
    fprintf('Test 3: Complete Transformation (abc → dq → abc)\n');
    ia = 1.0; ib = -0.5; ic = -0.5;
    theta = pi/6;  % 30 degrees
    [id, iq] = abc_to_dq(ia, ib, ic, theta);
    [ia_back, ib_back, ic_back] = dq_to_abc(id, iq, theta);

    fprintf('  Input:  ia=%.3f, ib=%.3f, ic=%.3f, θ=%.3f rad\n', ia, ib, ic, theta);
    fprintf('  dq:     d=%.3f, q=%.3f\n', id, iq);
    fprintf('  Output: ia=%.3f, ib=%.3f, ic=%.3f\n', ia_back, ib_back, ic_back);

    error3 = max([abs(ia-ia_back), abs(ib-ib_back), abs(ic-ic_back)]);
    fprintf('  Max Error: %.6f %s\n\n', error3, ternary(error3 < 1e-10, '✓ PASS', '✗ FAIL'));

    fprintf('========================================\n\n');

end

function result = ternary(condition, true_val, false_val)
    if condition
        result = true_val;
    else
        result = false_val;
    end
end
