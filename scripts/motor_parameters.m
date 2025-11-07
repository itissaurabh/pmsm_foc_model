%% PMSM Motor Parameters
% This script defines all parameters for the PMSM motor and FOC controller
% Author: Generated for PMSM FOC with Hall Sensors Project
% Date: 2025-11-07

clear all;
clc;

%% Motor Electrical Parameters
% These parameters define the electrical characteristics of the PMSM

motor.Rs = 0.285;              % Stator resistance [Ohm]
motor.Ld = 0.00085;            % d-axis inductance [H]
motor.Lq = 0.00085;            % q-axis inductance [H] (same as Ld for surface PMSM)
motor.Ls = motor.Ld;           % Average stator inductance [H]
motor.lambda_m = 0.0118;       % Permanent magnet flux linkage [Wb]
motor.Ke = 0.0118;             % Back-EMF constant [V/(rad/s)]
motor.Kt = 1.5 * motor.lambda_m; % Torque constant [Nm/A]

%% Motor Mechanical Parameters
% These parameters define the mechanical characteristics

motor.P = 4;                   % Number of pole pairs
motor.J = 0.00001;             % Moment of inertia [kg.m^2]
motor.B = 0.00001;             % Friction coefficient [N.m.s/rad]

%% Motor Ratings
% Rated values for the motor

motor.V_rated = 24;            % Rated voltage [V]
motor.I_rated = 2.5;           % Rated current [A]
motor.I_max = 5.0;             % Maximum current [A]
motor.speed_rated = 3000;      % Rated speed [RPM]
motor.omega_rated = motor.speed_rated * 2*pi/60; % Rated angular velocity [rad/s]
motor.torque_rated = motor.Kt * motor.I_rated;   % Rated torque [Nm]

%% Inverter Parameters
% DC-link and switching parameters

inverter.Vdc = 28;             % DC-link voltage [V]
inverter.Vdc_min = 20;         % Minimum DC-link voltage [V]
inverter.Vdc_max = 32;         % Maximum DC-link voltage [V]
inverter.fsw = 10000;          % PWM switching frequency [Hz]
inverter.Tsw = 1/inverter.fsw; % PWM period [s]
inverter.dead_time = 1e-6;     % Dead-time [s]

%% Hall Sensor Parameters
% Configuration for Hall effect sensors

hall.num_sensors = 3;          % Number of Hall sensors (always 3)
hall.offset_angle = 0;         % Hall sensor offset angle [rad]
hall.resolution = pi/3;        % Angular resolution (60 degrees) [rad]
hall.noise_enabled = false;    % Enable Hall sensor noise simulation
hall.noise_amplitude = 0.01;   % Noise amplitude [rad]

% Hall sensor placement (electrical angles)
hall.angle_A = 0;              % Hall A position [rad]
hall.angle_B = 2*pi/3;         % Hall B position [rad]
hall.angle_C = 4*pi/3;         % Hall C position [rad]

%% Current Controller Parameters (Inner Loop)
% PI controller gains for d-axis and q-axis current control

% Desired current loop bandwidth
current_ctrl.bandwidth = 1000; % [Hz]
current_ctrl.omega_cc = 2*pi * current_ctrl.bandwidth; % [rad/s]

% Calculate PI gains based on motor parameters
current_ctrl.tau_e = motor.Ls / motor.Rs; % Electrical time constant

% Current controller gains (Tuned for good response)
current_ctrl.Kp_d = motor.Ls * current_ctrl.omega_cc; % P gain for d-axis
current_ctrl.Ki_d = motor.Rs * current_ctrl.omega_cc; % I gain for d-axis
current_ctrl.Kp_q = motor.Ls * current_ctrl.omega_cc; % P gain for q-axis
current_ctrl.Ki_q = motor.Rs * current_ctrl.omega_cc; % I gain for q-axis

% Anti-windup limits
current_ctrl.integrator_max = inverter.Vdc / 2;
current_ctrl.integrator_min = -inverter.Vdc / 2;

% Output limits
current_ctrl.output_max = inverter.Vdc / sqrt(3);
current_ctrl.output_min = -inverter.Vdc / sqrt(3);

% Current reference limits
current_ctrl.id_ref = 0;       % d-axis current reference (id=0 control)
current_ctrl.iq_max = motor.I_max;
current_ctrl.iq_min = -motor.I_max;

%% Speed Controller Parameters (Outer Loop)
% PI controller gains for speed control

% Desired speed loop bandwidth (typically 10x slower than current loop)
speed_ctrl.bandwidth = 100;    % [Hz]
speed_ctrl.omega_sc = 2*pi * speed_ctrl.bandwidth; % [rad/s]

% Mechanical time constant
speed_ctrl.tau_m = motor.J * motor.Rs / (motor.Kt^2);

% Speed controller gains (Tuned for good response)
speed_ctrl.Kp = 0.05;          % Proportional gain
speed_ctrl.Ki = 2.0;           % Integral gain

% Anti-windup limits for speed controller
speed_ctrl.integrator_max = motor.I_max;
speed_ctrl.integrator_min = -motor.I_max;

% Output limits (these become iq reference limits)
speed_ctrl.output_max = motor.I_max;
speed_ctrl.output_min = -motor.I_max;

%% Simulation Parameters
% Settings for Simulink simulation

sim.Ts = 1e-5;                 % Simulation time step [s] (100 kHz)
sim.Ts_control = inverter.Tsw; % Control loop sample time [s]
sim.Ts_speed = 10 * inverter.Tsw; % Speed loop sample time [s]
sim.stop_time = 2.0;           % Simulation stop time [s]

% Initial conditions
sim.initial_speed = 0;         % Initial rotor speed [rad/s]
sim.initial_position = 0;      % Initial rotor position [rad]
sim.initial_id = 0;            % Initial d-axis current [A]
sim.initial_iq = 0;            % Initial q-axis current [A]

%% Load Parameters
% External load torque configuration

load.type = 'step';            % Load type: 'constant', 'step', 'ramp', 'sine'
load.torque_constant = 0;      % Constant load torque [Nm]
load.torque_step_time = 1.0;   % Time of load step [s]
load.torque_step_value = 0.01; % Load torque step value [Nm]
load.torque_max = 0.05;        % Maximum load torque [Nm]

%% Reference Speed Profile
% Speed reference configuration

ref.type = 'step';             % Reference type: 'constant', 'step', 'ramp', 'sine'
ref.speed_initial = 0;         % Initial speed reference [RPM]
ref.speed_final = 1000;        % Final speed reference [RPM]
ref.speed_step_time = 0.2;     % Time of speed step [s]
ref.ramp_rate = 500;           % Ramp rate [RPM/s]
ref.sine_amplitude = 500;      % Sine amplitude [RPM]
ref.sine_frequency = 1;        % Sine frequency [Hz]

% Convert reference speeds to rad/s
ref.omega_initial = ref.speed_initial * 2*pi/60;
ref.omega_final = ref.speed_final * 2*pi/60;

%% Display Parameters
% Print key parameters to console

fprintf('\n========================================\n');
fprintf('PMSM Motor Parameters Summary\n');
fprintf('========================================\n');
fprintf('Motor Resistance (Rs):     %.3f Ohm\n', motor.Rs);
fprintf('Motor Inductance (Ls):     %.5f H\n', motor.Ls);
fprintf('Flux Linkage (lambda_m):   %.5f Wb\n', motor.lambda_m);
fprintf('Torque Constant (Kt):      %.5f Nm/A\n', motor.Kt);
fprintf('Pole Pairs (P):            %d\n', motor.P);
fprintf('Rated Speed:               %d RPM\n', motor.speed_rated);
fprintf('Rated Current:             %.2f A\n', motor.I_rated);
fprintf('Rated Torque:              %.4f Nm\n', motor.torque_rated);
fprintf('\n');
fprintf('DC-link Voltage:           %.1f V\n', inverter.Vdc);
fprintf('PWM Frequency:             %.1f kHz\n', inverter.fsw/1000);
fprintf('\n');
fprintf('Current Loop Bandwidth:    %.1f Hz\n', current_ctrl.bandwidth);
fprintf('Current Kp (d-axis):       %.5f\n', current_ctrl.Kp_d);
fprintf('Current Ki (d-axis):       %.5f\n', current_ctrl.Ki_d);
fprintf('\n');
fprintf('Speed Loop Bandwidth:      %.1f Hz\n', speed_ctrl.bandwidth);
fprintf('Speed Kp:                  %.5f\n', speed_ctrl.Kp);
fprintf('Speed Ki:                  %.5f\n', speed_ctrl.Ki);
fprintf('\n');
fprintf('Simulation Time Step:      %.1f us\n', sim.Ts*1e6);
fprintf('Control Sample Time:       %.1f us\n', sim.Ts_control*1e6);
fprintf('========================================\n\n');

%% Save workspace for Simulink
% Save all parameters to workspace so Simulink can access them
save('motor_workspace.mat');

fprintf('Parameters loaded successfully!\n');
fprintf('Ready to run Simulink simulation.\n\n');
