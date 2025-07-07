% MATLAB Script for Thesis-Quality Trend Plots

% Clear workspace and close figures
clear;
clc;
close all;

%% 1. Simulate Theoretical Data for Roll Subsystem
% As roll angle diverges from 0, required absolute motor voltage increases.
% This is modeled as a smooth quadratic relationship.
roll_angle = linspace(-5, 5, 100); % Roll angle in degrees, from -5 to 5
% Simulate required absolute voltage: minimum at 0, increasing quadratically
% Use absolute value to ensure positive voltage as requested "absolute motor voltage"
base_voltage_roll = 0; % Minimum absolute voltage at 0 degrees (changed from 0.5 to 0)
sensitivity_roll = 0.05; % How quickly voltage increases with angle squared

required_voltage_roll = base_voltage_roll + sensitivity_roll * (roll_angle.^2); % Smooth quadratic, always positive

%% 2. Simulate Theoretical Data for Pitch Subsystem
% As pitch angle moves from -30 to 30 degrees, voltage increases towards a plateau of 2V.
% This is modeled using a sigmoid-like function for a smooth transition to a plateau.
pitch_angle = linspace(-30, 30, 100); % Pitch angle in degrees, from -30 to 30
% Sigmoid function to go from ~1.2V to ~2V
% Parameters for sigmoid:
min_voltage_pitch = 1.2;
max_voltage_pitch = 2.0;
midpoint = 0; % Center of the angle range
steepness = 0.1; % Controls how quickly it rises

% Sigmoid formula: min + (max - min) / (1 + exp(-k * (x - x0)))
required_voltage_pitch = min_voltage_pitch + (max_voltage_pitch - min_voltage_pitch) ./ (1 + exp(-steepness * (pitch_angle - midpoint)));

%% 3. Create Thesis-Quality Plots (Separate Figures)

% --- Figure 1: Roll Subsystem Trend ---
figure('Position', [100, 100, 700, 500], 'Color', 'w'); % Adjust figure size
plot(roll_angle, required_voltage_roll, 'b-', 'LineWidth', 2.5); % Blue line, thick

% Customize plot aesthetics
title('Theoretical Roll Subsystem: Absolute Motor Voltage vs. Angle', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Roll Angle (degrees)', 'FontSize', 12);
ylabel('Absolute Motor Voltage (V)', 'FontSize', 12);
grid on; % Add grid lines
set(gca, 'FontSize', 10, 'FontName', 'Arial', 'Box', 'on', 'XColor', [0.3 0.3 0.3], 'YColor', [0.3 0.3 0.3]); % Axis font, box, color
xlim([-5 5]); % Set x-axis limits explicitly as requested
ylim([0, max(required_voltage_roll)*1.1]); % Ensure y-axis starts from 0 and has buffer

% --- Figure 2: Pitch Subsystem Trend ---
figure('Position', [850, 100, 700, 500], 'Color', 'w'); % Adjust figure size and position
plot(pitch_angle, required_voltage_pitch, 'r-', 'LineWidth', 2.5); % Red line, thick

% Customize plot aesthetics
title('Theoretical Pitch Subsystem: Required Motor Voltage vs. Angle', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Pitch Angle (degrees)', 'FontSize', 12);
ylabel('Required Motor Voltage (V)', 'FontSize', 12);
grid on; % Add grid lines
set(gca, 'FontSize', 10, 'FontName', 'Arial', 'Box', 'on', 'XColor', [0.3 0.3 0.3], 'YColor', [0.3 0.3 0.3]); % Axis font, box, color
xlim([-30 30]); % Set x-axis limits explicitly as requested
ylim([min_voltage_pitch*0.9, max_voltage_pitch*1.1]); % Auto-adjust y-limits with buffer

%% 4. Save the Figures
% Save the figures in high-resolution formats suitable for a thesis.
% Adjust the filename and format as needed.

% Option 1: Save as PDF (vector graphics, best for text and lines)
% exportgraphics(figure(1), 'Roll_Subsystem_Theoretical_Trend.pdf', 'ContentType', 'vector');
% exportgraphics(figure(2), 'Pitch_Subsystem_Theoretical_Trend.pdf', 'ContentType', 'vector');

% Option 2: Save as high-resolution PNG (good for web or if PDF causes issues)
exportgraphics(figure(1), 'Roll_Subsystem_Theoretical_Trend.png', 'Resolution', 300);
exportgraphics(figure(2), 'Pitch_Subsystem_Theoretical_Trend.png', 'Resolution', 300);

disp('Theoretical plots generated and saved as PNG files.');
