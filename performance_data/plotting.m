% MATLAB Script for Thesis Plotting
% Author: Your Name/Gemini
% Date: July 7, 2025 (Updated)

clear; close all; clc;

%% Configuration
% Define the CSV file names
pitchFiles = {'pitch_LQI_FF.csv', 'pitch_P_NOFF.csv'};
rollFiles = {'roll_LQR.csv', 'roll_P.csv'};

% Define column indices (1-based)
% These have been updated based on your latest information.
time_idx = 1;          % Column for Time (common for all files)

% Pitch-specific indices
pitch_effort_idx = 6;  % Column for Pitch Joint Effort
pitch_position_idx = 7;% Column for Pitch Joint Position
pitch_velocity_idx = 8;% Column for Pitch Joint Velocity

% Roll-specific indices
roll_effort_idx = 9;   % Column for Roll Joint Effort
roll_position_idx = 10;% Column for Roll Joint Position
roll_velocity_idx = 11;% Column for Roll Joint Velocity


% Plotting parameters for thesis quality
lineWidth = 1.5;
fontSizeTitle = 14;
fontSizeLabels = 12;
fontSizeLegend = 10;
gridAlpha = 0.5;
gridLineStyle = ':';

% Output directory for saved figures
outputDir = 'Thesis_Plots';
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

% Line styles and colors for combined error plots (cycling through red/blue)
errorLineStyles = {'r-', 'b-', 'r--', 'b--'}; % Red solid, Blue solid, Red dashed, Blue dashed

%% Process Pitch Data
disp('Processing Pitch Data...');

% Initialize figure for combined Pitch Error plot BEFORE the loop
figure('Name', 'Combined Pitch Joint Error');
set(gcf, 'WindowState', 'maximized'); % Maximize figure for saving PNG
hold on; % Hold on to plot multiple lines on the same figure
pitchErrorLegends = {}; % To store legend entries for combined error plot

for i = 1:length(pitchFiles)
    fileName = pitchFiles{i};
    disp(['Loading file: ', fileName]);

    % Load data from CSV
    try
        data = readmatrix(fileName);
    catch
        warning(['Could not read file: ', fileName, '. Skipping.']);
        continue;
    end

    % Extract data based on pitch-specific indices
    time = data(:, time_idx);
    % Normalize time to start from 0
    time = time - time(1);
    joint_position = data(:, pitch_position_idx);
    joint_effort = data(:, pitch_effort_idx);
    joint_velocity = data(:, pitch_velocity_idx);

    % Calculate Joint Error
    joint_error = joint_effort - joint_position;

    % Extract base name for plot titles and file saving
    [~, baseName, ~] = fileparts(fileName);

    % Determine case name for legend
    if contains(baseName, 'LQI_FF', 'IgnoreCase', true)
        caseName = 'Pitch with LQI and Feedforward control';
    elseif contains(baseName, 'P_NOFF', 'IgnoreCase', true) || contains(baseName, 'P', 'IgnoreCase', true)
        caseName = 'Pitch with Proportional control';
    else
        caseName = strrep(baseName, '_', ' '); % Fallback
    end


    %% Plot 1: JOINT Velocity vs Time (Pitch) - Individual Plot
    figure('Name', [baseName, ' - Joint Velocity']);
    set(gcf, 'WindowState', 'maximized'); % Maximize figure for saving PNG
    plot(time, joint_velocity, 'b-', 'LineWidth', lineWidth); % Blue line
    grid on;
    set(gca, 'GridAlpha', gridAlpha, 'GridLineStyle', gridLineStyle);
    title(['Pitch Joint Velocity (PWM Value) vs Time (', caseName, ')'], 'FontSize', fontSizeTitle);
    xlabel('Time (s)', 'FontSize', fontSizeLabels);
    ylabel('PWM Value', 'FontSize', fontSizeLabels); % Updated label
    set(gca, 'FontSize', fontSizeLabels); % Set tick label font size
    saveas(gcf, fullfile(outputDir, [baseName, '_Velocity.fig']));
    saveas(gcf, fullfile(outputDir, [baseName, '_Velocity.png']));
    disp(['Generated plot: ', baseName, ' - Joint Velocity']);

    %% Plot 2: JOINT Effort and JOINT Position vs Time (Pitch) - Individual Plot
    figure('Name', [baseName, ' - Joint Effort & Position']);
    set(gcf, 'WindowState', 'maximized'); % Maximize figure for saving PNG
    hold on;
    plot(time, joint_effort, 'r-', 'LineWidth', lineWidth);  % Red solid line for Effort
    plot(time, joint_position, 'b--', 'LineWidth', lineWidth); % Blue dashed line for Position
    hold off;
    grid on;
    set(gca, 'GridAlpha', gridAlpha, 'GridLineStyle', gridLineStyle);
    title(['Pitch Commanded and Actual Position vs Time (', caseName, ')'], 'FontSize', fontSizeTitle);
    xlabel('Time (s)', 'FontSize', fontSizeLabels);
    ylabel('Position (degrees)', 'FontSize', fontSizeLabels); % Updated label
    legend('Commanded Position', 'Actual Position', 'Location', 'best', 'FontSize', fontSizeLegend); % Updated legend
    set(gca, 'FontSize', fontSizeLabels);
    saveas(gcf, fullfile(outputDir, [baseName, '_Effort_Position.fig']));
    saveas(gcf, fullfile(outputDir, [baseName, '_Effort_Position.png']));
    disp(['Generated plot: ', baseName, ' - Joint Effort & Position']);

    %% Plot 3: JOINT Error vs Time (Pitch) - Combined Plot
    % Plot on the combined error figure initialized before the loop
    figure(findobj('Name', 'Combined Pitch Joint Error')); % Select the combined error figure
    plot(time, joint_error, errorLineStyles{mod(i-1, length(errorLineStyles)) + 1}, 'LineWidth', lineWidth);
    pitchErrorLegends{end+1} = [caseName ' Error']; % Updated legend entry
    disp(['Added data to combined Pitch Error plot: ', baseName]);
    disp(' '); % Add a newline for better readability between files
end

% Finalize and save the combined Pitch Error plot AFTER the loop
figure(findobj('Name', 'Combined Pitch Joint Error'));
hold off;
grid on;
set(gca, 'GridAlpha', gridAlpha, 'GridLineStyle', gridLineStyle);
title('Combined Pitch Joint Error (Commanded - Actual Position) vs Time', 'FontSize', fontSizeTitle);
xlabel('Time (s)', 'FontSize', fontSizeLabels);
ylabel('Joint Error (degrees)', 'FontSize', fontSizeLabels); % Updated label
legend(pitchErrorLegends, 'Location', 'best', 'FontSize', fontSizeLegend);
set(gca, 'FontSize', fontSizeLabels);
saveas(gcf, fullfile(outputDir, 'Combined_Pitch_Error.fig'));
saveas(gcf, fullfile(outputDir, 'Combined_Pitch_Error.png'));
disp('Generated combined plot: Combined Pitch Joint Error');


%% Process Roll Data
disp('Processing Roll Data...');

% Initialize figure for combined Roll Error plot BEFORE the loop
figure('Name', 'Combined Roll Joint Error');
set(gcf, 'WindowState', 'maximized'); % Maximize figure for saving PNG
hold on; % Hold on to plot multiple lines on the same figure
rollErrorLegends = {}; % To store legend entries for combined error plot

for i = 1:length(rollFiles)
    fileName = rollFiles{i};
    disp(['Loading file: ', fileName]);

    % Load data from CSV
    try
        data = readmatrix(fileName);
    catch
        warning(['Could not read file: ', fileName, '. Skipping.']);
        continue;
    end

    % Extract data based on roll-specific indices
    time = data(:, time_idx);
    % Normalize time to start from 0
    time = time - time(1);
    joint_position = data(:, roll_position_idx);
    joint_effort = data(:, roll_effort_idx);
    joint_velocity = data(:, roll_velocity_idx);

    % Calculate Joint Error
    joint_error = joint_effort - joint_position;

    % Extract base name for plot titles and file saving
    [~, baseName, ~] = fileparts(fileName);

    % Determine case name for legend
    if contains(baseName, 'LQR', 'IgnoreCase', true)
        caseName = 'Roll with LQR control';
    elseif contains(baseName, 'P', 'IgnoreCase', true)
        caseName = 'Roll with Proportional control';
    else
        caseName = strrep(baseName, '_', ' '); % Fallback
    end

    %% Plot 1: JOINT Velocity vs Time (Roll) - Individual Plot
    figure('Name', [baseName, ' - Joint Velocity']);
    set(gcf, 'WindowState', 'maximized'); % Maximize figure for saving PNG
    plot(time, joint_velocity, 'b-', 'LineWidth', lineWidth); % Blue line
    grid on;
    set(gca, 'GridAlpha', gridAlpha, 'GridLineStyle', gridLineStyle);
    title(['Roll Joint Velocity (PWM Value) vs Time (', caseName, ')'], 'FontSize', fontSizeTitle);
    xlabel('Time (s)', 'FontSize', fontSizeLabels);
    ylabel('PWM Value', 'FontSize', fontSizeLabels); % Updated label
    set(gca, 'FontSize', fontSizeLabels);
    saveas(gcf, fullfile(outputDir, [baseName, '_Velocity.fig']));
    saveas(gcf, fullfile(outputDir, [baseName, '_Velocity.png']));
    disp(['Generated plot: ', baseName, ' - Joint Velocity']);

    %% Plot 2: JOINT Effort and JOINT Position vs Time (Roll) - Individual Plot
    figure('Name', [baseName, ' - Joint Effort & Position']);
    set(gcf, 'WindowState', 'maximized'); % Maximize figure for saving PNG
    hold on;
    plot(time, joint_effort, 'r-', 'LineWidth', lineWidth);  % Red solid line for Effort
    plot(time, joint_position, 'b--', 'LineWidth', lineWidth); % Blue dashed line for Position
    hold off;
    grid on;
    set(gca, 'GridAlpha', gridAlpha, 'GridLineStyle', gridLineStyle);
    title(['Roll Commanded and Actual Position vs Time (', caseName, ')'], 'FontSize', fontSizeTitle);
    xlabel('Time (s)', 'FontSize', fontSizeLabels);
    ylabel('Position (degrees)', 'FontSize', fontSizeLabels); % Updated label
    legend('Commanded Position', 'Actual Position', 'Location', 'best', 'FontSize', fontSizeLegend); % Updated legend
    set(gca, 'FontSize', fontSizeLabels);
    saveas(gcf, fullfile(outputDir, [baseName, '_Effort_Position.fig']));
    saveas(gcf, fullfile(outputDir, [baseName, '_Effort_Position.png']));
    disp(['Generated plot: ', baseName, ' - Joint Effort & Position']);

    %% Plot 3: JOINT Error vs Time (Roll) - Combined Plot
    % Plot on the combined error figure initialized before the loop
    figure(findobj('Name', 'Combined Roll Joint Error')); % Select the combined error figure
    plot(time, joint_error, errorLineStyles{mod(i-1, length(errorLineStyles)) + 1}, 'LineWidth', lineWidth);
    rollErrorLegends{end+1} = [caseName ' Error']; % Updated legend entry
    disp(['Added data to combined Roll Error plot: ', baseName]);
    disp(' '); % Add a newline for better readability between files
end

% Finalize and save the combined Roll Error plot AFTER the loop
figure(findobj('Name', 'Combined Roll Joint Error'));
hold off;
grid on;
set(gca, 'GridAlpha', gridAlpha, 'GridLineStyle', gridLineStyle);
title('Combined Roll Joint Error (Commanded - Actual Position) vs Time', 'FontSize', fontSizeTitle);
xlabel('Time (s)', 'FontSize', fontSizeLabels);
ylabel('Joint Error (degrees)', 'FontSize', fontSizeLabels); % Updated label
legend(rollErrorLegends, 'Location', 'best', 'FontSize', fontSizeLegend);
set(gca, 'FontSize', fontSizeLabels);
saveas(gcf, fullfile(outputDir, 'Combined_Roll_Error.fig'));
saveas(gcf, fullfile(outputDir, 'Combined_Roll_Error.png'));
disp('Generated combined plot: Combined Roll Joint Error');

disp('Script execution complete. Plots saved in the "Thesis_Plots" directory.');