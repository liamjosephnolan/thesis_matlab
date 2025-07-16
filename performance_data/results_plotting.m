% MATLAB Script for Thesis Plotting
% Author: Your Name/Gemini
% Date: July 9, 2025 (CRITICAL FIX: Correct colors for Combined Pitch/Roll Error in non-trajectory data)
clear; close all; clc;

%% Configuration
% Define the CSV file names
pitchFiles = {'pitch_LQI_FF.csv', 'pitch_P_NOFF.csv'};
rollFiles = {'roll_LQR.csv', 'roll_P.csv'};
% Trajectory following data
trajFollowFiles = {'p_traj_follow.csv', 'lqr_traj_follow.csv'};
% NEW: Specific file for the DESIRED 3D trajectory path
desired3DPathFile = 'traj_3.csv'; 

% Define column indices (1-based)
time_idx = 1;          % Column for Time (common for all files)
% Pitch-specific indices
pitch_effort_idx = 6;  % Column for Pitch Joint Effort (now Commanded Position)
pitch_position_idx = 7;% Column for Pitch Joint Position (now Actual Position)
pitch_velocity_idx = 8;% Column for Pitch Joint Velocity
% Roll-specific indices
roll_effort_idx = 9;   % Column for Roll Joint Effort (now Commanded Position)
roll_position_idx = 10;% Column for Roll Joint Position (now Actual Position)
roll_velocity_idx = 11;% Column for Roll Joint Velocity

% For traj_3.csv: Assuming time, x, y, z (adjust if structure is different)
traj3_time_idx = 1;
traj3_x_idx = 2;
traj3_y_idx = 3;
traj3_z_idx = 4;


% Plotting parameters for thesis quality
lineWidth = 1.5;
fontSizeTitle = 14;
fontSizeLabels = 12;
fontSizeLegend = 10;
gridAlpha = 0.5;
gridLineStyle = ':';

% Defined Line Colors
desired_color = 'k'; % Black for Desired/Commanded
lqr_color = 'r';     % Red for LQR Actual
p_color = 'b';       % Blue for P-Control / LQI+FF Actual

% Line styles for actual positions and errors
actual_line_style = '--';
commanded_line_style = '-';

% Line styles and colors for combined error plots in NON-TRAJECTORY data (for differentiation)
% This will ensure different colors for LQI+FF vs P-Control, and LQR vs P-Control in the basic cases.
combined_error_line_styles = {'b-', 'r-'}; % Blue for first, Red for second in combined error plots.


%% Process Pitch Data (Individual Control Performance)
disp('--- Processing Pitch Data (Individual Control Performance) ---');
% Initialize figure for combined Pitch Error plot BEFORE the loop
figure('Name', 'Combined Pitch Joint Error');
set(gcf, 'WindowState', 'maximized');
hold on;
pitchErrorLegends = {}; 

for i = 1:length(pitchFiles)
    fileName = pitchFiles{i};
    disp(['Loading file: ', fileName]);
    try
        data = readmatrix(fileName);
    catch
        warning(['Could not read file: ', fileName, '. Skipping.']);
        continue;
    end

    time = data(:, time_idx);
    time = time - time(1); % Normalize time to start from 0
    joint_position = data(:, pitch_position_idx);
    joint_effort = data(:, pitch_effort_idx); % This is now Commanded Position
    joint_error = joint_effort - joint_position; % Commanded - Actual

    [~, baseName, ~] = fileparts(fileName);
    % Determine case name for legend and plot color for individual plots
    if contains(baseName, 'LQI_FF', 'IgnoreCase', true)
        caseName = 'Pitch with LQI and Feedforward control';
        actual_plot_color_individual = p_color; % Still blue for individual plot
    elseif contains(baseName, 'P_NOFF', 'IgnoreCase', true) || contains(baseName, 'P', 'IgnoreCase', true)
        caseName = 'Pitch with Proportional control';
        actual_plot_color_individual = p_color; % Still blue for individual plot
    else
        caseName = strrep(baseName, '_', ' '); 
        actual_plot_color_individual = 'g'; % Default if unexpected
    end

    %% Plot 1: JOINT Velocity vs Time (Pitch) - Individual Plot
    figure('Name', [baseName, ' - Joint Velocity']);
    set(gcf, 'WindowState', 'maximized');
    plot(time, data(:, pitch_velocity_idx), actual_plot_color_individual, 'LineWidth', lineWidth);
    grid on;
    set(gca, 'GridAlpha', gridAlpha, 'GridLineStyle', gridLineStyle);
    title(['Pitch Joint Motor Speed vs Time (', caseName, ')'], 'FontSize', fontSizeTitle);
    xlabel('Time (s)', 'FontSize', fontSizeLabels);
    ylabel('PWM Value', 'FontSize', fontSizeLabels);
    set(gca, 'FontSize', fontSizeLabels);
    disp(['Displayed plot: ', baseName, ' - Joint Velocity']);

    %% Plot 2: JOINT Commanded and Actual Position vs Time (Pitch) - Individual Plot
    figure('Name', [baseName, ' - Joint Commanded & Actual Position']);
    set(gcf, 'WindowState', 'maximized');
    hold on;
    plot(time, joint_effort, desired_color, 'LineStyle', commanded_line_style, 'LineWidth', lineWidth, 'DisplayName', 'Commanded Position');
    plot(time, joint_position, actual_plot_color_individual, 'LineStyle', actual_line_style, 'LineWidth', lineWidth, 'DisplayName', 'Actual Position');
    hold off;
    grid on;
    set(gca, 'GridAlpha', gridAlpha, 'GridLineStyle', gridLineStyle);
    title(['Pitch Commanded vs Actual Position vs Time (', caseName, ')'], 'FontSize', fontSizeTitle);
    xlabel('Time (s)', 'FontSize', fontSizeLabels);
    ylabel('Position (degrees)', 'FontSize', fontSizeLabels);
    legend('show', 'Location', 'best', 'FontSize', fontSizeLegend);
    set(gca, 'FontSize', fontSizeLabels);
    disp(['Displayed plot: ', baseName, ' - Joint Commanded & Actual Position']);

    %% Plot 3: JOINT Error vs Time (Pitch) - Combined Plot
    figure(findobj('Name', 'Combined Pitch Joint Error'));
    % Use combined_error_line_styles for differentiation in this specific plot
    plot(time, joint_error, combined_error_line_styles{mod(i-1, length(combined_error_line_styles)) + 1}, 'LineWidth', lineWidth);
    pitchErrorLegends{end+1} = [caseName ' Error'];
    disp(['Added data to combined Pitch Error plot: ', baseName]);
    disp(' ');
end

% Finalize the combined Pitch Error plot AFTER the loop
figure(findobj('Name', 'Combined Pitch Joint Error'));
hold off;
grid on;
set(gca, 'GridAlpha', gridAlpha, 'GridLineStyle', gridLineStyle);
title('Pitch Joint Error (Commanded - Actual) vs Time', 'FontSize', fontSizeTitle);
xlabel('Time (s)', 'FontSize', fontSizeLabels);
ylabel('Joint Error (degrees)', 'FontSize', fontSizeLabels);
legend(pitchErrorLegends, 'Location', 'best', 'FontSize', fontSizeLegend);
set(gca, 'FontSize', fontSizeLabels);
disp('Displayed combined plot: Combined Pitch Joint Error');


disp('--- Processing Roll Data (Individual Control Performance) ---');
% Initialize figure for combined Roll Error plot BEFORE the loop
figure('Name', 'Combined Roll Joint Error');
set(gcf, 'WindowState', 'maximized');
hold on;
rollErrorLegends = {}; 

for i = 1:length(rollFiles)
    fileName = rollFiles{i};
    disp(['Loading file: ', fileName]);
    try
        data = readmatrix(fileName);
    catch
        warning(['Could not read file: ', fileName, '. Skipping.']);
        continue;
    end

    time = data(:, time_idx);
    time = time - time(1); % Normalize time to start from 0
    joint_position = data(:, roll_position_idx);
    joint_effort = data(:, roll_effort_idx); % This is now Commanded Position
    joint_error = joint_effort - joint_position; % Commanded - Actual

    [~, baseName, ~] = fileparts(fileName);
    % Determine case name for legend and plot color for individual plots
    if contains(baseName, 'LQR', 'IgnoreCase', true)
        caseName = 'Roll with LQR control';
        actual_plot_color_individual = lqr_color; % Red for individual LQR plot
    elseif contains(baseName, 'P', 'IgnoreCase', true)
        caseName = 'Roll with Proportional control';
        actual_plot_color_individual = p_color; % Blue for individual P-Control plot
    else
        caseName = strrep(baseName, '_', ' '); 
        actual_plot_color_individual = 'g'; % Default if unexpected
    end

    %% Plot 1: JOINT Velocity vs Time (Roll) - Individual Plot
    figure('Name', [baseName, ' - Joint Velocity']);
    set(gcf, 'WindowState', 'maximized');
    plot(time, data(:, roll_velocity_idx), actual_plot_color_individual, 'LineWidth', lineWidth);
    grid on;
    set(gca, 'GridAlpha', gridAlpha, 'GridLineStyle', gridLineStyle);
    title(['Roll Joint Motor Speed vs Time (', caseName, ')'], 'FontSize', fontSizeTitle);
    xlabel('Time (s)', 'FontSize', fontSizeLabels);
    ylabel('PWM Value', 'FontSize', fontSizeLabels);
    set(gca, 'FontSize', fontSizeLabels);
    disp(['Displayed plot: ', baseName, ' - Joint Velocity']);

    %% Plot 2: JOINT Commanded and Actual Position vs Time (Roll) - Individual Plot
    figure('Name', [baseName, ' - Joint Commanded & Actual Position']);
    set(gcf, 'WindowState', 'maximized');
    hold on;
    plot(time, joint_effort, desired_color, 'LineStyle', commanded_line_style, 'LineWidth', lineWidth, 'DisplayName', 'Commanded Position');
    plot(time, joint_position, actual_plot_color_individual, 'LineStyle', actual_line_style, 'LineWidth', lineWidth, 'DisplayName', 'Actual Position');
    hold off;
    grid on;
    set(gca, 'GridAlpha', gridAlpha, 'GridLineStyle', gridLineStyle);
    title(['Roll Commanded vs Actual Position vs Time (', caseName, ')'], 'FontSize', fontSizeTitle);
    xlabel('Time (s)', 'FontSize', fontSizeLabels);
    ylabel('Position (degrees)', 'FontSize', fontSizeLabels);
    legend('show', 'Location', 'best', 'FontSize', fontSizeLegend);
    set(gca, 'FontSize', fontSizeLabels);
    disp(['Displayed plot: ', baseName, ' - Joint Commanded & Actual Position']);

    %% Plot 3: JOINT Error vs Time (Roll) - Combined Plot
    figure(findobj('Name', 'Combined Roll Joint Error'));
    % Use combined_error_line_styles for differentiation in this specific plot
    plot(time, joint_error, combined_error_line_styles{mod(i-1, length(combined_error_line_styles)) + 1}, 'LineWidth', lineWidth);
    rollErrorLegends{end+1} = [caseName ' Error'];
    disp(['Added data to combined Roll Error plot: ', baseName]);
    disp(' ');
end

% Finalize the combined Roll Error plot AFTER the loop
figure(findobj('Name', 'Combined Roll Joint Error'));
hold off;
grid on;
set(gca, 'GridAlpha', gridAlpha, 'GridLineStyle', gridLineStyle);
title('Roll Joint Error (Commanded - Actual) vs Time', 'FontSize', fontSizeTitle);
xlabel('Time (s)', 'FontSize', fontSizeLabels);
ylabel('Joint Error (degrees)', 'FontSize', fontSizeLabels);
legend(rollErrorLegends, 'Location', 'best', 'FontSize', fontSizeLegend);
set(gca, 'FontSize', fontSizeLabels);
disp('Displayed combined plot: Combined Roll Joint Error');

disp('--- Processing Trajectory Data (Main Focus for Comparisons) ---');

% --- NEW: Load and plot the single DESIRED 3D path ---
disp(['Loading desired 3D path from: ', desired3DPathFile]);
try
    desiredPathData = readmatrix(desired3DPathFile);
    % Assuming traj_3.csv contains time, x, y, z in columns 1,2,3,4 respectively
    desired_x = desiredPathData(:, traj3_x_idx);
    desired_y = desiredPathData(:, traj3_y_idx);
    desired_z = desiredPathData(:, traj3_z_idx);

    figure('Name', 'Desired 3D Trajectory Path');
    set(gcf, 'WindowState', 'maximized');
    plot3(desired_x, desired_y, desired_z, desired_color, 'LineWidth', lineWidth, 'DisplayName', 'Desired Trajectory Path');
    grid on;
    set(gca, 'GridAlpha', gridAlpha, 'GridLineStyle', gridLineStyle);
    title('Desired 3D Trajectory Path (from traj_3.csv)', 'FontSize', fontSizeTitle);
    xlabel('X Position', 'FontSize', fontSizeLabels);
    ylabel('Y Position', 'FontSize', fontSizeLabels);
    zlabel('Z Position', 'FontSize', fontSizeLabels);
    legend('show', 'Location', 'best', 'FontSize', fontSizeLegend);
    view(3); % Standard 3D view
    axis tight;
    disp('Displayed the single Desired 3D Trajectory Path plot.');
catch
    warning(['Could not read desired 3D path file: ', desired3DPathFile, '. Skipping this plot.']);
end

% --- Process p_traj_follow.csv and lqr_traj_follow.csv for comparisons ---
trajData = struct(); % Initialize struct to store data

for i = 1:length(trajFollowFiles)
    fileName = trajFollowFiles{i};
    disp(['Loading file: ', fileName]);
    try
        data = readmatrix(fileName);
    catch
        warning(['Could not read file: ', fileName, '. Skipping.']);
        continue;
    end

    time = data(:, time_idx);
    time = time - time(1); % Normalize time to start from 0
    
    roll_effort = data(:, roll_effort_idx);    % Commanded Roll Position
    roll_position = data(:, roll_position_idx); % Actual Roll Position
    pitch_effort = data(:, pitch_effort_idx);  % Commanded Pitch Position
    pitch_position = data(:, pitch_position_idx); % Actual Pitch Position

    [~, baseName, ~] = fileparts(fileName);
    if contains(baseName, 'p_traj_follow', 'IgnoreCase', true)
        key = 'P'; 
        caseName = 'P Control Trajectory Following';
    elseif contains(baseName, 'lqr_traj_follow', 'IgnoreCase', true)
        key = 'LQR'; 
        caseName = 'LQR Control Trajectory Following';
    else
        key = baseName; 
        caseName = strrep(baseName, '_', ' ');
    end
    
    trajData.(key).time = time;
    trajData.(key).roll_effort = roll_effort; 
    trajData.(key).roll_position = roll_position;
    trajData.(key).pitch_effort = pitch_effort;
    trajData.(key).pitch_position = pitch_position;
    trajData.(key).caseName = caseName; 

    disp(['Loaded trajectory data: ', caseName]);
end

% --- Generate comparison plots for Trajectory Following Data ---
if isfield(trajData, 'P') && isfield(trajData, 'LQR') % Ensure both files were loaded

    disp('Generating Trajectory Follow Comparison Plots...');
    
    %% Individual Commanded & Actual Position vs Time Plots for Trajectory Following
    fileKeys = fieldnames(trajData); 
    for k = 1:length(fileKeys)
        currentKey = fileKeys{k};
        data = trajData.(currentKey);
        
        actual_plot_color_traj = p_color; % Default to blue (P-Control)
        if strcmp(currentKey, 'LQR')
            actual_plot_color_traj = lqr_color; % Change to red for LQR
        end

        % Plot: Roll Commanded and Actual Position vs Time
        figure('Name', [data.caseName, ' - Roll Commanded & Actual Position vs Time']);
        set(gcf, 'WindowState', 'maximized');
        hold on;
        plot(data.time, data.roll_effort, desired_color, 'LineStyle', commanded_line_style, 'LineWidth', lineWidth, 'DisplayName', 'Commanded Position');
        plot(data.time, data.roll_position, actual_plot_color_traj, 'LineStyle', actual_line_style, 'LineWidth', lineWidth, 'DisplayName', 'Actual Position');
        hold off;
        grid on;
        set(gca, 'GridAlpha', gridAlpha, 'GridLineStyle', gridLineStyle);
        title([data.caseName, ': Roll Commanded & Actual Position vs Time'], 'FontSize', fontSizeTitle);
        xlabel('Time (s)', 'FontSize', fontSizeLabels);
        ylabel('Position (degrees)', 'FontSize', fontSizeLabels);
        legend('show', 'Location', 'best', 'FontSize', fontSizeLegend);
        set(gca, 'FontSize', fontSizeLabels);
        disp(['Displayed 2D plot: ', currentKey, ' Roll Commanded & Actual Position vs Time']);

        % Plot: Pitch Commanded and Actual Position vs Time
        figure('Name', [data.caseName, ' - Pitch Commanded & Actual Position vs Time']);
        set(gcf, 'WindowState', 'maximized');
        hold on;
        plot(data.time, data.pitch_effort, desired_color, 'LineStyle', commanded_line_style, 'LineWidth', lineWidth, 'DisplayName', 'Commanded Position');
        plot(data.time, data.pitch_position, actual_plot_color_traj, 'LineStyle', actual_line_style, 'LineWidth', lineWidth, 'DisplayName', 'Actual Position');
        hold off;
        grid on;
        set(gca, 'GridAlpha', gridAlpha, 'GridLineStyle', gridLineStyle);
        title([data.caseName, ': Pitch Commanded & Actual Position vs Time'], 'FontSize', fontSizeTitle);
        xlabel('Time (s)', 'FontSize', fontSizeLabels);
        ylabel('Position (degrees)', 'FontSize', fontSizeLabels);
        legend('show', 'Location', 'best', 'FontSize', fontSizeLegend);
        set(gca, 'FontSize', fontSizeLabels);
        disp(['Displayed 2D plot: ', currentKey, ' Pitch Commanded & Actual Position vs Time']);
    end


    %% Combined Error Plots for Trajectory Follow Data
    
    % Calculate errors
    roll_error_p = trajData.P.roll_effort - trajData.P.roll_position; 
    pitch_error_p = trajData.P.pitch_effort - trajData.P.pitch_position; 

    roll_error_lqr = trajData.LQR.roll_effort - trajData.LQR.roll_position; 
    pitch_error_lqr = trajData.LQR.pitch_effort - trajData.LQR.pitch_position; 

    % Plot Combined Trajectory Follow Roll Error
    figure('Name', 'Combined Trajectory Follow Roll Error');
    set(gcf, 'WindowState', 'maximized');
    hold on;
    plot(trajData.P.time, roll_error_p, p_color, 'LineStyle', commanded_line_style, 'DisplayName', [trajData.P.caseName ' Roll Error'], 'LineWidth', lineWidth);
    plot(trajData.LQR.time, roll_error_lqr, lqr_color, 'LineStyle', commanded_line_style, 'DisplayName', [trajData.LQR.caseName ' Roll Error'], 'LineWidth', lineWidth);
    hold off;
    grid on;
    set(gca, 'GridAlpha', gridAlpha, 'GridLineStyle', gridLineStyle);
    title('Roll Joint Error (Commanded - Actual) vs Time: P vs LQR Trajectory Following', 'FontSize', fontSizeTitle);
    xlabel('Time (s)', 'FontSize', fontSizeLabels);
    ylabel('Error (Commanded - Actual) (PWM Value - degrees)', 'FontSize', fontSizeLabels);
    legend('show', 'Location', 'best', 'FontSize', fontSizeLegend);
    set(gca, 'FontSize', fontSizeLabels);
    disp('Displayed combined plot: Trajectory Follow Roll Error');

    % Plot Combined Trajectory Follow Pitch Error
    figure('Name', 'Combined Trajectory Follow Pitch Error');
    set(gcf, 'WindowState', 'maximized');
    hold on;
    plot(trajData.P.time, pitch_error_p, p_color, 'LineStyle', commanded_line_style, 'DisplayName', [trajData.P.caseName ' Pitch Error'], 'LineWidth', lineWidth);
    plot(trajData.LQR.time, pitch_error_lqr, lqr_color, 'LineStyle', commanded_line_style, 'DisplayName', [trajData.LQR.caseName ' Pitch Error'], 'LineWidth', lineWidth);
    hold off;
    grid on;
    set(gca, 'GridAlpha', gridAlpha, 'GridLineStyle', gridLineStyle);
    title('Pitch Joint Error (Commanded - Actual) vs Time: P vs LQR Trajectory Following', 'FontSize', fontSizeTitle);
    xlabel('Time (s)', 'FontSize', fontSizeLabels);
    ylabel('Error (Commanded - Actual) (PWM Value - degrees)', 'FontSize', fontSizeLabels);
    legend('show', 'Location', 'best', 'FontSize', fontSizeLegend);
    set(gca, 'FontSize', fontSizeLabels);
    disp('Displayed combined plot: Trajectory Follow Pitch Error');


    %% Combined Commanded & Actual Position vs Time Plots (Trajectory Following)
    % Only one "Commanded Position" line is plotted for both.

    % Roll Combined Commanded & Actual
    disp('--- Generating Trajectory Following Roll Control Methods Time Response Comparison ---');
    figure('Name', 'Trajectory Following Roll Control Methods Time Response Comparison');
    set(gcf, 'WindowState', 'maximized');
    hold on;
    % Commanded Position (using LQR data, as it's identical to P's)
    plot(trajData.LQR.time, trajData.LQR.roll_effort, desired_color, 'LineStyle', commanded_line_style, 'LineWidth', lineWidth, 'DisplayName', 'Commanded Position');
    % LQR Actual Position
    plot(trajData.LQR.time, trajData.LQR.roll_position, lqr_color, 'LineStyle', actual_line_style, 'LineWidth', lineWidth, 'DisplayName', 'LQR Actual Position');
    % P-Control Actual Position
    plot(trajData.P.time, trajData.P.roll_position, p_color, 'LineStyle', actual_line_style, 'LineWidth', lineWidth, 'DisplayName', 'P-Control Actual Position');
    
    hold off;
    grid on;
    set(gca, 'GridAlpha', gridAlpha, 'GridLineStyle', gridLineStyle);
    title('Roll Joint Commanded & Actual Position vs Time: LQR vs P-Control (Trajectory Following)', 'FontSize', fontSizeTitle);
    xlabel('Time (s)', 'FontSize', fontSizeLabels);
    ylabel('Position (degrees)', 'FontSize', fontSizeLabels);
    legend('show', 'Location', 'best', 'FontSize', fontSizeLegend);
    set(gca, 'FontSize', fontSizeLabels);
    disp('Displayed plot: Trajectory Following Roll Control Methods Time Response Comparison');


    % Pitch Combined Commanded & Actual
    disp('--- Generating Trajectory Following Pitch Control Methods Time Response Comparison ---');
    figure('Name', 'Trajectory Following Pitch Control Methods Time Response Comparison');
    set(gcf, 'WindowState', 'maximized');
    hold on;
    % Commanded Position (using LQR data, as it's identical to P's)
    plot(trajData.LQR.time, trajData.LQR.pitch_effort, desired_color, 'LineStyle', commanded_line_style, 'LineWidth', lineWidth, 'DisplayName', 'Commanded Position');
    % LQR Actual Position
    plot(trajData.LQR.time, trajData.LQR.pitch_position, lqr_color, 'LineStyle', actual_line_style, 'LineWidth', lineWidth, 'DisplayName', 'LQR Actual Position');
    % P-Control Actual Position
    plot(trajData.P.time, trajData.P.pitch_position, p_color, 'LineStyle', actual_line_style, 'LineWidth', lineWidth, 'DisplayName', 'P-Control Actual Position');
    
    hold off;
    grid on;
    set(gca, 'GridAlpha', gridAlpha, 'GridLineStyle', gridLineStyle);
    title('Pitch Joint Commanded & Actual Position vs Time: LQR vs P-Control (Trajectory Following)', 'FontSize', fontSizeTitle);
    xlabel('Time (s)', 'FontSize', fontSizeLabels);
    ylabel('Position (degrees)', 'FontSize', fontSizeLabels);
    legend('show', 'Location', 'best', 'FontSize', fontSizeLegend);
    set(gca, 'FontSize', fontSizeLabels);
    disp('Displayed plot: Trajectory Following Pitch Control Methods Time Response Comparison');

else
    warning('Skipping Trajectory Follow comparison plots as "p_traj_follow.csv" or "lqr_traj_follow.csv" data could not be fully loaded.');
end

disp('Script execution complete. All plots displayed fullscreen.');