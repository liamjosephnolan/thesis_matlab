clc;

%% User Toggle
% 0 = Full script (clear all, system ID, LQR)
% 1 = LQR design only (reuses previously identified systems)
runMode = 0;

if runMode == 0
    clear all; close all; clc;
    runMode = 0;
end

if runMode == 1
    clc;

end

%% Define roll and pitch grid values (manually ordered)
roll_vals = [-20, 0, 20];
pitch_vals = [-23, 0, 23];  % Note: exclude -3 since you want a clean 3x3

%% Define your grid points with files and explicit indices in tables
% Add your data files here, along with the roll/pitch angles and their corresponding indices in the 3x3 table
gridPoints = [
    struct('file', 'prbs_roll_m20_pitch_m15.csv', 'roll', -20, 'pitch', -15, ...
           'row_idx', 1, 'col_idx', 1, 'Qpos', 5000, 'Qvel', 0.25, 'Qi', 0.005, 'R', 0.1);
    struct('file', 'prbs_roll_0_pitch_m15.csv', 'roll', 0, 'pitch', -15, ...
           'row_idx', 1, 'col_idx', 2, 'Qpos', 5000, 'Qvel', 0.25, 'Qi', 0.005, 'R', 0.1);
    struct('file', 'prbs_roll_20_pitch_m15.csv', 'roll', 20, 'pitch', -15, ...
           'row_idx', 1, 'col_idx', 3, 'Qpos', 5000, 'Qvel', 0.25, 'Qi', 0.005, 'R', 0.1);
    struct('file', 'prbs_roll_m20_pitch_0.csv', 'roll', -20, 'pitch', 0, ...
           'row_idx', 2, 'col_idx', 1, 'Qpos', 5000, 'Qvel', 0.25, 'Qi', 0.005, 'R', 0.1)
    struct('file', 'prbs_roll_0_pitch_0.csv', 'roll', 0, 'pitch', 0, ...
           'row_idx', 2, 'col_idx', 2, 'Qpos', 5000, 'Qvel', 0.25, 'Qi', 0.005, 'R', 0.1);
    struct('file', 'prbs_roll_20_pitch_0.csv', 'roll', 20, 'pitch', 0, ...
           'row_idx', 2, 'col_idx', 3, 'Qpos', 5000, 'Qvel', 0.25, 'Qi', 0.005, 'R', 0.1);
    struct('file', 'prbs_roll_m20_pitch_15.csv', 'roll', -20, 'pitch', 15, ...
           'row_idx', 3, 'col_idx', 1, 'Qpos', 5000, 'Qvel', 0.25, 'Qi', 0.005, 'R', 0.1);
    struct('file', 'prbs_roll_0_pitch_15.csv', 'roll', 0, 'pitch', 15, ...
           'row_idx', 3, 'col_idx', 2, 'Qpos', 5000, 'Qvel', 0.25, 'Qi', 0.005, 'R', 0.1);
    struct('file', 'prbs_roll_20_pitch_15.csv', 'roll', 20, 'pitch', 15, ...
           'row_idx', 3, 'col_idx', 3, 'Qpos', 5000, 'Qvel', 0.25, 'Qi', 0.005, 'R', 0.1);
];

%% Full Workflow: Clear, System ID, Plots
if runMode == 0

    %% System Identification Loop
    results = struct();
    for i = 1:numel(gridPoints)
        % --- Data Loading ---
        fileName = gridPoints(i).file;
        dataTbl = readtable(fileName);

        t = dataTbl{:,1} - dataTbl{1,1};
        pos = dataTbl{:,7};
        u = dataTbl{:,8};
        Ts = mean(diff(t));

        % --- Raw Data Plots ---
        figure(1);
        subplot(2, numel(gridPoints), 2*i-1);
        plot(t, pos, 'b', 'LineWidth', 1.5);
        title(sprintf('Pos: R=%d°, P=%d°', gridPoints(i).roll, gridPoints(i).pitch));
        xlabel('Time (s)'); ylabel('Position (deg)');
        grid on;

        subplot(2, numel(gridPoints), 2*i);
        plot(t, u, 'r', 'LineWidth', 1.5);
        title(sprintf('Input: R=%d°, P=%d°', gridPoints(i).roll, gridPoints(i).pitch));
        xlabel('Time (s)'); ylabel('Control Input');
        grid on;

        % --- System Identification ---
        sys_data = iddata(pos, u, Ts);
        sys_est = tfest(sys_data, 3);
        ss_sys = ss(sys_est);

        % --- Model Validation ---
        [y_sim, fit] = compare(sys_data, sys_est);

        figure(2);
        subplot(1, numel(gridPoints), i);
        plot(t, pos, 'b', y_sim.SamplingInstants, y_sim.OutputData, 'r--', 'LineWidth', 1.5);
        title(sprintf('Fit: R=%d°, P=%d° (%.1f%%)', gridPoints(i).roll, gridPoints(i).pitch, fit));
        xlabel('Time (s)'); ylabel('Position (deg)');
        legend('Measured', 'Simulated');
        grid on;

        % Store Identified System and table indices
        results(i).angles = [gridPoints(i).roll, gridPoints(i).pitch];
        results(i).A = ss_sys.A;
        results(i).B = ss_sys.B;
        results(i).fit = fit;
        results(i).fileName = fileName;
        results(i).row_idx = gridPoints(i).row_idx;
        results(i).col_idx = gridPoints(i).col_idx;
    end

    % Save Results to Workspace
    assignin('base', 'results', results);
end

%% LQR Controller Design
if runMode == 0 || runMode == 1
    if runMode == 1
        if ~exist('results', 'var')
            error('No identified systems found in workspace. Run with runMode = 0 first.');
        end
    end

    % Initialize empty tables with NaNs
    Kp_table = NaN(3,3);
    Kd_table = NaN(3,3);
    Ki_table = NaN(3,3);

    % Compute LQR gains and fill tables using explicit row/col indices
    for i = 1:numel(results)
        % Build Q and R matrices
        Q = diag([gridPoints(i).Qpos, gridPoints(i).Qvel, gridPoints(i).Qi]);
        R = gridPoints(i).R;

        % LQR gain calculation
        [K, ~, ~] = lqr(results(i).A, results(i).B, Q, R);
        results(i).K = K;

        % Fill the tables at the manually assigned index
        r = results(i).row_idx;
        c = results(i).col_idx;

        Kp_table(r, c) = K(1);
        Kd_table(r, c) = K(2);
        Ki_table(r, c) = K(3);
    end

    % Display gains
    fprintf('=== LQR Controller Gains ===\n');
    for i = 1:numel(results)
        fprintf('\nOperating Point %d:\n', i);
        fprintf('Roll: %d°, Pitch: %d°\n', results(i).angles(1), results(i).angles(2));
        fprintf('Fit: %.1f%%\n', results(i).fit);
        fprintf('LQR Gains: [%.3f, %.3f, %.3f]\n', results(i).K);
    end

    % === Print C-style Gain Tables ===
    fprintf('\n// Gain tables for Pitch (3x3)\n');

    % Kp
    fprintf('const double Kp_pitch_table[3][3] = {\n');
    for r = 1:3
        fprintf('    {%.5f, %.5f, %.5f}', Kp_table(r,1), Kp_table(r,2), Kp_table(r,3));
        if r < 3
            fprintf(',  // Roll %d\n', roll_vals(r));
        else
            fprintf('   // Roll %d\n', roll_vals(r));
        end
    end
    fprintf('};\n\n');

    % Kd
    fprintf('const double Kd_pitch_table[3][3] = {\n');
    for r = 1:3
        fprintf('    {%.5f, %.5f, %.5f}', Kd_table(r,1), Kd_table(r,2), Kd_table(r,3));
        if r < 3
            fprintf(',  // Roll %d\n', roll_vals(r));
        else
            fprintf('   // Roll %d\n', roll_vals(r));
        end
    end
    fprintf('};\n\n');

    % Ki
    fprintf('const double Ki_pitch_table[3][3] = {\n');
    for r = 1:3
        fprintf('    {%.5f, %.5f, %.5f}', Ki_table(r,1), Ki_table(r,2), Ki_table(r,3));
        if r < 3
            fprintf(',  // Roll %d\n', roll_vals(r));
        else
            fprintf('   // Roll %d\n', roll_vals(r));
        end
    end
    fprintf('};\n');
end
