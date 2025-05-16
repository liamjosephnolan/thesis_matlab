clear all;
close all;
clc;

%% Load CSV Files and Parse Metadata
folderPath = pwd;
fileList = dir(fullfile(folderPath, 'roll_step_response_roll_*_pitch_*.csv'));

% Initialize struct array
data = [];

for k = 1:length(fileList)
    fileName = fileList(k).name;

    % Extract roll and pitch angles from filename
    tokens = regexp(fileName, 'roll_step_response_roll_([-]?\d+)_pitch_([-]?\d+)\.csv', 'tokens');

    if ~isempty(tokens)
        rollAngle = str2double(tokens{1}{1});
        pitchAngle = str2double(tokens{1}{2});

        % Load table data
        tblData = readtable(fullfile(folderPath, fileName), 'VariableNamingRule', 'preserve');

        % Append data to struct
        entry.roll = rollAngle;
        entry.pitch = pitchAngle;
        entry.time = tblData{:,1};
        entry.roll_position = tblData{:,10};
        entry.roll_velocity = tblData{:,11};

        data = [data; entry];
    end
end

numPlots = length(data);

%% Plot Raw Roll Position Step Responses
figure('Name', 'Roll Position Step Responses');
for i = 1:numPlots
    subplot(3,3,i);
    plot(data(i).time, data(i).roll_position);
    title(sprintf('Roll: %d° Pitch: %d°', data(i).roll, data(i).pitch));
    xlabel('Time (s)');
    ylabel('Roll Position');
    grid on;
end
sgtitle('Raw Roll Position vs Time');

%% System Identification (2nd-Order Transfer Functions)
velocity_threshold = 0.1;
figure('Name', 'System Identification Results');

for i = 1:numPlots
    currData = data(i);
    roll_time = currData.time;
    roll_position = currData.roll_position;
    roll_velocity = currData.roll_velocity;

    % Detect step change
    roll_step_idx = find(abs(diff(roll_velocity)) > velocity_threshold, 1) - 100;

    if isempty(roll_step_idx)
        warning('No step detected for Roll=%d Pitch=%d', currData.roll, currData.pitch);
        continue;
    end

    % Trim data after step
    roll_trim_idx = roll_step_idx:min(roll_step_idx+1000, length(roll_time));

    if length(roll_trim_idx) < 10
        warning('Insufficient data after step for Roll=%d Pitch=%d', currData.roll, currData.pitch);
        continue;
    end

    roll_y = roll_position(roll_trim_idx) - roll_position(roll_trim_idx(1));
    roll_t = roll_time(roll_trim_idx) - roll_time(roll_trim_idx(1));
    roll_u = roll_velocity(roll_trim_idx) - roll_velocity(roll_trim_idx(1));

    Ts = mean(diff(roll_t));
    roll_data_iddata = iddata(roll_y, roll_u, Ts);

    % Estimate transfer function
    try
        roll_sys = tfest(roll_data_iddata, 2);
        data(i).tf_model = roll_sys;
    catch ME
        warning('System ID failed for Roll=%d Pitch=%d: %s', currData.roll, currData.pitch, ME.message);
        continue;
    end

    % Plot comparison
    subplot(3,3,i);
    compare(roll_data_iddata, roll_sys);
    title(sprintf('Roll: %d° Pitch: %d°', currData.roll, currData.pitch));
    grid on;
end
sgtitle('System Identification Comparison (2nd-Order TF)');

%% LQR Design and Closed-Loop Step Response
default_Q = diag([500, 0.25]);
default_R = 0.0075;
t_sim = linspace(0, 10, 1000);

figure('Name', 'Open-loop vs LQR Closed-loop Step Responses');

for i = 1:numPlots
    if ~isfield(data(i), 'tf_model') || isempty(data(i).tf_model)
        warning('No TF model for Roll=%d Pitch=%d, skipping...', data(i).roll, data(i).pitch);
        continue;
    end

    % Convert to state-space
    roll_sys_ss = ss(data(i).tf_model);
    A = roll_sys_ss.A;
    B = roll_sys_ss.B;
    C = roll_sys_ss.C;
    D = roll_sys_ss.D;

    % Controllability check
    if rank(ctrb(A,B)) < size(A,1)
        warning('System at Roll=%d Pitch=%d is uncontrollable.', data(i).roll, data(i).pitch);
        continue;
    else
        fprintf('System at Roll=%d Pitch=%d is controllable.\n', data(i).roll, data(i).pitch);
    end

    % Display A matrix
    fprintf('A matrix at Roll=%d Pitch=%d:\n', data(i).roll, data(i).pitch);
    disp(A);

    % LQR Design
    Q = default_Q;
    R = default_R;

    [K, ~, ~] = lqr(A, B, Q, R);

    fprintf('K matrix at Roll=%d Pitch=%d:\n', data(i).roll, data(i).pitch);
    disp(K);


    % Closed-loop system
    A_cl = A - B*K;
    sys_cl = ss(A_cl, B, C, D);

    % Simulate step responses
    [y_cl, t_cl] = step(sys_cl, t_sim);
    [y_ol, t_ol] = step(data(i).tf_model, t_sim);

    % Plot open vs closed loop
    subplot(3,3,i);
    plot(t_ol, y_ol, 'r--', t_cl, y_cl, 'b-');
    title(sprintf('Roll: %d° Pitch: %d°', data(i).roll, data(i).pitch));
    legend('Open-loop', 'Closed-loop', 'Location', 'best');
    xlabel('Time (s)');
    ylabel('Roll Position');
    grid on;

    % Save to struct
    data(i).A = A;
    data(i).B = B;
    data(i).C = C;
    data(i).D = D;
    data(i).K = K;
    data(i).Q = Q;
    data(i).R = R;
    data(i).sys_cl = sys_cl;
    data(i).A_cl = A_cl;
end

sgtitle('Open-loop vs Closed-loop Step Response (LQR Controlled)');


for i = 1:numPlots
    if isfield(data(i), 'A')
        eigVals = eig(data(i).A);
        fprintf('Eigenvalues at Roll=%d Pitch=%d: ', data(i).roll, data(i).pitch);
        disp(eigVals.');
    end
end

for i = 1:numPlots
    if isfield(data(i), 'A')
        poles = eig(data(i).A);
        for p = 1:length(poles)
            wn = abs(poles(p));
            zeta = -real(poles(p)) / wn;
            fprintf('Sys %d | Pole: %.3f%+.3fj | ωn=%.3f | ζ=%.3f\n', i, real(poles(p)), imag(poles(p)), wn, zeta);
        end
    end
end


% Create a new figure for eigenvalues
figure;
hold on;
colors = lines(numPlots); % Distinct colors for each system

for i = 1:numPlots
    if isfield(data(i), 'A')
        eigVals = eig(data(i).A);
        % Plot eigenvalues for this system
        plot(real(eigVals), imag(eigVals), 'x', 'Color', colors(i,:), 'MarkerSize', 10, 'LineWidth', 2);
        % Label position next to first eigenvalue point
        text(real(eigVals(1)) + 0.1, imag(eigVals(1)), sprintf('R%d P%d', data(i).roll, data(i).pitch), 'Color', colors(i,:), 'FontSize', 8);
    end
end

xlabel('Real Part');
ylabel('Imaginary Part');
title('Eigenvalues of A Matrices for Each Roll/Pitch Position');
grid on;
axis equal;
xlim auto;
ylim auto;
hold off;
