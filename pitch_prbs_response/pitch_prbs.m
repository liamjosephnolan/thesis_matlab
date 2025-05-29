clear all
close all
clc

% List of data files and associated roll/pitch metadata
dataFiles = {
    'prbs_roll_0_pitch_m23.csv',  [0, -23];
    'prbs_roll_0_pitch_m3.csv',   [0, -3];
};

% LQR tuning matrices
pitch_Q_base = diag([800, 0.0025, 0.005]);  % base 3x3 Q matrix
pitch_R_scalar = 0.03;                     % scalar R

% Preallocate results storage
results = struct();

for i = 1:size(dataFiles, 1)
    % Load data table
    fileName = dataFiles{i,1};
    angles   = dataFiles{i,2};
    dataTbl  = readtable(fileName);

    % Extract time, position, and input
    t   = dataTbl{:,1} - dataTbl{1,1};
    pos = dataTbl{:,7};
    u   = dataTbl{:,8};
    Ts  = mean(diff(t));  % infer sample time from data

    % Build iddata object
    sys_data = iddata(pos, u, Ts);

    % Estimate 3rd order system using tfest
    np = 3; % number of poles
    nz = 0; % number of zeros
    sys_est = tfest(sys_data, np, nz);

    % Convert to state-space (minimal realization)
    ss_sys = ss(sys_est);
    ss_sys = minreal(ss_sys);

    % Build LQR controller with provided Q and R
    [K,~,~] = lqr(ss_sys.A, ss_sys.B, pitch_Q_base, pitch_R_scalar);

    % Store results
    results(i).fileName = fileName;
    results(i).roll     = angles(1);
    results(i).pitch    = angles(2);
    results(i).A        = ss_sys.A;
    results(i).B        = ss_sys.B;
    results(i).C        = ss_sys.C;
    results(i).D        = ss_sys.D;
    results(i).K        = K;

    % Plot raw position vs time
    figure;
    subplot(2,1,1);
    plot(t, pos, 'b', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Position (deg)');
    title(['Position vs. Time - ', fileName], 'Interpreter','none');
    grid on;

    % Plot input signal vs time
    subplot(2,1,2);
    plot(t, u, 'r', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Input Signal');
    title(['Input Signal vs. Time - ', fileName], 'Interpreter','none');
    grid on;
end

% Display system matrices and LQR gains
for i = 1:numel(results)
    disp(['Results for file: ', results(i).fileName]);
    disp(['Roll: ', num2str(results(i).roll), '°, Pitch: ', num2str(results(i).pitch), '°']);
    disp('A ='); disp(results(i).A);
    disp('B ='); disp(results(i).B);
    disp('C ='); disp(results(i).C);
    disp('D ='); disp(results(i).D);
    disp('LQR Gains K ='); disp(results(i).K);
    disp('-------------------------------------------');
end
