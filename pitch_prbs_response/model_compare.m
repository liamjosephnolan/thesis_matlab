
clear all; close all; clc;

files = dir('*.csv');

pitch_Q_base = diag([800, 0.0025, 0.005]);  % base 3x3 Q matrix
pitch_R_scalar = 0.03;                    % scalar R

% Initialize arrays to store results
roll_all = [];
pitch_all = [];
model_order_all = []; % 3 or 4
Kp_all = [];
Ki_all = [];
Kd_all = [];

for i = 1:length(files)
    filename = files(i).name;

    % Extract roll and pitch from filename
    tokens = regexp(filename, 'prbs_roll_([m]?\d+)_pitch_([m]?\d+)\.csv', 'tokens');
    if isempty(tokens)
        disp(['Skipping ', filename]);
        continue;
    end

    roll = str2double(strrep(tokens{1}{1}, 'm', '-'));
    pitch = str2double(strrep(tokens{1}{2}, 'm', '-'));

    dataTbl = readtable(filename, 'VariableNamingRule', 'preserve');
    t = dataTbl{:,1} - dataTbl{1,1};
    pos = dataTbl{:,7};
    u   = dataTbl{:,8};

    % Uniform time vector and interpolation
    Ts = mean(diff(t));
    t_uniform = t(1):Ts:t(end);
    pos_uniform = interp1(t, pos, t_uniform, 'linear');
    u_uniform = interp1(t, u, t_uniform, 'linear');

    % Remove NaNs and ensure column vectors
    nan_idx = isnan(pos_uniform) | isnan(u_uniform);
    t_uniform(nan_idx) = [];
    pos_uniform(nan_idx) = [];
    u_uniform(nan_idx) = [];
    pos_uniform = pos_uniform(:);
    u_uniform = u_uniform(:);

    % Downsample data to speed up computations
    d = 10;
    t_ds = t_uniform(1:d:end);
    pos_ds = pos_uniform(1:d:end);
    u_ds = u_uniform(1:d:end);
    Ts_ds = Ts * d;

    % Create iddata object
    data = iddata(pos_ds, u_ds, Ts_ds);

    fprintf('\nProcessing file: %s\n', filename);
    fprintf('Roll: %d°, Pitch: %d°\n', roll, pitch);

    % Identify 3rd and 4th order TF models
    sys3 = tfest(data, 3);
    sys4 = tfest(data, 4);

    % Different state-space realizations for 3rd order
    ss_sys3 = ss(sys3);
    ss_min3 = minreal(ss_sys3);
    ss_bal3 = balreal(ss_sys3);

    % Poles for 3rd order
    poles3 = pole(ss_sys3);
    fprintf('\n3rd Order System Poles:\n'); disp(poles3);

    % Display 3rd order matrices for standard ss
    fprintf('\n3rd Order TFEST State-space matrices (Standard ss):\n');
    fprintf('A =\n'); disp(ss_sys3.A);
    fprintf('B =\n'); disp(ss_sys3.B);
    fprintf('C =\n'); disp(ss_sys3.C);
    fprintf('D =\n'); disp(ss_sys3.D);

    % Display minimal realization matrices for 3rd order
    fprintf('\n3rd Order Minimal Realization matrices:\n');
    fprintf('A =\n'); disp(ss_min3.A);
    fprintf('B =\n'); disp(ss_min3.B);
    fprintf('C =\n'); disp(ss_min3.C);
    fprintf('D =\n'); disp(ss_min3.D);

    % Display balanced realization matrices for 3rd order
    fprintf('\n3rd Order Balanced Realization matrices:\n');
    fprintf('A =\n'); disp(ss_bal3.A);
    fprintf('B =\n'); disp(ss_bal3.B);
    fprintf('C =\n'); disp(ss_bal3.C);
    fprintf('D =\n'); disp(ss_bal3.D);

    % Different state-space realizations for 4th order
    ss_sys4 = ss(sys4);
    ss_min4 = minreal(ss_sys4);
    ss_bal4 = balreal(ss_sys4);

    % Poles for 4th order
    poles4 = pole(ss_sys4);
    fprintf('\n4th Order System Poles:\n'); disp(poles4);

    % Display 4th order matrices for standard ss
    fprintf('\n4th Order TFEST State-space matrices (Standard ss):\n');
    fprintf('A =\n'); disp(ss_sys4.A);
    fprintf('B =\n'); disp(ss_sys4.B);
    fprintf('C =\n'); disp(ss_sys4.C);
    fprintf('D =\n'); disp(ss_sys4.D);

    % Display minimal realization matrices for 4th order
    fprintf('\n4th Order Minimal Realization matrices:\n');
    fprintf('A =\n'); disp(ss_min4.A);
    fprintf('B =\n'); disp(ss_min4.B);
    fprintf('C =\n'); disp(ss_min4.C);
    fprintf('D =\n'); disp(ss_min4.D);

    % Display balanced realization matrices for 4th order
    fprintf('\n4th Order Balanced Realization matrices:\n');
    fprintf('A =\n'); disp(ss_bal4.A);
    fprintf('B =\n'); disp(ss_bal4.B);
    fprintf('C =\n'); disp(ss_bal4.C);
    fprintf('D =\n'); disp(ss_bal4.D);

    % Simulate responses
    y3 = lsim(sys3, u_ds, t_ds);
    y4 = lsim(sys4, u_ds, t_ds);

    % Plot measured vs model outputs
    figure;
    plot(t_ds, pos_ds, 'k', t_ds, y3, 'r--', t_ds, y4, 'b--', 'LineWidth', 1.2);
    legend('Measured', 'TFEST 3rd order', 'TFEST 4th order');
    title(sprintf('Response comparison | Roll %d° Pitch %d°', roll, pitch));
    xlabel('Time (s)');
    ylabel('Position');

    % --- LQR for 3rd order ---
    A = ss_sys3.A;
    B = ss_sys3.B;

    n_states = size(A,1);
    n_inputs = size(B,2);
    Q = zeros(n_states);
    Q(1:min(end,size(pitch_Q_base,1)), 1:min(end,size(pitch_Q_base,1))) = pitch_Q_base(1:min(end,n_states), 1:min(end,n_states));
    if isscalar(pitch_R_scalar)
        R = pitch_R_scalar * eye(n_inputs);
    else
        R = pitch_R_scalar;
    end

    if rank(ctrb(A,B)) < n_states
        disp('3rd order system uncontrollable — skipping LQR.');
        roll_all(end+1) = roll;
        pitch_all(end+1) = pitch;
        model_order_all(end+1) = 3;
        Kp_all(end+1) = NaN;
        Ki_all(end+1) = NaN;
        Kd_all(end+1) = NaN;
    else
        K = lqr(A,B,Q,R);
        fprintf('\n3rd Order LQR Gains:\n'); disp(K);

        roll_all(end+1) = roll;
        pitch_all(end+1) = pitch;
        model_order_all(end+1) = 3;

        if length(K) >= 3
            Kp_all(end+1) = K(1);
            Kd_all(end+1) = K(2);
            Ki_all(end+1) = K(3);
        else
            Kp_all(end+1) = NaN;
            Ki_all(end+1) = NaN;
            Kd_all(end+1) = NaN;
        end
    end

    % --- LQR for 4th order ---
    A = ss_sys4.A;
    B = ss_sys4.B;

    n_states = size(A,1);
    n_inputs = size(B,2);
    Q = zeros(n_states);
    Q(1:min(end,size(pitch_Q_base,1)), 1:min(end,size(pitch_Q_base,1))) = pitch_Q_base(1:min(end,n_states), 1:min(end,n_states));
    if isscalar(pitch_R_scalar)
        R = pitch_R_scalar * eye(n_inputs);
    else
        R = pitch_R_scalar;
    end

    if rank(ctrb(A,B)) < n_states
        disp('4th order system uncontrollable — skipping LQR.');
        roll_all(end+1) = roll;
        pitch_all(end+1) = pitch;
        model_order_all(end+1) = 4;
        Kp_all(end+1) = NaN;
        Ki_all(end+1) = NaN;
        Kd_all(end+1) = NaN;
    else
        K = lqr(A,B,Q,R);
        fprintf('\n4th Order LQR Gains:\n'); disp(K);

        roll_all(end+1) = roll;
        pitch_all(end+1) = pitch;
        model_order_all(end+1) = 4;

        if length(K) >= 3
            Kp_all(end+1) = K(1);
            Kd_all(end+1) = K(2);
            Ki_all(end+1) = K(3);
        else
            Kp_all(end+1) = NaN;
            Ki_all(end+1) = NaN;
            Kd_all(end+1) = NaN;
        end
    end
end

% Create and display tables for Kp, Kd, Ki including model order
T_Kp = table(roll_all', pitch_all', model_order_all', Kp_all', ...
    'VariableNames', {'Roll_deg', 'Pitch_deg', 'Model_Order', 'Kp'});
T_Kd = table(roll_all', pitch_all', model_order_all', Kd_all', ...
    'VariableNames', {'Roll_deg', 'Pitch_deg', 'Model_Order', 'Kd'});
T_Ki = table(roll_all', pitch_all', model_order_all', Ki_all', ...
    'VariableNames', {'Roll_deg', 'Pitch_deg', 'Model_Order', 'Ki'});

fprintf('\nSummary Table for Kp gains:\n');
disp(T_Kp);

fprintf('\nSummary Table for Kd gains:\n');
disp(T_Kd);

fprintf('\nSummary Table for Ki gains:\n');
disp(T_Ki);
