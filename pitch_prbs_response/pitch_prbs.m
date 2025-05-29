clear all
close all
clc

% Get list of CSV files in directory
files = dir('*.csv');

% Initialize result storage
results = table;
Kp_table = table;
Kd_table = table;
Ki_table = table;

% Loop over files
for i = 1:length(files)
    filename = files(i).name;
    
        % Extract roll and pitch from filename using regexp
    tokens = regexp(filename, 'prbs_roll_([m]?\d+)_pitch_([m]?\d+)\.csv', 'tokens');
    
    if isempty(tokens)
        disp(['Skipping file (name format not matched): ', filename]);
        continue
    end
    
    roll_str = tokens{1}{1};
    pitch_str = tokens{1}{2};
    
    % Convert 'm' prefix to negative number
    if startsWith(roll_str, 'm')
        roll_angle = -str2double(erase(roll_str, 'm'));
    else
        roll_angle = str2double(roll_str);
    end
    
    if startsWith(pitch_str, 'm')
        pitch_angle = -str2double(erase(pitch_str, 'm'));
    else
        pitch_angle = str2double(pitch_str);
    end

    % Load data
    data = readtable(filename, 'VariableNamingRule', 'preserve');
    
    % Extract time and normalize
    t = data{:,1} - data{1,1};
    
    % Extract pitch position and pitch velocity
    pos = data{:,7};  % Pitch position
    u = data{:,8};  % Pitch velocity
    
    % Sampling time
    Ts = mean(diff(t));
    
    % Prepare iddata object for system ID
    sys_data = iddata(pos, u, Ts);
    
    % Construct a valid variable name using roll and pitch
    varName = sprintf('sysdata_R%d_P%d', roll_angle, pitch_angle);
    varName = matlab.lang.makeValidName(varName);  % ensure it's a valid MATLAB variable name
    
    % Assign to a variable in the base workspace
    assignin('base', varName, sys_data);
        
    % Fit transfer function model (3rd order)
    sys_tf = tfest(sys_data, 3);
    
    % Convert to state-space (controllable canonical form)
    sys_ss = ss(sys_tf);
    
    % Resample to uniform time vector
    t_uniform = (0:Ts:t(end))';
    vel_uniform = interp1(t, u, t_uniform);
    pos_uniform = interp1(t, pos, t_uniform);
    
    % Simulate the fitted transfer function response
    pos_sim = lsim(sys_tf, vel_uniform, t_uniform);
    
    % Plot measured vs fitted response
    figure;
    plot(t_uniform, pos_uniform, 'b', 'LineWidth', 1.5); hold on;
    plot(t_uniform, pos_sim, 'r--', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Position (rad)');
    title(sprintf('Measured vs Fitted - Roll: %d°, Pitch: %d°', roll_angle, pitch_angle));
    legend('Measured', 'Fitted');
    grid on;
    
    % Extract state-space matrices
    [A, B, C, D] = ssdata(sys_ss);
    
    % Display matrices cleanly
    fprintf('\nFile: %s\n', filename);
    fprintf('A matrix:\n'); disp(A)
    fprintf('B matrix:\n'); disp(B)
    fprintf('C matrix:\n'); disp(C)
    fprintf('D matrix:\n'); disp(D)
    
    % Check controllability
    if rank(ctrb(A, B)) < size(A, 1)
        error('System is uncontrollable at Roll %d°, Pitch %d°', roll_angle, pitch_angle);
    end
    
    % Define LQR tuning weights
    Q = diag([500, 0.0015, 0.001]);  % adjust values as needed
    R = 0.01;    

    % Calculate LQR gains
    [K,~,~] = lqr(A, B, Q, R);
    
    % Separate K into Kp, Kd, Ki (if applicable)
    if size(K,2) == 2
        Kp = K(1);
        Kd = K(2);
        Ki = 0;
    else
        Kp = K(1);
        Kd = K(2);
        Ki = K(3);
    end
    
    % Store full result in master table
    results = [results;
        table(roll_angle, pitch_angle, Kp, Kd, Ki, {A}, {B}, {C}, {D}, ...
        'VariableNames', {'Roll', 'Pitch', 'Kp', 'Kd', 'Ki', 'A', 'B', 'C', 'D'})];
    
    % Store in individual K tables for clean summary display
    Kp_table = [Kp_table; table(roll_angle, pitch_angle, Kp)];
    Kd_table = [Kd_table; table(roll_angle, pitch_angle, Kd)];
    Ki_table = [Ki_table; table(roll_angle, pitch_angle, Ki)];
end

% Display summary tables
disp('================= Kp Gains =================');
disp(Kp_table)

disp('================= Kd Gains =================');
disp(Kd_table)

disp('================= Ki Gains =================');
disp(Ki_table)
