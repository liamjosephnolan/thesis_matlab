clc
clear all
close all

%% ========== Roll LQR Calculation ==========
% Read CSV file with preserved headers
roll_data = readtable('roll_step.csv', 'VariableNamingRule', 'preserve');

% Extract time and normalize
roll_time = roll_data.time - roll_data.time(1);

% Extract roll data using original column names
roll_position = roll_data.('roll_position');
roll_velocity = roll_data.('roll_velocity');
roll_effort = roll_data.('roll_effort');

% Plot raw data
figure;
subplot(2,1,1); 
plot(roll_time, roll_position); 
ylabel('Roll Position');
title('Roll Raw Data');

subplot(2,1,2); 
plot(roll_time, roll_velocity); 
ylabel('Roll Velocity');
xlabel('Time (s)');

% Find step change time
velocity_threshold = 0.1; % Adjust based on noise level
roll_step_idx = find(abs(diff(roll_velocity)) > velocity_threshold, 1);

if isempty(roll_step_idx)
    error('No significant step change detected in roll velocity!');
end

% Trim data around step
roll_trim_idx = roll_step_idx:min(roll_step_idx+500, length(roll_time));
if length(roll_trim_idx) < 10
    error('Not enough roll data points after step for analysis');
end

% Prepare data for system ID
roll_y = roll_position(roll_trim_idx) - roll_position(roll_trim_idx(1));
roll_t = roll_time(roll_trim_idx) - roll_time(roll_trim_idx(1));
roll_u = roll_velocity(roll_trim_idx) - roll_velocity(roll_trim_idx(1));

% Fit transfer function (2nd-order)
roll_data_iddata = iddata(roll_y, roll_u, mean(diff(roll_t)));
roll_sys = tfest(roll_data_iddata, 2);

% Compare results
figure;
compare(roll_data_iddata, roll_sys);
title('Roll Model Validation');
legend('Actual', 'Fitted');
grid on;

% Convert to state-space
roll_ss_sys = ss(roll_sys);
roll_A = roll_ss_sys.A;
roll_B = roll_ss_sys.B;
roll_C = roll_ss_sys.C;
roll_D = roll_ss_sys.D;

% Check controllability
if rank(ctrb(roll_A, roll_B)) == size(roll_A, 1)
    disp('Roll System is controllable');
else
    error('Roll System is uncontrollable! Check model.');
end

% LQR weight matrices
roll_Q = diag([500, 0.25]);  % Penalize position error more than velocity
roll_R = 0.0075;             % Penalize control effort

% Compute LQR gain
[roll_K, roll_S, roll_CLP] = lqr(roll_A, roll_B, roll_Q, roll_R);

% Closed-loop system
roll_A_cl = roll_A - roll_B * roll_K;
roll_sys_cl = ss(roll_A_cl, roll_B, roll_C, roll_D);

% Simulate step response
t_sim = linspace(0, 10, 1000);
[roll_y_cl, roll_t_cl] = step(roll_sys_cl, t_sim);

% Plot
figure;
step(roll_sys, 'r--', roll_sys_cl, 'b-');
legend('Open-loop', 'Closed-loop (LQR)');
title('Roll Step Response Comparison');

%% ========== Pitch LQR Calculation ==========
% Read CSV file with preserved headers
pitch_data = readtable('prbs_pitch_V5.csv', 'VariableNamingRule', 'preserve');

% Extract time and normalize
pitch_time = pitch_data.time - pitch_data.time(1);

% Extract pitch data using original column names
pitch_position = pitch_data.('pitch_position');
pitch_velocity = pitch_data.('pitch_velocity');

% Plot raw data
figure;
subplot(2,1,1); 
plot(pitch_time, pitch_position); 
ylabel('Pitch Position');
title('Pitch Raw Data');

subplot(2,1,2); 
plot(pitch_time, pitch_velocity); 
ylabel('Pitch Velocity');
xlabel('Time (s)');


% Prepare data for system ID
pitch_y = pitch_position;
pitch_t = pitch_time;
pitch_u =  pitch_velocity;

% Fit transfer function (2nd-order)
pitch_data_iddata = iddata(pitch_y, pitch_u, mean(diff(pitch_t)));
pitch_sys = tfest(pitch_data_iddata, 3);

% Compare results
figure;
compare(pitch_data_iddata, pitch_sys);
title('Pitch Model Validation');
legend('Actual', 'Fitted');
grid on;

% Convert to state-space
pitch_ss_sys = ss(pitch_sys);
pitch_A = pitch_ss_sys.A;
pitch_B = pitch_ss_sys.B;
pitch_C = pitch_ss_sys.C;
pitch_D = pitch_ss_sys.D;
% Check controllability
if rank(ctrb(pitch_A, pitch_B)) == size(pitch_A, 1)
    disp('Pitch System is controllable');
else
    error('Pitch System is uncontrollable! Check model.');
end



% Augment system for integral action
pitch_A_aug = [pitch_A, zeros(2,1);
               -pitch_C, 0];
pitch_B_aug = [pitch_B;
               0];

% Augmented Q matrix (includes integral of position error)
pitch_Q_aug = diag([600, 0.15, .00005]);  % adjust 100 to tune integrator aggressiveness
pitch_R = 0.0025;    

% Compute LQI gain
[pitch_K_aug, pitch_S, pitch_CLP] = lqr(pitch_A_aug, pitch_B_aug, pitch_Q_aug, pitch_R);

% Extract Kx and Ki from augmented gain
pitch_Kx = pitch_K_aug(1, 1:2);  % state feedback gains
pitch_Ki = -pitch_K_aug(1, 3);    % integral gain

% Display results
disp('Pitch System LQI Gains:');
disp(['Kx = ', mat2str(pitch_Kx)]);
disp(['Ki = ', num2str(pitch_Ki)]);
