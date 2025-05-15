clear all;
close all;
clc

load("ss1.mat")
pitch_sys = ss1;


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



% Get sizes
[n, ~] = size(pitch_A);
[~, m] = size(pitch_B);

% Augment A matrix
pitch_A_aug = [pitch_A,         zeros(n,1);
               -pitch_C,        0];

% Augment B matrix
pitch_B_aug = [pitch_B;
               zeros(1,m)];

% Augmented Q matrix (includes integral of position error)
pitch_Q_aug = diag([.06, 0.000015 ,0, .0000005]);  % adjust 100 to tune integrator aggressiveness
pitch_R = 10;    

% Compute LQI gain
[pitch_K_aug, pitch_S, pitch_CLP] = lqr(pitch_A_aug, pitch_B_aug, pitch_Q_aug, pitch_R);

% Extract Kx and Ki from augmented gain
pitch_Kx = pitch_K_aug(1, 1:2);  % state feedback gains
pitch_Ki = -pitch_K_aug(1, 3);    % integral gain

% Display results
disp('Pitch System LQI Gains:');
disp(['Kx = ', mat2str(pitch_Kx)]);
disp(['Ki = ', num2str(pitch_Ki)]);
