
A = ss1.A;
B = ss1.B;
C = ss1.C;
D = ss1.D;


% LQR weight matrices (can adjust independently)
Q = diag([.1, 0.001]);  % Penalize position error more than velocity
R = 10;             % Penalize control effort

% Compute LQR gain
[K, S, CLP] = lqr(A, B, Q, R);