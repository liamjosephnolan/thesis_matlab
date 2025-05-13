
%% 1. Load and Prepare Data
% Load PRBS Data for Pitch
dataTable = readtable('PRBS_pitch_v4.csv');

% Extract data columns (corrected based on your clarification)
time = dataTable.time;
input = dataTable.pitch_velocity;     % Control input = velocity
output = dataTable.pitch_position;    % Measured output = position

Ts = mean(diff(time));  % Average sampling time

% Create iddata object
data = iddata(output, input, Ts);
data.TimeUnit = 'seconds';
data.InputName = 'VelocityCommand';
data.OutputName = 'PitchPosition';
data.InputUnit = 'deg/s';      % Adjust units as needed
data.OutputUnit = 'deg';       % Adjust units as needed
