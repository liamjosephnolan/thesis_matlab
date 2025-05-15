% Load the CSV file
filename = 'roll_step_response_roll_-15_pitch_0.csv';
data = readtable(filename);

% Get the time column (assuming it's the first column)
time = data{:, 1};  

% Extract roll position and roll velocity by column index
% From your CSV: 
% Roll position is column 10
% Roll velocity is column 11

roll_position = data{:, 10};
roll_velocity = data{:, 11};

% Plot roll position
figure;
subplot(2,1,1);
plot(time, roll_position);
title('Roll Position vs Time');
xlabel('Time (s)');
ylabel('Roll Position (deg/rad/whatever)');

% Plot roll velocity
subplot(2,1,2);
plot(time, roll_velocity);
title('Roll Velocity vs Time');
xlabel('Time (s)');
ylabel('Roll Velocity (units/s)');

% Optional: Link x-axes for easy zooming
linkaxes(findall(gcf,'type','axes'),'x')
