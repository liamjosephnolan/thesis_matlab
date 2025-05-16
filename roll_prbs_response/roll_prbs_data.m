clear all;
close all;
clc;

% Set folder path to current directory
folderPath = pwd;

% Get list of PRBS CSV files
fileList = dir(fullfile(folderPath, 'prbs_roll*_pitch*.csv'));

% Helper function to convert 'm20' to -20, '0' to 0, '20' to 20
convertAngle = @(str) str2double(regexprep(str, 'm', '-'));

% Loop through files
for k = 1:length(fileList)
    fileName = fileList(k).name;

    % Extract roll and pitch angles from filename
    tokens = regexp(fileName, 'prbs_roll_(m?\d+)_pitch_(m?\d+)\.csv', 'tokens');

    if ~isempty(tokens)
        rollAngleStr = tokens{1}{1};
        pitchAngleStr = tokens{1}{2};

        rollAngle = convertAngle(rollAngleStr);
        pitchAngle = convertAngle(pitchAngleStr);

        % Load table data (no need to preserve headers)
        tblData = readtable(fullfile(folderPath, fileName), 'VariableNamingRule', 'preserve');

        % Extract time, roll position (col 10), and roll velocity (col 11)
        timeVec = tblData{:,1};
        roll_position = tblData{:,10};
        roll_velocity = tblData{:,11};

        % Compute sample time
        Ts = mean(diff(timeVec));

        % Create iddata object
        prbs_iddata = iddata(roll_position, roll_velocity, Ts);

        % Create a variable name like data_0_20 or data_m20_m20
        varName = sprintf('data_%s_%s', rollAngleStr, pitchAngleStr);

        % Store as a struct with data
        S.roll = rollAngle;
        S.pitch = pitchAngle;
        S.time = timeVec;
        S.roll_position = roll_position;
        S.roll_velocity = roll_velocity;
        S.Ts = Ts;
        S.iddata = prbs_iddata;

        % Dynamically assign to variable in workspace
        assignin('base', varName, S);

        fprintf('Loaded %s\n', varName);

    else
        warning('Filename "%s" does not match expected pattern.', fileName);
    end
end
