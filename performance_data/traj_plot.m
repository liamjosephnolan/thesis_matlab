% --- User Adjustable Parameters ---
% Define the filename
clear all
close all
clc
filename = 'traj_3.csv';

% Define the column indices for X, Y, and Z positions within the CSV file.
% Based on your provided CSV structure:
% Column 2 for X, Column 3 for Y, Column 4 for Z
x_col = 2;
y_col = 3;
z_col = 4;

% Define the start and end row indices for plotting.
start_index = 1;
end_index = inf; 
% --- End of User Adjustable Parameters ---


% Read the data from the CSV file.
try
    % Open the file
    fileID = fopen(filename, 'r');
    if fileID == -1
        error('Failed to open the file. Please ensure heck.csv is in the current directory.');
    end

    % Skip the header line
    fgetl(fileID); 

    % Dynamically build the format string based on column indices
    % *** THIS IS THE KEY CHANGE ***
    % Based on your CSV example, there are 4 columns: __time, x, y, z
    num_columns_in_file = 4; 
    
    format_string = '';
    for i = 1:num_columns_in_file
        if i == x_col || i == y_col || i == z_col
            format_string = [format_string, '%f ']; % Read X, Y, or Z as float
        else
            format_string = [format_string, '%*f ']; % Skip other columns (e.g., __time) as float
                                                     % Changed to %*f as your __time column is numeric
        end
    end
    format_string = strtrim(format_string); % Remove trailing space
    
    % Read the data using textscan
    data = textscan(fileID, format_string, 'Delimiter', ',', 'CollectOutput', true);
    fclose(fileID);

    % The data is returned as a cell array, extract the numeric array
    positions = data{1};

    % The rest of your script for extracting and plotting is correct after this fix.
    % It correctly maps the read columns to X, Y, Z based on their relative order.
    
    % Create a mapping from original column index to the new 'positions' array index
    original_cols = sort([x_col, y_col, z_col]);
    
    % Determine the actual column index in 'positions' for X, Y, Z
    x_pos_idx = find(original_cols == x_col);
    y_pos_idx = find(original_cols == y_col);
    z_pos_idx = find(original_cols == z_col);

    X_full = positions(:, x_pos_idx);
    Y_full = positions(:, y_pos_idx);
    Z_full = positions(:, z_pos_idx);

    % Apply start and end indices
    total_rows = length(X_full);
    
    % Adjust end_index if it's Inf or greater than total_rows
    if isinf(end_index) || end_index > total_rows
        end_index = total_rows;
    end

    % Validate start_index
    if start_index < 1
        warning('start_index adjusted to 1 as it was less than 1.');
        start_index = 1;
    end

    % Ensure start_index does not exceed end_index
    if start_index > end_index
        error('Start index (%d) cannot be greater than the end index (%d).', start_index, end_index);
    end
    
    % Select the relevant data segment
    X = X_full(start_index:end_index);
    Y = Y_full(start_index:end_index);
    Z = Z_full(start_index:end_index);

    % Create the 3D plot
    figure; % Opens a new figure window
    plot3(X, Y, Z, '-b', 'LineWidth', 1.5); % Plots the trajectory as a blue line
    grid on; % Adds a grid to the plot

    % Add labels and title
    xlabel('X Position');
    ylabel('Y Position');
    zlabel('Z Position');
    title(sprintf('3D Trajectory Plot from %s (Rows %d to %d)', filename, start_index, end_index));

    % Customize the view
    view(3); % Sets the 3D view (azimuth, elevation)
    axis equal; % Ensures equal scaling for all axes

catch ME
    fprintf('An error occurred: %s\n', ME.message);
    fprintf('Please ensure the file "%s" exists, the column indices are correct, and the specified row range is valid.\n', filename);
end