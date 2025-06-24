figure();
plot(test.l_rover.NewTime, test.l_rover.LinearAccelX, 'Color', 'black');
xlabel('Time (ms)');
ylabel('Acceleration (a.u.)');
title('Left Rover Acceleration Signal');
hold on;

% Y limits for patch rectangles
y = min(test.l_rover.LinearAccelX);
height = max(test.l_rover.LinearAccelX) - y;

global test_global;
test_global = test;

for i = 1:size(test.l_gait_periods, 1)
    x_start = test.l_gait_periods.NewTime_start(i);
    x_end = test.l_gait_periods.NewTime_end(i);

    % Determine color based on initial Gait value
    gait_val = test.l_gait_periods.Gait(i);
    switch gait_val
        case 0
            fc = [1, 0, 0]; % red
        case 1
            fc = [0, 1, 0]; % green
        case 2
            fc = [1, 1, 1]; % white
        otherwise
            fc = [0.5, 0.5, 0.5]; % fallback gray
    end

    % Define vertices for patch
    x_patch = [x_start, x_end, x_end, x_start];
    y_patch = [y, y, y + height, y + height];

    % Create patch and assign callback
    p = patch(x_patch, y_patch, fc, ...
        'EdgeColor', 'none', ...
        'FaceAlpha', 0.1, ...
        'UserData', i, ...
        'ButtonDownFcn', @toggleGaitPatch);
end

% Callback function
function toggleGaitPatch(src, ~)
    global test_global;

    i = src.UserData; % Get row index
    current = test_global.l_gait_periods.Gait(i);
    new = mod(current + 1, 3); % Cycle 0 → 1 → 2 → 0

    % Update color
    switch new
        case 0
            fc = [1, 0, 0]; % red
        case 1
            fc = [0, 1, 0]; % green
        case 2
            fc = [1, 1, 1]; % white
    end

    set(src, 'FaceColor', fc);

    % Update data
    test_global.l_gait_periods.Gait(i) = new;
    fprintf('Patch %d updated to gait value %d\n', i, new);
end
