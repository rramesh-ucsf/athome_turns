function updated_test = turn_gui(test)

%TURN_GUI allows manual review of potential turns and straight walking 
%
%   turn_gui(test)
%
%   Input:
%       test - Structure containing turn_table, which has identified turns and straight walking
%
%   Clickable colored patches represent turn states:
%     - Red:   Turn = 0 = Straight Walking
%     - Green: Turn = 1 = Turning
%     - White: Turn = 2 = Excluded (e.g., non-walking) 
%
%   Clicking a patch cycles its turn state: 0 → 1 → 2 → 0

    global test_global;
    test_global = test;

    fig = figure('Name', 'Turn GUI', 'NumberTitle', 'off');
    plot(test.l_rover.NewTime, test.l_rover.LinearAccelX, 'Color', 'black');
    xlabel('Time (ms)');
    ylabel('Acceleration (a.u.)');
    title('Left Rover Acceleration Signal');
    hold on;

    % Y limits for patch rectangles
    y = min(test.l_rover.LinearAccelX);
    height = max(test.l_rover.LinearAccelX) - y;

    % Store struct in a persistent variable for callback access
    setappdata(fig, 'test_struct', test);

    for i = 1:size(test.turn_table.Turn,1)
        x_start = test.turn_table.Start(i);
        x_end   = test.turn_table.End(i);

        gait_val = test.turn_table.Turn(i);
        fc = gait_color(gait_val);

        % Define patch
        x_patch = [x_start, x_end, x_end, x_start];
        y_patch = [y, y, y + height, y + height];

        patch(x_patch, y_patch, fc, ...
            'EdgeColor', 'none', ...
            'FaceAlpha', 0.1, ...
            'UserData', i, ...
            'ButtonDownFcn', @toggleGaitPatch);
    end

    uicontrol('Style', 'pushbutton', ...
          'String', 'Save and Close', ...
          'Units', 'normalized', ...
          'Position', [0.4 0.01 0.2 0.05], ...
          'Callback', @(~,~) saveAndClose(fig));
    
    uiwait(fig);
    updated_test = test_global;

end

function toggleGaitPatch(src, ~)

    global test_global

    i = src.UserData;
    current = test_global.turn_table.Turn(i);
    new = mod(current + 1, 3);  % Cycle 0→1→2→0

    fc = gait_color(new);
    set(src, 'FaceColor', fc);

    % Update turn state value in struct
    turn_table = test_global.turn_table; 
    turn_table.Turn(i) = new;           
    test_global.turn_table = turn_table;       

    %fprintf('Period %d updated to turn value %d\n', i, new);
    if new == 0
        fprintf('Period %d updated to straight walking\n', i);
    elseif new == 1
        fprintf('Period %d updated to turning\n', i);
    else
        fprintf('Period %d updated to non-walking/excluded\n', i);
    end
end

function fc = gait_color(val)
    % Return RGB color based on gait value
    switch val
        case 0
            fc = [1, 0, 0];   % red
        case 1
            fc = [0, 1, 0];   % green
        case 2
            fc = [1, 1, 1];   % white
        otherwise
            fc = [0.5, 0.5, 0.5]; % gray fallback
    end
end

function saveAndClose(fig)
    uiresume(fig);
    close(fig);
end

