function output_struct = identify_turns(cleaned_struct, turn_thresh, turn_interv, turn_tol, walk_thresh, troubleshoot)

%IDENTIFY_TURNS detects potential turns using Rover heading signal. 
%
%   output_struct = IDENTIFY_TURNS(cleaned_struct, turn_thresh, turn_interv, turn_tol)
%
%   Inputs:
%       cleaned_struct - Struct containing cleaned Rover data
%       turn_thresh    - Threshold for angular velocity to identify a turn
%       turn_interv    - How long should potential turn periods be (in ms)
%       turn_tol       - How much leeway is there for a period to be
%       considered a turn (distance from local angular velocity max/min in
%       ms)
%       walk_thresh    - Threshold for heading variance to identify
%       straight walking
%       troubleshoot - Set to 1 if there are issues and some extra figures
%       will pop up (i.e. all local min/max, heading variance)
%
%   Output:
%       output_struct - Struct containing potential turn periods and annotations
%
%   This function identifies potential turns vs straight walking periods by
%   calculating angular velocity and identifying local max/min above or
%   below a certain threshold. 
%
%   Example:
%       output_struct = identify_turns(cleaned_struct, 0.1, 1000, 200, 500, 0);
%

deriv = diff(cleaned_struct.l_gait.Heading_Smooth);
deriv = [deriv(1); deriv];

[peaks, peak_locs] = findpeaks(deriv,'MinPeakHeight',turn_thresh);
peak_locs = peak_locs - cleaned_struct.delay/1000;

[troughs, trough_locs] = findpeaks(-1*deriv,'MinPeakHeight',turn_thresh);
trough_locs = trough_locs - cleaned_struct.delay/1000;

peaktroughs = [peaks; -1*troughs];
peaktrough_locs = [peak_locs; trough_locs];

[peaktrough_locs, sort_idx] = sort(peaktrough_locs);
peaktroughs = peaktroughs(sort_idx);

if troubleshoot == 1
    figure()
    ax(1) = subplot(311);
    plot(cleaned_struct.l_rover.NewTime, cleaned_struct.l_rover.LinearAccelX,'Color','black');
    xlabel('Time (ms)')
    ylabel('Acceleration (a.u.)')
    title('Original Left Rover Acceleration Signal')
    ax(2) = subplot(312);
    plot(cleaned_struct.l_gait.NewTime,cleaned_struct.l_gait.Heading_Interp,Color='black');
    hold on
    plot(cleaned_struct.l_gait.NewTime - cleaned_struct.delay/cleaned_struct.smooth_interv,cleaned_struct.l_gait.Heading_Smooth,Color='blue');
    ylabel('Heading (°)')
    xlabel('Time (ms)')
    title(sprintf('Left Rover Heading Signal with Interpolation and %.2fs Smoothing', cleaned_struct.smooth_interv/1000))
    linkaxes(ax, 'x')
    xlim([0 inf]);
    ax(3) = subplot(313);
    plot(cleaned_struct.l_gait.NewTime - cleaned_struct.delay/cleaned_struct.smooth_interv,deriv,Color='red');
    hold on 
    yline(turn_thresh,'k:',LineWidth=2)
    yline(-1*turn_thresh,'k:',LineWidth=2)
    plot(peaktrough_locs, peaktroughs, 'k.',MarkerSize=10)
    ylabel('Angular Velocity (°/ms)')
    xlabel('Time (ms)')
    title('Left Rover Angular Velocity')
    linkaxes(ax, 'x')
    xlim([0 inf]);
end

%% Reduce clustered local angular velocity minima or maxima to single representative values (turns)

cluster_thresh = 500; % how close do local min/max indices have to be in order to be considered a cluster 

peaktrough_diff = diff(peaktrough_locs);
new_cluster = [true; peaktrough_diff > cluster_thresh];
cluster_id = cumsum(new_cluster);
peaktroughs_meds = zeros(max(cluster_id),1);
peaktrough_locs_meds = zeros(max(cluster_id),1);
for k = 1:max(cluster_id)
    cluster_members = peaktrough_locs(cluster_id == k);
    peaktrough_locs_meds(k) = round(median(cluster_members));
    [~, idx] = min(abs(cleaned_struct.l_gait.NewTime - cleaned_struct.delay/cleaned_struct.smooth_interv - round(median(cluster_members))));
    peaktroughs_meds(k) = deriv(idx);
end

if troubleshoot == 1
    figure()
    ax(1) = subplot(311);
    plot(cleaned_struct.l_rover.NewTime, cleaned_struct.l_rover.LinearAccelX,'Color','black');
    xlabel('Time (ms)')
    ylabel('Acceleration (a.u.)')
    title('Original Left Rover Acceleration Signal')
    ax(2) = subplot(312);
    plot(cleaned_struct.l_gait.NewTime,cleaned_struct.l_gait.Heading_Interp,Color='black');
    hold on
    plot(cleaned_struct.l_gait.NewTime - cleaned_struct.delay/cleaned_struct.smooth_interv,cleaned_struct.l_gait.Heading_Smooth,Color='blue');
    ylabel('Heading (°)')
    xlabel('Time (ms)')
    title(sprintf('Left Rover Heading Signal with Interpolation and %.2fs Smoothing', cleaned_struct.smooth_interv/1000))
    linkaxes(ax, 'x')
    xlim([0 inf]);
    ax(3) = subplot(313);
    plot(cleaned_struct.l_gait.NewTime - cleaned_struct.delay/cleaned_struct.smooth_interv,deriv,Color='red');
    hold on 
    yline(turn_thresh,'k:',LineWidth=2)
    yline(-1*turn_thresh,'k:',LineWidth=2)
    plot(peaktrough_locs_meds, peaktroughs_meds, 'k.',MarkerSize=10)
    ylabel('Angular Velocity (°/ms)')
    xlabel('Time (ms)')
    title('Left Rover Angular Velocity - Clusters Reduced to Representative Values')
    linkaxes(ax, 'x')
    xlim([0 inf]);
end

%% Set up table to store movement state data and label potential turns 

Start = (0:turn_interv:max(cleaned_struct.l_gait.NewTime))';
Start = Start(1:end-1,1);
End = (turn_interv:turn_interv:max(cleaned_struct.l_gait.NewTime))';
Turn = zeros(size(Start));
Heading_Var = zeros(size(Start));
Heading_Mean = zeros(size(Start));
turn_table = table(Start, End, Turn, Heading_Var, Heading_Mean, 'VariableNames', {'Start', 'End', 'Turn', 'Heading_Var', 'Heading_Mean'});

for i = 1:size(turn_table,1)
    turn_count = sum((peaktrough_locs_meds >= (turn_table.Start(i) - turn_tol) & (peaktrough_locs_meds <= (turn_table.End(i) + turn_tol))));
    if turn_count >= 1
        turn_table.Turn(i) = 1;
    end
end

%% Identify periods of low heading variability (straight walking)

for i = 1:size(turn_table,1)
    t1 = turn_table.Start(i);
    t2 = turn_table.End(i);
    [~, idx1] = min(abs((cleaned_struct.l_gait.NewTime - cleaned_struct.delay/cleaned_struct.smooth_interv) - t1));
    [~, idx2] = min(abs((cleaned_struct.l_gait.NewTime - cleaned_struct.delay/cleaned_struct.smooth_interv) - t2));
    turn_table.Heading_Var(i) = var(cleaned_struct.l_gait.Heading_Smooth(idx1:idx2));
    turn_table.Heading_Mean(i) = mean(cleaned_struct.l_gait.Heading_Smooth(idx1:idx2));
end

if troubleshoot == 1
    figure()
    ax(1) = subplot(411);
    plot(cleaned_struct.l_rover.NewTime, cleaned_struct.l_rover.LinearAccelX,'Color','black');
    xlabel('Time (ms)')
    ylabel('Acceleration (a.u.)')
    title('Original Left Rover Acceleration Signal')
    ax(2) = subplot(412);
    plot(cleaned_struct.l_gait.NewTime,cleaned_struct.l_gait.Heading_Interp,Color='black');
    hold on
    plot(cleaned_struct.l_gait.NewTime - cleaned_struct.delay/cleaned_struct.smooth_interv,cleaned_struct.l_gait.Heading_Smooth,Color='blue');
    ylabel('Heading (°)')
    xlabel('Time (ms)')
    title(sprintf('Left Rover Heading Signal with Interpolation and %.2fs Smoothing', cleaned_struct.smooth_interv/1000))
    linkaxes(ax, 'x')
    xlim([0 inf]);
    ax(3) = subplot(413);
    plot(cleaned_struct.l_gait.NewTime - cleaned_struct.delay/cleaned_struct.smooth_interv,deriv,Color='red');
    hold on 
    yline(turn_thresh,'k:',LineWidth=2)
    yline(-1*turn_thresh,'k:',LineWidth=2)
    plot(peaktrough_locs_meds, peaktroughs_meds, 'k.',MarkerSize=10)
    ylabel('Angular Velocity (°/ms)')
    xlabel('Time (ms)')
    title('Left Rover Angular Velocity - Clusters Reduced to Representative Values')
    ax(4) = subplot(414);
    hold on
    for i = 1:height(turn_table)
        if turn_table.Turn(i) == 0
            x = [turn_table.Start(i), turn_table.End(i), turn_table.End(i), turn_table.Start(i)];
            y = [0, 0, turn_table.Heading_Var(i), turn_table.Heading_Var(i)];
            fill(x, y, [0.2 0.6 1], 'EdgeColor', 'k', 'FaceAlpha', 0.7);
        end
    end
    yline(walk_thresh,'k:',LineWidth=2)
    xlabel('Time (ms)');
    ylabel('Heading Variance');
    title('Heading Variance for Each Non-Turn Interval');
    linkaxes(ax, 'x')
    xlim([0 inf]);
end

for i = 1:height(turn_table)
    if turn_table.Turn(i) == 0
            if turn_table.Heading_Mean(i) ~= 0 & turn_table.Heading_Var(i) < walk_thresh
                turn_table.Turn(i) = 0;
            else
                turn_table.Turn(i) = 2;
            end
    end
end
            
%% Visualize potential periods of turns vs straight walking 

figure()
ax(1) = subplot(311);
plot(cleaned_struct.l_rover.NewTime, cleaned_struct.l_rover.LinearAccelX,'Color','black',LineWidth=1);
hold on
xlabel('Time (ms)')
ylabel('Acceleration (a.u.)')
title('Original Left Rover Acceleration Signal')
ax(2) = subplot(312);
plot(cleaned_struct.l_gait.NewTime,cleaned_struct.l_gait.Heading_Interp,Color='black',LineWidth=1);
hold on
plot(cleaned_struct.l_gait.NewTime - cleaned_struct.delay/cleaned_struct.smooth_interv,cleaned_struct.l_gait.Heading_Smooth,Color='blue',LineWidth=1);
ylabel('Heading (°)')
xlabel('Time (ms)')
title(sprintf('Left Rover Heading Signal with Interpolation and %.2fs Smoothing', cleaned_struct.smooth_interv/1000))
linkaxes(ax, 'x')
ax(3) = subplot(313);
plot(cleaned_struct.l_gait.NewTime - cleaned_struct.delay/cleaned_struct.smooth_interv,deriv,Color='red',LineWidth=1);
hold on 
plot(peaktrough_locs_meds, peaktroughs_meds, 'k.',MarkerSize=10)
ylabel('Angular Velocity (°/ms)')
xlabel('Time (ms)')
title(sprintf('Left Rover Angular Velocity - Potential Turns with %.2fs Tolerance', turn_tol/1000))
linkaxes(ax, 'x')
for i = 1:height(turn_table)
    if turn_table.Turn(i) == 1
        t1 = turn_table.Start(i);
        t2 = turn_table.End(i);
        for j = 1:3
            subplot(3,1,j)
            yl = ylim;  
            fill([t1 t2 t2 t1], [yl(1) yl(1) yl(2) yl(2)], ...
                [147 197 114]/255, ...       
                'EdgeColor', 'none', ...
                'FaceAlpha', 0.5);       
        end
    elseif turn_table.Turn(i) == 0
        t1 = turn_table.Start(i);
        t2 = turn_table.End(i);
        for j = 1:3
            subplot(3,1,j)
            yl = ylim;  
            fill([t1 t2 t2 t1], [yl(1) yl(1) yl(2) yl(2)], ...
                [220 20 60]/255, ...       
                'EdgeColor', 'none', ...
                'FaceAlpha', 0.2);       
        end
    end
end
xlim([0 inf]);

fprintf('%d potential turns were identified\n', max(cluster_id));

output_struct = cleaned_struct; 
output_struct.turn_table = turn_table; 

end