function post_align_struct = chunk(post_align_struct, period, tolerance, fig)

% FOR RCS_02, REACTIVATE THE Z_SCORE SECTIONS
% FOR RCS_05, DEACTIVATE THE Z_SCORE SECTIONS
% FOR RCS_03, HAVE TO GO WITH METHOD 2 FOR OVERALL GAIT AND NEED TO LOOSEN RESTRICTIONS (SWITCH RCS LOSS TESTER TO 0.5, COMMENT OUT STRICT RESTRICTIONS IN OVERALL GAIT, AND SWITCH TO METHOD 2) 

post_align_struct.period = period; 

% chunk left leg data

ms = period*1000;
num_periods = ceil(post_align_struct.l_gait.NewTime(end)/ms);

post_align_struct.l_gait_periods = table();
post_align_struct.l_gait_periods = addvars(post_align_struct.l_gait_periods,[0:ms:ms*(num_periods-1)]','NewVariableNames','NewTime_start');
post_align_struct.l_gait_periods = addvars(post_align_struct.l_gait_periods,[ms:ms:(ms*num_periods)]','NewVariableNames','NewTime_end');
post_align_struct.l_gait_periods.NewTime_end(end) = post_align_struct.l_gait.NewTime(end);
post_align_struct.l_gait_periods = addvars(post_align_struct.l_gait_periods,zeros(num_periods,1),'NewVariableNames','Gait');

rover_loss_store = [];
rcs_loss_store = [];
lfp_loss_store = [];
chunk_mean_store = [];

for i = 1:num_periods
    start_idx = double(post_align_struct.l_gait_periods.NewTime_start(i) + 1);
    end_idx = double(post_align_struct.l_gait_periods.NewTime_end(i) + 1);
    if end_idx > size(post_align_struct.l_gait.Gait,1)
        end_idx = size(post_align_struct.l_gait.Gait,1);
        post_align_struct.l_gait_periods.NewTime_end(i) = end_idx;
    end

    % check if the period is mostly walking or nonwalking 
    if sum(post_align_struct.l_gait.Gait(start_idx:end_idx)) >= 0.5*sum(ones(end_idx - start_idx + 1,1))
        post_align_struct.l_gait_periods.Gait(i) = 1;
    elseif sum(post_align_struct.l_gait.Gait(start_idx:end_idx)) <= (1-tolerance)*(end_idx - start_idx + 1)
        post_align_struct.l_gait_periods.Gait(i) = 0;
    else
        post_align_struct.l_gait_periods.Gait(i) = 3;
    end

    
    [~,loss_test_start_idx] = min(abs(post_align_struct.l_rover.NewTime - start_idx));
    [~,loss_test_end_idx] = min(abs(post_align_struct.l_rover.NewTime - end_idx));
    loss_tester = loss_test_end_idx - loss_test_start_idx;
    rover_loss_store = [rover_loss_store; loss_tester];

    % REACTIVATE THE FOLLOWING TO EXCLUDE ROVER LOSS
    %if loss_tester < 0.99*(ms/10)
    %    post_align_struct.l_gait_periods.Gait(i) = 2;
    %end

    % this section checks to see whether RCS has a lot of signal loss
    % within that period 
    [~,rcs_loss_test_start_idx] = min(abs(post_align_struct.l_rcs_accel.NewTime - start_idx));
    [~,rcs_loss_test_end_idx] = min(abs(post_align_struct.l_rcs_accel.NewTime - end_idx));
    rcs_loss_tester = rcs_loss_test_end_idx - rcs_loss_test_start_idx;
    rcs_loss_store = [rcs_loss_store; rcs_loss_tester];
    chunk_mean = mean(post_align_struct.l_rcs_accel.XSamples(rcs_loss_test_start_idx:rcs_loss_test_end_idx));
    chunk_mean_store = [chunk_mean_store; chunk_mean];
    if rcs_loss_tester < 0.5*65*period
        post_align_struct.l_gait_periods.Gait(i) = 2;
    end
%     [~,lfp_loss_test_start_idx] = min(abs(post_align_struct.l_rcs_lfp.NewTime - start_idx));
%     [~,lfp_loss_test_end_idx] = min(abs(post_align_struct.l_rcs_lfp.NewTime - end_idx));
%     lfp_loss_tester = lfp_loss_test_end_idx - lfp_loss_test_start_idx;
%     lfp_loss_store = [lfp_loss_store; lfp_loss_tester];
%     if lfp_loss_tester < 0.99*500*period
%         post_align_struct.l_gait_periods.Gait(i) = 2;
%     end
    %post_align_struct.l_gait_periods.Gait(i) = round(mean(post_align_struct.l_gait.Gait(start_idx:end_idx)));
end

% if post_align_struct.l_gait_periods.Gait(1) == 0
%     post_align_struct.l_gait_periods.Gait(1) = 2;
% end
% 
% for i = 2:size(post_align_struct.l_gait_periods.Gait,1)-1
%     if post_align_struct.l_gait_periods.Gait(i) == 1
%         if post_align_struct.l_gait_periods.Gait(i-1) == 0 || post_align_struct.l_gait_periods.Gait(i+1) == 2
%             if post_align_struct.l_gait_periods.Gait(i+1) == 2 || post_align_struct.l_gait_periods.Gait(i+1) == 0
%                 post_align_struct.l_gait_periods.Gait(i) = 2;
%             end
%         end
%     end
% end
% 

% z_scored_chunk_means = zscore(chunk_mean_store);
% for i = 1:length(post_align_struct.l_gait_periods.Gait)
%     if z_scored_chunk_means(i) < -1.5 || z_scored_chunk_means(i) > 1.5 
%         post_align_struct.l_gait_periods.Gait(i) = 2;
%     end
% end

% Find indices of zero elements
zero_indices = find(post_align_struct.l_gait_periods.Gait == 0);

% Check each zero sequence
for i = 1:length(zero_indices)
    start_idx = zero_indices(i); % Starting index of the zero sequence
    end_idx = start_idx; % Ending index of the zero sequence
    
    % Find the ending index of the zero sequence
    while end_idx < length(post_align_struct.l_gait_periods.Gait) && post_align_struct.l_gait_periods.Gait(end_idx + 1) == 0
        end_idx = end_idx + 1;
    end
    
    % Check if the zero sequence is bordered by two consecutive 2's
    if start_idx > 2 && end_idx < length(post_align_struct.l_gait_periods.Gait) - 1
        if post_align_struct.l_gait_periods.Gait(start_idx - 2) == 2 && post_align_struct.l_gait_periods.Gait(start_idx - 1) == 2 && ...
           post_align_struct.l_gait_periods.Gait(end_idx + 1) == 2 && post_align_struct.l_gait_periods.Gait(end_idx + 2) == 2
            post_align_struct.l_gait_periods.Gait(start_idx:end_idx) = 2; % Replace the zero sequence with 2's
        end
    elseif start_idx > 2 && end_idx >= length(post_align_struct.l_gait_periods.Gait) - 1
        if post_align_struct.l_gait_periods.Gait(start_idx - 2) == 2 && post_align_struct.l_gait_periods.Gait(start_idx - 1) == 2
            post_align_struct.l_gait_periods.Gait(start_idx:end_idx) = 2;
        end
    end
end

if fig == 1 || nargin == 2
    figure()
    ax(1) = subplot(411);
    scatter(post_align_struct.l_rcs_accel.NewTime,post_align_struct.l_rcs_accel.XSamples,1)
    title('L RC+S Accelerometry')
    hold on 
    for i = 1:num_periods
        x = post_align_struct.l_gait_periods.NewTime_start(i);
        y = mean(post_align_struct.l_rcs_accel.XSamples) - 0.5*max(abs(post_align_struct.l_rcs_accel.XSamples));
        width = post_align_struct.l_gait_periods.NewTime_end(i)-post_align_struct.l_gait_periods.NewTime_start(i);
        height = max(abs(post_align_struct.l_rcs_accel.XSamples));
        if post_align_struct.l_gait_periods.Gait(i) == 1
            rectangle('Position',[x,y,width,height],'FaceColor',[0, 1, 0, 0.1],'EdgeColor','none');
        elseif post_align_struct.l_gait_periods.Gait(i) == 0
            rectangle('Position',[x,y,width,height],'FaceColor',[1, 0, 0, 0.1],'EdgeColor','none');
        end
    end
    hold off
    ax(2) = subplot(412);
    scatter(post_align_struct.l_rover.NewTime,post_align_struct.l_rover.LinearAccelX,1)
    title('L Rover Accelerometry')
    hold on
    for i = 1:num_periods
        x = post_align_struct.l_gait_periods.NewTime_start(i);
        y = 0 - max(post_align_struct.l_rover.LinearAccelX);
        width = post_align_struct.l_gait_periods.NewTime_end(i)-post_align_struct.l_gait_periods.NewTime_start(i);
        height = 2.1*max(post_align_struct.l_rover.LinearAccelX);
        if post_align_struct.l_gait_periods.Gait(i) == 1
            rectangle('Position',[x,y,width,height],'FaceColor',[0, 1, 0, 0.1],'EdgeColor','none');
        elseif post_align_struct.l_gait_periods.Gait(i) == 0
            rectangle('Position',[x,y,width,height],'FaceColor',[1, 0, 0, 0.1],'EdgeColor','none');
        end
    end
    hold off;
    ax(3) = subplot(413);
    scatter(post_align_struct.l_rcs_lfp.NewTime,post_align_struct.l_rcs_lfp.key1,1)
    title('L RCS Key 1 LFP')
    hold on
    for i = 1:num_periods
        x = post_align_struct.l_gait_periods.NewTime_start(i);
        y = 0 - max(post_align_struct.l_rover.LinearAccelX);
        width = post_align_struct.l_gait_periods.NewTime_end(i)-post_align_struct.l_gait_periods.NewTime_start(i);
        height = 2.1*max(post_align_struct.l_rover.LinearAccelX);
        if post_align_struct.l_gait_periods.Gait(i) == 1
            rectangle('Position',[x,y,width,height],'FaceColor',[0, 1, 0, 0.1],'EdgeColor','none');
        elseif post_align_struct.l_gait_periods.Gait(i) == 0
            rectangle('Position',[x,y,width,height],'FaceColor',[1, 0, 0, 0.1],'EdgeColor','none');
        end
    end
    ylim([-0.3 0.3])
    ax(4) = subplot(414);
    title('Signal Issue Identification')
    hold on
    z_scored_chunk_means = zscore(chunk_mean_store);
    for i = 1:num_periods
        x = post_align_struct.l_gait_periods.NewTime_start(i);
        y = 1;
        width = post_align_struct.l_gait_periods.NewTime_end(i)-post_align_struct.l_gait_periods.NewTime_start(i);
        height = 1;
        if rcs_loss_store(i)< 0.95*max(rcs_loss_store) || rover_loss_store(i)< 0.95*max(rover_loss_store)  % || lfp_loss_store(i) < 0.95*max(lfp_loss_store)
            rectangle('Position',[x,y,width,height],'FaceColor',[1, 0, 0, 1],'EdgeColor','none');
        end
        if z_scored_chunk_means(i) > 1.5 || z_scored_chunk_means(i) < -1.5
            rectangle('Position',[x,y,width,height],'FaceColor',[0, 0, 1, 1],'EdgeColor','none');
        end
    end
    hold off;
    linkaxes(ax,'x');
end

%% repeat for right data
ms = period*1000;
num_periods = ceil(post_align_struct.r_gait.NewTime(end)/ms);

post_align_struct.r_gait_periods = table();
post_align_struct.r_gait_periods = addvars(post_align_struct.r_gait_periods,[0:ms:ms*(num_periods-1)]','NewVariableNames','NewTime_start');
post_align_struct.r_gait_periods = addvars(post_align_struct.r_gait_periods,[ms:ms:(ms*num_periods)]','NewVariableNames','NewTime_end');
post_align_struct.r_gait_periods.NewTime_end(end) = post_align_struct.r_gait.NewTime(end);
post_align_struct.r_gait_periods = addvars(post_align_struct.r_gait_periods,zeros(num_periods,1),'NewVariableNames','Gait');

if isfield(post_align_struct,'r_rcs_accel')
    rcs_loss_store = [];
    lfp_loss_store = [];
    chunk_mean_store = [];
end

for i = 1:num_periods
    start_idx = double(post_align_struct.r_gait_periods.NewTime_start(i) + 1);
    end_idx = double(post_align_struct.r_gait_periods.NewTime_end(i) + 1);
    if end_idx > size(post_align_struct.r_gait.Gait,1)
        end_idx = size(post_align_struct.r_gait.Gait,1);
        post_align_struct.r_gait_periods.NewTime_end(i) = end_idx;
    end
    if sum(post_align_struct.r_gait.Gait(start_idx:end_idx)) >= 0.5*sum(ones(end_idx - start_idx + 1,1))
        post_align_struct.r_gait_periods.Gait(i) = 1;
    elseif sum(post_align_struct.r_gait.Gait(start_idx:end_idx)) <= (1-tolerance)*(end_idx - start_idx + 1)
        post_align_struct.r_gait_periods.Gait(i) = 0;
    else
        post_align_struct.r_gait_periods.Gait(i) = 3;
    end
    [~,loss_test_start_idx] = min(abs(post_align_struct.r_rover.NewTime - start_idx));
    [~,loss_test_end_idx] = min(abs(post_align_struct.r_rover.NewTime - end_idx));
    loss_tester = loss_test_end_idx - loss_test_start_idx;
    % REACTIVATE THE FOLLOWING TO EXCLUDE ROVER LOSS
    %if loss_tester < 0.99*(ms/10)
    %    post_align_struct.r_gait_periods.Gait(i) = 2;
    %end
    if isfield(post_align_struct,'r_rcs_accel')
        [~,rcs_loss_test_start_idx] = min(abs(post_align_struct.r_rcs_accel.NewTime - start_idx));
        [~,rcs_loss_test_end_idx] = min(abs(post_align_struct.r_rcs_accel.NewTime - end_idx));
        rcs_loss_tester = rcs_loss_test_end_idx - rcs_loss_test_start_idx;
        rcs_loss_store = [rcs_loss_store; rcs_loss_tester];
        chunk_mean = mean(post_align_struct.r_rcs_accel.XSamples(rcs_loss_test_start_idx:rcs_loss_test_end_idx));
        chunk_mean_store = [chunk_mean_store; chunk_mean];
    else
        [~,rcs_loss_test_start_idx] = min(abs(post_align_struct.l_rcs_accel.NewTime - start_idx));
        [~,rcs_loss_test_end_idx] = min(abs(post_align_struct.l_rcs_accel.NewTime - end_idx));
        rcs_loss_tester = rcs_loss_test_end_idx - rcs_loss_test_start_idx;
    end
    if rcs_loss_tester < 0.5*65*period
        post_align_struct.r_gait_periods.Gait(i) = 2;
    end
%     [~,lfp_loss_test_start_idx] = min(abs(post_align_struct.l_rcs_lfp.NewTime - start_idx));
%     [~,lfp_loss_test_end_idx] = min(abs(post_align_struct.l_rcs_lfp.NewTime - end_idx));
%     lfp_loss_tester = lfp_loss_test_end_idx - lfp_loss_test_start_idx;
%     lfp_loss_store = [lfp_loss_store; lfp_loss_tester];
%     if lfp_loss_tester < 0.99*500*period
%         post_align_struct.r_gait_periods.Gait(i) = 2;
%     end
    %post_align_struct.r_gait_periods.Gait(i) = round(mean(post_align_struct.r_gait.Gait(start_idx:end_idx)));
end

% if post_align_struct.r_gait_periods.Gait(1) == 0
%     post_align_struct.r_gait_periods.Gait(1) = 2;
% end
% 
% for i = 2:size(post_align_struct.r_gait_periods.Gait,1)-1
%     if post_align_struct.r_gait_periods.Gait(i) == 1
%         if post_align_struct.r_gait_periods.Gait(i-1) == 0 || post_align_struct.r_gait_periods.Gait(i+1) == 2
%             if post_align_struct.r_gait_periods.Gait(i+1) == 2 || post_align_struct.r_gait_periods.Gait(i+1) == 0
%                 post_align_struct.r_gait_periods.Gait(i) = 2;
%             end
%         end
%     end
% end
% 
% for i = 2:size(post_align_struct.r_gait_periods.Gait,1)-1
%     if post_align_struct.r_gait_periods.Gait(i) == 0
%         if post_align_struct.r_gait_periods.Gait(i-1) == 2
%             if post_align_struct.r_gait_periods.Gait(i+1) == 2 || post_align_struct.r_gait_periods.Gait(i+1) == 0
%                 post_align_struct.r_gait_periods.Gait(i) = 2;
%             end
%         end
%     end
% end
% 
% for i = 1:size(post_align_struct.r_gait_periods.Gait,1)
%     if post_align_struct.r_gait_periods.Gait(i) == 2
%         for j = (1+i):size(post_align_struct.r_gait_periods.Gait,1)
%             if post_align_struct.r_gait_periods.Gait(j) == 0 || post_align_struct.r_gait_periods.Gait(j) == 1
%                 continue
%             else
%                 complete_size = j-i;
%                 if complete_size < round(60/period)
%                     post_align_struct.r_gait_periods.Gait(i:j) = 2;
%                 end
%             end
%         end
%     end
% end

% z_scored_chunk_means = zscore(chunk_mean_store);
% for i = 1:(min(length(post_align_struct.r_gait_periods.Gait),length(z_scored_chunk_means)))
%     if z_scored_chunk_means(i) < -1.5 || z_scored_chunk_means(i) > 1.5 
%         post_align_struct.r_gait_periods.Gait(i) = 2;
%     end
% end

% Find indices of zero elements
zero_indices = find(post_align_struct.r_gait_periods.Gait == 0);

% Check each zero sequence
for i = 1:length(zero_indices)
    start_idx = zero_indices(i); % Starting index of the zero sequence
    end_idx = start_idx; % Ending index of the zero sequence
    
    % Find the ending index of the zero sequence
    while end_idx < length(post_align_struct.r_gait_periods.Gait) && post_align_struct.r_gait_periods.Gait(end_idx + 1) == 0
        end_idx = end_idx + 1;
    end
    
    % Check if the zero sequence is bordered by two consecutive 2's
    if start_idx > 2 && end_idx < length(post_align_struct.r_gait_periods.Gait) - 1
        if post_align_struct.r_gait_periods.Gait(start_idx - 2) == 2 && post_align_struct.r_gait_periods.Gait(start_idx - 1) == 2 && ...
           post_align_struct.r_gait_periods.Gait(end_idx + 1) == 2 && post_align_struct.r_gait_periods.Gait(end_idx + 2) == 2
            post_align_struct.r_gait_periods.Gait(start_idx:end_idx) = 2; % Replace the zero sequence with 2's
        end
    elseif start_idx > 2 && end_idx >= length(post_align_struct.r_gait_periods.Gait) - 1
        if post_align_struct.r_gait_periods.Gait(start_idx - 2) == 2 && post_align_struct.r_gait_periods.Gait(start_idx - 1) == 2
            post_align_struct.r_gait_periods.Gait(start_idx:end_idx) = 2;
        end
    end
end

%% Plot R figure 
r_num_periods = min(size(post_align_struct.r_gait_periods,1),size(rover_loss_store,1));
if fig == 1 || nargin == 2
    figure()
    ax(1) = subplot(411);
    if isfield(post_align_struct,'r_rcs_accel')
        scatter(post_align_struct.r_rcs_accel.NewTime,post_align_struct.r_rcs_accel.XSamples,1)
        title('R RC+S Accelerometry')
        hold on 
        for i = 1:r_num_periods
            x = post_align_struct.r_gait_periods.NewTime_start(i);
            y = mean(post_align_struct.r_rcs_accel.XSamples) - 0.5*max(abs(post_align_struct.r_rcs_accel.XSamples));
            width = post_align_struct.r_gait_periods.NewTime_end(i)-post_align_struct.r_gait_periods.NewTime_start(i);
            height = max(abs(post_align_struct.r_rcs_accel.XSamples));
            if post_align_struct.r_gait_periods.Gait(i) == 1
                rectangle('Position',[x,y,width,height],'FaceColor',[0, 1, 0, 0.1],'EdgeColor','none');
            elseif post_align_struct.r_gait_periods.Gait(i) == 0
                rectangle('Position',[x,y,width,height],'FaceColor',[1, 0, 0, 0.1],'EdgeColor','none');
            end
        end
        hold off
    else
        scatter(post_align_struct.l_rcs_accel.NewTime,post_align_struct.l_rcs_accel.XSamples,1)
        title('L RC+S Accelerometry')
        hold on 
        for i = 1:r_num_periods
            x = post_align_struct.r_gait_periods.NewTime_start(i);
            y = mean(post_align_struct.l_rcs_accel.XSamples) - 0.5*max(abs(post_align_struct.l_rcs_accel.XSamples));
            width = post_align_struct.r_gait_periods.NewTime_end(i)-post_align_struct.r_gait_periods.NewTime_start(i);
            height = max(abs(post_align_struct.l_rcs_accel.XSamples));
            if post_align_struct.r_gait_periods.Gait(i) == 1
                rectangle('Position',[x,y,width,height],'FaceColor',[0, 1, 0, 0.1],'EdgeColor','none');
            elseif post_align_struct.r_gait_periods.Gait(i) == 0
                rectangle('Position',[x,y,width,height],'FaceColor',[1, 0, 0, 0.1],'EdgeColor','none');
            end
        end
        hold off
    end
    ax(2) = subplot(412);
    scatter(post_align_struct.r_rover.NewTime,post_align_struct.r_rover.LinearAccelX,1)
    title('R Rover Accelerometry')
    hold on
    for i = 1:num_periods
        x = post_align_struct.r_gait_periods.NewTime_start(i);
        y = 0 - max(post_align_struct.r_rover.LinearAccelX);
        width = post_align_struct.r_gait_periods.NewTime_end(i)-post_align_struct.r_gait_periods.NewTime_start(i);
        height = 2.1*max(post_align_struct.r_rover.LinearAccelX);
        if post_align_struct.r_gait_periods.Gait(i) == 1
            rectangle('Position',[x,y,width,height],'FaceColor',[0, 1, 0, 0.1],'EdgeColor','none');
        elseif post_align_struct.r_gait_periods.Gait(i) == 0
            rectangle('Position',[x,y,width,height],'FaceColor',[1, 0, 0, 0.1],'EdgeColor','none');
        end
    end
    hold off;
    ax(3) = subplot(413);
    plot(post_align_struct.l_rcs_lfp.NewTime,post_align_struct.l_rcs_lfp.key2)
    title('L RCS Key 2 LFP')
    hold on
    for i = 1:num_periods
        x = post_align_struct.r_gait_periods.NewTime_start(i);
        y = 0 - max(post_align_struct.r_rover.LinearAccelX);
        width = post_align_struct.r_gait_periods.NewTime_end(i)-post_align_struct.r_gait_periods.NewTime_start(i);
        height = 2.1*max(post_align_struct.r_rover.LinearAccelX);
        if post_align_struct.r_gait_periods.Gait(i) == 1
            rectangle('Position',[x,y,width,height],'FaceColor',[0, 1, 0, 0.1],'EdgeColor','none');
        elseif post_align_struct.r_gait_periods.Gait(i) == 0
            rectangle('Position',[x,y,width,height],'FaceColor',[1, 0, 0, 0.1],'EdgeColor','none');
        end
    end
    ylim([-0.3 0.3])
    ax(4) = subplot(414);
    title('Signal Issue Identification')
    hold on
    z_scored_chunk_means = zscore(chunk_mean_store);
    for i = 1:r_num_periods
        x = post_align_struct.r_gait_periods.NewTime_start(i);
        y = 1;
        width = post_align_struct.r_gait_periods.NewTime_end(i)-post_align_struct.r_gait_periods.NewTime_start(i);
        height = 1;
        if rcs_loss_store(i)< 0.95*max(rcs_loss_store) %|| rover_loss_store(i)< 0.95*max(rover_loss_store) % || lfp_loss_store(i) < 0.95*max(lfp_loss_store)
            rectangle('Position',[x,y,width,height],'FaceColor',[1, 0, 0, 1],'EdgeColor','none');
        end
        if z_scored_chunk_means(i) > 1.5 || z_scored_chunk_means(i) < -1.5
            rectangle('Position',[x,y,width,height],'FaceColor',[0, 0, 1, 1],'EdgeColor','none');
        end
    end
    hold off;
    linkaxes(ax,'x');
end

% plot left and right rover accelerometry 

%new_num_periods = min(size(post_align_struct.l_gait_periods,1),size(post_align_struct.r_gait_periods,1));
% if fig == 1 || nargin == 2
%     figure()
%     ax(1) = subplot(211);
%     plot(post_align_struct.l_rover.NewTime,post_align_struct.l_rover.LinearAccelX)
%     title('L Rover Accelerometry')
%     hold on
%     for i = 1:new_num_periods
%         x = post_align_struct.l_gait_periods.NewTime_start(i);
%         y = 0 - max(post_align_struct.l_rover.LinearAccelX);
%         width = post_align_struct.l_gait_periods.NewTime_end(i)-post_align_struct.l_gait_periods.NewTime_start(i);
%         height = 2.1*max(post_align_struct.l_rover.LinearAccelX);
%         if post_align_struct.l_gait_periods.Gait(i) == 1
%             rectangle('Position',[x,y,width,height],'FaceColor',[0, 1, 0, 0.1],'EdgeColor','none');
%         elseif post_align_struct.l_gait_periods.Gait(i) == 0
%             rectangle('Position',[x,y,width,height],'FaceColor',[1, 0, 0, 0.1],'EdgeColor','none');
%         end
%     end
%     hold off;
%     ax(2) = subplot(212);
%     plot(post_align_struct.r_rover.NewTime,post_align_struct.r_rover.LinearAccelX)
%     title('R Rover Accelerometry')
%     hold on
%     for i = 1:new_num_periods
%         x = post_align_struct.r_gait_periods.NewTime_start(i);
%         y = 0 - max(post_align_struct.r_rover.LinearAccelX);
%         width = post_align_struct.r_gait_periods.NewTime_end(i)-post_align_struct.r_gait_periods.NewTime_start(i);
%         height = 2.1*max(post_align_struct.r_rover.LinearAccelX);
%         if post_align_struct.r_gait_periods.Gait(i) == 1
%             rectangle('Position',[x,y,width,height],'FaceColor',[0, 1, 0, 0.1],'EdgeColor','none');
%         elseif post_align_struct.r_gait_periods.Gait(i) == 0
%             rectangle('Position',[x,y,width,height],'FaceColor',[1, 0, 0, 0.1],'EdgeColor','none');
%         end
%     end
%     hold off;
%     linkaxes(ax,'x');
% end

% Method 1: BOTH L and R have to be gait periods
% if size(post_align_struct.l_gait_periods,1) <= size(post_align_struct.r_gait_periods,1)
%     overall_gait_periods = post_align_struct.l_gait_periods;
% else
%     overall_gait_periods = post_align_struct.r_gait_periods;
% end
% overall_gait_periods.Gait(:) = 0;
% 
% for i = 1:size(overall_gait_periods,1)
%     if post_align_struct.l_gait_periods.Gait(i,1) == 1 & post_align_struct.r_gait_periods.Gait(i,1) == 1
%         overall_gait_periods.Gait(i,1) = 1;
%     elseif post_align_struct.l_gait_periods.Gait(i,1) == 0 & post_align_struct.r_gait_periods.Gait(i,1) == 0
%         overall_gait_periods.Gait(i,1) = 0;
%     else
%         overall_gait_periods.Gait(i,1) = 2;
%     end
% end

% Method 2: EITHER L and R have to be gait periods
if size(post_align_struct.l_gait_periods,1) <= size(post_align_struct.r_gait_periods,1)
    overall_gait_periods = post_align_struct.l_gait_periods;
else
    overall_gait_periods = post_align_struct.r_gait_periods;
end
overall_gait_periods.Gait(:) = 0;

for i = 1:size(overall_gait_periods,1)
    if post_align_struct.l_gait_periods.Gait(i,1) == 1 | post_align_struct.r_gait_periods.Gait(i,1) == 1
        overall_gait_periods.Gait(i,1) = 1;
    elseif post_align_struct.l_gait_periods.Gait(i,1) == 0 & post_align_struct.r_gait_periods.Gait(i,1) == 0
        overall_gait_periods.Gait(i,1) = 0;
    else
        overall_gait_periods.Gait(i,1) = 2;
    end
end

% This portion below is a stricter data loss restriction, but probably can
% be commented out if wanting to go looser

% for i = 2:size(overall_gait_periods,1)-1
%     if overall_gait_periods.Gait(i) == 0 || overall_gait_periods.Gait(i) == 1
%         if overall_gait_periods.Gait(i-1) == 2
%             if overall_gait_periods.Gait(i+1) == 2 
%                 overall_gait_periods.Gait(i) = 2;
%             end
%         end
%     end
% end
% overall_gait_periods.Gait(1:2) = 2;
% overall_gait_periods.Gait(end-2:end) = 2;

if fig == 1
    figure()
    ax(1) = subplot(211);
    scatter(post_align_struct.l_rcs_accel.NewTime,post_align_struct.l_rcs_accel.XSamples,1)
    title('L RCS Accelerometry')
    hold on
    for i = 1:size(overall_gait_periods,1)
        x = overall_gait_periods.NewTime_start(i);
        y = 0 - max(post_align_struct.l_rcs_accel.XSamples);
        width = overall_gait_periods.NewTime_end(i)-overall_gait_periods.NewTime_start(i);
        height = 2.1*max(post_align_struct.l_rcs_accel.XSamples);
        if overall_gait_periods.Gait(i) == 1
                rectangle('Position',[x,y,width,height],'FaceColor',[0, 1, 0, 0.1],'EdgeColor','none');
        elseif overall_gait_periods.Gait(i) == 0
                rectangle('Position',[x,y,width,height],'FaceColor',[1, 0, 0, 0.1],'EdgeColor','none');
        end
    end
    hold off;
    ax(2) = subplot(212);
    scatter(post_align_struct.l_rover.NewTime,post_align_struct.l_rover.LinearAccelX,1)
    %plot(post_align_struct.l_rover.NewTime,post_align_struct.l_rover.LinearAccelX)
    title('L Rover Accelerometry')
    hold on
    for i = 1:size(overall_gait_periods)
        x = overall_gait_periods.NewTime_start(i);
        y = 0 - max(post_align_struct.l_rover.LinearAccelX);
        width = overall_gait_periods.NewTime_end(i)-overall_gait_periods.NewTime_start(i);
        height = 2.1*max(post_align_struct.l_rover.LinearAccelX);
        if overall_gait_periods.Gait(i) == 1
                rectangle('Position',[x,y,width,height],'FaceColor',[0, 1, 0, 0.1],'EdgeColor','none');
        elseif overall_gait_periods.Gait(i) == 0
                rectangle('Position',[x,y,width,height],'FaceColor',[1, 0, 0, 0.1],'EdgeColor','none');
        end
    hold off
    linkaxes(ax,'x');
    end
end

% figure()
% plot(post_align_struct.l_rover.NewTime,post_align_struct.l_rover.LinearAccelX)
% title('Left Rover Accelerometry')
% hold on
% for i = 1:size(overall_gait_periods)
%     x = overall_gait_periods.NewTime_start(i);
%     y = 0 - max(post_align_struct.l_rover.LinearAccelX);
%     width = overall_gait_periods.NewTime_end(i)-overall_gait_periods.NewTime_start(i);
%     height = 2.1*max(post_align_struct.l_rover.LinearAccelX);
%     if overall_gait_periods.Gait(i) == 1
%             rectangle('Position',[x,y,width,height],'FaceColor',[0, 1, 0, 0.1],'EdgeColor','none');
%     elseif overall_gait_periods.Gait(i) == 0
%             rectangle('Position',[x,y,width,height],'FaceColor',[1, 0, 0, 0.1],'EdgeColor','none');
%     end
% xlabel('Time (ms)')
% hold off

post_align_struct.overall_gait_periods = overall_gait_periods;

end
