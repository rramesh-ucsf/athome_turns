function post_align_struct = post_align(file,fig)

load(file)

if isfield(aligned_data,"right_LFP_table") == 1

    % create new time stream where t=0 is start of recordings (alignment point)
    l_rcs_lfp_time = aligned_data.left_LFP_table.DerivedTime - aligned_data.left_LFP_table.DerivedTime(1);
    r_rcs_lfp_time = aligned_data.right_LFP_table.DerivedTime - aligned_data.right_LFP_table.DerivedTime(1);
    l_rcs_accel_time = aligned_data.left_Accel_table.DerivedTime - aligned_data.left_Accel_table.DerivedTime(1);
    r_rcs_accel_time = aligned_data.right_Accel_table.DerivedTime - aligned_data.right_Accel_table.DerivedTime(1);
    l_rover_time = 1000*seconds(aligned_data.Rover.Left.DateTime - aligned_data.Rover.Left.DateTime(1));
    r_rover_time = 1000*seconds(aligned_data.Rover.Right.DateTime - aligned_data.Rover.Right.DateTime(1));

    if l_rcs_lfp_time(end,1) < r_rcs_lfp_time(end,1)
        l_rcs_lfp_crop = size(l_rcs_lfp_time,1);
        end_time = l_rcs_lfp_time(end,1);
        [~, r_rcs_lfp_crop] = min(abs(r_rcs_lfp_time - end_time));
        [~, l_rover_crop] = min(abs(l_rover_time - end_time));
        [~, r_rover_crop] = min(abs(r_rover_time - end_time));
        [~, l_rcs_accel_crop] = min(abs(l_rcs_accel_time - end_time));
        [~, r_rcs_accel_crop] = min(abs(r_rcs_accel_time - end_time));
    else
        r_rcs_lfp_crop = size(r_rcs_lfp_time,1);
        end_time = r_rcs_lfp_time(end,1);
        [~, l_rcs_lfp_crop] = min(abs(l_rcs_lfp_time - end_time));
        [~, l_rover_crop] = min(abs(l_rover_time - end_time));
        [~, r_rover_crop] = min(abs(r_rover_time - end_time));
        [~, l_rcs_accel_crop] = min(abs(l_rcs_accel_time - end_time));
        [~, r_rcs_accel_crop] = min(abs(r_rcs_accel_time - end_time));
    end

    % concatenate signals with the new time stream 
    l_rcs_lfp_sig = [aligned_data.left_LFP_table(1:l_rcs_lfp_crop,:), table(l_rcs_lfp_time(1:l_rcs_lfp_crop),'VariableNames',{'NewTime'})];
    r_rcs_lfp_sig = [aligned_data.right_LFP_table(1:r_rcs_lfp_crop,:), table(r_rcs_lfp_time(1:r_rcs_lfp_crop),'VariableNames',{'NewTime'})];
    l_rover_sig = [aligned_data.Rover.Left(1:l_rover_crop,:), table(l_rover_time(1:l_rover_crop),'VariableNames',{'NewTime'})];
    r_rover_sig = [aligned_data.Rover.Right(1:r_rover_crop,:), table(r_rover_time(1:r_rover_crop),'VariableNames',{'NewTime'})];
    l_rcs_accel_sig = [aligned_data.left_Accel_table(1:l_rcs_accel_crop,:), table(l_rcs_accel_time(1:l_rcs_accel_crop),'VariableNames',{'NewTime'})];
    r_rcs_accel_sig = [aligned_data.right_Accel_table(1:r_rcs_accel_crop,:), table(r_rcs_accel_time(1:r_rcs_accel_crop),'VariableNames',{'NewTime'})];

    % place aligned and cropped signals into a structure post_align_struct
    post_align_struct.l_rcs_accel = l_rcs_accel_sig;
    post_align_struct.r_rcs_accel = r_rcs_accel_sig;
    post_align_struct.l_rcs_lfp = l_rcs_lfp_sig;
    post_align_struct.r_rcs_lfp = r_rcs_lfp_sig;
    post_align_struct.l_rover = l_rover_sig;
    post_align_struct.r_rover = r_rover_sig;

    % add the year of recording and filename for a future step
    
    splitStr_1 = strsplit(file, '/');
    splitStr_2 = strsplit(splitStr_1{end}, '_');
    splitStr_3 = strsplit(splitStr_2{end},'.');
    filename = [splitStr_2{end-1} '_' splitStr_3{1}];
    post_align_struct.filename = filename;

    if contains(file,'21_') == 1
        post_align_struct.year = 2021;
    elseif contains(file,'22_') == 1
        post_align_struct.year = 2022;
    elseif contains(file,'23_') == 1
        post_align_struct.year = 2023;
    else
        post_align_struct.year = 2024;
    end

    % plot the aligned and cropped signals on unified time stream
    if fig == 1 || nargin == 1
        figure() 
        ax(1) = subplot(311);
        plot(l_rcs_accel_sig.NewTime,l_rcs_accel_sig.XSamples)
        title('L RC+S Accelerometry')
        ax(2) = subplot(312);
        plot(l_rover_sig.NewTime, l_rover_sig.LinearAccelX)
        hold on
        plot(r_rover_sig.NewTime, r_rover_sig.LinearAccelX)
        title('Superimposed L and R Rover')
        ax(3) = subplot(313);
        plot(r_rcs_accel_sig.NewTime, r_rcs_accel_sig.XSamples)
        title('R RC+S Accelerometry')
%         ax(4) = subplot(414);
%         if size(l_rover_sig.LinearAccelX,1) < size(r_rover_sig.LinearAccelX,1)
%             [c,lags] = xcorr(l_rcs_accel_sig.XSamples,l_rover_sig.LinearAccelX + r_rover_sig.LinearAccelX(1:size(l_rover_sig.LinearAccelX,1),1),200);
%             [~, ideal_lag_idx] = max(abs(c));
%             ideal_lag = lags(ideal_lag_idx);
%             hold on
%             plot(l_rover_sig.NewTime, l_rover_sig.LinearAccelX + r_rover_sig.LinearAccelX(1:size(l_rover_sig.LinearAccelX,1),1))
%             plot(l_rover_sig.NewTime + ideal_lag, l_rover_sig.LinearAccelX + r_rover_sig.LinearAccelX(1:size(l_rover_sig.LinearAccelX,1),1))
%             title('Lagged Sum Signal')
%         else
%             [c,lags] = xcorr(l_rcs_accel_sig.XSamples,l_rover_sig.LinearAccelX(1:size(r_rover_sig.LinearAccelX,1),1) + r_rover_sig.LinearAccelX,200);
%             [~, ideal_lag_idx] = max(abs(c));
%             ideal_lag = lags(ideal_lag_idx);
%             hold on
%             plot(l_rover_sig.NewTime, l_rover_sig.LinearAccelX + r_rover_sig.LinearAccelX(1:size(l_rover_sig.LinearAccelX,1),1))        
%             plot(l_rover_sig.NewTime + ideal_lag, l_rover_sig.LinearAccelX + r_rover_sig.LinearAccelX(1:size(l_rover_sig.LinearAccelX,1),1))
%             title('Lagged Sum Signal')
%         end
         linkaxes(ax,'x');
    end

    post_align_struct.right_accel_taxis = ((post_align_struct.r_rcs_accel.DerivedTime - post_align_struct.r_rcs_accel.DerivedTime(1))/1000);
    post_align_struct.left_accel_taxis = ((post_align_struct.l_rcs_accel.DerivedTime - post_align_struct.l_rcs_accel.DerivedTime(1))/1000);
    post_align_struct.right_taxis = ((post_align_struct.r_rcs_lfp.DerivedTime - post_align_struct.r_rcs_lfp.DerivedTime(1))/1000);
    post_align_struct.left_taxis = ((post_align_struct.l_rcs_lfp.DerivedTime - post_align_struct.l_rcs_lfp.DerivedTime(1))/1000);

else 

    % create new time stream where t=0 is start of recordings (alignment point)
    l_rcs_lfp_time = aligned_data.left_LFP_table.DerivedTime - aligned_data.left_LFP_table.DerivedTime(1);
    l_rcs_accel_time = aligned_data.left_Accel_table.DerivedTime - aligned_data.left_Accel_table.DerivedTime(1);
    l_rover_time = 1000*seconds(aligned_data.Rover.Left.DateTime - aligned_data.Rover.Left.DateTime(1));
    r_rover_time = 1000*seconds(aligned_data.Rover.Right.DateTime - aligned_data.Rover.Right.DateTime(1));

    l_rcs_lfp_crop = size(l_rcs_lfp_time,1);
    end_time = l_rcs_lfp_time(end,1);
    [~, l_rover_crop] = min(abs(l_rover_time - end_time));
    [~, r_rover_crop] = min(abs(r_rover_time - end_time));
    [~, l_rcs_accel_crop] = min(abs(l_rcs_accel_time - end_time));

    l_rcs_lfp_sig = [aligned_data.left_LFP_table(1:l_rcs_lfp_crop,:), table(l_rcs_lfp_time(1:l_rcs_lfp_crop),'VariableNames',{'NewTime'})];
    l_rover_sig = [aligned_data.Rover.Left(1:l_rover_crop,:), table(l_rover_time(1:l_rover_crop),'VariableNames',{'NewTime'})];
    r_rover_sig = [aligned_data.Rover.Right(1:r_rover_crop,:), table(r_rover_time(1:r_rover_crop),'VariableNames',{'NewTime'})];
    l_rcs_accel_sig = [aligned_data.left_Accel_table(1:l_rcs_accel_crop,:), table(l_rcs_accel_time(1:l_rcs_accel_crop),'VariableNames',{'NewTime'})];

    % plot the aligned and cropped signals on unified time stream
    if fig == 1 || nargin == 1
        figure() 
        ax(1) = subplot(311);
        plot(l_rcs_accel_sig.NewTime,l_rcs_accel_sig.XSamples)
        title('L RC+S Accelerometry')
        ax(2) = subplot(312);
        plot(l_rover_sig.NewTime, l_rover_sig.LinearAccelX)
        hold on
        plot(r_rover_sig.NewTime, r_rover_sig.LinearAccelX)
        title('L Rover Accelerometry')
        ax(4) = subplot(313);
        plot(r_rover_sig.NewTime, r_rover_sig.LinearAccelX)
        title('R Rover Accelerometry')
        linkaxes(ax,'x');
    end

    % place aligned and cropped signals into a structure post_align_struct
    post_align_struct.l_rcs_accel = l_rcs_accel_sig;
    post_align_struct.l_rcs_lfp = l_rcs_lfp_sig;
    post_align_struct.l_rover = l_rover_sig;
    post_align_struct.r_rover = r_rover_sig;

    % add the year of recording and filename for a future step

    splitStr_1 = strsplit(file, '/');
    splitStr_2 = strsplit(splitStr_1{end}, '_');
    splitStr_3 = strsplit(splitStr_2{end},'.');
    filename = [splitStr_2{end-1} '_' splitStr_3{1}];
    post_align_struct.filename = filename;

    if contains(file,'21_') == 1
        post_align_struct.year = 2021;
    elseif contains(file,'22_') == 1
        post_align_struct.year = 2022;
    elseif contains(file,'23_') == 1
        post_align_struct.year = 2023;
    else
        post_align_struct.year = 2024;
    end

    post_align_struct.left_accel_taxis = ((post_align_struct.l_rcs_accel.DerivedTime - post_align_struct.l_rcs_accel.DerivedTime(1))/1000);
    post_align_struct.left_taxis = ((post_align_struct.l_rcs_lfp.DerivedTime - post_align_struct.l_rcs_lfp.DerivedTime(1))/1000);

end 

post_align_struct.DeviceSettings = aligned_data.DeviceSettings;

end

