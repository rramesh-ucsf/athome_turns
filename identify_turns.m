function output_struct = identify_turns(cleaned_struct, thresh)

deriv = diff(cleaned_struct.l_gait.Heading_Smooth);
deriv = [deriv(1); deriv];

[peaks, peak_locs] = findpeaks(deriv,'MinPeakHeight',thresh);
peak_locs = peak_locs - cleaned_struct.delay/1000;

[troughs, trough_locs] = findpeaks(-1*deriv,'MinPeakHeight',thresh);
trough_locs = trough_locs - cleaned_struct.delay/1000;

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
yline(thresh,'k:',LineWidth=2)
yline(-1*thresh,'k:',LineWidth=2)
plot(peak_locs, peaks, 'k.',MarkerSize=10)
plot(trough_locs, -1*troughs, 'k.',MarkerSize=10)
ylabel('Angular Velocity (°/ms)')
xlabel('Time (ms)')
title('Left Rover Angular Velocity')
linkaxes(ax, 'x')
xlim([0 inf]);

end