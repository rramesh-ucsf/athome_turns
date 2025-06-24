%% Read and compile all Rover MPGAs for specified subject

rover_stack = rover_report('RCS_05');

%% Load in and visualize an aligned RC+S-Rover file 

post_align_struct = post_align('/Volumes/dwang3_shared/Patient Data/RC+S Data/gait_RCS_05/Rover/Data/Aligned Data/RCS05_Rover_RCS_040924_02.mat',1);

%% Label the heading information 

