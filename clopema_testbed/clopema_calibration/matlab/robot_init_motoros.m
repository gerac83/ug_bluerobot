% Get path to ROS workspace
[voopRes voopT] = system('echo $ROS_WORKSPACE');

% Add motoros toolbox to the path
addpath(genpath([strcat(voopT) '/clopema_matlab/matlab/']));

% Clear vars
clear voopRes voopT
