% performPickAndPlace.m: sample script to perform a simple pick and place
% operation.

%% Initialize libraries
initialize

%% specify the pick and place positions
pickPose = [19, 15, 2];
placePose = [19.5 -15 2];

%% perform the simulation
simulationPickAndPlace(pickPose, placePose);

% perform pick and place hardware
%hardwarePickAndPlace(pickPose, placePose)
