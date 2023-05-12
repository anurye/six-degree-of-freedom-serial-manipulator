%% This script is for testing pick and place using the actual hardware %%
%% Testing using simulation
visualize = true;
% Serial link object
robot = actual;

% Home position
q_home = [0, 0, 0, 0, 0, 0];

% Pick (Approach pick and Pick)
% Approach pick
T_home = direct_kine(dh, q_home);

% Calculating T_pick
% T_pick_approach desired
% Remove the translation of T_home and substitute it with desired one
T_home(1:3, 4) = zeros(3, 1);
T_pick_app = translate([8 -26 7])*T_home*RPY2T([0, -pi/2, 0]);

% Get q for approaching pick
q_pick_app = inv_kine(T_pick_app, dh, q_home);

% Trajectory
t = get_time(q_home, q_pick_app);
% q_home joint angles for home position; q_pick_app is joint angles for
% pick_approach position (q1 ... q6) and t is a time vector
Q_pick_app = joint_traj(@LSPB, q_home, q_pick_app, t);

if visualize
    % visualization
    robot.plot(Q_pick_app);
end

% T_pick desired (TASK SPACE)
Temp_T_pick_app = T_pick_app;
Temp_T_pick_app(1:3, 4) = zeros(3, 1);
T_pick = translate([8 -26 2])*Temp_T_pick_app*RPY2T([0, 0, 0]);
Q_pick = task_traj(@LSPB, T_pick_app, T_pick);
if visualize
    % visualization
    robot.plot(Q_pick);
end

% Retract back to the approach pick position
Q_retract_back_pick = flip(Q_pick);
if visualize
    % visualization
    robot.plot(Q_retract_back_pick);
end

% Place
% Approach place
% T_place_approach desired
T_pick_app(1:3, 4) = zeros(3, 1);
T_place_app = translate([8 26 7])*T_pick_app*RPY2T([0, 0, 0]);

% Compute q_place_approach desired
q_place_app = inv_kine(T_place_app, dh, q_home);

% Trajectory
t = get_time(q_pick_app, q_place_app);
Q_place_app = joint_traj(@LSPB, q_pick_app, q_place_app, t);

if visualize
    % visualization
    robot.plot(Q_place_app);
end

% T_place_desired
Temp_T_place_app = T_place_app;
Temp_T_place_app(1:3, 4) = zeros(3, 1);
T_place = translate([8 26 2])*Temp_T_place_app*RPY2T([0, 0, 0]);
Q_place = task_traj(@LSPB, T_place_app, T_place);

if visualize
    % visualization
    robot.plot(Q_place);
end

% Retract to approach place
Q_retract_back_place = flip(Q_place);

if visualize
    % visualization
    robot.plot(Q_retract_back_place);
end

% Go back to home position
% Get_time
t = get_time(q_place_app, q_home);
Q_home = joint_traj(@LSPB, q_place_app, q_home, t);

if visualize
    % Visualize
    robot.plot(Q_home)
end

%% Hardware Test

