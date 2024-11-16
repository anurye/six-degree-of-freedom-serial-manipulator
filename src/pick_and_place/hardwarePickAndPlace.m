function hardwarePickAndPlace(pickPose, placePose)
%HARDWAREPICKANDPLACE performs a pick-and-place operation using hardware
%
% Parameters
% ----------
% pickPose : list
%   The Cartesian coordinates [x, y, z] of the pick position.
%
% placePose : list
%   The Cartesian coordinates [x, y, z] of the place position.
%

% Check if appropriate arguments are provided
if numel(pickPose) ~= 3
    error("pick position should have 3 elements")
end

if numel(placePose) ~= 3
    error("Place position should have 3 elements")
end

% convert the pick and place positions into a row vector
pickPose = reshape(pickPose, 1, []);
placePose = reshape(placePose, 1, []);

% calculate intermediate positions
pickApp = pickPose; pickApp(3) = pickApp(3) + 5;
placeApp = placePose; placeApp(3) = placeApp(3) + 5;

% Home position
q_home = [0, 0, 0, 0, 0, 0];

% ######################## Pick #############################
% Approach pick
T_home = direct_kine(dh, q_home);

% Calculating T_pick
% T_pick_approach desired
% Remove the translation of T_home and substitute it with desired one
T_home(1:3, 4) = zeros(3, 1);
T_pick_app = translate(pickApp)*T_home*RPY2T([0, -pi/2, 0]);

% Get q for approaching pick
q_pick_app = inv_kine(T_pick_app, dh, q_home);

% Trajectory
t = get_time(q_home, q_pick_app);
[Q_pick_app, dQ_pick_app] = joint_traj(@LSPB, q_home, q_pick_app, t);

% Task space
T_temp = T_pick_app;
T_temp(1:3, 4) = zeros(3, 1);
T_pick = translate(pickPose)*T_temp*RPY2T([0, 0, 0]);
[Q_pick, dQ_pick] = task_traj(@LSPB, T_pick_app, T_pick);

% Retract back to the approach pick position
[Q_retract_back_pick, dQ_retract_back_pick] = task_traj(@LSPB, T_pick, T_pick_app);

%################### Place #######################################
% Approach place
% T_place_approach desired
T_pick_app(1:3, 4) = zeros(3, 1);
T_place_app = translate(placeApp)*T_pick_app*RPY2T([0, 0, 0]);

% Compute q_place_approach desired
q_place_app = inv_kine(T_place_app, dh, q_home);

% Trajectory
t = get_time(q_pick_app, q_place_app);
[Q_place_app, dQ_place_app] = joint_traj(@LSPB, q_pick_app, q_place_app, t);

% Place
T_temp = T_place_app;
T_temp(1:3, 4) = zeros(3, 1);
T_place = translate(placePose)*T_temp*RPY2T([0, 0, 0]);
[Q_place, dQ_place] = task_traj(@LSPB, T_place_app, T_place);

% Retract to approach place
[Q_retract_back_place, dQ_retract_back_place] = task_traj(@LSPB, T_place, T_place_app);

%##################### Go back to home position ###############
% Get_time
t = get_time(q_place_app, q_home);
[Q_home, dQ_home] = joint_traj(@LSPB, q_place_app, q_home, t);


%################## Send data to the hardware ##################
% delay
d = 1;

% Open port
port_num = open_port;

% Enable torque
for DXL_ID = 1:7
    enable_torque(DXL_ID, port_num);
end

% Home pose
qm_home = [2033;
           2053;
           1010;
           500 ;
           510 ;
           507
            ];

% Velocity to go to home
vel_home = 60*ones(1, 6);
write_data(qm_home', vel_home, port_num);
pause(d)

% Approach pick
Qm_pick_app = traj_pos_joint2motor(Q_pick_app);
dQm_pick_app = traj_vel_joint2motor(dQ_pick_app);
%dQm_pick_app = 60*ones(size(dQ_pick_app));
% Send velocity and trajectory to the manipulator
sync_send(Qm_pick_app, dQm_pick_app, port_num);
pause(d);

% Open gripper
gripper('open', port_num);
pause(d);

% Go to Pick
Qm_pick = traj_pos_joint2motor(Q_pick);
dQm_pick = traj_vel_joint2motor(dQ_pick);
% Send velocity and trajectory to the manipulator
sync_send(Qm_pick, dQm_pick, port_num);
pause(d);

% Close gripper
gripper('close', port_num);
pause(d);

% Retract back to pre-pick
Qm_retract_back_pick = traj_pos_joint2motor(Q_retract_back_pick);
dQm_retract_back_pick = traj_vel_joint2motor(dQ_retract_back_pick);
%dQm_retract_back_pick = 20*ones(size(dQ_retract_back_pick));
sync_send(Qm_retract_back_pick, dQm_retract_back_pick, port_num);
pause(d);

% Place approach
Qm_place_app = traj_pos_joint2motor(Q_place_app);
dQm_place_app = traj_vel_joint2motor(dQ_place_app);
% Send velocity and trajectory to the manipulator
sync_send(Qm_place_app, dQm_place_app, port_num);
pause(d);

% Go to place
Qm_place = traj_pos_joint2motor(Q_place);
dQm_place = traj_vel_joint2motor(dQ_place);
% Send velocity and trajectory to the manipulator
sync_send(Qm_place, dQm_place, port_num);
pause(d);

% Open gripper
gripper('open', port_num);
pause(d);

% Retract back to approach place
Qm_retract_back_place = traj_pos_joint2motor(Q_retract_back_place);
dQm_retract_back_place = traj_vel_joint2motor(dQ_retract_back_place);
% Send velocity and trajectory to the manipulator
sync_send(Qm_retract_back_place, dQm_retract_back_place, port_num);
pause(d);

% Close gripper
gripper('close', port_num);
pause(d);

% Go to home pose
Qm_home = traj_pos_joint2motor(Q_home);
dQm_home = traj_vel_joint2motor(dQ_home);
% Send velocity and trajectory to the manipulator
sync_send(Qm_home, dQm_home, port_num);
pause(d);

% Option to disaple torque
if strcmp(input('Disable torque? (y/n): ', 's'), 'y')
    for DXL_ID = 1:7
        disable_torque(DXL_ID, port_num);
    end
end

% close port
close_port(port_num);

end
