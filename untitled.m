%% Trajectory test
visualize = false;
% Serial link object
robot = actual;

% Home position
q_home = [0, 0, 0, 0, 0, 0];

%% #####################################################
%             JOINT SPACE TRAJECTORY TEST
%% Pick
% Approach pick
T_home = direct_kine(dh, q_home);

% Calculating T_peack
% T_pick_approach desired
% Remove the translation of T_home and substitute it with desired one
T_home(1:3, 4) = zeros(3, 1);
T_pick_app = translate([8 -26 0])*T_home*RPY2T([0, -pi/2, 0]);

% Get q for approaching pick
q_pick_app = inv_kine(T_pick_app, dh, q_home);

% Trajectory
t = get_time(q_home, q_pick_app);
% q_home joint angles for home position; q_pick_app is joint angles for
% pick_approach position (q1 ... q6) and t is a time vector
[Q_pick_app Qd_pick_app] = joint_traj(@LSPB, q_home, q_pick_app, t);

if visualize
    % visualization
    robot.plot(Q_pick_app);
end

% T_pick desired
T_pick_app(1:3, 4) = zeros(3, 1);
T_pick = translate([8 -26 2])*T_pick_app*RPY2T([0, 0, 0]);

% Calculate q for pick
q_pick = inv_kine(T_pick, dh, q_home);

% Trajectory
% get time
t = get_time(q_pick_app, q_pick);
[Q_pick Qd_pick] = joint_traj(@LSPB, q_pick_app, q_pick, t);

if visualize
    % visualization
    robot.plot(Q_pick);
end

%% Retract back to the approach pick position
t = get_time(q_pick, q_pick_app);
[Q_retract_back_pick Qd_retract_back_pick] = joint_traj(@LSPB, q_pick, q_pick_app, t);

if visualize
    % visualization
    robot.plot(Q_retract_back_pick);
end

%% Place
% Approach place
% T_place_approach desired
T_pick_app(1:3, 4) = zeros(3, 1);
T_place_app = translate([8 26 7])*T_pick_app*RPY2T([0, 0, 0]);

% Compute q_place_approach desired
q_place_app = inv_kine(T_place_app, dh, q_home);

% Trajectory
t = get_time(q_pick_app, q_place_app);
[Q_place_app Qd_place_app]= joint_traj(@LSPB, q_pick_app, q_place_app, t);

if visualize
    % visualization
    robot.plot(Q_place_app);
end

% T_place_desired
T_place_app(1:3, 4) = zeros(3, 1);
T_place = translate([8 26 2])*T_place_app*RPY2T([0, 0, 0]);

% Calculate q for place
q_place = inv_kine(T_place, dh, q_home);

% Trajectory
t = get_time(q_place_app, q_place);
[Q_place Qd_place] = joint_traj(@LSPB, q_place_app, q_place, t);

if visualize
    % visualization
    robot.plot(Q_place);
end

%% Retract to approach place
t = get_time(q_place, q_place_app);
[Q_retract_back_place Qd_retract_back_place] = joint_traj(@LSPB, q_place, q_place_app, t);

if visualize
    % visualization
    robot.plot(Q_retract_back_place);
end

%% Go back to home position
% Get_time
t = get_time(q_place_app, q_home);
[Q_home Qd_home] = joint_traj(@LSPB, q_place_app, q_home, t);

if visualize
    % Visualize
    robot.plot(Q_home)
end

%% Hardware
% Transform pick approach to motor position
% Approach pick
% pose
Qm_pick_app = traj_pos_joint2motor(Q_pick_app);
% vel
Qdm_pick_app = traj_vel_joint2motor(Qd_pick_app);

% Pick
%pos
Qm_pick = traj_pos_joint2motor(Q_pick);
%vel
Qdm_pick = traj_vel_joint2motor(Qd_pick);

% Back to pick approach
%pos
Qm_retract_back_pick = traj_pos_joint2motor(Q_retract_back_pick);
%vel
Qdm_retract_back_pick = traj_vel_joint2motor(Qd_retract_back_pick);

% Place
% Approach place
%pos
Qm_place_app = traj_pos_joint2motor(Q_place_app);
%vel
Qdm_place_app = traj_vel_joint2motor(Qd_place_app);

% Place
%pos
Qm_place = traj_pos_joint2motor(Q_place);
%vel
Qdm_place = traj_vel_joint2motor(Qd_place);

% Back to place approach
%pos
Qm_retract_back_place = traj_pos_joint2motor(Q_retract_back_place);
%vel
Qdm_retract_back_place = traj_vel_joint2motor(Qd_retract_back_place);

% Go back to home
Qm_home = traj_pos_joint2motor(Q_home);
Qdm_home = traj_vel_joint2motor(Qd_home);


%% Send to hardware %%
%
qm_home = [2032;
           2020;
           1010;
           500 ;
           510 ;
           494 
           ];
write_pos_and_vel(qm_home', zeros(1, 6))
pause(0.1)

% Pick approach
sync_send(Qm_place_app, Qdm_place_app);
pause(0.1)

% Pick
sync_send(Qm_pick, Qdm_pick);
pause(0.1)

% Back to pick approach
sync_send(Qm_retract_back_pick, Qdm_retract_back_pick);
pause(0.1)

% Place approach
sync_send(Qm_place_app, Qdm_place_app);
pause(0.1)

% Place
sync_send(Qm_place, Qdm_place);
pause(0.1)

% Back to place approach
sync_send(Qm_retract_back_place, Qdm_retract_back_place);
pause(0.1)

% Go home
sync_send(Qm_home, Qdm_home)
pause(0.1)

% Terminate session
terminate_session;





