function test_trajectory(space, varargin)
% This function test the joint space if space = "j" and task space if
% space = "t". It accepts additional boolean argument for visualization

% Check varargin
switch length(varargin)
    case 0
        visualize = false;
    case 1
        visualize = varargin{1};
    otherwise
        error("Wrong number of additional arguments provided: %d\n", length(varargin))
end

%% Trajectory test
% Serial link object
robot = actual;

% Home position
q_home = [0, 0, 0, 0, 0, 0];

%% #####################################################
%             JOINT SPACE TRAJECTORY TEST
if strcmp(space, "j")
    %% Pick
    % Approach pick
    T_home = direct_kine(dh, q_home);

    % Calculating T_peack
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

    % T_pick desired
    T_pick_app(1:3, 4) = zeros(3, 1);
    T_pick = translate([8 -26 2])*T_pick_app*RPY2T([0, 0, 0]);

    % Calculate q for pick
    q_pick = inv_kine(T_pick, dh, q_home);

    % Trajectory
    % get time
    t = get_time(q_pick_app, q_pick);
    Q_pick = joint_traj(@LSPB, q_pick_app, q_pick, t);

    if visualize
        % visualization
        robot.plot(Q_pick);
    end

    %% Retract back to the approach pick position
    t = get_time(q_pick, q_pick_app);
    Q_retract_back_pick = joint_traj(@LSPB, q_pick, q_pick_app, t);

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
    Q_place_app = joint_traj(@LSPB, q_pick_app, q_place_app, t);

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
    Q_place = joint_traj(@LSPB, q_place_app, q_place, t);

    if visualize
        % visualization
        robot.plot(Q_place);
    end

    %% Retract to approach place
    t = get_time(q_place, q_place_app);
    Q_retract_back_place = joint_traj(@LSPB, q_place, q_place_app, t);

    if visualize
        % visualization
        robot.plot(Q_retract_back_place);
    end

    %% Go back to home position
    % Get_time
    t = get_time(q_place_app, q_home);
    Q_home = joint_traj(@LSPB, q_place_app, q_home, t);

    if visualize
        % Visualize
        robot.plot(Q_home)
    end

elseif strcmp(space, "t")
    %% ####################################################
    %                 TASK SPACE TRAJECTORY TEST
    % We want to go in straight line from Ti to Tf
    T_home = direct_kine(dh, q_home);

    % Initial pose
    T_home(1:3, 4) = zeros(3, 1);
    Ti = translate([10, 3, 15])*T_home;

    % Final pose
    Tf = translate([10 25 15])*T_home;

    % Task space trajectory
    Q_task = task_traj(@LSPB, Ti, Tf);

    if visualize
        % Visualize
        robot.plot(Q_task)
    end
end
