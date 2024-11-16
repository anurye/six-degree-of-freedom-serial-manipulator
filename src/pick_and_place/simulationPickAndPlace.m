function Q_trajectory = simulationPickAndPlace(pickPose, placePose, varargin)
% simulationPickAndPlace - Simulates a pick-and-place operation in a virtual environment
%
% Parameters
% ----------
% pickPose : vector, shape (1, 3)
%   The Cartesian coordinates [x, y, z] of the pick position.
%
% placePose : vector, shape (1, 3)
%   The Cartesian coordinates [x, y, z] of the place position.
%
% varargin{1} : logical (optional)
%   A single optional logical value to enable or disable simulation visualization.
%   - `true` (default): Enables visualization.
%   - `false`: Disables visualization.
%
% Returns
% -------
% Q_trajectory : matrix
%   A matrix where each row represents the joint configuration at a specific time step
%   for the entire pick-and-place operation.
%

% Check if appropriate arguments are provided
if numel(pickPose) ~= 3
    error("pick position should have 3 elements")
end

if numel(placePose) ~= 3
    error("Place position should have 3 elements")
end

% parse varargin
if ~isempty(varargin) && numel(varargin) ~= 1
    error("simulationPickAndPlace: the optional argument should be a single logical parameter")
end

if isempty(varargin)
    visualize = 1;
else
    visualize = varargin{1};
end

% convert the pick and place positions into a row vector
pickPose = reshape(pickPose, 1, []);
placePose = reshape(placePose, 1, []);

% calculate intermediate positions
pickApp = pickPose; pickApp(3) = pickApp(3) + 5;
placeApp = placePose; placeApp(3) = placeApp(3) + 5;

% Serial link object
robot = actual;

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
Q_pick_app = joint_traj(@LSPB, q_home, q_pick_app, t);

% Task space
T_temp = T_pick_app;
T_temp(1:3, 4) = zeros(3, 1);
T_pick = translate(pickPose)*T_temp*RPY2T([0, 0, 0]);
Q_pick = task_traj(@LSPB, T_pick_app, T_pick);

% Retract back to the approach pick position
Q_retract_back_pick = task_traj(@LSPB, T_pick, T_pick_app);

%################### Place #######################################
% Approach place
% T_place_approach desired
T_pick_app(1:3, 4) = zeros(3, 1);
T_place_app = translate(placeApp)*T_pick_app*RPY2T([0, 0, 0]);

% Compute q_place_approach desired
q_place_app = inv_kine(T_place_app, dh, q_home);

% Trajectory
t = get_time(q_pick_app, q_place_app);
Q_place_app = joint_traj(@LSPB, q_pick_app, q_place_app, t);

% Place
T_temp = T_place_app;
T_temp(1:3, 4) = zeros(3, 1);
T_place = translate(placePose)*T_temp*RPY2T([0, 0, 0]);
Q_place = task_traj(@LSPB, T_place_app, T_place);

% Retract to approach place
Q_retract_back_place = task_traj(@LSPB, T_place, T_place_app);

%##################### Go back to home position ###############
% Get_time
t = get_time(q_place_app, q_home);
Q_home = joint_traj(@LSPB, q_place_app, q_home, t);

% Combine all the trajectory points
Q_trajectory = [Q_pick_app; Q_pick; Q_retract_back_pick;
    Q_place_app; Q_place; Q_retract_back_place;
    Q_home];

%########################## Visualize ########################
if visualize
    % Initialize the end-effector positions
    endEffectorPositions = zeros(length(Q_trajectory), 3);

    % Get the end-effector positions for each time step
    for i = 1:numrows(Q_trajectory)
        % Get end-effector position at current time step
        T = robot.fkine(Q_trajectory(i, :));
        T = double(T);
        endEffectorPosition = T(1:3, 4);

        % Store the end-effector position
        endEffectorPositions(i, :) = endEffectorPosition;
    end

    % Visualize robot configurations and plot end-effector trajectory as a red line
    plot3(endEffectorPositions(:, 1), endEffectorPositions(:, 2), endEffectorPositions(:, 3), 'r', 'LineWidth', 2);
    hold on;
    robot.plot(Q_trajectory);
    hold off;
end


%{
    %% ####################################################
    %                 TASK SPACE TRAJECTORY TEST
    % We want to create a right angle triangle
    T_home = direct_kine(dh, q_home);

    % Corner 1
    T_home(1:3, 4) = zeros(3, 1);
    T_corner_1 = translate([10, 0, 0])*T_home*RPY2T([0, -pi/2, 0]);
    q1 = inv_kine(T_corner_1, dh, q_home);

    % Corner 2
    T_temp = T_corner_1;
    T_temp(1:3, 4) = zeros(3, 1);
    T_corner_2 = translate([10, 20, 0])*T_temp*RPY2T([0, 0, 0]);
    q2 = inv_kine(T_corner_2, dh, q_home);

    % Corner 3
    T_temp = T_corner_2;
    T_temp(1:3, 4) = zeros(3, 1);
    T_corner_3 = translate([20, 0, 0])*T_temp*RPY2T([0, 0, 0]);
    q3 = inv_kine(T_corner_3, dh, q_home);

    % Task space trajectory from corner 1 to corner 2
    Q_12 = task_traj(@LSPB, T_corner_1, T_corner_2);

    % Task space trajectory from corner 2 to corner 3
    Q_23 = task_traj(@LSPB, T_corner_2, T_corner_3);

    % Task space trajectory from corner 3 to corner 1
    Q_31 = task_traj(@LSPB, T_corner_3, T_corner_1);

    % Combine all of them to one trajectory
    Q_task = [Q_12; Q_23; Q_31];

    % The same trajectory in joint space
    Q_joint = [joint_traj(@LSPB, q1, q2, get_time(q1, q2));
        joint_traj(@LSPB, q2, q3, get_time(q2, q3));
        joint_traj(@LSPB, q3, q1, get_time(q3, q1))];


        % Initialize the end-effector positions
        endEffectorPositions_j = zeros(length(Q_task), 3);

        % Get the end-effector positions for each time step
        for i = 1:numrows(Q_task)
            % Get end-effector position at current time step
            T = robot.fkine(Q_task(i, :));
            T = double(T);
            endEffectorPosition = T(1:3, 4);

            % Store the end-effector position
            endEffectorPositions_j(i, :) = endEffectorPosition;
        end

        % Initialize the end-effector positions
        endEffectorPositions_t = zeros(length(Q_joint), 3);
        for i = 1:numrows(Q_joint)
            % Get end-effector position at current time step
            T = robot.fkine(Q_joint(i, :));
            T = double(T);
            endEffectorPosition = T(1:3, 4);

            % Store the end-effector position
            endEffectorPositions_t(i, :) = endEffectorPosition;
        end

        % Visualize robot configurations and plot end-effector trajectory
        plot3(endEffectorPositions_t(:, 1), endEffectorPositions_t(:, 2), endEffectorPositions_t(:, 3), 'b-', 'LineWidth', 2);
        hold on;
        plot3(endEffectorPositions_j(:, 1), endEffectorPositions_j(:, 2), endEffectorPositions_j(:, 3), 'r-', 'LineWidth', 2);
        robot.plot(Q_task);
        hold off;
%}

end
