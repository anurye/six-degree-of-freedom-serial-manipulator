function [Q, dQ] = task_traj(fhandle, Ti, Tf)
%TASK_TRAJ generates a task-space trajectory for a manipulator
%
% Parameters
% ----------
% fhandle : function_handle
%   A function handle to compute the trajectory interpolation (e.g., linear interpolation).
%
% Ti : matrix
%   Initial homogeneous transformation matrix specifying the starting pose of the manipulator in task space.
%
% Tf : matrix
%   Final homogeneous transformation matrix specifying the ending pose of the manipulator in task space.
%
% Returns
% -------
% Q : matrix
%   The joint positions for the manipulator at each trajectory step.
%
% dQ : matrix
%   The joint velocities for the manipulator at each trajectory step.
%

% Check the first argument is a function handle if not print error
if ~isa(fhandle, 'function_handle')
    error("The first argument must be function handle")
end

% Calculating the time
% get the initial and final goint angles
q0 = inv_kine(Ti, dh, zeros(1, 6));
qf = inv_kine(Tf, dh, zeros(1, 6));

% Get time spets
t = get_time(q0, qf);
dt = t(2) - t(1);

% Convert homogenous tranformation matrix to rpy
% For initial pose
[pos_i, rpy_i] = T2RPY(Ti);
Ti = [pos_i rpy_i];

% For final pose
[pos_f, rpy_f] = T2RPY(Tf);
Tf = [pos_f rpy_f];

% Perform the interpolation
% Position
qx = linspace(Ti(1), Tf(1), length(t));
qy = linspace(Ti(2), Tf(2), length(t));
qz = linspace(Ti(3), Tf(3), length(t));

% Orientation
g = linspace(Ti(4), Tf(4), length(t));
b = linspace(Ti(5), Tf(5), length(t));
a = linspace(Ti(6), Tf(6), length(t));

% Perform inverse kinematics at each interpolation points
% Initialize variable to hold joint angles and velocities
Q = zeros(length(qx), 6);
dQ = zeros(length(qx), 6);

% Initialize 3D matrix to hold all transformations
T = zeros(4,4,length(qx));

for i=1:length(qx)
    T(:,:,i) = translate([qx(i) qy(i) qz(i)])*RPY2T([g(i) b(i) a(i)]);
    if i > 1
        Q(i, :) = inv_kine(T(:,:,i), dh, zeros(1, 6));
        dQ(i, :) = abs(Q(i, :) - Q(i-1, :))/dt;
    else
        Q(i, :) = inv_kine(T(:,:,i), dh, zeros(1, 6));
    end
end

% take the average velocity
for i = 1:length(dQ)
    dQ(i, :) = sum(dQ, 1)/numrows(dQ);
end
dQ = max(min(dQ, 1023), 90);
end