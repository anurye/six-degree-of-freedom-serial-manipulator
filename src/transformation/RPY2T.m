function T = RPY2T(rpy)
% RPY2T converts roll, pitch, and yaw angles to a homogeneous transformation matrix
%
% Parameters
% ----------
% rpy : list
%   A vector containing the roll, pitch, and yaw angles (in rad):
%   - rpy(1): Roll angle
%   - rpy(2): Pitch angle
%   - rpy(3): Yaw angle
%
% Returns
% -------
% T : matrix
%   The homogeneous transformation matrix corresponding to the specified roll, pitch, and yaw angles.
%

% Extract the roll, pitch, yaw angles
g = rpy(1); b = rpy(2); a = rpy(3);

% Rotation matrix
R = Rotz(a)*Roty(b)*Rotx(g);

% Convert R to homogenous matrix
T = [R zeros(3, 1);
     zeros(1, 3) 1];
end

function R = Rotz(a)
% Rotation around z-axis
R = [cos(a) -sin(a) 0;
     sin(a)  cos(a) 0;
       0      0    1];
end

function R = Roty(b)
% Rotation around y-axis
R = [cos(b)   0    sin(b);
        0     1        0;
    -sin(b)   0    cos(b)];
end

function R = Rotx(g)
% Rotation around x-axis
R = [1      0       0;
     0  cos(g) -sin(g);
     0  sin(g)  cos(g)];
end