function dQm = traj_vel_joint2motor(dQ)
% TRAJ_VEL_JOINT2MOTOR converts a joint-space velocity trajectory to motor velocity units
%
% Parameters
% ----------
% dQ : matrix
%   A joint-space velocity trajectory where each row represents joint 
%   velocities (in radians per second) for all six joints at a time step.
%
% Returns
% -------
% dQm : matrix
%   A motor-space velocity trajectory where each row represents motor 
%   velocities (in manipulator units) for all six joints at a time step.
%

% Convert velocity from rad/sec to rpm
dQ = (60/(2*pi)).*dQ;

% Initialize the size
dQm = zeros(size(dQ));

for i = 1:numrows(dQ)
    % Limit the velocity between 1023 and 5
    dQm(i, :) = max(min(vel_rpm2dxl(abs(dQ(i, :))), 1023), 30);
end

end