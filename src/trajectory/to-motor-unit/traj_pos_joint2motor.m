function Qm = traj_pos_joint2motor(Qj)
%TRAJ_POS_JOINT2MOTOR converts a joint-space trajectory to motor positions in manipulator units
%
% Parameters
% ----------
% Qj : matrix
%   A joint-space trajectory where each row represents joint angles (in rad)
%   for all six joints at a time step.
%
% Returns
% -------
% Qm : matrix
%   A motor-space trajectory where each row represents motor positions 
%   (in manipulator units) for all six joints.
%

% Initialize the size of Qm
Qm = zeros(size(Qj));
for i = 1:numrows(Qj)
    Qm(i, :) = joint2motor(Qj(i, :));
end

end
