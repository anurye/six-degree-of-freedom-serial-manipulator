function Qm = traj_pos_joint2motor(Qj)
% This function accepts the joint trajectory and returns this trajectory in
% motor unit

% Initialize the size of Qm
Qm = zeros(size(Qj));
for i = 1:numrows(Qj)
    Qm(i, :) = joint2motor(Qj(i, :));
end

end
