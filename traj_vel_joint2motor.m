function Qdm = traj_vel_joint2motor(Qd)
% This function accepts the velocity trajectory in rpm and transforms it
% into the motor unit

% Initialize the size
Qdm = zeros(size(Qd));

for i = 1:numrows(Qd)
    Qdm(i, :) = vel_rpm2dxl(Qd(i, :));
end

end