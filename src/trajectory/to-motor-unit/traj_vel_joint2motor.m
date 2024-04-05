function dQm = traj_vel_joint2motor(dQ)
% This function accepts the velocity trajectory in rad/sec and transforms it
% into the motor unit

% Convert velocity from rad/sec to rpm
dQ = (60/(2*pi)).*dQ;

% Initialize the size
dQm = zeros(size(dQ));

for i = 1:numrows(dQ)
    % Limit the velocity between 1023 and 5
    dQm(i, :) = max(min(vel_rpm2dxl(abs(dQ(i, :))), 1023), 30);
end

end