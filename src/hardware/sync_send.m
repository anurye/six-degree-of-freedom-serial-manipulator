function sync_send(Q, dQ, port_num)
% SYNC_SEND sends position and velocity data for all six joints sequentially
%
% Parameters
% ----------
% Q : matrix
%     A matrix where each col represents position values for the six joints.
%
% dQ : matrix
%     A matrix where each col represents velocity values for the six joints.
%
% port_num : int
%     The identifier of the port through which the data is sent.
%

% delay between consequetive send
%dt = 0.01;
dt = dt + 0.1*dt;
for i = 1:numrows(Q)
    write_data(Q(i, :), dQ(i, :), port_num);
    %write_pos_and_vel(Q(i, :), dQ(i, :), port_num);
    pause(dt);
end

end
