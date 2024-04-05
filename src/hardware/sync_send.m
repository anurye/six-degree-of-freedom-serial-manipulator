function sync_send(Q, dQ, port_num)
% This function sends position and velocity for all six joints one at 
% a time through the write_pos_and_vel function.

% delay between consequetive send
%dt = 0.01;
dt = dt + 0.1*dt;
for i = 1:numrows(Q)
    write_data(Q(i, :), dQ(i, :), port_num);
    %write_pos_and_vel(Q(i, :), dQ(i, :), port_num);
    pause(dt);
end

end
