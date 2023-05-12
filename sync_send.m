function sync_send(Q, Qd)
% This function sends position and velocity for all six joints one at 
% a time through the write_pos_and_vel function.

for i = 1:numrows(Q)
    if i == numrows(Q)
        write_pos_and_vel(Q(i, :), Qd(i, :), true);
    end
    write_pos_and_vel(Q(i, :), Qd(i, :));
end

end
