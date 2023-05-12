function terminate_session
% This function sends torque disable to all dynamixels using the
% disable_torque function

for DXL_ID = 1:6
    disable_torque(DXL_ID);
end

end