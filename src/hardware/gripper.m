function gripper(option, port_num)
% This function is used for opening and closing the gripper

% Determine what operation is required, based on that set motor position
switch option
    case 'open'
        dxl_goal_position = 560;
    case 'close'
        dxl_goal_position = 190;
    otherwise
        error("Wrong option is provided for gripper")
end

% Protocol version
PROTOCOL_VERSION            = 1.0;          % See which protocol version is used in the Dynamixel

% Default setting
DXL_ID                      = 7;            % Dynamixel ID: 1
                                            % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
DXL_MINIMUM_POSITION_VALUE  = 1610;          % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 2318;         % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 10;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

ADDR_MX_GOAL_POSITION       = 30;

dxl_comm_result = COMM_TX_FAIL;             % Communication result

dxl_error = 0;                              % Dynamixel error


% Write goal position
write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end


end