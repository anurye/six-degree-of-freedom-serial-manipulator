function  write_data(q, qd, port_num)
%WRITE_DATA sends position and velocity commands to the Dynamixel manipulator
%
% Parameters
% ----------
% q : list
%     A vector containing the desired position values for the six joints.
%
% qd : list
%     A vector containing the desired velocity values for the six joints.
%
% port_num : int
%     The identifier of the port used for communication.
%

lib_name = '';

if strcmp(computer, 'PCWIN')
    lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
    lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
    lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
    lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
    lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_sync_write.h');
end

% Control table address
ADDR_PRO_TORQUE_ENABLE       = 24;         % Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION       = 30;
ADDR_PRO_PRESENT_POSITION    = 36;
ADDR_PRO_MOVING_SPEED        = 32;

% Data Byte Length
LEN_PRO_GOAL_POSITION           = 2;
LEN_PRO_MOVING_SPEED            = 2;


% Protocol version
PROTOCOL_VERSION            = 1.0;          % See which protocol version is used in the Dynamixel


DXL_MINIMUM_POSITION_VALUE  = -150000;      % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 150000;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

% Initialize Groupsyncwrite instance
group_num = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
group_num_vel = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_PRO_MOVING_SPEED, LEN_PRO_MOVING_SPEED);


dxl_comm_result = COMM_TX_FAIL;             % Communication result
dxl_addparam_result = false;                % AddParam result

dxl_error = 0;                              % Dynamixel error
dxl_present_position = 0;                   % Present position

% Write moving speed for all motors using loop
for i = 1:6
    dxl_addparam_result = groupSyncWriteAddParam(group_num_vel, i, typecast(int32(qd(i)), 'uint32'), LEN_PRO_MOVING_SPEED);
    if dxl_addparam_result ~= true
        fprintf('[ID:%03d] groupSyncWrite addparam failed', i);
        return;
    end
end

% Syncwrite goal position
groupSyncWriteTxPacket(group_num_vel);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
end

% Clear syncwrite parameter storage
groupSyncWriteClearParam(group_num_vel);

% Similarly write goal position using loop
for i = 1:6
    dxl_addparam_result = groupSyncWriteAddParam(group_num, i, typecast(int32(q(i)), 'uint32'), LEN_PRO_GOAL_POSITION);
    if dxl_addparam_result ~= true
        fprintf('[ID:%03d] groupSyncWrite addparam failed', i);
        return;
    end
end

% Syncwrite goal position
groupSyncWriteTxPacket(group_num);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
end

% Clear syncwrite parameter storage
groupSyncWriteClearParam(group_num);


end