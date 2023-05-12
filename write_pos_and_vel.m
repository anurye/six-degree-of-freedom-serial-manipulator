function  write_pos_and_vel(q, qd)
% This function is modified version of sync_write script
% Accepts the psition and velocity and write this to the manipulator.

% Handle varargin
%{
if length(varargin) ~= 1
    error("Wrong number of additional arguments provided.")
else
    close_port_condition = true;
end
%}
close_port_condition = true;

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

% Default setting
BAUDRATE                    = 1000000;

DEVICENAME                  = 'COM8';       % Check which port is being used on your controller
                                            % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
                                            
TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = -150000;      % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 150000;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed


% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

% Initialize Groupsyncwrite instance
group_num = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
group_num_vel = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_PRO_MOVING_SPEED, LEN_PRO_MOVING_SPEED);


dxl_comm_result = COMM_TX_FAIL;             % Communication result
dxl_addparam_result = false;                % AddParam result

dxl_error = 0;                              % Dynamixel error
dxl_present_position = 0;                   % Present position

% Open port
if (openPort(port_num))
    fprintf('Succeeded to open the port!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    input('Press any key to terminate...\n');
    return;
end


% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Succeeded to change the baudrate!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

% Enable Dynamixel Torque DXL_ID 1
write1ByteTxRx(port_num, PROTOCOL_VERSION, 1, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end

% Enable Dynamixel Torque DXL_ID 2
write1ByteTxRx(port_num, PROTOCOL_VERSION, 2, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end

% Enable Dynamixel Torque DXL_ID 3
write1ByteTxRx(port_num, PROTOCOL_VERSION, 3, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end

% Enable Dynamixel Torque DXL_ID 4
write1ByteTxRx(port_num, PROTOCOL_VERSION, 4, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end

% Enable Dynamixel Torque DXL_ID 5
write1ByteTxRx(port_num, PROTOCOL_VERSION, 5, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end

% Enable Dynamixel Torque DXL_ID 6
write1ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end

%
% Write moving speed
% Add parameter storage for Dynamixel#1 moving speed
%vel = 3*velconv(qd(1),"mx");
%dxl_addparam_result = groupSyncWriteAddParam(group_num_vel, 1, typecast(int32(vel), 'uint32'), LEN_PRO_MOVING_SPEED);
dxl_addparam_result = groupSyncWriteAddParam(group_num_vel, 1, typecast(int32(qd(1)), 'uint32'), LEN_PRO_MOVING_SPEED);
if dxl_addparam_result ~= true
  fprintf('[ID:%03d] groupSyncWrite addparam failed', 1);
  return;
end

% Write moving speed
% Add parameter storage for Dynamixel#2 moving speed
%vel = 3*velconv(qd(2),"mx");
%dxl_addparam_result = groupSyncWriteAddParam(group_num_vel, 2, typecast(int32(vel), 'uint32'), LEN_PRO_MOVING_SPEED);
dxl_addparam_result = groupSyncWriteAddParam(group_num_vel, 2, typecast(int32(qd(2)), 'uint32'), LEN_PRO_MOVING_SPEED);
if dxl_addparam_result ~= true
  fprintf('[ID:%03d] groupSyncWrite addparam failed', 2);
  return;
end

% Write moving speed
% Add parameter storage for Dynamixel#3 moving speed
%vel = 3*velconv(qd(3),"mx");
%dxl_addparam_result = groupSyncWriteAddParam(group_num_vel, 3, typecast(int32(vel), 'uint32'), LEN_PRO_MOVING_SPEED);
dxl_addparam_result = groupSyncWriteAddParam(group_num_vel, 3, typecast(int32(qd(3)), 'uint32'), LEN_PRO_MOVING_SPEED);
if dxl_addparam_result ~= true
  fprintf('[ID:%03d] groupSyncWrite addparam failed', 3);
  return;
end

% Write moving speed
% Add parameter storage for Dynamixel#4 moving speed
%vel = 3*velconv(qd(4),"ax");
%dxl_addparam_result = groupSyncWriteAddParam(group_num_vel, 4, typecast(int32(vel), 'uint32'), LEN_PRO_MOVING_SPEED);
dxl_addparam_result = groupSyncWriteAddParam(group_num_vel, 4, typecast(int32(qd(4)), 'uint32'), LEN_PRO_MOVING_SPEED);
if dxl_addparam_result ~= true
  fprintf('[ID:%03d] groupSyncWrite addparam failed', 4);
  return;
end

% Write moving speed
% Add parameter storage for Dynamixel#5 moving speed
%vel = 3*velconv(qd(5),"ax");
%dxl_addparam_result = groupSyncWriteAddParam(group_num_vel, 5, typecast(int32(vel), 'uint32'), LEN_PRO_MOVING_SPEED);
dxl_addparam_result = groupSyncWriteAddParam(group_num_vel, 5, typecast(int32(qd(5)), 'uint32'), LEN_PRO_MOVING_SPEED);
if dxl_addparam_result ~= true
  fprintf('[ID:%03d] groupSyncWrite addparam failed', 5);
  return;
end

% Write moving speed
% Add parameter storage for Dynamixel#6 moving speed
%vel = 3*velconv(qd(6),"ax");
%dxl_addparam_result = groupSyncWriteAddParam(group_num_vel, 6, typecast(int32(vel), 'uint32'), LEN_PRO_MOVING_SPEED);
dxl_addparam_result = groupSyncWriteAddParam(group_num_vel, 6, typecast(int32(qd(6)), 'uint32'), LEN_PRO_MOVING_SPEED);
if dxl_addparam_result ~= true
  fprintf('[ID:%03d] groupSyncWrite addparam failed', 6);
  return;
end

% Syncwrite goal position
groupSyncWriteTxPacket(group_num_vel);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
end

% Clear syncwrite parameter storage
groupSyncWriteClearParam(group_num_vel);

%}

% Write goal position
% Add parameter storage for Dynamixel#1 goal position
%pos = pos_r2m(q(1),"mx");
%dxl_addparam_result = groupSyncWriteAddParam(group_num, 1, typecast(int32(pos), 'uint32'), LEN_PRO_GOAL_POSITION);
dxl_addparam_result = groupSyncWriteAddParam(group_num, 1, typecast(int32(q(1)), 'uint32'), LEN_PRO_GOAL_POSITION);
if dxl_addparam_result ~= true
  fprintf('[ID:%03d] groupSyncWrite addparam failed', 1);
  return;
end

% Write goal position
% Add parameter storage for Dynamixel#2 goal position
%pos = pos_r2m(q(2),"mx");
%dxl_addparam_result = groupSyncWriteAddParam(group_num, 2, typecast(int32(pos), 'uint32'), LEN_PRO_GOAL_POSITION);
dxl_addparam_result = groupSyncWriteAddParam(group_num, 2, typecast(int32(q(2)), 'uint32'), LEN_PRO_GOAL_POSITION);
if dxl_addparam_result ~= true
  fprintf('[ID:%03d] groupSyncWrite addparam failed', 2);
  return;
end

% Write goal position
% Add parameter storage for Dynamixel#3 goal position
%pos = pos_r2m(q(3),"mx");
%dxl_addparam_result = groupSyncWriteAddParam(group_num, 3, typecast(int32(pos), 'uint32'), LEN_PRO_GOAL_POSITION);
dxl_addparam_result = groupSyncWriteAddParam(group_num, 3, typecast(int32(q(3)), 'uint32'), LEN_PRO_GOAL_POSITION);
if dxl_addparam_result ~= true
  fprintf('[ID:%03d] groupSyncWrite addparam failed', 3);
  return;
end

% Write goal position
% Add parameter storage for Dynamixel#4 goal position
%pos = pos_r2m(q(4),"ax");
%dxl_addparam_result = groupSyncWriteAddParam(group_num, 4, typecast(int32(pos), 'uint32'), LEN_PRO_GOAL_POSITION);
dxl_addparam_result = groupSyncWriteAddParam(group_num, 4, typecast(int32(q(4)), 'uint32'), LEN_PRO_GOAL_POSITION);
if dxl_addparam_result ~= true
  fprintf('[ID:%03d] groupSyncWrite addparam failed', 4);
  return;
end

% Write goal position
% Add parameter storage for Dynamixel#5 goal position
%pos = pos_r2m(q(5),"ax");
%dxl_addparam_result = groupSyncWriteAddParam(group_num, 5, typecast(int32(pos), 'uint32'), LEN_PRO_GOAL_POSITION);
dxl_addparam_result = groupSyncWriteAddParam(group_num, 5, typecast(int32(q(5)), 'uint32'), LEN_PRO_GOAL_POSITION);
if dxl_addparam_result ~= true
  fprintf('[ID:%03d] groupSyncWrite addparam failed', 5);
  return;
end

% Write goal position
% Add parameter storage for Dynamixel#6 goal position
%pos = pos_r2m(q(6),"ax");
%dxl_addparam_result = groupSyncWriteAddParam(group_num, 6, typecast(int32(pos), 'uint32'), LEN_PRO_GOAL_POSITION);
dxl_addparam_result = groupSyncWriteAddParam(group_num, 6, typecast(int32(q(6)), 'uint32'), LEN_PRO_GOAL_POSITION);
if dxl_addparam_result ~= true
  fprintf('[ID:%03d] groupSyncWrite addparam failed', 6);
  return;
end

% Syncwrite goal position
groupSyncWriteTxPacket(group_num);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
end

% Clear syncwrite parameter storage
groupSyncWriteClearParam(group_num);

% Close port
if close_port_condition
    closePort(port_num);
end

% Unload Library
unloadlibrary(lib_name);

end