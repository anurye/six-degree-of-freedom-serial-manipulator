function port_num = close_port(port_num)
%CLOSE_PORT closes the specified port and unloads the corresponding library
%
% Parameters
% ----------
% port_num : int
%     The identifier for the port to be closed.
%
% Returns
% -------
%   []
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

% Close port
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);

end
