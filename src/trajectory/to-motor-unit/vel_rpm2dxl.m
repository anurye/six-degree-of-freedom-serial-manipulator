function vm = vel_rpm2dxl(v_rpm, varargin)
%VEL_RPM2DXL converts velocities from RPM to motor units (Dynamixel's own unit)
%
% Parameters
% ----------
% v_rpm : scalar or vector
%   - If scalar: A single joint velocity in RPM.
%   - If vector: A vector of joint velocities in RPM for all joints (length must be 6).
%
% varargin : string, optional
%   - Specifies the motor type when `v_rpm` is scalar. Acceptable values:
%       - "ax": For AX-series motors (0.111 RPM/unit).
%       - "mx": For MX-series motors (0.114 RPM/unit).
%   - If `v_rpm` is a vector, no additional arguments are required.
%
% Returns
% -------
% vm : scalar or vector
%     The converted velocity in Dynamixel's motor units. The format matches the input `v_rpm`:
%     - If `v_rpm` is scalar, `vm` is scalar.
%     - If `v_rpm` is a vector, `vm` is a vector of velocities in motor units.
%

% Handle varargin
if length(varargin) > 1
    error("Wrong number of additional arguments provided: %d\n", length(varargin))
end

if isscalar(v_rpm) && isempty(varargin)
    error("For a single joint velocity the motor type has to be specified.")
elseif ~isscalar(v_rpm) && ~isempty(varargin)
    error("No additional argument is required if vector of joint velocities is provided")
elseif ~isscalar(v_rpm) && length(v_rpm) ~= 6
    error("If vector of joint velocities is provided the length has to be 6")
end

% Velocity conversion units if vector of velocities is provoded
conv_unit_vecs = [1023/116.2 1023/116.2 1023/116.2...
                  1023/114   1023/114   1023/114];

if isscalar(v_rpm)
    type = varargin{1};
    switch type
        case "ax"
            % 0 ~ 1,023(0x3FF) can be used, and the unit is about 0.111rpm
            % If it is 1023, it is about 114rpm.
            vm = round(v_rpm * 1023/114);
        case "mx"
            % 0~1023 (0X3FF) can be used, and the unit is about 0.114rpm
            % If it is 1023, it is about 116.62rpm
            vm = round(v_rpm * 1023/116.2);
        otherwise
            error("Wrong motor type provided%s\n", type)
    end
else
    % For vector of joint angles
    % Percent
    p = 0;
    vm = v_rpm .* conv_unit_vecs;
    vm = vm + p*vm;
    vm = ceil(vm);
end
end
