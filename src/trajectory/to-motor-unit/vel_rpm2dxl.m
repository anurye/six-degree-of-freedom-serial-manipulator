function vm = vel_rpm2dxl(v_rpm, varargin)
% Convert velocity from rpm to dxl own unit

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
