function qm = joint2motor(qj, varargin)
% Accepts joint angles n rad and returns it in the manupulator unit

% Handle varargin
if length(varargin) > 1
    error("Wrong number of additional arguments provided: %d\n", length(varargin))
end

if isscalar(qj) && isempty(varargin)
    error("For a single joint angle the motor type has to be specified.")
elseif ~isscalar(qj) && ~isempty(varargin)
    error("No additional argument is required if vector of joint angles is provided")
elseif ~isscalar(qj) && length(qj) ~= 6
    error("If vector of joint angles is provided the length has to be 6")
end

% for vector of joint angles
conv_unit_vec = [4095/deg2rad(360) 4095/deg2rad(360) 4095/deg2rad(360)...
                 1023/deg2rad(300) 1023/deg2rad(300) 1023/deg2rad(300)];

% Limits of motor positions
qm_limits = [1023 3514;
             1210 3030;
             1010 3010;
             220  790 ;
             206  510 ;
             202  810 ;
            ];
% Motor position at manipulator home position
qm_home = [2032;
           2020;
           1010;
           500 ;
           510 ;
           494 
           ];

if isscalar(qj)
    type = varargin{1};
    switch type
        case "ax"
            conv_unit = 1023/deg2rad(300); % cnversion unit from degree to manupulator unit
            qm = qj*conv_unit;
            return;
        case "mx"
            conv_unit = 4095/deg2rad(360);
            qm = qj*conv_unit;
            return;
        otherwise
            error("Wrong manupulator type")
    end
else
    % vectors of joint angle
    qm = qj .* conv_unit_vec;
end

% Compensate for the home position offset
qm = qm + qm_home';

% Check for limits;
for i = 1:numcols(qm)
    % Limit the motor positions between the given max and min ranges
    qm(i) = max(min(qm(i), qm_limits(i, 2)), qm_limits(i, 1));
end

end
