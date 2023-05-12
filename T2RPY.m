function [pos, rpy] = T2RPY(T)
% End effector rpy angles calculation
r11 = T(1, 1); r12 = T(1, 2); r13 = T(1, 3);
r21 = T(2, 1); r22 = T(2, 2); r23 = T(2, 3);
r31 = T(3, 1); r32 = T(3, 2); r33 = T(3, 3);

% Let
zero = 1e-4;

if abs(abs(r31) - 1) > zero
    % cos(beta) != 0
    % alpha
    a = atan2(r21, r11);

    % beta
    b = atan2(-r31, sqrt(r32^2 + r33^2));

    % gama
    g = atan2(r32, r33);

elseif abs(abs(r31) - 1) < zero && r31 < 0
    % beta = 90
    % alpha
    a = 0;

    % beta
    b = pi/2;

    % gama
    g = atan2(r12, r13);

elseif abs(abs(r31) - 1) < zero && r31 > 0
    % beta = -90
    % alpha
    a = 0;

    % beta
    b = -pi/2;

    % gama
    g = -atan2(r12, r22);
end

switch nargout
    case 1
        rpy = [g b a];
    case 2
        pos = T(1:3, end)';
        rpy = [g b a];
    otherwise
        error("Wrong number of output arguments provided")
end

end