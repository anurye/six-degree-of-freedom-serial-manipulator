function [q, dq, ddq] = LSPB(q0, qf, T, v)
% LSPB generates a trapezoidal velocity (LSPB) trajectory
%
% Parameters
% ----------
% q0 : double
%   Initial joint position (in radians).
%
% qf : double
%   Final joint position (in radians).
%
% T : scalar or vector
%   - If scalar: The number of time steps for the trajectory.
%   - If vector: The time vector for the trajectory.
%
% v : double, optional
%   Desired maximum velocity for the trajectory. 
%   If not provided, it will be calculated automatically.
%
% Returns
% -------
% q : list
%   Joint positions at each time step.
%
% dq : list
%   Joint velocities at each time step.
%
% ddq : list
%   Joint accelerations at each time step.
%

% Check if time vector is provided. if num of step is provided create t.
if isscalar(T)
    t = (0:T-1)';
else
    t = T(:);
end

% Calculate final time
tf = max(t(:));

% Check if velocity is provided if not calculate it
switch nargin
    case 4
        % Velocity provided
        V = v;
    case 3
        % Velocity not provided so calculate it.
        % Make it in between acceptable range for V.
        V = 1.5*(qf - q0)/tf;
    otherwise
        error("Wrong number of argument provided: %d.\n", nargin);
end

% Check if the provided velocity is appropriate
if abs(V) < abs(qf - q0)/tf
    error("The velocity %.4f is too small.\n", V)
elseif abs(V) > 2*abs(qf - q0)/tf
    error("The velocity %.4f is too large.\n", V)
end

% Determine sign of velocity
V = abs(V)*sign(qf - q0);

% Check if initial and final joint angles are the same
if q0 == qf
    q = ones(size(t))*q0;
    dq = zeros(size(t));
    ddq = zeros(size(t));
    return
end

% Calculate blend time and acceleration
tb = (q0 - qf + V*tf)/V;
a = V/tb;

% Initialize the return values
q = zeros(length(t), 1);
dq = zeros(length(t), 1);
ddq = zeros(length(t), 1);

for i = 1: length(t)
    tc = t(i);  % current time instant
    if tc <= tb
        % portion 1 (Parabolic) linear velocity
        q(i) = q0 + a/2*tc^2;
        dq(i) = a*tc;
        ddq(i) = a;

    elseif tc <= tf - tb
        % protion 2, linear segment constant velocity
        q(i) = (qf + q0 - V*tf)/2 + V*tc;
        dq(i) = V;
        ddq(i) = 0;
        
    else
        % portion 3, parabolic
        q(i) = qf - a/2*tf^2 + a*tf*tc - a/2*tc^2;
        dq(i) = a*tf - a*tc;
        ddq(i) = -a;
    end
end

end