function [Q, dQ, ddQ] = joint_traj(fhandle, q0, qf, T, V)
% Check correct number of input is provided
if nargin < 4 || nargin > 5
    error("Wrong number of argument provided.\n")
end

% q0 and qf must have equal number of elements
if numcols(q0) ~= numcols(qf)
    error("q0 and qf must be of the same size'\n")
end

% Check the first argument is a function handle if not print error
if ~isa(fhandle, 'function_handle')
    error("The first argument must be function handle")
end

% Check if T is not scalar
% Before that stor T in temp variable we will need it later
Ttemp = T;

% Scalar flag, will be needed later
s = true;
if ~isscalar(T)
    T = length(T);
    s = false;
end

% Initialize return values
Q = zeros(T, numcols(q0));
dQ = zeros(T, numcols(q0));
ddQ = zeros(T, numcols(q0));

% Calculate the trajectory for all joints with the use of fhandle
% if s (scalar flag) is true then we pass T to fhandle if not we pas Ttemp
if ~s
    T = Ttemp;
end

if nargin == 4
    for i = 1:numcols(q0)
        [Q(:, i), dQ(:, i), ddQ(:, i)] = fhandle(q0(i), qf(i), T);
    end
elseif nargin == 5
    % Check if V have the same columns as q0
    if numcols(V) ~= numcols(q0)
        error("V must have the same number of columns as q0 and qf.\n")
    end

    for i = 1:numcols(q0)
        [Q(:, i), dQ(:, i), ddQ(:, i)] = fhandle(q0(i), qf(i), T, V(i));
    end
end

end