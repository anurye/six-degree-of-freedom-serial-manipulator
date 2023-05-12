function Q = task_traj(fhandle, Ti, Tf)
% Accepts the initial and final homogenous matrices and computes the task
% space trakectory

% Check the first argument is a function handle if not print error
if ~isa(fhandle, 'function_handle')
    error("The first argument must be function handle")
end

% Calculating the time
% get the initial and final goint angles
q0 = inv_kine(Ti, dh, zeros(1, 6));
qf = inv_kine(Tf, dh, zeros(1, 6));

% Get time spets
t = get_time(q0, qf);

% Convert homogenous tranformation matrix to rpy
% For initial pose
[pos_i, rpy_i] = T2RPY(Ti);
Ti = [pos_i rpy_i];

% For final pose
[pos_f, rpy_f] = T2RPY(Tf);
Tf = [pos_f rpy_f];

% Perform the interpolation
% Position
qx = fhandle(Ti(1), Tf(1), t);
qy = fhandle(Ti(2), Tf(2), t);
qz = fhandle(Ti(3), Tf(3), t);

% Orientation
g = fhandle(Ti(4), Tf(4), t);
b = fhandle(Ti(5), Tf(5), t);
a = fhandle(Ti(6), Tf(6), t);

% Perform inverse kinematics at each interpolation points
% Initialize variable to hold joint angles
Q = zeros(length(qx), 6);

% Initialize 3D matrix to hold all transformations
T = zeros(4,4,length(qx));

% actual
%r = actual;

for i=1:length(qx)
    T(:,:,i) = translate([qx(i) qy(i) qz(i)])*RPY2T([g(i) b(i) a(i)]);
    Q(i, :) = inv_kine(T(:,:,i), dh, zeros(1, 6));
    %Q(i, :) = r.ikine(T(:,:,i));
end

end