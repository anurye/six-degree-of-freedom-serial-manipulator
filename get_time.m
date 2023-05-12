function T = get_time(q0, qf)
% calculates the longest tf and return time steps
% from specification
vmax_rpm = [55 63 55 59 59 59];
vmax_rad = vmax_rpm./(60/(2*pi));

% Take safe value (25%)
vmax = vmax_rad.*0.25;  

% Let's assume acceleration
a = 4;

tf = ((abs(qf - q0))./vmax) + (vmax./a);

% round to upper integer
tf = ceil(max(tf));

% change in t
dt = 0.1;

% time step
T = 0:dt:tf;

end