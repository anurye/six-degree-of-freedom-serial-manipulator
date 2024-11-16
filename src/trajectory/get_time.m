function T = get_time(q0, qf)
%GET_TIME computes the time steps for a trajectory based on initial and final joint angles
%
% Parameters
% ----------
% q0 : list
%   Initial joint angles (in rad) of the manipulator.
%
% qf : list
%   Final joint angles (in rad) of the manipulator.
%
% Returns
% -------
% T : vector
%   A vector of time steps from 0 to the total trajectory time `tf` with a fixed sampling time.
%

% from specification
vmax_rpm = [55 63 55 59 59 59];
vmax_rad = vmax_rpm./(60/(2*pi));

% Take safe value (60%)
vmax = vmax_rad.*0.2;  

% Let's assume acceleration
a = 4;

tf = ((abs(qf - q0))./vmax) + (vmax./a);

% round to upper integer
tf = ceil(max(tf));

% change in t
%dt = 0.01;

% time step
T = 0:dt:tf;

end