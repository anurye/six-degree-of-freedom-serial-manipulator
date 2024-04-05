%% This script is for ploting LSBP trajectory
clear;
clc;
%% 1. For a single joint
q0 = 0;
qf = pi;
T = get_time(q0, qf);
[q, dq] = LSPB(q0, qf, T);

figure;
subplot(211)
plot(q)
xlabel("t(s)")
ylabel("q(rad)")
grid
title("Position")
axis tight

subplot(212)
plot(dq)
xlabel("t(s)")
ylabel("vel")
grid
title("Velocity")
axis tight

%% 2. for multiple joints
q0 = zeros(1, 6);
qf = [pi pi/2 pi/4 pi/6 pi/8 pi/10];

T = get_time(q0, qf);
[Q, dQ, ddQ] = joint_traj(@LSPB, q0, qf, T);

figure;
subplot(211)
plot(Q)
xlabel("t(s)")
ylabel("q(rad)")
grid
title("Position")
axis tight
legend("q1", "q2", "q3", "q4", "q5", "q6", "Location","west")

subplot(212)
plot(dQ)
xlabel("t(s)")
ylabel("vel")
grid
title("Velocity")
axis tight

