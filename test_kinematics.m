%% Kinematics test
close all; clear; clc;

% Specify joint angles
q1 = 4;
q2 = 2;
q3 = 0.8;
q4 = 1;
q5 = 0;
q6 = 3;
q = [q1 q2 q3 q4 q5 q6];


%% Forward Kinematics Test
fprintf("#######################################################\n\n")
fprintf("            Direct Kinematics Test                     \n\n")
% solve the DKP for given joint variable values
[Tee, E]= direct_kine(dh, q);

% Actual computed using petercorke robotics-toolbox
robot = actual;
Tee_a = robot.fkine(q);

% Display result
disp("Computed fk: ")
[pos_computed, rpy_computed] = T2RPY(Tee);
fprintf("\tX\t\t\tY\t\tZ\t\tgama\t\tbeta\t\talpha\n")
disp([pos_computed rpy_computed])

disp("Actual fk: ")
rpy_actual = tr2rpy(Tee_a);
[~, pos_actual] = tr2rt(Tee_a);
disp([pos_actual' rpy_actual])

%% Inverse Kinematics Test
fprintf("\n\n###################################################\n\n")
fprintf("                  Inverse Kinematics Test              \n\n")
disp("Given q:")
disp(q)

Tee = direct_kine(dh, q);

qc = inv_kine(Tee, dh, q);
disp("Computed q:")
disp(qc)

qtb = robot.ikine(Tee);
disp("Using toolbox: ")
disp(qtb)

disp("Given Tee:")
disp(Tee)
disp("Computed Tee")
disp(direct_kine(dh, qc))

