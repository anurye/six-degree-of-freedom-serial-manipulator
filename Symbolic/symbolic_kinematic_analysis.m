clear;clc;
syms q1 q2 q3 q4 q5 q6...
     d1 d4 d6...
     a2 a3...
     r11 r12 r13...
     r21 r22 r23...
     r31 r32 r33...
     qx  qy  qz...
     px  py  pz

% Desired endeffector orientation
Rd60 = [r11 r12 r13;
        r21 r22 r23;
        r31 r32 r33];
% Desired endeffector position
q = [qx; qy; qz];

% Wrist center
p = [px py pz];

% Homogenous transformation matrices
T10 = dh_matrix(q1,   d1,    0,      0);
T21 = dh_matrix(q2,    0,    0,  -pi/2);
T32 = dh_matrix(q3,    0,   a2,      0);
T43 = dh_matrix(q4,   d4,   a3,  -pi/2);
T54 = dh_matrix(q5,    0,    0,   pi/2);
T65 = dh_matrix(q6,   d6,    0,  -pi/2);


% Intermediate matrices
T20 = simplify(T10*T21);
T30 = simplify(T20*T32);
T40 = simplify(T30*T43);
T50 = simplify(T40*T54);
T60 = simplify(T50*T65);

%% Jacobian
[z1, p1] = findJ(T10);
[z2, p2] = findJ(T20);
[z3, p3] = findJ(T30);
[z4, p4] = findJ(T40);
[z5, p5] = findJ(T50);
[z6, p6] = findJ(T60);

% Jw
Jw = [z1 z2 z3 z4 z5 z6];

% Jv
P6_1 = simplify(p6 - p1);
P6_2 = simplify(p6 - p2);
P6_3 = simplify(p6 - p3);
P6_4 = simplify(p6 - p4);
P6_5 = simplify(p6 - p5);
P6_6 = simplify(p6 - p6);

jv1 = simplify(cross(z1, P6_1));
jv2 = simplify(cross(z2, P6_2));
jv3 = simplify(cross(z3, P6_3));
jv4 = simplify(cross(z4, P6_4));
jv5 = simplify(cross(z5, P6_5));
jv6 = simplify(cross(z6, P6_6));

Jv = [jv1 jv2 jv3 jv4 jv5 jv6];

% J
J = [Jv; Jw];

% determinant of jacobian
deter = simplify(det(J));

%% Inverse kinematics
Td60 = [r11     r12     r13     qx;
        r21     r22     r23     qy;
        r31     r32     r33     qz;
        0       0       0       1];
% Solve for q1, isolate q1
T51 = simplify(T10\T50);
Td50 = simplify(Td60/T65);
Td51 = simplify(T10\Td50);
% 51T24 and 51Td24


% Solve for q2
T41 = simplify(T51/T54);
Td41 = simplify(Td51/T54);
% 41T14 and 41Td14, 41T34 and 41Td34


% Solve for q3
T52 = simplify(T21\T51);
Td52 = simplify(T21\Td51);
% 52T24 and 52Td24, 52T14 and 52Td14


% Solve for q5
T53 = simplify(T32\T52);
Td53 = simplify(T32\Td52);
% 53T22 and 53Td22

% Solve for joint 4
% 53T32 and 53Td32, 53T12 and 53Td12

% Solve for joint 6
% 65T11, 65T31 
Td65 = simplify(T50\Td60);


% Singularity sin(q5) = 0
T61 = simplify(T10\T60);
Td61 = simplify(T10\Td60);
