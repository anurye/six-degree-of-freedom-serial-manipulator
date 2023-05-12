function robot = actual()
% offset angle
q1_off = 0;
q2_off = -pi/2;
q3_off = -pi/2;
q4_off = 0;
q5_off = pi/2;
q6_off = 0;

% jont offset
d1 = 5;
d2 = 0;
d3 = 0;
d4 = 13;
d5 = 0;
d6 = 10;

% Link length
a0 = 0;
a1 = 0;
a2 = 15;
a3 = 7.5;
a4 = 0;
a5 = 0;

% twist angle
alpha0 = 0;
alpha1 = -pi/2;
alpha2 = 0;
alpha3 = -pi/2;
alpha4 = pi/2;
alpha5 = -pi/2;

% Define the links
l(1) = Link([0 d1 a0 alpha0], 'modified'); l(1).offset = q1_off;
l(2) = Link([0 d2 a1 alpha1], 'modified'); l(2).offset = q2_off;
l(3) = Link([0 d3 a2 alpha2], 'modified'); l(3).offset = q3_off;
l(4) = Link([0 d4 a3 alpha3], 'modified'); l(4).offset = q4_off;
l(5) = Link([0 d5 a4 alpha4], 'modified'); l(5).offset = q5_off;
l(6) = Link([0 d6 a5 alpha5], 'modified'); l(6).offset = q6_off;

% Robot object
robot = SerialLink(l, 'name', 'Arm');

end