function dh_table = dh()
% DH generates the Denavit-Hartenberg (DH) parameter table of the manipulator
%
% Returns
% -------
% dh_table : matrix, shape (6, 4)
%     - Column 1: Joint angle offset (theta)
%     - Column 2: Link offset (d)
%     - Column 3: Link length (a)
%     - Column 4: Twist angle (alpha)
%

% Joint angles
q1_off = 0;
q2_off = -pi/2;
q3_off = -pi/2;
q4_off = 0;
q5_off = pi/2;
q6_off = 0;

% Joint offsets
d1 = 5; 
d2 = 0;
d3 = 0;
d4 = 13;
d5 = 0;
d6 = 10;

% Link lengths
a0 = 0;
a1 = 0;
a2 = 15;
a3 = 7.5;
a4 = 0;
a5 = 0;

% Twist angles
alp0 = 0;
alp1 = -pi/2;
alp2 = 0;
alp3 = -pi/2;
alp4 = pi/2;
alp5 = -pi/2;

% dh matrix
dh_table = [q1_off   d1   a0  alp0
            q2_off   d2   a1  alp1
            q3_off   d3   a2  alp2
            q4_off   d4   a3  alp3
            q5_off   d5   a4  alp4
            q6_off   d6   a5  alp5    
            ];
end