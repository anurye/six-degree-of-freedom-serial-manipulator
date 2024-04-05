function [Tee, E] = direct_kine(dh, q)
% Retruns the direct kinematics interms of homogenous matrix also interms
% of euler angles in two outputs are specified (if nargout == 2)  

% Offset angle
q_off = dh(:, 1);

% Joint angle
q1 = q(1) + q_off(1);
q2 = q(2) + q_off(2);
q3 = q(3) + q_off(3);
q4 = q(4) + q_off(4);
q5 = q(5) + q_off(5);
q6 = q(6) + q_off(6);

% Link length
a = dh(:, 3);
a2 = a(3);
a3 = a(4);

% Joint offset
d = dh(:, 2);
d1 = d(1);
d4 = d(4);
d6 = d(6);

% Let
s1 = sin(q1); s2 = sin(q2); s4 = sin(q4); s5 = sin(q5); s6 = sin(q6);
c1 = cos(q1); c2 = cos(q2); c4 = cos(q4); c5 = cos(q5); c6 = cos(q6);
s23 = sin(q2 + q3); c23 = cos(q2 + q3);

% n
r11 = s6*(c4*s1 - c23*c1*s4) + c6*(c5*(s1*s4 + c23*c1*c4) - s23*c1*s5);
r21 = -s6*(c1*c4 + c23*s1*s4) - c6*(c5*(c1*s4 - c23*c4*s1) + s23*s1*s5);
r31 = s23*s4*s6 - c6*(c23*s5 + s23*c4*c5);

% o
r12 = c6*(c4*s1 - c23*c1*s4) - s6*(c5*(s1*s4 + c23*c1*c4) - s23*c1*s5);
r22 = s6*(c5*(c1*s4 - c23*c4*s1) + s23*s1*s5) - c6*(c1*c4 + c23*s1*s4);
r32 = s6*(c23*s5 + s23*c4*c5) + s23*c6*s4;

% k
r13 = -s5*(s1*s4 + c23*c1*c4) - s23*c1*c5;
r23 = s5*(c1*s4 - c23*c4*s1) - s23*c5*s1;
r33 = s23*c4*s5 - c23*c5;

% Position
qx = c1*(a3*c23 - d4*s23 + a2*c2) - d6*(s5*(s1*s4 + c23*c1*c4) + s23*c1*c5);
qy = s1*(a3*c23 - d4*s23 + a2*c2) + d6*(s5*(c1*s4 - c23*c4*s1) - s23*c5*s1);
qz = d1 - d4*c23 - a3*s23 - a2*s2 - d6*(c23*c5 - s23*c4*s5);

% Homogenous matrix
T = [r11    r12     r13    qx;
     r21    r22     r23    qy;
     r31    r32     r33    qz;
       0      0       0     1];

% End effector euler angles
r11 = T(1, 1); r12 = T(1, 2); r13 = T(1, 3);
r21 = T(2, 1); r22 = T(2, 2); r23 = T(2, 3);
r31 = T(3, 1); r32 = T(3, 2); r33 = T(3, 3);

% Direct kinematics in terms of Euler angles and  end-effector position
% Let
zero = 1e-5;

if abs(abs(r33) - 1) > zero
    % sin(theta) != 0
    psi = atan2(r32, -r31);
    phi = atan2(r23, r13);
    theta = atan2(sqrt(r13^2 + r23^2), r33);

elseif abs(abs(r33) - 1) < zero && r33 > 0
    % cos(theta) = 1
    theta = 0;
    psi = 0;
    phi = atan2(r21, r11);

elseif abs(abs(r33) - 1) < zero && r33 < 0
    % cos(theta) = -1
    theta = pi;
    phi = 0;
    psi = atan2(r12, r22);
end

% Direct kinematics interms of position and euler angles
pos_plus_euler = [qx, qy, qz, phi, theta, psi];

switch nargout
    case 1
        Tee = T;
    case 2
        Tee = T;
        E = pos_plus_euler;
end

end