function q = inv_kine(Td, dh, q_previous)
% Offset angle
q_off = dh(:, 1)';

% Previous joint angles take into account offset
%q_previous = q_previous - q_off;

% Link length
a = dh(:, 3);
a2 = a(3);
a3 = a(4);

% Joint offset
d = dh(:, 2);
d1 = d(1);
d4 = d(4);
d6 = d(6);

% Elements of the desired homogenous matrix
r11 = Td(1, 1); r12 = Td(1, 2); r13 = Td(1, 3);
r21 = Td(2, 1); r22 = Td(2, 2); r23 = Td(2, 3);
r31 = Td(3, 1); r32 = Td(3, 2); r33 = Td(3, 3);
qx = Td(1, 4);
qy = Td(2, 4);
qz = Td(3, 4);

% Let
zero = 1e-5;

% Joint 1
if abs(qy - d6*r23) < zero && abs(qx - d6*r13) < zero
    error("Singularity in joint 1")
else
    % Possible values of q1
    q11 = atan2(qy - d6*r23, qx - d6*r13);
    q12 = atan2(-(qy - d6*r23), -(qx - d6*r13));

    % Chose the one that is closer to the previous
    if abs(angdiff(q11, q_previous(1))) < abs(angdiff(q12, q_previous(1)))
        q1 = q11;
    else
        q1 = q12;
    end
end

% Joint 2
E = (qx - d6*r13)*cos(q1) + (qy - d6*r23)*sin(q1);
E2 = E^2;

F = d1 + d6*r33 - qz;
F2 = F^2;

K = (E2 + F2 + a2^2 - a3^2 - d4^2) / (2 * a2);

r = sqrt(E2 + F2);

if r < zero
    error("Singularity in joint 2");
else
    phi = atan2(F, E);
    % phi - q2
    phi_q2 = acos(K/r);

    if ~isreal(phi_q2)
        error("Point not reachable");
    end
    % Possible values of q2
    q21 = wrapToPi(phi - phi_q2);
    q22 = wrapToPi(phi - phi_q2);

    % Choose the one closer to the previous
    if abs(angdiff(q21, q_previous(2))) < abs(angdiff(q22, q_previous(2)))
        q2 = q21;
    else
        q2 = q22;
    end
end

% Joint 3
% Let
num = E*cos(q2) + F*sin(q2) - a2;
den = F*cos(q2) - E*sin(q2);

q3 = wrapToPi(atan2(a3, d4) - atan2(num, den));

% Joint 5
% Let
A = r13*cos(q1) + r23*sin(q1);

q5 = atan2(sqrt(1 - (-r33*cos(q2 + q3) - A*sin(q2 + q3))^2), -r33*cos(q2 + q3) - A*sin(q2 + q3));
q5 = wrapToPi(q5);

%{
q52 = atan2(sqrt(1 - (-r33*cos(q2 + q3) - A*sin(q2 + q3))^2), -r33*cos(q2 + q3) - A*sin(q2 + q3));
% Chose the one that is closer to the previous
if abs(angdiff(q51, q_previous(5))) < abs(angdiff(q52, q_previous(5)))
    q5 = q51;
else
    q5 = q52;
end
%}

% Joint 4
if abs(sin(q5)) < zero
    q4 = q_previous(4);
else
    % Let
    B = r23*cos(q1) - r13*sin(q1);
    q4 = atan2(B, r33*sin(q2 + q3) - A*cos(q2 + q3));
end

% Joint 6
if abs(sin(q5)) < zero
    % Let
    q6 = atan2(r21*cos(q1) - r11*sin(q1), r22*cos(q1) - r12*sin(q1));
else
    % Let
    c1 = cos(q1); s1 = sin(q1);
    c2 = cos(q2); s2 = sin(q2);
    c3 = cos(q3); s3 = sin(q3);
    c4 = cos(q4); s4 = sin(q4);
    c5 = cos(q5); s5 = sin(q5);

    num = r12*(c5*s1*s4 + c1*(c2*(c3*c4*c5 - s3*s5) - s2*(c4*c5*s3 + c3*s5)))...
        -r22*(c4*c5*s1*s2*s3 + c1*c5*s4 + c3*s1*s2*s5 + c2*s1*(s3*s5 - c3*c4*c5))...
        +r32*(s3*(s2*s5 - c2*c4*c5) - c3*(c4*c5*s2 + c2*s5));
    num = -num;

    den = r11*(c5*s1*s4 + c1*(c2*(c3*c4*c5 - s3*s5) - s2*(c4*c5*s3 + c3*s5)))...
        -r21*(c4*c5*s1*s2*s3 + c1*c5*s4 + c3*s1*s2*s5 + c2*s1*(s3*s5 - c3*c4*c5))...
        +r31*(s3*(s2*s5 - c2*c4*c5) - c3*(c4*c5*s2 + c2*s5));

    q6 = atan2(num, den);

end

% Therefore,
q = [q1 q2 q3 q4 q5 q6];

%{
for i = 1:length(q)
    if q(i) > 0
        q(i) = q(i) - q_off(i);
    else
        q(i) = q(i) + q_off(i);
    end
end
%}

q = wrapToPi(q - q_off);

end