function T = dh_matrix(q, d, a, alpha)

T = [cos(q)                         -sin(q)                  0                  a;
     sin(q)*cospi(alpha/pi)  cos(q)*cospi(alpha/pi)   -sinpi(alpha/pi)     -d*sinpi(alpha/pi);
     sin(q)*sinpi(alpha/pi)  cos(q)*sinpi(alpha/pi)    cospi(alpha/pi)      d*cospi(alpha/pi);
           0                  0                 0                  1
    ];
end
