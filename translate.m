function T = translate(pos)
% Returns a transformation matrix representing translation
T = eye(4);
T(1:3, 4) = pos';
end