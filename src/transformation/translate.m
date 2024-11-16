function T = translate(pos)
%TRANSLATE creates a homogeneous transformation matrix for a translation
%
% Parameters
% ----------
% pos : list
%     A vector specifying the translation along the x, y, and z axes:
%
% Returns
% -------
% T : matrix
%   The homogeneous transformation matrix representing the translation:
%

T = eye(4);
T(1:3, 4) = pos';

end