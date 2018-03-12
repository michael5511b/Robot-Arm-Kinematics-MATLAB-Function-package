% dhTransform - Returns the homogenous transform corresponding
%               to the provide DH parameters for a link.
%
%   H = dhTransform(a, d, alpha, theta)
%
%       With the input of DH parameters a, d, alpha and theta, this
%       function will return the homogeneous transformation matrix, which
%       is the product of transX(a), rotX(alpha), transZ(d), rotZ(theta).
%
%   a, alpha, d, theta = the DH parameters
%   H = the 4x4 homogeneous transformation matrix corresponding to the DH
%       parameters
%
% Michael Cheng
% CWID: 10820067
% MENG 544: Robot Mechanics: Kinematics, Dynamics, and Control
% 9/29/2016

function H = dhTransform(a, d, alpha, theta)
H = [1 0 0 a;0 1 0 0;0 0 1 0;0 0 0 1] * [rotX(alpha) [0;0;0];0 0 0 1] * [1 0 0 0;0 1 0 0;0 0 1 d;0 0 0 1] * [rotZ(theta) [0;0;0];0 0 0 1];
end