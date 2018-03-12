% rotX - Generates a rotation matrix rotating about the X axis by theta.
%
%   R = rotX(theta) -
%
%       By inputing a theta, in radians, this function will generate a 
%       3x3 rotation matrix. When a vector is multiplied by this rotation 
%       matrix R, the vector will rotate about the X axis by theta. 
% 
%   R = the 3x3 rotation matrix about the X axis by theta
%   theta = the angle of rotation about the X axis, in radians
% 
% Michael Cheng
% CWID: 10820067
% MENG 544: Robot Mechanics: Kinematics, Dynamics, and Control
% 9/29/2016

function R = rotX(theta)
R=[1 0 0;0 cos(theta) -sin(theta);0 sin(theta) cos(theta)];
end