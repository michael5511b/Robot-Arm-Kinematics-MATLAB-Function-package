% rotY - Generates a rotation matrix rotating about the Y axis by theta.
%
%   R = rotY(theta) -
%
%       By inputing a theta, in radians, this function will generate a 
%       3x3 rotation matrix. When a vector is multiplied by this rotation 
%       matrix R, the vector will rotate about the Y axis by theta. 
% 
%   R = the 3x3 rotation matrix about the Y axis by theta
%   theta = the angle of rotation about the Y axis, in radians
% 
% Michael Cheng
% CWID: 10820067
% MENG 544: Robot Mechanics: Kinematics, Dynamics, and Control
% 9/29/2016

function R = rotY(theta)
R=[cos(theta) 0 sin(theta);0 1 0;-sin(theta) 0 cos(theta)];
end