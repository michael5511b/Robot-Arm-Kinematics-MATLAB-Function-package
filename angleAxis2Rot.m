% angleAxis2Rot - Returns the rotation matrix encoded by a rotation of theta 
%                 radians about the unit vector k axis.
%
%   R = angleAxis2Rot(k, theta)
%
%       by inputing the vector k as the rotation axis, and theta as the
%       rotation angle about the k axis, this function will return the
%       rotation matrix for this rotation. Using the unit quaternion
%       parameters.
%        
% 
%   R = the rotation matrix of the input rotation
%   k = the rotation axis in the form of 3x1 matrix
%   theta = the rotation angle about the k axis
%
% Michael Cheng
% CWID: 10820067
% MENG 544: Robot Mechanics: Kinematics, Dynamics, and Control
% 9/29/2016

function R = angleAxis2Rot(k, theta)
e1 = k(1).*sin(theta./2);
e2 = k(2).*sin(theta./2);
e3 = k(3).*sin(theta./2);
e4 = cos(theta./2);
R = [1-(2.*((e2).^2))-(2.*((e3).^2)) 2.*(e1.*e2-e3.*e4) 2.*(e1.*e3+e2.*e4);2.*(e1.*e2+e3.*e4) 1-(2.*((e1).^2))-(2.*((e3).^2)) 2.*(e2.*e3-e1.*e4);2.*(e1.*e3-e2.*e4) 2.*(e2.*e3+e1.*e4) 1-(2.*((e1).^2))-(2.*((e2).^2))];
end