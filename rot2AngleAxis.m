% rot2AngleAxis - Returns the angle and axis corresponding to a
%                 rotation matrix.
%
%   [k, theta] = rot2AngleAxis(R)
%
%       by inputing the rotation matrix, the function will return the
%       array, consisting the corresponding rotation axis k, and rotation 
%       angle theta
%
%   
%   k = the rotation axis in the form of 3x1 matrix
%   theta = the rotation angle about the k axis
%   R = the input rotation matrix
%
% Michael Cheng
% CWID: 10820067
% MENG 544: Robot Mechanics: Kinematics, Dynamics, and Control
% 9/29/2016

function [k, theta] = rot2AngleAxis(R)

theta = acos((R(1,1)+R(2,2)+R(3,3)-1)./2);
k = (1./(2.*sin(theta)))*[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];

end