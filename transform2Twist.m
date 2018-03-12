% transform2Twist - Returns the twist vector corresponding to the
%                   provided homogenous transform matrix. 
%
%   t = transform2Twist(H)
%
%       With the input of a homogeneous transformation matrix, the function 
%       returns the twist function correspondingly, in the form of a 6x1 matrix
%       with the first 3 elements being the v vector and the last three elements 
%       being the w vector.
%
%   t = the 6 element input twist vector in the form of 6x1 matrix
%   H = the 4x4 homogeneous transformation matrix
%
% Michael Cheng
% CWID: 10820067
% MENG 544: Robot Mechanics: Kinematics, Dynamics, and Control
% 9/29/2016

function t = transform2Twist(H)
R = [H(1,1) H(1,2) H(1,3);H(2,1) H(2,2) H(2,3);H(3,1) H(3,2) H(3,3)];
theta = acos((trace(R)-1)./2);
Wunit = (1./(2.*sin(theta)))*[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];
w = Wunit.*theta;
Wunitcp = [0 -Wunit(3) Wunit(2);Wunit(3) 0 -Wunit(1);-Wunit(2) Wunit(1) 0];
v = inv(((eye(3)-R)*Wunitcp)+(theta*w*transpose(w)))*[H(1,4);H(2,4);H(3,4)];
t = [v;w];
end