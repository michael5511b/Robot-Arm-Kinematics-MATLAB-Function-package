% twist2Transform - Returns the homogenous transformation matrix
%                   corresponding to a 6 element twist vector.
%
%   H = twist2Transform(t)
%
%      With the twist vector in the form of 6x1 matrix, which consists the
%      w and the v vectors, this function calculates theta, the unit w
%      vector, and the matrix packing of the cross product operator of unit
%      w vector.
% 
%   H = the homogeneous transformation matrix for the twist vector
%   t = the 6 element twist vector in the form of 6x1 matrix
%
% Michael Cheng
% CWID: 10820067
% MENG 544: Robot Mechanics: Kinematics, Dynamics, and Control
% 9/29/2016

function H = twist2Transform(t)
v = [t(1);t(2);t(3)];
w = [t(4);t(5);t(6)];
theta = (((t(4)).^2)+((t(5)).^2)+((t(6)).^2)).^(1/2);
Wunit = w./theta;
Wunitcp = [0 -Wunit(3) Wunit(2);Wunit(3) 0 -Wunit(1);-Wunit(2) Wunit(1) 0];
e = (cos(theta)*eye(3))+(sin(theta)*Wunitcp)+((1-cos(theta))*Wunit*transpose(Wunit));
d = (((eye(3)-e)*Wunitcp)+(theta*Wunit*transpose(Wunit)))*v;
g = [0 0 0 1];
H = [e d;g];
end


