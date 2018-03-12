% cpMatrix - Returns the matrix packing of the cross product operator.
%
%   X = cpMatrix(w)
%
%       By inputing a vector, this function will return a 3x3 matrix, this
%       matrix multiplied by another vector, will generate the cross
%       product of the two vectors.
%        
% 
%   X = the matrix packing the cross product operator of the input vector
%   w = the input vector in the form of a 3x1 matrix
%
% Michael Cheng
% CWID: 10820067
% MENG 544: Robot Mechanics: Kinematics, Dynamics, and Control
% 9/29/2016

function X = cpMatrix(w)
X=[0 -w(3) w(2);w(3) 0 -w(1);-w(2) w(1) 0];
end