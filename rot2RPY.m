% rot2RPY - Generates the roll, pitch, yaw angles from a rotational transformation matrix.
%
%   [roll, pitch, yaw] = rot2RPY(R) -
%
%       By inputing the rotation matrix, this function will generate the 
%       roll, pitch, yaw angles of this rotation matrix in an output array. 
% 
%   roll = the angle of rotation about the X axis, in radians
%   pitch = the angle of rotation about the Y axis, in radians 
%   yaw = the angle of rotation about the Z axis, in radians
%   R = the input 3x3 rotation matrix
%
% Michael Cheng
% CWID: 10820067
% MENG 544: Robot Mechanics: Kinematics, Dynamics, and Control
% 9/29/2016

function [roll, pitch, yaw] = rot2RPY(R)
pitch = atan2(-R(3,1), (((R(1,1)).^2) + ((R(2,1)).^2)).^(1/2));
yaw = atan2(R(2,1)./cos(pitch), R(1,1)./cos(pitch));
roll = atan2(R(3,2)./cos(pitch), R(3,3)./cos(pitch));
end