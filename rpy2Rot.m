% rpy2Rot - Generates a rotational transformation matrix from roll, pitch, yaw angles.
%
%   R = rpy2Rot(roll, pitch, yaw) -
%
%       By inputing roll, pitch, yaw angles, in radians, this function will 
%       generate a transformation matrix. When a vector is multiplied by this 
%       rotation matrix R, the vector will rotate about X, Y, Z axis by the 
%       angle of roll, pitch, yaw. 
% 
%   R = the 3x3 rotational transformation matrix after roll, pitch, yaw rotations
%   roll = the angle of rotation about the X axis, in radians
%   pitch = the angle of rotation about the Y axis, in radians 
%   yaw = the angle of rotation about the Z axis, in radians
%
% Michael Cheng
% CWID: 10820067
% MENG 544: Robot Mechanics: Kinematics, Dynamics, and Control
% 9/29/2016

function R = rpy2Rot (roll, pitch, yaw)
R=[cos(yaw) -sin(yaw) 0;sin(yaw) cos(yaw) 0;0 0 1]*[cos(pitch) 0 sin(pitch);0 1 0;-sin(pitch) 0 cos(pitch)]*[1 0 0;0 cos(roll) -sin(roll);0 sin(roll) cos(roll)];
end