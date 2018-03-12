% createLink - Creates a structure with the DH parameters and some other
%              parameters.
%
%   L = createLink(a, d, alpha, theta, centOfMass, mass, inertia, isRotary)
%
%       With the input of DH parameters a, d, alpha and theta, and with the
%       input of the position of the center of mass, the mass of the link,
%       the inertia of the link, and the info of the type of link stored in
%       the isRotary parameter.
%
%   L = the structure consisting the information of the link
%   a, alpha, d, theta = the DH parameters
%   centOfMass = the position of the center of mass
%   mass = the mass of the link
%   inertia = the inertia of the link
%   isRotary = determine whether the link is rotary or prismatic, true (1)
%              for rotary, false (2) for prismatic
%
% Michael Cheng
% CWID: 10820067
% MENG 544: Robot Mechanics: Kinematics, Dynamics, and Control
% 9/29/2016

function L = createLink(a, d, alpha, theta, centOfMass, mass, inertia)



if isempty(theta)==1
    TF_isRotary=true;
elseif isempty(d)==1
    TF_isRotary=false;
else
    TF_isRotary=2;
end

L = struct('a', a, 'd', d, 'alpha', alpha, 'theta', theta, 'com', centOfMass, 'mass', mass, 'inertia', inertia, 'isRotary', TF_isRotary);
end