% dhFwdKine1 - Returns the forward kinematics of a manipulator
%              with the provided DH parameter set. 
%
%   H = dhFwdKine1(linkList, paramList)
%
%       With the input of the links consist in the array linkList, and the
%       current states of the joint variables consist in the array
%       paramList, using the DH transform, this function will return the
%       homogeneous transformation matrix. In this first version of the
%       forward kinematics function, the function checks the isRotary
%       parameter to determine if the link is rotary or prismatic. If the
%       link is rotary, the variable is for the theta parameter. If the
%       link is prismatic, the variable is for the d parameter.
%
%   linkList = the array consisting all the link structures, every
%              structure consists all the information need for the link
%   paramList = the array that consists the variables of all the links
%
% Michael Cheng
% CWID: 10820067
% MENG 544: Robot Mechanics: Kinematics, Dynamics, and Control
% 9/29/2016

function H = dhFwdKine(linkList, paramList)
A = length(linkList);
%syms n m;
z=1;
for n = 1:1:A
    
    if linkList(n).isRotary == 1
        linkList(n).theta=paramList(z);
        z=z+1;
    elseif linkList(n).isRotary == 0
        linkList(n).d = paramList(z);
        z=z+1;
    else
        
    end
end
H = 1;
for m = 1:1:A
    H = H*dhTransform(linkList(m).a,linkList(m).d,linkList(m).alpha,linkList(m).theta);
end
end