% velocityJacobian - Returns the velocity jacobian of the manipulator given 
%                    an array of links created by the createLink function and 
%                    the current joint variables. 
%
%   Jv = velocityJacobian( linkList, paramList )
%
%       By inputting the link list and the current state parameter list,
%       this function returns the velocity jacobian of this set of links.
%
%   linkList = the array consisting all the link structures, every
%              structure consists all the information need for the link
%   paramList = the array that consists the joint variables
%
% Michael Cheng
% CWID: 10820067
% MENG 544: Robot Mechanics: Kinematics, Dynamics, and Control
% 11/13/2016
function Jv = velocityJacobian( linkList, paramList )

H=dhFwdKine(linkList,paramList);
A=length(linkList);

%Putting paramList values in to linkList
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

R0_N=H(1:3,1:3);
TN_0=inv(H);

%Prellocating arrays
TN_i=sym(zeros(4,4,A));
z=sym(zeros(3,1,A));
d=sym(zeros(3,1,A));
JvN=sym(zeros(6,A));

%Getting the list of TN_i
T_cur=TN_0;
for n=1:1:A
    T_cur=T_cur*dhTransform(linkList(n).a,linkList(n).d,linkList(n).alpha,linkList(n).theta);
    TN_i(:,:,n)=T_cur;
end

%Getting the list of z from TN_i
for m=1:1:A
    z(:,:,m)=TN_i(1:3,1:3,m)*[0;0;1];
end

%Getting the list of d from TN_i
for p=1:1:A
    d(:,:,p)=-TN_i(1:3,4,p);
end

%Construct JvN with z and d
for q=1:1:A
    JvN(:,q)=[cross(z(:,:,q),d(:,:,q));z(:,:,q)];
end

%Get Jv from JvN
Jv=[R0_N zeros(3,3);zeros(3,3) R0_N]*JvN;

