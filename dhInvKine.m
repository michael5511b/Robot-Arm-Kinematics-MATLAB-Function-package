% dhInvKine - Returns the parameter list necessary to achieve a desired 
%             homogenous transform and the residual error in that transform. 
%
%   [paramList, error] = dhInvKine (linkList, desTransform, paramListGuess) 
%
%       With the input of the links consist in the array linkList, the
%       desired transformation matrix which represents the desired tool position, 
%       and an initial parameter guess, which will be corrected closer and
%       closer to the desired values throughout the function loop. The
%       function will return the final joint values and the residual error.
%
%   linkList = the array consisting all the link structures, every
%              structure consists all the information need for the link
%   desTransform = the desired transformation matrix of the desired tool
%                  position
%   paramListGuess = an initial guess of where the joint variables
%                    currently are, could be the joints current position
%   paramList = returns the desired joint variable positions
%   error = the residual error form the function loop
%
% Michael Cheng
% CWID: 10820067
% MENG 544: Robot Mechanics: Kinematics, Dynamics, and Control
% 11/13/2016

function [paramList, error] = dhInvKine (linkList, desTransform, paramListGuess) 

%desired
Rd = desTransform(1:3,1:3);
Pd = desTransform(1:3,4);
theta_d = acos((Rd(1,1)+Rd(2,2)+Rd(3,3)-1)./2);
k_d = (1./(2.*sin(theta_d)))*[Rd(3,2)-Rd(2,3);Rd(1,3)-Rd(3,1);Rd(2,1)-Rd(1,2)];
q_d1 = k_d(1).*sin(theta_d./2);
q_d2 = k_d(2).*sin(theta_d./2);
q_d3 = k_d(3).*sin(theta_d./2);
q_d0 = cos(theta_d./2);
Td = [Pd;q_d0;q_d1;q_d2;q_d3];

%check which link has no variables, ensure which joint variables don't change
A=length(linkList);
z=1;
for G=1:1:A
    if linkList(G).isRotary==2
        Y(z)=G;
        z=z+1;
    end
end

%current
q_current = paramListGuess;
go=true;
while go
curTransform = dhFwdKine(linkList, q_current);
Rc = curTransform(1:3,1:3);
Pc = curTransform(1:3,4);
theta_c = acos((Rc(1,1)+Rc(2,2)+Rc(3,3)-1)./2);
k_c = (1./(2.*sin(theta_c)))*[Rc(3,2)-Rc(2,3);Rc(1,3)-Rc(3,1);Rc(2,1)-Rc(1,2)];
q_c1 = k_c(1).*sin(theta_c./2);
q_c2 = k_c(2).*sin(theta_c./2);
q_c3 = k_c(3).*sin(theta_c./2);
q_c0 = cos(theta_c./2);
Tc = [Pc;q_c0;q_c1;q_c2;q_c3];
C = (1/2)*[-q_c1 -q_c2 -q_c3;q_c0 -q_c3 q_c2;q_c3 q_c0 -q_c1;-q_c2 q_c1 q_c0];
Jvc = velocityJacobian( linkList, q_current );
Jg = [eye(3) zeros(3);zeros(4,3) C]*Jvc;
%pseudo inverse
[U,S,V] = svd(Jg);
%[m,n]=size(Jg);
%r=rank(S);
%SR=S(1:r,1:r);
%SRc=[SR^-1 zeros(r,m-r);zeros(n-r,r) zeros(n-r,m-r)];
%Jg_pseuInv=V*SRc*U.';
Jg_pseuInv=V*pinv(S)*U';
e=Td-Tc;
q_delta=-Jg_pseuInv*e;
q_current=q_current-q_delta;
%the loop stops when all values in delta q is smaller than 0.001
if (abs(q_delta(:))<=0.001)
    go = false;
end
end

%joints that aren't rotary don't change, so those joint values remains same
%as the paramList
q_delta(Y)=0;
q_current(Y)=paramListGuess(Y);
paramList = q_current;
error = q_delta;

paramList = double(paramList);
error = double(error);
