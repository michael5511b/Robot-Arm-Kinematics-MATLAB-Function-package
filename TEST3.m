link1=createLink(0,0,0,   [],[0;0;0]    ,0,    [0 0 0;0 0 0;0 0 0]);
link2=createLink(0,0,pi/2,[],[0;0;0.125],5.325,[0.031 0 0;0 0.031 0;0 0 0.00666]);
link3=createLink(1,0,0,   [],[0.5;0;0]  ,21.3, [1.788 0 0;0 1.788 0;0 0 0.027]);
link4=createLink(0.5,0,0, 0 ,[0.25;0;0] ,10.65,[0.229 0 0;0 0.229 0;0 0 0.013]);
linkList=[link1 link2 link3 link4];

paramList = [pi/2;2*pi/3;-pi/4;0];
paramListDot = [10;5;15;0];
paramListDDot = [2;-5;1;0];

baseDynamics.linA = [0;0;0];
baseDynamics.angV = [0;0;0];
baseDynamics.angA = [0;0;0];
endEffectorWrench = [0;0;0;0;0;0];
gravityDirection = [0;0;-1];

[jointTorques, Jv, JvDot] = newtonEuler( linkList,paramList, paramListDot, paramListDDot,baseDynamics, endEffectorWrench,gravityDirection );
disp(Jv)
disp((JvDot))
disp(jointTorques)