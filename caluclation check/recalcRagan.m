

clear all;
clc;
close all
attEuler=[1; 0.3 ;0] %roll pitch yaw xyz                    zyx

 q0 = angle2quat( 0, 0.3 ,1,'zyx') %confirmed 
%  rotmat0=quat2rotm(q0)
 state=[0 0 0 0 0 0 0 0 1 q0]';
 q = [state(10);state(11);state(12);state(13)]/norm(state(10:13))
rotMat = quat2rotm(q')
rotMat2=quat2rotmat(q);
state=[1 0.3 0];
%rotm = eul2rotm([0 0.3 1],'zyx') 
R=[cos(state(2))*cos(state(3)) cos(state(3))*sin(state(2))*sin(state(1))-cos(state(1))*sin(state(3)) cos(state(3))*cos(state(1))*sin(state(2))+sin(state(1))*sin(state(3));
    cos(state(2))*sin(state(3)) sin(state(3))*sin(state(2))*sin(state(1))+cos(state(1))*cos(state(3)) sin(state(3))*cos(state(1))*sin(state(2))-sin(state(1))*cos(state(3));
    -sin(state(2)) cos(state(2))*sin(state(1)) cos(state(1))*cos(state(2))]

