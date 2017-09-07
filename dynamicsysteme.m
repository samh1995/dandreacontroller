function [stateDeriv]= dynamicsysteme(t, state, f,desirestate)
% Propagates quadrotor dyanimcs using ode45
% Inputs: time, state and control signal 
% Output: derivative of the state


  % new state (u,v,w,p,q,r,X,Y,Z,q0,q1,q2,q3) uvw in body fram
global g m IB Izzp IT l Dt Kf Kt  damping_ratio nat_freq  
global  wbar fbar wbbar nbar K
q = [state(10);state(11);state(12);state(13)]/norm(state(10:13));
R = quat2rotmat(q); %% from body to inertial
state = reshape(state,[max(size(state)),1]);
stateDeriv = zeros(13,1);

% %% Rotational Matrix 
%  Rx=[cos(state(2))*cos(state(3)), cos(state(3))*sin(state(2))*sin(state(1))-cos(state(1)*sin(state(3)), ...
%      cos(state(3))*cos(state(1))*sin((state(2))+sin((state(1))*sin((state(3))];
%  Ry=[cos(state(2))*sin(state(3)), sin(state(3))*sin(state(2))*sin(state(1))+cos(state(1)*cos(state(3)), ...
%      sin(state(3))*cos(state(1))*sin((state(2))-sin((state(1))*cos((state(3))];
%  Rz=[-sin(state(2)), cos(state(2))*sin(state(1)), cos(state(1))*cos(state(2))];
%  R=[Rx;Ry;Rz];
 %R=[cos(state(2))*cos(state(3)) cos(state(3))*sin(state(2))*sin(state(1))-cos(state(1))*sin(state(3)) cos(state(3))*cos(state(1))*sin(state(2))+sin(state(1))*sin(state(3));
  %  cos(state(2))*sin(state(3)) sin(state(3))*sin(state(2))*sin(state(1))+cos(state(1))*cos(state(3)) sin(state(3))*cos(state(1))*sin(state(2))-sin(state(1))*cos(state(3));
    %-sin(state(2)) cos(state(2))*sin(state(1)) cos(state(1))*cos(state(2))];
j=1;
for j=1:3
    if f(j)>3.8
       f(j)=3.8;
    end
   if f(j)<0.2
       f(j)=0.2;
   end
   w(j)=sqrt(abs(f(j))/Kf) ;
 
end
w(4)=0;
f(4)=0;
% fGravity = inv(R)*[0; 0; -m*g];                 % force of gravity, inertial fram
% fThrust = [0; 0; Kf*sum(w.^2)];   % force of thrust, expressed in inertial frame
fThrust = R*[0; 0; Kf*sum(w.^2)];

fGravity = [0; 0; -m*g];   
% stateDeriv(1:3) = (fGravity  +fThrust - m*cross(state(4:6),state(1:3)))/m;
stateDeriv(1:3) = (fGravity  +fThrust)/m;
stateDeriv(4)=(1/IB(1,1))*((f(2))*l-state(5)*state(6)*(IT(3,3)-IT(1,1))-Izzp*state(5)*(sqrt(abs(f(1))/Kf) +sqrt(abs(f(2))/Kf) +sqrt(abs(f(3))/Kf)+ sqrt(abs(f(4))/Kf) ));
stateDeriv(5)=(1/IB(1,1))*((f(3)-f(1))*l+state(4)*state(6)*(IT(3,3)-IT(1,1))+Izzp*state(4)*(sqrt(abs(f(1))/Kf) +sqrt(abs(f(2))/Kf) +sqrt(abs(f(3))/Kf)+ sqrt(abs(f(4))/Kf)));
stateDeriv(6)=(1/IB(3,3))*(-Dt*state(6)+Kt*(f(1)-f(2)+f(3)-f(4)));

% stateDeriv(8)=state(11);
% stateDeriv(9)=state(12);
stateDeriv(7:9)=state(1:3);
% stateDeriv(10:13) = -0.5*quatmultiply([0;state(4:6)],q);
stateDeriv(10:13) = 0.5*quatmultiply(q,[0;state(4:6)]);

% stateDeriv(10)=((cos(state(1))*cos(state(3))*sin(state(2))+sin(state(1))*sin(state(3)))/m)*(f(1)+f(2)+f(3)+f(4));
% stateDeriv(11)=((cos(state(1))*sin(state(3))*sin(state(2))-sin(state(1))*cos(state(3)))/m)*(f(1)+f(2)+f(3)+f(4));
% stateDeriv(12)=(1/m)*(((f(1)+f(2)+f(3)+f(4))*cos(state(1))*cos(state(2)))-m*g);
% stateDeriv(10)=((cos(state(1))*cos(state(3))*sin(state(2))+sin(state(1))*sin(state(3)))/m)*u(1)-(Dt/m)*state(10);
% stateDeriv(11)=((cos(state(1))*sin(state(3))*sin(state(2))-sin(state(1))*cos(state(3)))/m)*u(1)-(Dt/m)*state(11);
% stateDeriv(12)=(1/m)*(((f(1)+f(2)+f(3))*cos(state(1))*cos(state(2)))-m*g);


end
