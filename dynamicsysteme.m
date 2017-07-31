function [stateDeriv]= dynamicsysteme(t, state, dt, f)
% Propagates quadrotor dyanimcs using ode45
% Inputs: time, state and control signal 
% Output: derivative of the state
  global g m IB Izzp IT l Dt Kf Kt  damping_ratio nat_freq 
    
    global  wbar fbar wbbar nbar K

stateDeriv = zeros(12,1);

%% Rotational Matrix 
% Rx=[cos(state(2))*cos(state(3)), cos(state(3))*sin(state(2))*sin(state(1))-cos(state(1)*sin(state(3)), ...
%     cos(state(3))*cos(state(1))*sin((state(2))+sin((state(1))*sin((state(3))];
% Ry=[cos(state(2))*sin(state(3)), sin(state(3))*sin(state(2))*sin(state(1))+cos(state(1)*cos(state(3)), ...
%     sin(state(3))*cos(state(1))*sin((state(2))-sin((state(1))*cos((state(3))];
% Rz=[-sin(state(2)), cos(state(2))*sin(state(1)), cos(state(1))*cos(state(2))];
% R=[Rx;Ry;Rz];

j=1;
for j=1:4
   w(j)=sqrt(abs(f(j))/Kf) ;
    
end


stateDeriv(1)=state(4)+state(5)*sin(state(1))*tan(state(2))+state(6)*cos(state(1))*tan(state(2));
stateDeriv(2)=state(5)*cos(state(1))-state(6)*sin(state(1));
stateDeriv(3)=1/cos(state(2))*(state(5)*sin(state(1))+state(6)*cos(state(1)));
stateDeriv(4)=(1/IB(1,1))*((f(2)-f(4))*l+state(5)*state(6)*(IT(3,3)-IT(1,1))+Izzp*state(5)*(sqrt(abs(f(1))/Kf) +sqrt(abs(f(2))/Kf) +sqrt(abs(f(3))/Kf)+ sqrt(abs(f(4))/Kf) ));
stateDeriv(5)=(1/IB(1,1))*((f(3)-f(1))*l+state(4)*state(6)*(IT(3,3)-IT(1,1))+Izzp*state(4)*(sqrt(abs(f(1))/Kf) +sqrt(abs(f(2))/Kf) +sqrt(abs(f(3))/Kf)+ sqrt(abs(f(4))/Kf)));
stateDeriv(6)=(1/IB(3,3))*(-Dt*state(6)+Kt*(f(1)-f(2)+f(3)-f(4)));
stateDeriv(7)=state(10);
stateDeriv(8)=state(11);
stateDeriv(9)=state(12);
stateDeriv(10)=((cos(state(1))*cos(state(3))*sin(state(2))+sin(state(1))*sin(state(3)))/m)*(f(1)+f(2)+f(3)+f(4));
stateDeriv(11)=((cos(state(1))*sin(state(3))*sin(state(2))-sin(state(1))*cos(state(3)))/m)*(f(1)+f(2)+f(3)+f(4));
stateDeriv(12)=(1/m)*(((f(1)+f(2)+f(3)+f(4))*cos(state(1))*cos(state(2)))-m*g);
% stateDeriv(10)=((cos(state(1))*cos(state(3))*sin(state(2))+sin(state(1))*sin(state(3)))/m)*u(1)-(Dt/m)*state(10);
% stateDeriv(11)=((cos(state(1))*sin(state(3))*sin(state(2))-sin(state(1))*cos(state(3)))/m)*u(1)-(Dt/m)*state(11);
% stateDeriv(12)=(1/m)*(((f(1)+f(2)+f(3))*cos(state(1))*cos(state(2)))-m*g);


end
