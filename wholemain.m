clear all;
clc;
format long
close all
initparams;
initequil;
global g m IB Izzp IT l Dt Kf Kt  damping_ratio nat_freq 
global  wbar fbar wbbar nbar K
    
 euler=[0.5;1;5] %roll pitch yaw xyz                  
 q0 = angle2quat(euler(3),euler(2),euler(1),'ZYX')

 state=[0 0 0 0 0 0 0 0 1 q0]';
 f=[1.25 1.25 1.25 0]';
   
 %desired state (u,v,w,p,q,r,X,Y,Z,q0,q1,q2,q3) uvw in body frame
 desiredstate=[0 0 0 0 0 0 1 1 5 1 0 0 0]';


 endTime =10;  % seconds
 dt = 1 / 200; % time step (Hz)
 t=0;
   
 Hist = inithist(state,t,f,desiredstate);
 W=1;

        
% LQR
[A,B,K]=SSsystem(); %finds K for LQR system % i guess its good 
%     C=[0 0 0 0];D=[0 0];
%     P=pole(idss(A-B*K, B,C,D))


for i = 0 : dt : endTime - dt
  
    % State Initialization
    q = [state(10);state(11);state(12);state(13)]/norm(state(10:13));
    R=quat2rotmat(q); %% from body to inertial
    state = reshape(state,[max(size(state)),1]); %make sure state is column vector
    Vinertial(1:3) = R*state(1:3) %R takes from body to inertial states 1:3 which arein body
    desiredVinertial(1:3) = R*desiredstate(1:3)

    %Outer position Controller
    errouter_d =[state(7)-desiredstate(7);state(8)-desiredstate(8);state(9)-desiredstate(9)];
    errouter_d_dot=[Vinertial(1)-desiredVinertial(1);Vinertial(2)-desiredVinertial(2);Vinertial(3)-desiredVinertial(3)]
    desired_accl=-2*damping_ratio*nat_freq*errouter_d_dot-(nat_freq)^2*errouter_d 
    

    % Computing n desired (eq45-46)
    f_total=norm(m/nbar(3)*(desired_accl-[0;0;-g]))
    n_desired=m/nbar(3)*inv(R)*(desired_accl-[0;0;-g])/norm(m/nbar(3)*(desired_accl-[0;0;-g]))
    s=[state(4)-wbbar(1);state(5)-wbbar(2);n_desired(1)-nbar(1);n_desired(2)-nbar(2)];
    u=-K*s

    % Computing desired thrusts 
    f=forces(u,f_total);
%     statederiv=dynamicsysteme(0, state,f,desiredstate)
    %accelinertialframe=(statederiv(1:3)+cross(state(4:6),state(1:3)))
    
    % Propagate dynamics.
    options = odeset('RelTol',1e-15); %tolerance   
    [t1ODE,stateODE]= ode45(@(t1ODE,stateODE) dynamicsysteme(t1ODE, stateODE, f,desiredstate),[i i+dt],state,options);
    state=stateODE(end,:)'
    t = t1ODE(end,:)- dt;
    q = [state(10);state(11);state(12);state(13)]/norm(state(10:13));
    R = quat2rotmat(q)
   
    
    Hist = updatehist(Hist, t, state, f);
    W=W+1
   
end

%%
% plotting(Hist.states, Hist.times,Hist.f);

 %% Visualize simulation.
%simvisualization(Hist.times, Hist.states, 'V1');