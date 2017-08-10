clear all;
clc;
close all
initparams;
initequil;
    global g m IB Izzp IT l Dt Kf Kt  damping_ratio nat_freq 
    global  wbar fbar wbbar nbar K
    
  %% Initial Conditions
%     state=[0.133899759 0 0 0 2.4579 18.2494 0 0 5 0 0 0 ]';
 state=[0 0 0 0 0 0 0 0 1 0 0 0 ]';
    f=[1.25 1.25 1.25 0]';
    desiredstate=[0.133899759 0 0 0 0 0 4 5 8 0 0 0 ]';

    endTime = 30;  % seconds
    dt = 1 / 200; % time step (Hz)
    t=0;
    
    Hist = inithist(state,t,f,desiredstate);
    W=1;

        
    %% LQR
    [A,B,K]=SSsystem(); %finds K for LQR system
    
%     C=[0 0 0 0];D=[0 0];
%     P=pole(idss(A-B*K, B,C,D))
for i = 0 : dt : endTime - dt
    % Rotational Matrix 
    R=[cos(state(2))*cos(state(3)) cos(state(3))*sin(state(2))*sin(state(1))-cos(state(1))*sin(state(3)) cos(state(3))*cos(state(1))*sin(state(2))+sin(state(1))*sin(state(3));
    cos(state(2))*sin(state(3)) sin(state(3))*sin(state(2))*sin(state(1))+cos(state(1))*cos(state(3)) sin(state(3))*cos(state(1))*sin(state(2))-sin(state(1))*cos(state(3));
    -sin(state(2)) cos(state(2))*sin(state(1)) cos(state(1))*cos(state(2))];

%% Outer position Controller
    errouter_d =[state(7)-desiredstate(7);state(8)-desiredstate(8);state(9)-desiredstate(9)];
    errouter_d_dot=[state(10)-desiredstate(10);state(11)-desiredstate(11);state(12)-desiredstate(12)];
    desired_accl=-2*damping_ratio*nat_freq*errouter_d_dot-(nat_freq)^2*errouter_d 
    

    % Computing n desired (eq45-46)
    f_total=norm(m/nbar(3)*(desired_accl-[0;0;-g]))
    n_desired=m/nbar(3)*inv(R)*(desired_accl-[0;0;-g])/norm(m/nbar(3)*(desired_accl-[0;0;-g]))

    s=[state(4)-wbbar(1);state(5)-wbbar(2);n_desired(1)-nbar(1);n_desired(2)-nbar(2)];
    u=-K*s

   %% Computing desired thrusts 
   % f = fsolve(@(x) forces(x,u,f_total), f);
    f=forces(u,f_total)
    
    % Propagate dynamics.
    options = odeset('RelTol',1e-3); %tolerance   
    [t1ODE,stateODE]= ode45(@(t1ODE,stateODE) dynamicsysteme(t1ODE, stateODE, f,desiredstate),[i i+dt],state,options);
    %statederiv=dynamicsysteme(0, state,f,desiredstate)

state=stateODE(end,:)';
t = t1ODE(end,:)- dt;
     
    Hist = updatehist(Hist, t, state, f);
   
    W=W+1
   
   %y(i,:)=y(i,:)-transpose(initialconditions); 
  
end
%%
set(0,'DefaultFigureWindowStyle','docked')
figure(1)
    plot(Hist.times,Hist.states(1,:)); hold on

      plot(Hist.times,desiredstate(1)*ones(size(Hist.times)));
%      plot([0 10],[0.133899759 0.133899759]);
  title('Roll value vs time (rads vs secs)')
  xlabel('t(s)')
 ylabel('Roll(rads)')
    saveas(figure (1),'state1.jpg')
figure(2)
    plot(Hist.times,Hist.states(2,:)); hold on

      plot(Hist.times,0*ones(size(Hist.times)));
    title('pitch value vs time (rads vs secs)')
    xlabel('t(s)')
    ylabel('Pitch(rads)')
    saveas(figure (2),'state2.jpg')
figure(3)
    plot(Hist.times,Hist.states(3,:))
    title('Yaw value vs time (rads vs secs)') 
    xlabel('t(s)')
    ylabel('Yaw(rads)')
    saveas(figure (3),'state3.jpg')
    
figure(4)
    plot(Hist.times,Hist.states(4,:)); hold on
      plot(Hist.times,wbbar(1)*ones(size(Hist.times)));
    title('Roll rate vs time (rads/s vs secs)')
    xlabel('t(s)')
    ylabel('p (rads/s)')
    saveas(figure (4),'state4.jpg')
    
figure(5)
    plot(Hist.times,Hist.states(5,:));
    hold on
    plot(Hist.times,wbbar(2)*ones(size(Hist.times)));
    title('pitch rate vs time (rads/s vs secs)')
    xlabel('t(s)')
    ylabel('q (rads/s)')
    saveas(figure (5),'state5.jpg')
  
figure(6)
    plot(Hist.times,Hist.states(6,:));
    hold on
    plot(Hist.times,wbbar(3)*ones(size(Hist.times)));
    title('Yaw rate vs time (rads/s vs secs)')
    xlabel('t(s)')
    ylabel('r (rads/s)')
   saveas(figure (6),'state6.jpg')
figure(7)
    plot(Hist.times,Hist.states(7,:));
    hold on
    plot(Hist.times,desiredstate(7)*ones(size(Hist.times)));
    title('x position vs time (m vs secs)')
    xlabel('t(s)')
    ylabel('x (m)')
  saveas(figure (7),'state7.jpg')
figure(8)
    plot(Hist.times,Hist.states(8,:));
    hold on
    plot(Hist.times,desiredstate(8)*ones(size(Hist.times)));
    title('y position vs time (m vs secs')
    xlabel('t(s)')
    ylabel('y(m)')
 saveas(figure (8),'state8.jpg')
figure(9)
    plot(Hist.times,Hist.states(9,:));
    hold on
    plot(Hist.times,desiredstate(9)*ones(size(Hist.times)));
    title('z position vs time (m vs secs')
    xlabel('t(s)')
    ylabel('z(m)')
   saveas(figure (9),'state9.jpg')

figure(10)
    plot(Hist.times,Hist.states(10,:));
    hold on
    plot(Hist.times,0*ones(size(Hist.times)));
    title('x rate vs time (m/s vs secs)')
    xlabel('t(s)')
    ylabel('x dot (m/s)')
    saveas(figure (10),'state10.jpg')
figure(11)
    plot(Hist.times,Hist.states(11,:));
    hold on
    plot(Hist.times,0*ones(size(Hist.times)));
    title('y  rate vs time (m/s vs secs)')
    xlabel('t(s)')
    ylabel('y dot (m/s)')
    saveas(figure (11),'state11.jpg')
figure(12)
    plot(Hist.times,Hist.states(12,:));
    hold on
    plot(Hist.times,0*ones(size(Hist.times)));
    title('z rate vs time (m/s vs secs)')
    xlabel('t(s)')
    ylabel('z dot (m/s)')
   saveas(figure (12),'state12.jpg')

figure(13)
  plot(Hist.times,Hist.f(1,:));hold on;
  plot(Hist.times,Hist.f(3,:));hold on;
  plot(Hist.times,Hist.f(2,:));

  xlabel('t(s)')
  ylabel('fi(N)')
  title('4 propeller forces vs time ')
  saveas(figure (13),'Propellers.jpg')
%%
figure(14)
    plot(Hist.times,Hist.states(4,:)); hold on
    plot(Hist.times,Hist.states(5,:)); hold on
    
    plot(Hist.times,Hist.states(6,:));

    title('omega (pqr) vs time (rads/s vs secs)')
    xlabel('t(s)')
    ylabel('r (rads/s)')
   saveas(figure (14),'pqr.jpg')
 
 %% Visualize simulation.
%simvisualization(Hist.times, Hist.states, 'V1');