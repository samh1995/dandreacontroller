clear all;
clc;
initparams;
initequil;
    global g m IB Izzp IT l Dt Kf Kt  damping_ratio nat_freq 
    
    global  wbar fbar wbbar nbar K
    state=[0 0 0 2 1 10 0 0 10 0 0 0]';
    f=[3 0 3 0]';
    desiredstate=[0 0 0 0 0 0 0 1 10 0 0 0];
   
    endTime = 2;  % seconds
    dt = 1 / 50; % time step (Hz)
t=0;
    %initialconditions=[iC(1);iC(2)-5.69;iC(3);iC(4)-0.289];
    %initialconditions=[11;2;0.95;0.3];

    Hist = inithist(state,t,f,desiredstate);
    W=1;

for i = 0 : dt : endTime - dt
    
    errouter_d =[state(7)-desiredstate(7);state(8)-desiredstate(8);state(9)-desiredstate(9)];
    errouter_d_dot=[state(10)-desiredstate(10);state(11)-desiredstate(11);state(12)-desiredstate(12)];
    desired_accl=-2*damping_ratio*nat_freq*errouter_d_dot-(nat_freq)^2*errouter_d ;
    
    %% Rotational Matrix 
R=[cos(state(2))*cos(state(3)) cos(state(3))*sin(state(2))*sin(state(1))-cos(state(1))*sin(state(3)) cos(state(3))*cos(state(1))*sin(state(2))+sin(state(1))*sin(state(3));
cos(state(2))*sin(state(3)) sin(state(3))*sin(state(2))*sin(state(1))+cos(state(1))*cos(state(3)) sin(state(3))*cos(state(1))*sin(state(2))-sin(state(1))*cos(state(3));
-sin(state(2)) cos(state(2))*sin(state(1)) cos(state(1))*cos(state(2))];



    f_total=m/nbar(3)*norm(desired_accl-[0;0;g])
    n_desired=m/nbar(3)*inv(R)*(desired_accl-[0;0;g])/f_total;
    
    %%LQR
    [A,B,K]=SSsystem(); %finds K for LQR system
    s=[state(4)-wbar(1);state(5)-wbar(2);n_desired(1)-nbar(1);n_desired(2)-nbar(2)];
    u=-K*s;
   % f = fsolve(@(x) forces(x,u,f_total), f);
    f=forces(u);
    options = odeset('RelTol',1e-3); %%tolerance
   % [tODE,y] = ode45(@(tODE, y) sys( y, A,B,K,tODE, dt),[i i+dt], initialconditions, options);
     [t1ODE,stateODE]= ode45(@(t1ODE,stateODE) dynamicsysteme(t1ODE, stateODE, dt, f),[i i+dt],state,options);
     state=stateODE(end,:)';
     t = t1ODE(end,:)- dt;
     
    Hist = updatehist(Hist, t, state, f);
   
    W=W+1
  
   %y(i,:)=y(i,:)-transpose(initialconditions); 
  
end

figure(1)
    plot(Hist.times,Hist.states(1,:))
    title('Roll value vs time (rads vs secs)')
    xlabel('t(s)')
    ylabel('Roll(rads)')
    saveas(figure (1),'state1.jpg')
figure(2)
    plot(Hist.times,Hist.states(2,:))
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
    plot(Hist.times,Hist.states(4,:))
    title('Roll rate vs time (rads/s vs secs)')
    xlabel('t(s)')
    ylabel('p (rads/s)')
    saveas(figure (4),'state4.jpg')
figure(5)
    plot(Hist.times,Hist.states(5,:))
    title('pitch rate vs time (rads/s vs secs)')
    xlabel('t(s)')
    ylabel('q (rads/s)')
    saveas(figure (5),'state5.jpg')
figure(6)
    plot(Hist.times,Hist.states(6,:))
    title('Yaw rate vs time (rads/s vs secs)')
    xlabel('t(s)')
    ylabel('r (rads/s)')
    saveas(figure (6),'state6.jpg')
figure(7)
    plot(Hist.times,Hist.states(7,:))
    title('x position vs time (m vs secs)')
    xlabel('t(s)')
    ylabel('x (m)')
    saveas(figure (7),'state7.jpg')
figure(8)
    plot(Hist.times,Hist.states(8,:))
    title('y position vs time (m vs secs')
    xlabel('t(s)')
    ylabel('y(m)')
    saveas(figure (8),'state8.jpg')
figure(9)
    plot(Hist.times,Hist.states(9,:))
    title('z position vs time (m vs secs')
    xlabel('t(s)')
    ylabel('z(m)')
    saveas(figure (9),'state9.jpg')

figure(10)
    plot(Hist.times,Hist.states(10,:))
    title('x rate vs time (m/s vs secs)')
    xlabel('t(s)')
    ylabel('x dot (m/s)')
    saveas(figure (10),'state10.jpg')
figure(11)
    plot(Hist.times,Hist.states(11,:))
    title('y  rate vs time (m/s vs secs)')
    xlabel('t(s)')
    ylabel('y dot (m/s)')
    saveas(figure (11),'state11.jpg')
figure(12)
    plot(Hist.times,Hist.states(12,:))
    title('z rate vs time (m/s vs secs)')
    xlabel('t(s)')
    ylabel('z dot (m/s)')
    saveas(figure (12),'state12.jpg')

figure(13)
  plot(Hist.times,Hist.f(1,:));hold on;
  plot(Hist.times,Hist.f(3,:));hold on;
  plot(Hist.times,Hist.f(2,:)); hold on
  plot(Hist.times,Hist.f(4,:))
  xlabel('t(s)')
  ylabel('fi(N)')
  title('4 propeller forces vs time ')
  saveas(figure (13),'Propellers.jpg')

figure(14)
 plot(Hist.times,Hist.f(4,:)) 
 title('Propeller 4 vs time')
 xlabel('t(s)')
 ylabel('f4(N)')
 saveas(figure (14),'Propellerf4.jpg')
         

% %    options = odeset('RelTol',1e-3); %%tolerance
% %     [tODE,errODE] = ode45(@(tODE, errODE) eq38error( errODE, tODE, dt),[i i+dt], err', options);