
function [] = plotting(Hist.states(:,:), Hist.times,Hist.f)


set(0,'DefaultFigureWindowStyle','docked')
figure(1)
    plot(Hist.times,Hist.states(1,:)); hold on
    plot(Hist.times,desiredstate(1)*ones(size(Hist.times)));
    title('Body Velocity in X')
    xlabel('t(s)')
    ylabel('m/s')
    saveas(figure (1),'state1.jpg')
figure(2)
    plot(Hist.times,Hist.states(2,:)); hold on
    plot(Hist.times,0*ones(size(Hist.times)));
    title('Body Velocity in y')
    xlabel('t(s)')
    ylabel('Pitch(rads)')
    saveas(figure (2),'state2.jpg')
figure(3)
    plot(Hist.times,Hist.states(3,:))
    title('Body Velocity in Z') 
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
    plot(Hist.times,Hist.f(1,:));hold on;
    plot(Hist.times,Hist.f(3,:));hold on;
    plot(Hist.times,Hist.f(2,:));
    xlabel('t(s)')
    ylabel('fi(N)')
    title('4 propeller forces vs time ')
    saveas(figure (10),'Propellers.jpg')
figure(11)
    plot(Hist.times,Hist.states(4,:)); hold on
    plot(Hist.times,Hist.states(5,:)); hold on
    plot(Hist.times,Hist.states(6,:));
    title('omega (pqr) vs time (rads/s vs secs)')
    xlabel('t(s)')
    ylabel('r (rads/s)')
    saveas(figure (11),'pqr.jpg') 
 figure(12)
    plot(Hist.times,Hist.states(10,:)); hold on
    plot(Hist.times,Hist.states(11,:)); hold on
    plot(Hist.times,Hist.states(12,:));hold on
    plot(Hist.times,Hist.states(13,:));
    title('quaterions vs time (rads/s vs secs)')
    xlabel('t(s)')
    ylabel('q')
    saveas(figure (12),'quaterion.jpg')
end