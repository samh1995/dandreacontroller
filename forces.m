%function x= forces(x,u,f_total)
function x= forces(u)
initparams;
initequil;
 global g m I Ixx Iyy Izz Izzp Ixxp IT Ip  l Dt Kf Kt 
 global  wbar fbar wbbar nbar K
    G=[1 1 1; -1 0 1; 0 1 0];
    H=[(fbar(1)+fbar(2)+fbar(3)); u(1)-fbar(1)+fbar(3);u(2)+fbar(2)];
%     [1 1 1; -1 0 1; 0 1 0]*[ x(1) x(2) x(3)]'=[(fbar(1)+fbar(2)+fbar(3)); u(1)-fbar(1)+fbar(3);u(2)+fbar(2)];
%     f(1)=u(1)-x(3)+fbar(3)+x(1)-fbar(1);
%     f(2)= -x(2) + u(2)+fbar(2);
%     f(3)=f_total-x(1) -x(2) -x(3) ;
x=linsolve(G,H);
x(4)=0
end