function []= initequil()
initparams;
    global fbar wbar Kf  nbar wbbar Rps
    
%  fbar=[2.05; 1.02; 2.05 ;0];
%  for i=1:4
%  wbar(i)=sqrt(fbar(i)/Kf);
%  end
%  wbbar=[0;5.69;18.89];
%  nbar=[0;0.289;0.958];
% Rps=8; 

 wbar=[582;362;585;0];
 for i=1:4
     fbar(i)=Kf*wbar(i)^2;
 end
 wbbar=[0.2;4.3;19.5];
 nbar=[0.01;0.2155;0.97744];
Rps=6; 

end