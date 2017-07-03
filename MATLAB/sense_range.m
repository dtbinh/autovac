function Xb=sense_range(Nsense,theta,Obs,r)
Xb=[];
for m=2:Nsense  % Compute the angles to be used around the circle
	theta(m,1)=theta(m-1,1)+(pi/180)*(360/Nsense); 
end
cobs=size(Obs,1);
for kk=1:cobs
    Xobs=Obs(kk,:)
for m=1:Nsense
		xb(:,m)=[Xobs(1,1)+r*cos(theta(m,1)); Xobs(1,2)+r*sin(theta(m,1))];
 end 
 Xb=[Xb;xb(1,:);xb(2,:)]
 plot(xb(1,:),xb(2,:))
hold on        
end
