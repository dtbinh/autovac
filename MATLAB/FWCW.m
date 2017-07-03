function [u_fwCW]= FWCW(x_obs,xnew,dt)
uA0=AO(x_obs,xnew,dt)
alpha=0.5; %speeding up parameter
theta=90;
R=[cosd(theta) -sind(theta);sind(theta) cosd(theta)];
u_fwCW=alpha*R*uA0;
       
