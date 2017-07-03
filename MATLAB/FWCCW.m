function [u_fwCCW] =FWCCW(x_obs,xnew,dt)
uA0=AO(x_obs,xnew,dt)
alpha=0.1;
theta=90;
R1=[cosd(-theta) -sind(-theta);sind(-theta) cosd(-theta)];
u_fwCCW=alpha*R1*uA0;
       