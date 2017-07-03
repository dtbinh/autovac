function [uA0]=AO(x_obs,xnew,dt)
v_cal2= 1; 
phi_cal2=-(atan2(x_obs(2,1)-xnew(2),x_obs(1,1)-xnew(1))-pi)*dt;
uA0=[v_cal2;phi_cal2];
   