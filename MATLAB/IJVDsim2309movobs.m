%AV path planning with Moving obstacles
%Seshadhri Srinivasan 23.09.2014
%Final version of IJVD file
%not complete has some problem
%clear 
clear all;close all;clc;
sprintf('%s', 'Behaviour based path-planning using MILP')
%1. prepare the arena
sprintf('%s','prepare the arena for AV')
xgoal=[20;20];
x_0=0;y_0=0;
init_pos=[x_0;y_0];
plot(x_0,y_0,'square','MarkerSize',15)
hold on
%this piece of code should be replaced for dynamic obstacles
Obs=[3 4; 4 3;12 10;14 11;14 14;18 16;17 19];
place_obstacles(Obs);
hold on
plot(xgoal(1,1),xgoal(2,1),'r--O','MarkerSize',30);
Nsense=36; %no. of circular points to sense
theta(:,1)=0*ones(Nsense,1);
Xb=[];r=1;
Xb=sense_range(Nsense,theta,Obs,r)
sprintf('%s','you have successfully created the arena for AV')

%2. Initialization
x_obs=Obs(:,:);
x_old=[0;0;0.5];
u=[1;0.5];
N_sim=300;epsilon1=1.5;
epsilon2=0.75;dt=1;kpv=0.1;
goal_delp=20.5;
goal_delm=20.5;
xnew=x_old;      
n_obs=size(Obs,1); %number of obstacles
epsilon3=3;

%3. Simulation of MPC
for i=1:70
    iter=i;
     x_old=xnew;
     %find the distance
     %update the state of the vehicle
      
     

     
  if((xnew(1,1)<=goal_delp)&& (xnew(2,1)<=goal_delm))
      %Measure the current position
      x_dist=xnew(1:2,1);
      n_obs=size(Obs,1); %number of obstacles
      Obs=Obs;%rand(n_obs,2)
      
      %find the distance
                for i=1:n_obs
                dist(i)=norm(x_dist-Obs(i,:)');
                end
    %Implement behaviour based control
   %find the min distance and the obstacle number
    [obs_dist,zdist_pred]=feval('SensorOutput',xnew',Obs);
     [min_dist,kmin]=min(obs_dist(1,:));
     x_obs=Obs(kmin,:)';
     x_obs=x_obs+rand(2,1);
     place_obstacles_mov(x_obs);
     hold on
     
%    min_dist=dist(1);
%    x_obs=[2;2];
    %  [x_obs,min_dist]=Obsmeas(n_obs,x_dist,Obs,xnew)
      
      %use heuristic rules
      
    
          
      
  if(min_dist>=epsilon1)
           fuel_cost=0.01
           safety_cost=0;  
          safety_limit=100;
%         blending_limit=100;
%         blending_cost=100;
         [uGTG]=GTG(xgoal,xnew,dt)
         u=uGTG
elseif(min_dist>=epsilon2 && min_dist<=epsilon1)
              [u_fwCW]= FWCW(x_obs,xnew,dt)
              [u_fwCCW]= FWCCW(x_obs,xnew,dt)
              [uGTG]=GTG(xgoal,xnew,dt)
              dec1=sum(uGTG.*u_fwCW);
              dec2=sum(uGTG.*u_fwCCW);   
              if(dec1>0)
                  u=u_fwCW;
              else
                  u=u_fwCCW;
             end
        else(min_dist <=epsilon2)
          [uA0]=AO(x_obs,xnew,dt);
      end
     % elseif (xnew(1,1)<=goal_delp)&& (xnew(2,1)>goal_delm)
%         [u_fwCCW]= FWCCW(x_obs,xnew,dt);   
%         u=u_fwCCW
%      
%           
%      elseif (xnew(1,1)>goal_delp)&& (xnew(2,1)<=goal_delm);
%        [u_fwCW]= FWCW(x_obs,xnew,dt);
%              u=u_fwCW;
%         
  else
          break
                  
  end

      xnew=update_leader(x_old(1,1),x_old(2,1),x_old(3,1),u)' %xnew
      plot(xnew(1,1), xnew(2,1),'square','MarkerSize',10);
      hold on
      
      
          
          
    
      
      
      
      
      
      

    
    

      end
    
    
    

