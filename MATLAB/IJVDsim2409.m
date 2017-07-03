%Seshadhri Srinivasan
%Final version of IJVD file
%static obstacle and hybrid controller
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
% Obs=[3 5; 4 3;12 10;14 14;15 16;18 14;19 20];
%Obs=[3 3; 8 4;5 4;12 6;14 12;14 4;18 7;17 14];
Obs=[3 3; 8 4;5 4;12 6;14 12;14 4;18 7;17 19];

place_obstacles(Obs);
plot(xgoal(1,1),xgoal(2,1),'r--O','MarkerSize',30);
Nsense=36; %no. of circular points to sense
theta(:,1)=0*ones(Nsense,1);
Xb=[];r=1;
Xb=sense_range(Nsense,theta,Obs,r)
sprintf('%s','you have successfully created the arena for AV')
sprintf('%s','you have successfully created the arena for AV')
U=[];
Error=[];
%2. Initialization
x_obs=Obs(:,:);
x_old=[0;0;0.5];
u=[1;0.5];
N_sim=300;epsilon1=1.5;
epsilon2=0.75;dt=1;kpv=0.1;
goal_delp=xgoal(1,1)+0.5;
goal_delm=xgoal(2,1)+0.5;
xnew=x_old;      
n_obs=size(Obs,1); %number of obstacles
X=[];Y=[];
%3. Simulation of MPC
for i=1:38
     iter=i; Error_sum=sum(Error);

    x_old=xnew;
     %find the distance
     %update the state of the vehicle
      

  if((xnew(1,1)<=goal_delp)&& (xnew(2,1)<=goal_delm))
      %Measure the current position
      x_dist=xnew(1:2,1);
      n_obs=size(Obs,1); %number of obstacles
    %find the distance
                for i=1:n_obs
                dist(i)=norm(x_dist-Obs(i,:)');
                end
    %Implement behaviour based control
   %find the min distance and the obstacle number
   [obs_dist,zdist_pred]=feval('SensorOutput',xnew',Obs);
     [min_dist,kmin]=min(obs_dist(1,:));
      x_obs=Obs(kmin,:)';
     x_obs=x_obs+rand(2,1)*0.5;
     place_obstacles_mov(x_obs);
     xdist=xnew(1:2,1);
     [M,z]=Newdist(x_obs,xdist);
     x_obs=M;
     %[min_dist,kmin]=min(obs_dist(1,:));
     
%    min_dist=dist(1);
%    x_obs=[2;2];
    %  [x_obs,min_dist]=Obsmeas(n_obs,x_dist,Obs,xnew)
      
      %use heuristic rules
        if(min_dist>=epsilon1)
        %GTG is to be selected
        fuel_cost=0.01;
        safety_cost=100;
        blending_cost1=10;
        blending_cost2=10;

      elseif(min_dist>=epsilon2 && min_dist<=epsilon1)
                  
              [u_fwCW]= FWCW(x_obs,xnew,dt)
              [u_fwCCW]= FWCCW(x_obs,xnew,dt)
              [uGTG]=GTG(xgoal,xnew,dt)
              dec1=sum(uGTG.*u_fwCW);
              dec2=sum(uGTG.*u_fwCCW);   
              if(dec1>0)
                  %clockwise follow wall needs to be selected
                  %u=u_fwCW;
                  fuel_cost=100;
                  safety_cost=100;
                  blending_cost1=0.01;
                  blending_cost2=100;
    
              elseif(dec2>0)
                  %Anti-clockwise follow wall needs to be selected
                  %u=u_fwCCW;
                  fuel_cost=100;
                  safety_cost=100;
                  blending_cost1=100;
                  blending_cost2=0.01;
              else
                  sprintf('%s','FW is wrong')
              end
      elseif(min_dist <=epsilon2)
            %Avoid obstacle has to be selected
            %[uA0]=AO(x_obs,xnew,dt);
             fuel_cost=100;
             safety_cost=0.01;  
             blending_cost1=100;
             blending_cost2=100;
      else
          sprintf('%s','check for the conditions')
        
        
   end

                  
  
    C1=fuel_cost;
    C2=safety_cost;
    C3=blending_cost1;
    C4=blending_cost2;
    
    %MILP selection
    f=[C1 C2 C3 C4];
    A=[1 1 1 1];
    B=[1];
    Aeq=[0 0 1 1];
    Beq=[1];
    M=[1,2];
    e=2^-24;
    %produce the IP solution
    f=[C1 C2 C3 C4];
          A=[1 1 0 0];
          B=[1];
          Aeq=[1 1 1 1];
          Beq=[1]
          %lb=[0;0;0;0];
          %ub=[1;1;1;1];
          M=[3,4];
          e=2^-24;
          [delta_i,v,s]= IP(f,A,B,Aeq,Beq,[],[],M,e)
          
          if(delta_i(1)>=1)
             [uGTG]=GTG(xgoal,xnew,dt);
              u=uGTG;
          elseif(delta_i(2)>=1)
              [uA0]=AO(x_obs,xnew,dt);
              u=uA0;
              elseif(delta_i(3)>=1)
                [u_fwCW]= FWCW(x_obs,xnew,dt);
              u=u_fwCW;
            
                   elseif (delta_i(4)>=1)
                 [u_fwCCW]= FWCCW(x_obs,xnew,dt);
                u=u_fwCCW;
         
          else
              sprintf('%s','something is perhaps wrong')
          end
      xnew=update_leader(x_old(1,1),x_old(2,1),x_old(3,1),u)' %xnew
      plot(xnew(1,1), xnew(2,1),'r-square','MarkerSize',10);
      hold on
      title('MILP based path-planning with dynamic obstacles')
      xlabel('x position')
      ylabel('y position')
    U=[U; u(2,1)].*dt; 
    X=[X xnew(1,1)];
      Y=[Y xnew(2,1)];
  else
      break;
  end
   
end
              T=size(X,2)-1;

          t=0:1:T
          figure(2)
          subplot(2,1,1)
          plot(t,X,'g-');
          hold on
          plot(t,Y,'r-')
          title('MILP based path-planning with dynamic obstacles')
          xlabel('time')
          ylabel('x y position')
          
          %figure(3)
          subplot(2,1,2)
          plot(t,U)
          xlabel('time')
          ylabel('\Phi in radians')
          
         
          
          
    
    
    
    %MILP based implementation
