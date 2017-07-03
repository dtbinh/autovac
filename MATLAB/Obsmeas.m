function [x_obs,min_dist]=Obsmeas(n_obs,x_dist,Obs,xnew)
for i=1:n_obs
      dist(i)=norm(x_dist-Obs(i,:)');
      end
                 [obs_dist,zdist_pred]=feval('SensorOutput',xnew,Obs);
                 [min_dist,kmin]=min(obs_dist(1,:));
                  x_obs=Obs(kmin,:)';
end
      