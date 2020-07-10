function [F_traj, M_traj] = run_sim()
max_time=4;
tstep    = 0.01; % this determines the time step at which the solution is given
max_iter = max_time/tstep; % max iteration
time=0;
des_z=zeros(max_iter,1);
curr_z=zeros(max_iter,1);
ttraj=zeros(max_iter,1);
F_traj=zeros(max_iter,1);
M_traj=zeros(max_iter,3);
for iter=1:max_iter
    t=time;
    [posd, veld, rotd, omegad, controld] = time_trajj(t);
    [posc, velc, rotc, omegac] = local(t);
    [Thrust,M] = controller(posc, velc, rotc, omegac ,posd, veld, rotd, omegad, controld);
    des_z(iter)=posd(3);
    curr_z(iter)=posc(3);
    F_traj(iter,1)=Thrust;
    M_traj(iter,:)=M;
    ttraj(iter)=time;
    time=t+tstep;    
end
figure(1)
plot(ttraj,des_z,'r');
hold on
plot(ttraj,curr_z,'b');
ylabel('z')
disp('Simulation Finished.');
end