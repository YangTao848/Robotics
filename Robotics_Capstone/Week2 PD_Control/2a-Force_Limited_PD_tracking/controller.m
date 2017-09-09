
function u = controller(params, t, x, xd)
  % Use params.traj(t) to get the reference trajectory
  % e.g. (x - params.traj(t)) represents the instaneous trajectory error

  % params can be initialized in the initParams function, which is called before the simulation starts
  
  % SOLUTION GOES HERE -------------
  kd=30; kp=100; eps=0.0001;
  v_des=(params.traj(t+eps)-params.traj(t-eps))/(2*eps);
  a_des=(params.traj(t+eps)+params.traj(t-eps)-2*params.traj(t))/(eps^2);
  u = kp* (params.traj(t)-x) +kd*(v_des-xd)+a_des;
  
  
end