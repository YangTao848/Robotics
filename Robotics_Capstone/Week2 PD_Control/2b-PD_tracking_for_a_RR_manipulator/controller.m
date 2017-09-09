
function u = controller(params, t, X)
  % 1. write out the forward kinematics, such that p = FK(theta1, theta2)
  % 2. Let e = p - params.traj(t) be the task-space error
  % 3. Calculate the manipulator Jacobian J = d p / d theta
  % 4. Use a "natural motion" PD controller, u = - kp * J^T * e - kd * [dth1; dth2]
  th1=X(1); th2=X(2);dth1=X(3); dth2=X(4);
  l=params.l; kp=3000;kd=2;
  p=[l*cos(th1)+l*cos(th1+th2);l*sin(th1)+l*sin(th1+th2)];
  J=[-p(2),-l*sin(th1+th2);p(1),l*cos(th1+th2)];
  u=-kp*J'*(p-params.traj(t))-kd*[dth1;dth2];
end

