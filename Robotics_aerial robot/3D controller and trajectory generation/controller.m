function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
kd3=5; kp3=60;
% Thrust
F = params.mass*params.gravity-params.mass*(-des_state.acc(3)+kd3*(state.vel(3)-des_state.vel(3))+kp3*(state.pos(3)-des_state.pos(3)));
phides=(1/params.gravity)*(des_state.acc(1)*sin(des_state.yaw)-des_state.acc(2)*cos(des_state.yaw));
thetades=(1/params.gravity)*(des_state.acc(1)*cos(des_state.yaw)+des_state.acc(2)*sin(des_state.yaw));

% Moment
k11=40; k12=1; 
k21=40; k22=1;
k31=40; k32=1;

M = zeros(3,1);
M(1,1)=k11*(phides-state.rot(1))+k12*(-state.omega(1) -35*(des_state.pos(2)-state.pos(2)+2*( des_state.vel(2)-state.vel(2) ) ));  %% rotate for phi and Y
M(2,1)=k21*(thetades-state.rot(2))+k22*(-state.omega(2)+35*(des_state.pos(1)-state.pos(1)+2*( des_state.vel(1)-state.vel(1) ) )); %%rotate for theta and X
M(3,1)=k31*(des_state.yaw-state.rot(3))+k32*(des_state.yawdot-state.omega(3)); %% rotate for omega

% =================== Your code ends here ===================

end
