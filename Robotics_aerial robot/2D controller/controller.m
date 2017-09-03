function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls
kdz=15 ; kpz=50; 
%kdz=0; kpz=0;
kpp=70; kdp=1;
kdy=15; kpy=5;
%kdy=0;kpy=0;
%phic=pi/6;

phic=-1/params.gravity *(des_state.acc(1)+kdy* (des_state.vel(1)-state.vel(1))+kpy*(des_state.pos(1)-state.pos(1)));
u1 = params.mass*(params.gravity+des_state.acc(2)+kdz*(des_state.vel(2)-state.vel(2))+kpz*(des_state.pos(2)-state.pos(2)));
u2 = kpp*(phic-state.rot)+kdp*( -1/params.gravity *(kdy*(des_state.acc(1)+params.gravity*state.rot)+kpy*(des_state.vel(1)-state.vel(1)))-state.omega);

% FILL IN YOUR CODE HERE

end

