function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters
kp=-100;
kv=-2* sqrt(-params.mass*kp);
u0 = params.mass*params.gravity + [kp,kv]*(s-s_des);
u=u0;
if(u0<params.u_min) 
    u=params.u_min;
else if(u0>params.u_max)
    u=params.u_max;
    end
end


% FILL IN YOUR CODE HERE


end

