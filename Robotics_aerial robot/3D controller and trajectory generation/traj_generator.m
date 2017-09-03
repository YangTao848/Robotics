function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

persistent waypoints0 traj_time d0 cx cy cz
if nargin > 2  %% [] [] are also input so when we intialize it nargin=3 
    d = waypoints(:,2:end) - waypoints(:,1:end-1); % 3*(n-1)
    d0 = 1.5 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);  %the distance between ti-1 and ti
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    N=size(waypoints0,2)-1;
 A=zeros(8*N,8*N);
 bx=zeros(8*N,1);
 by=zeros(8*N,1);
 bz=zeros(8*N,1);
 bx(1:N,1)=waypoints0(1,1:end-1)';
 bx(N+1:2*N,1)=waypoints0(1,2:end)';
 by(1:N,1)=waypoints0(2,1:end-1)';
 by(N+1:2*N,1)=waypoints0(2,2:end)';
 bz(1:N,1)=waypoints0(3,1:end-1)';
 bz(N+1:2*N,1)=waypoints0(3,2:end)';
for i=1:N;
    A(i,8*i-7)=1;          %P(Si-1)=wi-1
    for j=1:8;
        A(N+i,8*(i-1)+j)=1;   %P(Si)=wi
    end
end
for i=1:N-1;
    for j=1:7;
        A(2*N+i,8*(i-1)+j+1)=j/d0(i);
    end
        A(2*N+i,8*i+2)=-1/d0(i+1);
end
        A(3*N,2)=1/d0(1);
for i=1:N-1;
    for j=2:7;
        A(3*N+i,8*(i-1)+j+1)=j*(j-1)/(d0(i)^2);
    end
        A(3*N+i,8*i+3)=-2/(d0(i+1)^2);
end
    for k=1:7;
        A(4*N,8*(N-1)+k+1)=k/d0(N);   
    end
for i=1:N-1;
    for j=3:7;
        A(4*N+i,8*(i-1)+j+1)=j*(j-1)*(j-2)/(d0(i)^3);
    end
       A(4*N+i,8*i+4)=-6/(d0(i+1)^3);
end
       A(5*N,3)=2/(d0(1)^2);
for i=1:N-1;
    for j=4:7;
        A(5*N+i,8*(i-1)+j+1)=j*(j-1)*(j-2)*(j-3)/(d0(i)^4);
    end
        A(5*N+i,8*i+5)=-24/(d0(i+1)^4);
end
    for k=2:7;
        A(6*N,8*(N-1)+k+1)=k*(k-1)/(d0(N)^2);
    end
for i=1:N-1;
    for j=5:7;
        A(6*N+i,8*(i-1)+j+1)=j*(j-1)*(j-2)*(j-3)*(j-4)/(d0(i)^5);
    end
        A(6*N+i,8*i+6)=-120/(d0(i+1)^5);
end
        A(7*N,4)=(6/d0(1)^3);
for i=1:N-1;
    for j=6:7;
        A(7*N+i,8*(i-1)+j+1)=j*(j-1)*(j-2)*(j-3)*(j-4)*(j-5)/(d0(i)^6);
    end
        A(7*N+i,8*i+7)=-720/(d0(i+1)^6);
end
    for k=3:7;
        A(8*N,8*(N-1)+1+k)=k*(k-1)*(k-2)/(d0(N)^3);
    end
cx=pinv(A)*bx;
cy=pinv(A)*by;
cz=pinv(A)*bz;
else
    if(t > traj_time(end))
        t = traj_time(end);
        desired_state.pos=waypoints0(:,end);
        desired_state.vel = zeros(3,1);
        desired_state.acc = zeros(3,1);
        desired_state.yaw = 0;
        desired_state.yawdot = 0;
        return;
    end
    t_index = find(traj_time >= t,1);

    if(t_index > 1)
        t = t - traj_time(t_index-1);
    end
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
        desired_state.vel = [cx(2);cy(2);cz(2)]/d0(1);
        desired_state.acc = [cx(3);cy(3);cz(3)]*2/(d0(1)^2);
    else
        i=t_index-1;
        desx=0; desvx=0; desax=0;
        desy=0; desvy=0; desay=0;
        desz=0; desvz=0; desaz=0;
        for j=1:8;
            desx=desx+cx(8*(i-1)+j)*((t/d0(i))^(j-1));
            desy=desy+cy(8*(i-1)+j)*((t/d0(i))^(j-1));
            desz=desz+cz(8*(i-1)+j)*((t/d0(i))^(j-1));
        end
            desired_state.pos=[desx;desy;desz];
        for j=1:7;
            desvx=desvx+cx(8*(i-1)+1+j)*j*(t^(j-1))/(d0(i)^j);
            desvy=desvy+cy(8*(i-1)+1+j)*j*(t^(j-1))/(d0(i)^j);
            desvz=desvz+cz(8*(i-1)+1+j)*j*(t^(j-1))/(d0(i)^j);
        end
            desired_state.vel=[desvx;desvy;desvz];
        for j=1:6;
            desax=desax+cx(8*(i-1)+j+2)*j*(j+1)*(t^(j-1))/(d0(i)^j+1); 
            desay=desay+cy(8*(i-1)+j+2)*j*(j+1)*(t^(j-1))/(d0(i)^j+1); 
            desaz=desaz+cz(8*(i-1)+j+2)*j*(j+1)*(t^(j-1))/(d0(i)^j+1); 
        end
            desired_state.acc=[desax;desay;desaz];
        %scale = t/d0(t_index-1);
        %desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
    end
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end
%


    
end

%% Fill in your code here

% desired_state.pos = zeros(3,1);
% desired_state.vel = zeros(3,1);
% desired_state.acc = zeros(3,1);
% desired_state.yaw = 0;

