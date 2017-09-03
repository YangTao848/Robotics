clear
clc
close all

%% define parameters
M = 1;      % mass of the link
I = 1;      % moment of inertia about the COM
lc = 1/2;   % distance from the COM to the center of joint
l = 1;      % overall length of the link
robot = singleJointManipulator(I,M,lc,l);
robot1 = robot;  % a second robot with modelling errors

% set model parameters
robot = robot.setModelInertia(I);
robot = robot.setModelMass(M);
robot = robot.setModelLc(lc);
%% test kinematics
theta = pi/3;
robot = robot.setJointAngle(theta);
axis equal    % set plotting axis equal
robot.drawRobot

%% test plant dynamics without input torque
animateSwitch = 'on';    % switch of on/off for animation
t_end = 5;               % end time of simulation
ic = [-pi/3 0];          % initial condition
robot = robot.setJointTorque(0);

options = odeset('RelTol',1e-4,'AbsTol',1e-8*ones(1,2));
[td,Y] = robot.Dynamics(t_end,ic,options);

q = Y(:,1);
q_dot = Y(:,2);

E = zeros(length(td),1);
K = zeros(length(td),1);
V = zeros(length(td),1);
for i = 1:length(td)
    K(i) = robot.kineticEnergy(q_dot(i,:));
    V(i) = robot.potentialEnergy(q(i,:));
    E(i) = robot.totalEnergy(q(i,:),q_dot(i,:));
    if strcmp(animateSwitch,'on') && i > 1
        robot = robot.setJointAngle(q(i));
        robot.animateMotion(td(i)-td(i-1));
    elseif strcmp(animateSwitch,'off')
        continue
    end
end
close gcf
robot.plotEnergy(K,V,E,td)
%% test computed torque controller
ic = [0 0];       % initial condition: [qd,qd_dot]
fc = [pi/3 0];    % final condition: [qd,qd_dot]
ts =0; tf = 2;    % starting/completing time
[qds,qds_dot,qds_ddot] = robot.motionPlanning(ic,fc,ts,tf);

kp = 100; kd = 20;  % controller gains
robot = robot.setControlGains([kp;kd]);

options = odeset('RelTol',1e-4,'AbsTol',1e-8*ones(1,2));
[tct,Q] = robot.ComputedTorque(ts,tf,ic,options,qds,qds_dot,qds_ddot);
[qd,qd_dot,qd_ddot] = robot.motionEvaluation(qds,qds_dot,qds_ddot,tct);
u = robot.calcTorque(Q,qd,qd_dot,qd_ddot);

robot.plotTracking(qd,Q(:,1),tct)
robot.plotTorque(u,tct)

figure
for i = 1:length(tct)
    if strcmp(animateSwitch,'on') && i > 1
        robot = robot.setJointAngle(Q(i,1));
        robot.animateMotion(tct(i)-tct(i-1));
    elseif strcmp(animateSwitch,'off')
        continue
    end
end
close gcf
%% test controller with modelling errors
% set model parameters for robot1
robot1 = robot1.setModelInertia(1+0.5);
robot1 = robot1.setModelMass(1-0.5);
robot1 = robot1.setModelLc(1/2+0.2);

%% test computed torque controller with modeling error
ic = [0 0];
fc = [pi/3 0];
ts =0; tf = 2;
[qds,qds_dot,qds_ddot] = robot1.motionPlanning(ic,fc,ts,tf);

kp = 100; kd = 20;
robot1 = robot1.setControlGains([kp;kd]);
options = odeset('RelTol',1e-4,'AbsTol',1e-8*ones(1,2));
[tct,Q] = robot1.ComputedTorque(ts,tf,ic,options,qds,qds_dot,qds_ddot);
[qd,qd_dot,qd_ddot] = robot1.motionEvaluation(qds,qds_dot,qds_ddot,tct);
u = robot1.calcTorque(Q,qd,qd_dot,qd_ddot);

robot1.plotTracking(qd,Q(:,1),tct)
robot1.plotTorque(u,tct)

figure
for i = 1:length(tct)
    if strcmp(animateSwitch,'on') && i > 1
        robot1 = robot1.setJointAngle(Q(i,1));
        robot1.animateMotion(tct(i)-tct(i-1));
    elseif strcmp(animateSwitch,'off')
        continue
    end
end
close gcf