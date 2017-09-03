%Author: Chengxi Dong
clear, clc, close all
%set parameters
m1=1;m2=1;l1=1;l2=1;
I1=1;I2=1;
%boundary condition
q0=[0;0];  qd0=[0;0];
qt=[pi/3;pi/4]; qdt=[0;0];
tf=5;
%set gains
Kp=100;  Kd=20;
%generate trajectory
[a,b]= trajectory(q0,qd0,qt,qdt,tf);

ic = [ q0(1) qd0(1) q0(2) qd0(2)];
time = 5;
options = odeset('RelTol',1e-4,'AbsTol',1e-8*ones(1,4)); % solver options

[T,Y] = ode45(@(t,y) Controller(t,y,m1,m2,l1,l2,I1,I2,a,b,Kp,Kd),[0 time],ic,options); % Solve ODE with controller

%interpolation
y = interp1(T,Y,linspace(min(T),max(T)));
t_int=interp1(T,T,linspace(min(T),max(T)));

% plot video
 plotTracking(l1,l2,y,time,t_int,a,b,'Control without modeling error'); 
 
%error
q_des=[a(1)+a(2)*T+a(3)*T.^2+a(4)*T.^3,b(1)+b(2)*T+b(3)*T.^2+b(4)*T.^3];
q1_error=q_des(:,1)-Y(:,1);
q2_error=q_des(:,2)-Y(:,3);  %with respect to T

%get the input torque we have applied
torque=gettorque(Y,T,m1,m2,l1,l2,I1,I2,a,b,Kd,Kp);
figure;
%plot
suptitle('Simulation without error');
subplot(2,2,1);
plot(T,torque(:,1));hold on; plot(T,torque(:,2)); hold off; legend('torque1','torque2');
xlabel('time/[s]'); ylabel('torque/N*m');title('input vs t');
subplot(2,2,2);
plot(T,q_des(:,1),'*'); hold on; plot(T,Y(:,1),'LineWidth',2);hold off; legend('q1 desired','q1');
xlabel('time/[s]'); ylabel('angle/rad');title('q1 tracking');
subplot(2,2,3);
plot(T,q_des(:,2),'*'); hold on; plot(T,Y(:,3),'LineWidth',2);hold off; legend('q2 desired','q2');
xlabel('time/[s]'); ylabel('angle/rad');title('q2 tracking');
subplot(2,2,4);
plot(T,q1_error);hold on; plot(T,q2_error,'--'); hold off; xlabel('time/[s]'); ylabel('error/rad'); title('q1,q2 error');
legend('error of q1','error of q2');

%do it again with modeling error(sensor error)
simulation_with_paras_error;  % as given code, the parameters mismatch
simulation_with_sensor_error;  % I think there is a probability that the sensor can't get true states.
                               % So I add white noise to it and try simulation though the given codes didn't.

