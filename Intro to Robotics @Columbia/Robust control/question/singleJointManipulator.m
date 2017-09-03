classdef singleJointManipulator
    % SINGLEJOINTMANIPULATOR:
    %
    % this is a class example. This class defines a manipulator with a
    % single revolute joint. It contains functions that compute its
    % kinematics, dynamics, motion planning using 3rd order polynomial and
    % control using computed torque method.
    %
    % Author: Haohan Zhang
    % Affiliation: ROAR Lab @ Columbia University
    % Date: 11/21/2016
    
    properties
        inertia          % moment of inertia about center of mass
        mass             % mass of link
        lc               % distance between COM C and joint center O.
        linkLength       % length of the entire link
    end
    properties (Access = private)
        theta            % joint angle
        u                % input joint torque
        inertia_hat      % inertia in model
        mass_hat         % mass in model
        lc_hat           % lc in model
        controlGains     % control gains: [kp;kv]
    end
    
    methods
        % constructor
        function this = singleJointManipulator(I, M, L, l)
            % this is the constructor.
            if nargin > 0
                this.inertia = I;
                this.mass = M;
                this.lc = L;
                this.linkLength = l;
            else
                this.inertia = 1;
                this.mass = 1;
                this.lc = 1/2;
                this.linkLength = 1;
            end
        end
        
        % setters
        function this = setJointAngle(this,value)
            % this sets the joint angle theta.
            this.theta = value;
        end
        function this = setJointTorque(this,value)
            % this sets the joint torque u.
            this.u = value;
        end
        function this = setModelInertia(this,value)
            % this sets the inertia of model.
            this.inertia_hat = value;
        end
        function this = setModelMass(this,value)
            % this sets the mass of model.
            this.mass_hat = value;
        end
        function this = setModelLc(this,value)
            % this sets the lc of model.
            this.lc_hat = value;
        end
        function this = setControlGains(this,value)
            % this sets the gains for the outer loop controller.
            this.controlGains = value;
        end
        
        % getters
        function value = getJointAngle(this)
            % this returns the joint angle theta.
            value = this.theta;
        end
        function value = getJointTorque(this)
            % this returns the joint torque u.
            value = this.u;
        end
        function value = getModelInertia(this)
            % this returns the inertia of model.
            value = this.inertia_hat;
        end
        function value = getModelMass(this)
            % this returns the mass of model.
            value = this.mass_hat;
        end
        function value = getModelLc(this)
            % this returns the lc of model.
            value = this.lc_hat;
        end
        function value = getControlGains(this)
            % this returns the control gains for the outer loop.
            value = this.controlGains;
        end
        
        % kinematics
        function C = calcPosC(this)
            % this computes the position of point C (COM of the link).
            L = this.lc;
            q = this.getJointAngle;
            xc = L*cos(q); yc = L*sin(q);
            C = [xc;yc];
        end
        function P = calcPosP(this)
            % this computes the position of the tip P of this link.
            l = this.linkLength;
            q = this.getJointAngle;
            xp = l*cos(q); yp = l*sin(q);
            P = [xp;yp];
        end
        
        % motion planning
        function [qd,qd_dot,qd_ddot] = motionPlanning(~,ic,fc,ts,tf)
            % this plans a motion using a 3rd order polynomial based on
            % inital and final conditions. It returns the motion
            % expressions symbolically.
            b = [ic(1);fc(1);ic(2);fc(2)];
            A = [ts^3    ts^2  ts  1;
                 tf^3    tf^2  tf  1;
                 3*ts^2  2*ts  1   0;
                 3*tf^2  2*tf  1   0];
            par = A\b;
            
            % symbolic expression: qd = at^3+bt^2+ct+d
            syms T
            qd = par(1)*T^3 + par(2)*T^2 + par(3)*T + par(4);
            qd_dot = 3*par(1)*T^2 + 2*par(2)*T + par(3);
            qd_ddot = 6*par(1)*T + 2*par(2);
        end
        function [qd,qd_dot,qd_ddot] = motionEvaluation(~,qds,qds_dot,qds_ddot,t)
            % this evaluates the symbolic expression of motion numerically
            % based on the time vector t.
            syms T  
            qd = zeros(length(t),1);
            qd_dot = zeros(length(t),1);
            qd_ddot = zeros(length(t),1);
            for i = 1:length(t)
                qd(i) = double(subs(qds,T,t(i)));
                qd_dot(i) = double(subs(qds_dot,T,t(i)));
                qd_ddot(i) = double(subs(qds_ddot,T,t(i)));
            end
        end
        
        % dynamics
        function dy = equationOfMotion(t,y,this)
            % this evaluates the equation of motion.
            g = 9.8;
            I = this.inertia; M = this.mass; L = this.lc;
            tau = this.getJointTorque;
            dy = zeros(2,1);
            dy(1) = y(2);
            dy(2) = (1/(I+M*L^2))*(-M*g*L*cos(y(1)) + tau);
        end
        function [T,Y] = Dynamics(this,t_end,ic,options)
            % this integrate the motion according to equations of motion
            [T,Y] = ode45(@(t,y) equationOfMotion(t,y,this),[0 t_end],ic,options);
        end

        function K = kineticEnergy(this,q_dot)
            % this computes the kinetic energy of the system
            I = this.inertia; M = this.mass; L = this.lc;
            K = 1/2*(M*L^2 + I)*q_dot^2;
        end
        function V = potentialEnergy(this,q)
            % this returns the potential energy
            g = 9.8;
            M = this.mass;
            L = this.lc;
            V = M*g*L*sin(q);
        end
        function E = totalEnergy(this,q,q_dot)
            % this returns total energy
            K = this.kineticEnergy(q_dot);
            V = this.potentialEnergy(q);
            E = K + V;
        end
       
        % control
        function dq = controlLaw(t,q,this,qds,qds_dot,qds_ddot)
            % this sets up a controller using computed torque (inverse
            % dynamics) method.
            k = this.controlGains; kp = k(1); kd = k(2);
            
            % evaluate desired trajectory
            syms T
            qd = double(subs(qds,T,t));
            qd_dot = double(subs(qds_dot,T,t));
            qd_ddot = double(subs(qds_ddot,T,t));
            
            % parameter mismatch
            g = 9.8;
            M = this.mass; L = this.lc;
            M_hat = this.getModelMass; L_hat = this.getModelLc;
            h_hat = M_hat*g*L_hat*cos(q(1));
            h = M*g*L*cos(q(1));
            delta_h = h_hat - h;
            E = M\M_hat - eye(1);
            eta = E*(qd_ddot + kd*(qd_dot-q(2)) + kp*(qd-q(1))) + ...
                  M\delta_h;
            
            % ODEs
            dq = zeros(2,1);
            dq(1) = q(2);
            dq(2) = -kp*q(1) -kd*q(2) + qd_ddot + kp*qd + kd*qd_dot + eta;
        end
        function [T,Q] = ComputedTorque(this,ts,tf,ic,options,qd,qd_dot,qd_ddot)
            % this integrate the motion according to equations of motion
            [T,Q] = ode45(@(t,q) controlLaw(t,q,this,qd,qd_dot,qd_ddot),[ts tf],ic,options);
        end
        function [u,v] = calcTorque(this,Q,qd,qd_dot,qd_ddot)
            % this computes the control variable v and joint torque input
            % u.
            k = this.controlGains; kp = k(1); kd = k(2);
            v = qd_ddot + kd*(qd_dot-Q(:,2)) + kp*(qd-Q(:,1));
            
            g = 9.8;
            I_hat = this.getModelInertia; 
            M_hat = this.getModelMass; 
            L_hat = this.getModelLc;
            u = (I_hat+M_hat*L_hat^2)*v + M_hat*g*L_hat*cos(Q(:,1));
        end
        
        % plotting
        function drawRobot(this)
            % this draws this manipulator.
            O = [0;0];
            C = this.calcPosC;
            P = this.calcPosP;
            l = [O P].';
            hold on
            line(l(:,1),l(:,2),'linewidth',2,'color','b');
            plot(O(1),O(2),'k.','markersize',30);
            plot(C(1),C(2),'ko','markersize',15);
            plot(C(1),C(2),'kx','markersize',15);
            hold off
        end
        function animateMotion(this,dt)
            % this animates the motion of the robot
            axis([-1.2 1.2 -1.2 1.2]);
            this.drawRobot;
            drawnow
            pause(dt) % pause with a 'correct' timing
            clf
        end
        function plotEnergy(~,K,V,E,T)
            % this plots the kinetic energy, potential energy and total
            % energy.
            figure
            plot(T,[K,V,E],'linewidth',2.0)
            legend('Kinetic Energy','Potential Energy','Total Energy')
            xlabel('Time (s)')
            ylabel('Energy (J)')
            h = gca;
            h.FontSize = 16;
        end
        function plotTracking(~,qd,q,t)
            % this plots the performance of controller
            e = qd - q;  % tracking error
            figure
            subplot(2,1,1)
            plot(t,q,'linewidth',2.0)
            hold on
            plot(t,qd,'r--','linewidth',1.5)
            ylabel('Joint Angle \theta (rad)')
            legend('Actual trajectory','Desired trajectory')
            h = gca; h.FontSize = 16;
            subplot(2,1,2)
            plot(t,e,'k','linewidth',2.0)
            xlabel('Time (sec)')
            ylabel('Tracking Error (rad)')
            h = gca; h.FontSize = 16;
        end
        function plotTorque(~,u,t)
            % this plots input torque
            figure
            plot(t,u,'linewidth',1.5)
            grid on
            xlabel('Time (sec)')
            ylabel('Input Torque (Nm)')
            h = gca; h.FontSize = 16;
        end
    end
end

