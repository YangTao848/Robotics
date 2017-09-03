function dy=Controller_with_error(t,y,m1,m2,l1,l2,I1,I2,a,b,Kp,Kd)
    % Given the angular acceleration equations and initial conditions, solve for
    % the current rotation angles of the robot.
    %
    % State vaiables y = [q1,q1d,q2,q2d] are joint angles and joint angular velocity.
    %
    % input - t: time
    %         y: state variables 
    % output - dy
    g=9.81;
    q1 = y(1);
    qd1 = y(2);
    q2 = y(3);
    qd2 = y(4); 
    phi1=[(m1*l1/2+m2*l1)*g*cos(q1)+m2*(l2/2)*g*cos(q1+q2)
           m2*(l2/2)*g*cos(q1+q2)                       ];  
    h=-m2*l1*(l2/2)*sin(q2);
    
    C1= [h*qd2    h*(qd1+qd2);
        -h*qd1    0        ];
    A1=[I1+m2*l1^2+2*m2*l1*(l2/2)*cos(q2)+m1*(l1/2)^2+m2*(l2/2)^2+I2,    I2+m2*(l2/2)^2+m2*l1*(l2/2)*cos(q2);
        m2*l1*(l2/2)*cos(q2)+m2*(l2/2)^2+I2,                            I2+m2*(l2/2)^2                     ]; 
    dy = zeros(4,1);
    dy(1) = y(2);
    dy(3) = y(4);
    % Above are true matrix,below are what we measured.
    
    q1 = q1*(1+randn/20);
    qd1 = qd1*(1+randn/20);
    q2 = q2*(1+randn/20);
    qd2 = qd2*(1+randn/20); %   5% standard error when measuring the states.
    
   
    %TODO
    h=-m2*l1*(l2/2)*sin(q2);
    
    C= [h*qd2    h*(qd1+qd2);
        -h*qd1    0        ];
    
    phi=[(m1*l1/2+m2*l1)*g*cos(q1)+m2*(l2/2)*g*cos(q1+q2)
           m2*(l2/2)*g*cos(q1+q2)                       ];
     
    % follow hand-written part
    
    
    A=[I1+m2*l1^2+2*m2*l1*(l2/2)*cos(q2)+m1*(l1/2)^2+m2*(l2/2)^2+I2,    I2+m2*(l2/2)^2+m2*l1*(l2/2)*cos(q2);
        m2*l1*(l2/2)*cos(q2)+m2*(l2/2)^2+I2,                            I2+m2*(l2/2)^2                     ];  %follow the written part solution
    %desired state
    qdd_des=[2*a(3)+6*a(4)*t;2*b(3)+6*b(4)*t];
    qd_des=[a(2)+2*a(3)*(t)+3*a(4)*t^2;b(2)+2*b(3)*(t)+3*b(4)*t^2];
    q_des=[a(1)+a(2)*t+a(3)*t^2+a(4)*t^3;b(1)+b(2)*t+b(3)*t^2+b(4)*t^3];
    
    v=qdd_des+Kd*(qd_des-[qd1;qd2])+Kp*(q_des-[q1;q2]);
    
    u=A*v+phi+C*[qd1;qd2];
    
    % solve ODE
    
    b=-phi1-C1*[y(2);y(4)]+u; 
    x=A1\b;
    % dy(2) is double derivative of q1
    dy(2) = x(1,:);
    % dy(4) is double derivative of q2
    dy(4) = x(2,:);
   
end
