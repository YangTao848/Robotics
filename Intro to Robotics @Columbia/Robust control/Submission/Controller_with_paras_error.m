function dy=Controller_with_paras_error(t,y,m1,m2,l1,l2,I1,I2,a,b,Kp,Kd)
g=9.81;
    q1 = y(1);
    qd1 = y(2);
    q2 = y(3);
    qd2 = y(4);
    dy = zeros(4,1);  
    dy(1) = y(2);
    dy(3) = y(4);
    % h1 C1 A1 phi1 are true matrices that are not mismatched
    h1=-m2*l1*(l2/2)*sin(q2);
    
    C1= [h1*qd2    h1*(qd1+qd2);
        -h1*qd1    0        ];
    
    phi1=[(m1*l1/2+m2*l1)*g*cos(q1)+m2*(l2/2)*g*cos(q1+q2)
           m2*(l2/2)*g*cos(q1+q2)                       ];     
    % follow hand-written part    
    A1=[I1+m2*l1^2+2*m2*l1*(l2/2)*cos(q2)+m1*(l1/2)^2+m2*(l2/2)^2+I2,    I2+m2*(l2/2)^2+m2*l1*(l2/2)*cos(q2);
        m2*l1*(l2/2)*cos(q2)+m2*(l2/2)^2+I2,                            I2+m2*(l2/2)^2                     ];  %follow the written part solution
    %mismatch the parameters
    m1=1.5*m1; m2=1.5*m2; I1=0.5*I1; I2=0.5*I2;
    %calculate the matrices we estimate note that 1/2 changes to 0.7(as that in given code)
    h=-m2*l1*(l2*0.7)*sin(q2); 
    C= [h*qd2    h*(qd1+qd2);
        -h*qd1    0        ];   
    phi=[(m1*0.7+m2*l1)*g*cos(q1)+m2*(l2*0.7)*g*cos(q1+q2)
           m2*(l2*0.7)*g*cos(q1+q2)                       ];       
    A=[I1+m2*l1^2+2*m2*l1*(l2*0.7)*cos(q2)+m1*(l1*0.7)^2+m2*(l2*0.7)^2+I2,    I2+m2*(l2*0.7)^2+m2*l1*(l2*0.7)*cos(q2);
        m2*l1*(l2*0.7)*cos(q2)+m2*(l2*0.7)^2+I2,                            I2+m2*(l2*0.7)^2                     ]; 
    %desired state
    qdd_des=[2*a(3)+6*a(4)*t;2*b(3)+6*b(4)*t];
    qd_des=[a(2)+2*a(3)*(t)+3*a(4)*t^2;b(2)+2*b(3)*(t)+3*b(4)*t^2];
    q_des=[a(1)+a(2)*t+a(3)*t^2+a(4)*t^3;b(1)+b(2)*t+b(3)*t^2+b(4)*t^3];
    
    %compute torque
    v=qdd_des+Kd*(qd_des-[qd1;qd2])+Kp*(q_des-[q1;q2]);
    
    u=A*v+phi+C*[qd1;qd2];
    
    % solve ODE
    
    b=-phi1-C1*[qd1;qd2]+u; 
    x=A1\b;
    % dy(2) is double derivative of q1
    dy(2) = x(1,:);
    % dy(4) is double derivative of q2
    dy(4) = x(2,:);
end