function torque=gettorque(Y,T,m1,m2,l1,l2,I1,I2,a,b,Kd,Kp)
g=9.81;
torque=zeros(size(Y,1),2);

for i=1:size(Y,1)
    q1 = Y(i,1);
    qd1 = Y(i,2);
    q2 = Y(i,3);
    qd2 = Y(i,4);
    t=T(i,1);
    h=-m2*l1*(l2/2)*sin(q2);  
    C= [h*qd2    h*(qd1+qd2);
        -h*qd1    0        ];   
    phi=[(m1*l1/2+m2*l1)*g*cos(q1)+m2*(l2/2)*g*cos(q1+q2)
           m2*(l2/2)*g*cos(q1+q2)                       ];    
    % follow hand-written part  
    A=[I1+m2*l1^2+2*m2*l1*(l2/2)*cos(q2)+m1*(l1/2)^2+m2*(l2/2)^2+I2,    I2+m2*(l2/2)^2+m2*l1*(l2/2)*cos(q2);
        m2*l1*(l2/2)*cos(q2)+m2*(l2/2)^2+I2,                            I2+m2*(l2/2)^2                     ];  %follow the written part solution
    qdd_des=[2*a(3)+6*a(4)*t;2*b(3)+6*b(4)*t];
    qd_des=[a(2)+2*a(3)*(t)+3*a(4)*t^2;b(2)+2*b(3)*(t)+3*b(4)*t^2];
    q_des=[a(1)+a(2)*t+a(3)*t^2+a(4)*t^3;b(1)+b(2)*t+b(3)*t^2+b(4)*t^3];
    v=qdd_des+Kd*(qd_des-[qd1;qd2])+Kp*(q_des-[q1;q2]);
    u=A*v+phi+C*[qd1;qd2];
    torque(i,:)=u';  %corresponds to T(i)
end

end


