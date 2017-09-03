function [en,k,v] = getTotalEnergy(m1,m2,l1,l2,I1,I2,Y)

q1 = Y(:,1);
q2 = Y(:,3);
qd1 = Y(:,2);
qd2 = Y(:,4);
g=9.81;

k1 = 1/2*(I1 + m1*l1^2/4)*qd1.^2;
k2 = m2/2*(l1^2*qd1.^2+l2^2/4*(qd1+qd2).^2+l1*l2*(qd1.^2+qd1.*qd2).*cos(q2))+1/2*I2*(qd1+qd2).^2;

v1 = l1/2*sin(q1)*m1*g;
v2 = m2*g*(l1*sin(q1)+l2/2*sin(q1+q2));

k= k1+k2;
v = v1+v2;
en = k+v;


