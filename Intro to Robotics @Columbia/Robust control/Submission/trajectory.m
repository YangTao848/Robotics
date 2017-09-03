function [a,b] = trajectory(q0,qd0,qt,qdt,tf)  %q0=[q10;q20] 2x1   [a,b]=[[a1 a2 a3 a4]' [b1 b2 b3 b4]']
% q1=a1+a2*t+a3*t^2+a4*t^3
% q2=b1+b2*t+b3*t^2+b4*t^3
% solve for the coefficients of polynomials
temp=[1, 0, 0,  0;
      0, 1, 0,  0;
      1,tf,tf^2,tf^3;
      0, 1,2*tf,3*tf^2];
 A=[temp, zeros(4,4);
    zeros(4,4),temp];
 x=[q0(1);qd0(1);qt(1);qdt(1);q0(2);qd0(2);qt(2);qdt(2)];
 c=A\x;
 a=c(1:4,1);b=c(5:8,1);