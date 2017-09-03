function plotTracking(l1,l2,Y,time,t_int,a,b,tit)

q1 = Y(:,1);
q2 = Y(:,3);

%point A

Ax = l1*cos(q1);
Ay = l1*sin(q1);


%point B

Bx = Ax + l2*cos(q1+q2);
By = Ay + l2*sin(q1+q2);
figure;
for j=1:size(Y,1)
    %plot r1
    plot([0 Ax(j)],[0 Ay(j)],'color','b','linewidth',4);
    axis([-1*(l1+l2) l1+l2 -1*(l1+l2) l1+l2])
    hold on
    %plot r2
    plot([Ax(j) Bx(j)],[Ay(j) By(j)],'color','r','linewidth',4);
    legend('r1','r2')
    %plot desired centroid
    t=t_int(j);
    q_des=[a(1)+a(2)*t+a(3)*t^2+a(4)*t^3;
           b(1)+b(2)*t+b(3)*t^2+b(4)*t^3];
    xc1=(1/2)*l1*cos(q_des(1));
    yc1=(1/2)*l1*sin(q_des(1));
    xc2=l1*cos(q_des(1))+(1/2)*l2*cos(q_des(1)+q_des(2));
    yc2=l1*sin(q_des(1))+(1/2)*l2*sin(q_des(1)+q_des(2));
    plot(0,0,'k.','markersize',30);
    plot(xc1,yc1,'ko','markersize',15);
    plot(xc1,yc1,'kx','markersize',15);
    plot(xc2,yc2,'ko','markersize',15);
    plot(xc2,yc2,'kx','markersize',15);
    title(tit);  
    drawnow
    pause(time/size(Y,1)) % delay 0.05s
    hold off
    axis([-1*(l1+l2) l1+l2 -1*(l1+l2) l1+l2])
%     clf(h) % clear current figure
end


