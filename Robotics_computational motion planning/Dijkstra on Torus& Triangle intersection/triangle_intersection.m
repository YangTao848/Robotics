function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************
m1=1;m2=1;m3=1;m4=1;m5=1;m6=1;
for i=1:3;
    m1=m1*dif(P1(1,:),P1(2,:),P1(3,:),P2(i,:));
    m2=m2*dif(P1(2,:),P1(3,:),P1(1,:),P2(i,:));
    m3=m3*dif(P1(3,:),P1(1,:),P1(2,:),P2(i,:));
    m4=m4*dif(P2(1,:),P2(2,:),P2(3,:),P1(i,:));
    m5=m5*dif(P2(2,:),P2(3,:),P2(1,:),P1(i,:));
    m6=m6*dif(P2(3,:),P2(1,:),P2(2,:),P1(i,:));
end
m=m1+m2+m3+m4+m5+m6;
 if (m==0)
flag = true;
else
 flag=false;
end
% *******************************************************************
end