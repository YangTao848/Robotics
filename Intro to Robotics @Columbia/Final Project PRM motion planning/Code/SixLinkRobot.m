function fv = SixLinkRobot (cspace)
% Simulate a simple six link revolute chain
cspace=cspace';
q1=cspace(1)*pi/180; q2=cspace(2)*pi/180; d3=cspace(3); Euler(1,1:3)=cspace(1,4:6)*pi/180;
point=[-1,-1,0;-1,-1,d3;-1,1,0;-1,1,d3;1,-1,0;1,-1,d3;1,1,0;1,1,d3]; 
plane=[0,0,1,-d3;0,1,0,1;0,1,0,-1;0,0,1,0;1,0,0,-1;1,0,0,1];  % 6 planes, 8 points
%then find it in fixed frame
DH=[0,-pi/2,0,q1;
    0, pi/2,0,q2;
    0, 0 ,d3,0];
T=calcT0n(DH);
R=T(1:3,1:3); d=T(1:3,4);
fv(1,1).vertices=zeros(8,3);fv(1,2).vertices=zeros(8,3);
fv(1,1).faces=zeros(6,4);fv(1,2).faces=zeros(6,4);
for i=1:8
    fv(1,1).vertices(i,:)=(R*(point(i,:))')';
end
for j=1:6
    fv(1,2).faces(j,:)=[plane(j,1:3)*(R'),plane(j,4)]; 
end
 A=[cos(Euler(1)),  -sin(Euler(1)), 0;   
    sin(Euler(1)),  cos(Euler(1)),  0;
    0,0,1];
 B=[cos(Euler(2)), 0, sin(Euler(2));
      0,           1,       0       ;
    -sin(Euler(2)),0, cos(Euler(2))];
      
 C=[cos(Euler(3)),  -sin(Euler(3)), 0;
    sin(Euler(3)),  cos(Euler(3)),  0;
    0,0,1];
 R=A*B*C; 
 point=[-1,-1,0;-1,-1,20;-1,1,0;-1,1,20;1,-1,0;1,-1,20;1,1,0;1,1,20]; 
 plane=[0,0,1,-20;0,1,0,1;0,1,0,-1;0,0,1,0;1,0,0,-1;1,0,0,1]; 
 for i=1:8
    fv(1,2).vertices(i,:)=(R*(point(i,:))'+d)';
 end
 for j=1:6
    fv(1,2).faces(j,:)=[plane(j,1:3)*(R'),plane(j,1:3)*(R')*(-d)+plane(j,4)]; 
 end
