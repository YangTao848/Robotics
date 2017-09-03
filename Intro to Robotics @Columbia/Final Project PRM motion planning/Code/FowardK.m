function [point1,point2] = FowardK(x)
%get two link points
        x(1:2,1)=mod(x(1:2,1),360);
        x(4:6,1)=mod(x(4:6,1),360);    
        %p.Vertices = fv.vertices;
        q1=x(1)*pi/180; q2=x(2)*pi/180; d3=x(3); Euler(1:3,1)=x(4:6,1)*pi/180;
        DH=[0,-pi/2,0,q1;
            0, pi/2,0,q2;
             0, 0 ,d3,0];
        T=calcT0n(DH);
        d=T(1:3,4);
        point1=d;
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
        point2=d+R*[0;0;20];
end

