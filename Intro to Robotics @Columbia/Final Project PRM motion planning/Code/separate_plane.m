function separate = separate_plane(P1,P2,plane)
% P1 P2 are rows of point(x,y,z), plane =[A,B,C,D] the equation is Ax+By+Cz+D=0
% if all points of P1 and P2 are separated, then separate=true;
separate = false;
s1=sign(sum(P1.*plane(1,1:3),2)+plane(1,4)); %each row is the sign of Ax+By+Cz+D
s2=sign(sum(P2.*plane(1,1:3),2)+plane(1,4));
% we need s1 and s2 have different sign (or zero)
if((sum((s1>=0)-1)==0) && (sum((s2<=0)-1)==0))
    separate=true;
    return;
end
if((sum((s1<=0)-1)==0) && (sum((s2>=0)-1)==0))
    separate=true;
end

end
