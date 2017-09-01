function [endeff] = computeMiniForwardKinematics(rads1,rads2)

rads3=asin((sin((rads1-rads2)/2))/2);
l3=2*cos(rads3)-cos((rads1-rads2)/2);
ang=-(pi-(rads1+rads2)/2);
endeff = [l3*cos(ang),l3*sin(ang)];