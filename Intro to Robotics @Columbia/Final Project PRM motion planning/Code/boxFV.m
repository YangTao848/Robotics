function fv = boxFV (x1,x2,y1,y2,z1,z2)
% Make a simple box
fv.vertices = [x1,y1,z1;x2,y1,z1; x2,y2,z1;x1,y2,z1;x1,y1,z2;x2,y1,z2; x2,y2,z2;x1,y2,z2;];
fv.faces = [1,0,0,-x1;1,0,0,-x2;0,1,0,-y1;0,1,0,-y2;0,0,1,-z1;0,0,1,-z2];