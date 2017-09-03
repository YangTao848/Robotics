function route = GradientBasedPlanner (f, start_coords, end_coords, max_its)
% GradientBasedPlanner : This function plans a path through a 2D
% environment from a start to a destination based on the gradient of the
% function f which is passed in as a 2D array. The two arguments
% start_coords and end_coords denote the coordinates of the start and end
% positions respectively in the array while max_its indicates an upper
% bound on the number of iterations that the system can use before giving
% up.
% The output, route, is an array with 2 columns and n rows where the rows
% correspond to the coordinates of the robot as it moves along the route.
% The first column corresponds to the x coordinate and the second to the y coordinate

[gx, gy] = gradient (-f);
for i=1:size(f,1)
    for j=1:size(f,2)
        temp=sqrt(gx(i,j)^2+gy(i,j)^2);
        gx(i,j)=gx(i,j)/temp;
        gy(i,j)=gy(i,j)/temp;
    end
end
x=start_coords(1); y=start_coords(2);
route=start_coords;
flag=0;
%%% All of your code should be between the two lines of stars.
% *******************************************************************
for i=1:max_its
    x0=x;y0=y;
    x=x+gx(round(y0),round(x0));
    y=y+gy(round(y0),round(x0));
    route=[route;[x,y]];
    if(([x,y]-end_coords)*([x,y]-end_coords)'<4)
    break;
    end  
end
% *******************************************************************
end
