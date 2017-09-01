function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% % the number of grids for 1 meter.
 myResol = param.resol;
% % the initial map size in pixels
 myMap = zeros(param.size);
% % the origin of the map in pixels
 myOrigin = param.origin; 
% 
% % 4. Log-odd parameters 
 lo_occ = param.lo_occ;
 lo_free = param.lo_free; 
 lo_max = param.lo_max;
 lo_min = param.lo_min;

 N = size(pose,2);
 M = size(ranges,1);
 for j = 1:N % for each time,
% 
      i_x0=ceil(myResol*pose(1,j));
      i_y0=ceil(myResol*pose(2,j));
      theta=pose(3,j);
      pose_grid=[i_x0,i_y0];
      for i=1:M
        r=ranges(i,j);
        alpha=scanAngles(i);
        % Find grids hit by the rays (in the gird map coordinate)
        xocc=r*cos(alpha+theta)+pose(1,j);
        yocc=-r*sin(alpha+theta)+pose(2,j);
        occ_grid=ceil(myResol*[xocc,yocc]);
%       % Find occupied-measurement cells and free-measurement cells
        [freex, freey] = bresenham(pose_grid(1),pose_grid(2),occ_grid(1),occ_grid(2));
        free = sub2ind(size(myMap),freey+myOrigin(2),freex+myOrigin(1));
        occ =  sub2ind(size(myMap),occ_grid(2)+myOrigin(2),occ_grid(1)+myOrigin(1));
        % Update the log-odds
        myMap(free)=myMap(free)-lo_free;
            % Saturate the log-odd values  
        myMap(occ)=myMap(occ)+lo_occ;
       
%     % Visualize the map as needed
%    
      end
 end
 myMap(myMap<lo_min)=lo_min;
 myMap(myMap>lo_max)=lo_max;
figure,
imagesc(myMap); 
colormap('gray'); axis equal;
end
