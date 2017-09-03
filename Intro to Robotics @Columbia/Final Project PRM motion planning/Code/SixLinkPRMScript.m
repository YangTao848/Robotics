%
% SixLinkPRMScript.
%
clear all;

%% Drawing the robot
figure;

% The SixLinkRobot function computes layout of all of the links in the
% robot as a function of the 6 configuration space parameters. You can
% adjust these sixe numbers to see what happens.
%fv = SixLinkRobot ([-120 120 50 120 -120 60]');

%fv2 = SixLinkRobot ([0 0 0 0 0 180]');


% p = patch (fv);
% 
% p.FaceColor = 'red';
% p.EdgeColor = 'none';
% 
% p2 = patch(fv2);
% p2.FaceColor = 'green';
% p2.EdgeColor = 'none';
% 
% sz = 30;
% axis equal;
% axis (sz*[-1 1 -1 1]);


%% Add obstacles

obstacle(1,1) = boxFV(40, 50, 30, 40,50,30);
obstacle(1,2) = boxFV(40, 50, 15, 25,0,28);
obstacle(1,3) = boxFV(10, 15, 20, 50,5,20);
obstacle(1,4) = boxFV(-5 ,-20,-10,10,10,25);
obstacle(1,5) = boxFV(25,30,30,35,30,40);
% obstacle = appendFV (obstacle, boxFV(-20, 0, -20, -10));
% obstacle = appendFV (obstacle, transformFV(boxFV(-10, 10, -10, 10), 30, [-20 20]));

%patch (obstacle);

%% Build roadmap

nsamples = 1000;
neighbors = 5;

roadmap = PRM (@()(RandomSampleSixLink(obstacle)), @DistSixLink, @(x,y)(LocalPlannerSixLink(x,y,obstacle)), nsamples, neighbors);

%% Add nodes

roadmap2 = AddNode2PRM ([240 120 50 120 240 60]', roadmap, @DistSixLink, @(x,y)(LocalPlannerSixLink(x,y,obstacle)), neighbors);
roadmap2 = AddNode2PRM ([40 50 45 20 60 50]', roadmap2, @DistSixLink, @(x,y)(LocalPlannerSixLink(x,y,obstacle)), neighbors);

%% Plan a route

route = ShortestPathDijkstra(roadmap2.edges, roadmap2.edge_lengths, nsamples+1, nsamples+2);

%% Plot the trajectory
[endpoint1,endpoint2]=FowardK([40 50 45 20 60 50]');
v=VideoWriter('Final.avi');
    open(v);
for i = 2:length(route)
    x1 = roadmap2.samples(:,route(1,i-1));
    x_2 = roadmap2.samples(:,route(1,i));
    
    
    delta = x_2 - x1;
    
    t = delta > 180;
    delta(t) = delta(t) - 360;
    
    t = delta < -180;
    delta(t) = delta(t) + 360;
    
    delta(3)=x_2(3)-x1(3);
    
    n = ceil(sum(abs([delta(1:2);delta(4:6)]))*0.5+abs(x_2(3)-x1(3)));
    
    faces=[1 2 3 4;5 6 7 8;3 4 8 7;1 2 6 5;1 4 8 5;2 3 7 6];
    
    
    
    plot3(endpoint2(1),endpoint2(2),endpoint2(3),'o','markersize',10,'markerfacecolor','g')
    for iter = 1: size(obstacle,2)
        patch('Faces',faces,'Vertices',obstacle(1,iter).vertices)
    end
    hold on;
    grid minor;
    for j = 0:n       
        x =x1 + (j/n)*delta;
        x(1:2,1)=mod(x(1:2,1),360);
        x(4:6,1)=mod(x(4:6,1),360);
        fv = SixLinkRobot (x);     
        %p.Vertices = fv.vertices;
        [point1,point2]=FowardK(x);
        x_1=point1(1);y_1=point1(2);z_1=point1(3);
        x_2=point2(1);y_2=point2(2);z_2=point2(3);     
        h=plot3([0;x_1],[0;y_1],[0;z_1],'LineWidth',2,'Color','c');
        h2=plot3([x_1;x_2],[y_1;y_2],[z_1;z_2],'LineWidth',2,'Color','r');
        axis ([-50 50 -50 50 -50 50])
        pause(0.05);
        frame=getframe;
        writeVideo(v,frame);
        delete(h);delete(h2);
        if (CollisionCheck(fv, obstacle))
            fprintf (1, 'Ouch\n');
        end     
    end
        h=plot3([0;x_1],[0;y_1],[0;z_1],'LineWidth',2,'Color','c');
        h2=plot3([x_1;x_2],[y_1;y_2],[z_1;z_2],'LineWidth',2,'Color','r');
        hold off;
        
end
close(v);
