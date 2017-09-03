t=1:0.1:50;
b=t.^2;c=t.^3;
axis auto;
for i=1:size(t,2)
    drawnow;
    plot3([0;t(i)],[0;b(i)],[0;c(i)]); 
    pause(0.1);
end