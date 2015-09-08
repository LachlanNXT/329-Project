function [d, X, Y] = perpdist(robot, goal, obstacle)

x = 1; y = 2;
Mr = (robot(y)-goal(y))/(robot(x)-goal(x));
if Mr>999
    Mr = 999;
end
if Mr < -999
    Mr = -999;
end
Mo = -1/Mr;
Cr = robot(y) - Mr*robot(x);
Co = obstacle(y) - Mo*obstacle(x);
X = (Cr - Co)/(Mo - Mr);
Y = Mr * X + Cr;
d = Edist(obstacle,[X, Y]);
plot([robot(x),goal(x),obstacle(x),X],[robot(y),goal(y),obstacle(y),Y],'ro')
line([robot(x),goal(x)],[robot(y),goal(y)])
axis equal

end