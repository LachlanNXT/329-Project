function [angle, relative] = rota(pos, theta)

turn = [cos(theta) -sin(theta); sin(theta) cos(theta)];
relative = pos*turn;
angle = atan2(relative(2),relative(1));

figure
plot([0,pos(1)],[0,pos(2)])
hold on
plot([0,relative(1)],[0,relative(2)]);
axis equal

end