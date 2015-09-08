function SSL_2()
% ENB329 RoboCup F180 Small Size Simulator
% David Ball
% Queensland University of Technology, Australia

% Kinematic simulator with rendering
% This version has:
% Field (without corners or goals)
% ball movement with slow down
% robot movement (really just an object)
% ball-wall collisions
% ball-robot collisions
% there is no robot-wall collisions on purpose (for debugging later)

%     Copyright (C) 2013 David Ball (david.ball@qut.edu.au)
% 
%     This program is free software: you can redistribute it and/or modify
%     it under the terms of the GNU General Public len_micense as published by
%     the Free Software Foundation, either version 3 of the len_micense, or
%     (at your option) any later version.
% 
%     This program is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU General Public len_micense for more details.
% 
%     You should have received a copy of the GNU General Public len_micense
%     along with this program.  If not, see <http://www.gnu.org/licenses/>.
%
% MODIFIED BY LACHLAN ROBINSON GROUP 8 2015

% robot and ball state specifiers
close all
X = 1;
Y = 2;
T = 3;
VX = 4;
VY = 5;
VT = 6;

% robot velocity, angular velocity, heading
v = 0.5;
w = 0;
theta = 0;

% gains for proportional controller
kv = 0.5;
kw = 0.1;

% static field parameters
FIELD_SIZE_X = 2.0;
FIELD_SIZE_Y = 2.0;
GOAL_MIN_Y = 0.65;
GOAL_MAX_Y = 1.35;
BALL_RADIUS = 0.043 / 2.0;
ROBOT_RADIUS = 0.180 / 2.0;
BALL_SLOW = 1.0 - 0.01;
Dtol = 2.5;

% dynamic navigator state variables
score = 0;
kicker = 8;
seen = 0;
acquire = 0;
getBall = 0;
goToGoal = 0;
lastSeen = [0, 0];
obstruction = 0;
Lbumper = [0,0];
Rbumper = [0,0];

% figure and setup for main loop
fig_handle = figure('Position', [0 0 600 600]);
finished = 0;
TIME_STEP = 0.02;

% start positions and velocities
ball = [FIELD_SIZE_X/2.0+0.1 FIELD_SIZE_Y/2.0, 0, 0, 0, 0];
robot = [0 0 0, v*cos(theta), v*sin(theta), 0];
obstacles = [0.1 0.2; 0.75 0.4; 1.2 1.5];

time = 0;

% main simulation loop
while (finished == 0)
    try test = get(fig_handle); catch ME, break; end
    
    %Controller for a differential robot
    
    % robot position. All other positions are robot relative
    rPos = [robot(X), robot(Y)];
    posB = [ball(X), ball(Y)] - rPos;
    % angle of ball relative to robot
    thetaB = rot(posB, theta);

    % if the ball is within field of vision
    if abs(thetaB)<pi/6
        % note ball has been seen and save position
        seen = 1;
        if ~getBall && ~goToGoal
        acquire = 1;
        end
        lastSeen = [ball(X), ball(Y)];
    else
        
        % if not seen head to centre of field
        goal = [FIELD_SIZE_X/2.0, FIELD_SIZE_Y/2.0];
        seen = 0;
        
    end
    
    if acquire && ~getBall
        % if ball has been seen, set goal to just behind
        % last seen location of the ball
        goal = [lastSeen(1) - ROBOT_RADIUS*Dtol, lastSeen(2)];

        % if close enough to goal, move on
        if (norm(goal - rPos) <0.03)
            getBall = 1;
            acquire = 0;
        end
    end
    
    if getBall
        % if in position to get ball, get it
        % by setting goal position to ball location
        goal = lastSeen;
    
    end
    
    if goToGoal
        
        goal = [1.8,1];
        
        if (norm(goal - rPos) <0.03)
            goToGoal = 0;
        end
        
    end
        
    %% This Cell is for obstruction detection and avoidance
    
    % Determine square where obstacles might be a problem
    rx = rPos(1); ry = rPos(2);
    gx = goal(1); gy = goal(2);
    if rx < gx
        left = rx; right = gx;
    else
        left = gx; right = rx;
    end
    if ry > gy
        top = ry; bottom = gy;
    else
        top = gy; bottom = ry;
    end
    
    % check if obstacles or ball are in the square
    obslist = zeros(4,2);
    obscount = 1;
    if acquire == 1 && getBall == 0 && goToGoal == 0
        start = 0;
    else
        start = 1;
    end
    for obs_i = start:size(obstacles, 1)
        if obs_i == 0
            ox = lastSeen(1);
            oy = lastSeen(2);
        else
            ox = obstacles(obs_i, X);
            oy = obstacles(obs_i, Y);
        end
        
        if (ox-ROBOT_RADIUS<right && ox+ROBOT_RADIUS>left) && (oy-ROBOT_RADIUS<top && oy+ROBOT_RADIUS>bottom)
            obslist(obscount,:) = [ox, oy];
            obscount = obscount+1;
        end
    end
    
    if obscount > 1
    
    obsmax = obscount-1;
    obsdist = zeros(4,1) + 999;
    obslist2 = zeros(4,2);
    
    % find distance from robot to each of the obstacles
    for obscount = 1:obsmax
        obsdist(obscount) = Edist(rPos,obslist(obscount,:));
    end
    
    % place obstacles found into order from closest to farthest
    for obscount = 1:obsmax
        temp = find(obsdist == min(obsdist));
        obslist2(obscount,:) = obslist(temp,:);
        obsdist(temp) = 999;
    end
    
    obsdist2 = zeros(4,1) + 999;
    
    % determine perpendicular distance from path to obstacles
    for obscount = 1:obsmax
        obsdist2(obscount) = perpdist(rPos,goal,obslist2(obscount,:));
    end
    
    % determine if any obstacles are close enough to be a problem
    for obscount = 1:obsmax
        if obsdist2(obscount)< ROBOT_RADIUS*Dtol
            obstruction = 1;
            break
        else
            obstruction = 0;
        end
    end
    
    % put any problem obstacles into a list
    if obstruction
    obslist3 = zeros(4,2);
    problemcount = 1;
    for obscount = 1:obsmax
        if obsdist2(obscount)< ROBOT_RADIUS*Dtol
            obslist3(problemcount,:) = obslist2(obscount,:);
            problemcount = problemcount+1;
        end
    end
    
    [Dp, Xp, Yp] = perpdist(rPos,goal,obslist3(1,:));
    goal = [Xp + (ROBOT_RADIUS*Dtol/Dp)*(Xp - obslist3(1,1)), Yp + (ROBOT_RADIUS*Dtol/Dp)*(Yp - obslist3(1,2))];
       
    end
    
    
    else
        obstruction = 0;
    end
        
    %% Finished Obstruction Detection
    
    % Updating motion parameters (velocities)
    goalR = goal - rPos;
    thetaB = rot(goalR, theta);
    w = kw * thetaB;
    v = kv * norm(goalR) +0.1;
    
    theta = theta + w;
    if theta<-pi
        theta = theta+2*pi;
    end
    if theta>pi
        theta = theta-2*pi;
    end
    robot(VX) = v*cos(theta);
    robot(VY) = v*sin(theta);    
    
    % update the ball by the time step
    ball(VX) = ball(VX) * BALL_SLOW;
    ball(VY) = ball(VY) * BALL_SLOW;
    ball(X) = ball(X) + ball(VX) * TIME_STEP;
    ball(Y) = ball(Y) + ball(VY) * TIME_STEP;

    % update the robot by the time step
     robot(X) = robot(X) + robot(VX) * TIME_STEP;
     robot(Y) = robot(Y) + robot(VY) * TIME_STEP;   
     
     vPos = [robot(VX), robot(VY)];
     vlen = norm(vPos); vPos = vPos./vlen;
     [~, leftB] = rot(vPos, pi/6);
     [~, rightB] = rot(vPos, -pi/6);
     Lbumper(1) = robot(X) + leftB(1);
     Lbumper(2) = robot(Y) + leftB(2);
     Rbumper(1) = robot(X) + rightB(1);
     Rbumper(2) = robot(Y) + rightB(2);
     
     
    % collision detection
    % makes many assumptions like
    %   no energy lost on collisions
    %   no deformation of wall, robot or ball
    %   no slip during collision
    %   ball has zero mass, robot has infinte ball
    if (ball(X) < BALL_RADIUS)
       ball(VX) = -ball(VX);
    end
    if (ball(Y) < BALL_RADIUS)
       ball(VY) = -ball(VY);
    end
    if (ball(X) > FIELD_SIZE_X - BALL_RADIUS)
        
        if (ball(Y)<GOAL_MAX_Y && ball(Y)>GOAL_MIN_Y)
            score = score+1;
            obstruction = 0;
            getBall = 0;
            ball(X) = rand*2;
            ball(Y) = rand*2;
        end
        
       ball(VX) = -ball(VX);
    end
    if (ball(Y) > FIELD_SIZE_Y - BALL_RADIUS)
       ball(VY) = -ball(VY);
    end    
    
    for robot_i = 0:size(obstacles, 1)
        if robot_i == -2
            rx = Lbumper(1);
            ry = Lbumper(2);
            rvx = robot(VX)*kicker;
            rvy = robot(VY)*kicker;
        elseif robot_i == -1
            rx = Rbumper(1);
            ry = Rbumper(2);
            rvx = robot(VX)*kicker;
            rvy = robot(VY)*kicker;
        elseif robot_i == 0
            rx = robot(X);
            ry = robot(Y);
            rvx = robot(VX)*kicker;
            rvy = robot(VY)*kicker;
        else
            rx = obstacles(robot_i, X);
            ry = obstacles(robot_i, Y);
            rvx = 0;
            rvy = 0;
        end
        
        robot_ball_dist = sqrt((ball(X)-rx)^2 + (ball(Y)-ry)^2);
        if (robot_ball_dist < ROBOT_RADIUS + BALL_RADIUS)
            
            if robot_i == 0
                if seen
                    goToGoal = 1;
                end
            getBall = 0;
            end
            
            % from 'zylum' here: http://compsci.ca/v3/viewtopic.php?t=14897
            if ( ((rx - ball(X)) * (ball(VX) - rvx) + (ry - ball(Y)) * (ball(VY) - rvy)) > 0)
                nx = (ball(X) - rx) / (BALL_RADIUS + ROBOT_RADIUS);
                ny = (ball(Y) - ry) / (BALL_RADIUS + ROBOT_RADIUS); 
                a1 = ball(VX) * nx + ball(VY) * ny;
                a2 = rvx * nx + rvy * ny;
                p = 2 * (a1 - a2) / (2 * pi * BALL_RADIUS + 2 * pi * ROBOT_RADIUS);

                ball(VX) = ball(VX) - p * nx * 2 * pi * ROBOT_RADIUS;
                ball(VY) = ball(VY) - p * ny * 2 * pi * ROBOT_RADIUS;   

                % it doesn't make sense to change the robot velocity because
                % (a) it has a much higher mass and (b) the robot has actuators and controller
            end
        end
    end
    
    % RENDER SCENE
    h = subplot(1, 1, 1, 'replace');    
    hold on;    
    % draw field
    rectangle('Position', [0, 0, FIELD_SIZE_X, FIELD_SIZE_Y], 'FaceColor', 'g');
    % draw robot
    rectangle('Position', [robot(X) - ROBOT_RADIUS, robot(Y) - ROBOT_RADIUS, ROBOT_RADIUS * 2, ROBOT_RADIUS * 2], 'Curvature', [1 1], 'FaceColor', [0 0 1]);
    % draw the obstacles
    for robot_i = 1:size(obstacles, 1)
     rectangle('Position', [obstacles(robot_i, X) - ROBOT_RADIUS, obstacles(robot_i, Y) - ROBOT_RADIUS, ROBOT_RADIUS * 2, ROBOT_RADIUS * 2], 'Curvature', [1 1], 'FaceColor', [0 0 0]);       
    end
    % draw ball
    rectangle('Position', [ball(X) - BALL_RADIUS, ball(Y) - BALL_RADIUS, BALL_RADIUS * 2, BALL_RADIUS * 2], 'Curvature', [1 1], 'FaceColor', [1 0.6 0]);
    
    % RENDERING ADDITIONS:
    
    % draw goal
    rectangle('Position', [FIELD_SIZE_X, GOAL_MIN_Y, 0.05, GOAL_MAX_Y-GOAL_MIN_Y], 'FaceColor', 'r');
    % draw score
    text(0,2.1,num2str(score));
    % draw field of view and heading
    rPos = [robot(X), robot(Y)];
    vPos = [robot(X) + robot(VX), robot(Y) + robot(VY)];
    line([rPos(1), vPos(1)], [rPos(2), vPos(2)]);
    [~, leftA] = rot(vPos - rPos, pi/6);
    leftAG = leftA+rPos;
    line([rPos(1), leftAG(1)], [rPos(2), leftAG(2)]);
    [~, rightA] = rot(vPos - rPos, -pi/6);
    rightAG = rightA+rPos;
    line([rPos(1), rightAG(1)], [rPos(2), rightAG(2)]);
    % draw navigation lines
    plot([rPos(1), goal(1)], [rPos(2), goal(2)], 'r-');
    % is there an obstruction    
    text(1,2.1,num2str(obstruction));
    % sex axis
    axis([-0.2, FIELD_SIZE_X + 0.2, -0.2, FIELD_SIZE_Y + 0.2]);

    hold off;
    drawnow;   
    
    
    pause(TIME_STEP);   
   
    time = time + TIME_STEP;
end

end

function [angle, relative] = rot(pos, theta)

turn = [cos(theta) -sin(theta); sin(theta) cos(theta)];
relative = pos*turn;
angle = atan2(relative(2),relative(1));

end

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
%plot([robot(x),goal(x),obstacle(x),X],[robot(y),goal(y),obstacle(y),Y],'ro')
%line([robot(x),goal(x)],[robot(y),goal(y)])
%axis equal

end

function d = Edist(A,B)

d = sqrt((A(1) - B(1))^2 + (A(2) - B(2))^2);

end