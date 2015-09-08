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

X = 1;
Y = 2;
T = 3;
VX = 4;
VY = 5;
VT = 6;

FIELD_SIZE_X = 2.0;
FIELD_SIZE_Y = 2.0;
GOAL_MIN_Y = 0.65;
GOAL_MAX_Y = 1.35;
BALL_RADIUS = 0.043 / 2.0;
ROBOT_RADIUS = 0.180 / 2.0;
BALL_SLOW = 1.0 - 0.01;

fig_handle = figure('Position', [0 0 600 600]);
finished = 0;
TIME_STEP = 0.02;

% start positions and velocities
ball = [FIELD_SIZE_X/2.0+0.1 FIELD_SIZE_Y/2.0, 0, 0, 0, 0];
robot = [FIELD_SIZE_X/2.0 3.0 * FIELD_SIZE_Y/4.0 0, 0, -1, 0];

time = 0;

while (finished == 0)
    try test = get(fig_handle); catch ME, break; end
    
    % update the ball by the time step
    ball(VX) = ball(VX) * BALL_SLOW;
    ball(VY) = ball(VY) * BALL_SLOW;
    ball(X) = ball(X) + ball(VX) * TIME_STEP;
    ball(Y) = ball(Y) + ball(VY) * TIME_STEP;

    % update the robot by the time step
     robot(X) = robot(X) + robot(VX) * TIME_STEP;
     robot(Y) = robot(Y) + robot(VY) * TIME_STEP;    

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
       ball(VX) = -ball(VX);
    end
    if (ball(Y) > FIELD_SIZE_Y - BALL_RADIUS)
       ball(VY) = -ball(VY);
    end    
    
    robot_ball_dist = sqrt((ball(X)-robot(X))^2 + (ball(Y)-robot(Y))^2);
    if (robot_ball_dist < ROBOT_RADIUS + BALL_RADIUS)
        % from 'zylum' here: http://compsci.ca/v3/viewtopic.php?t=14897
        if ( ((robot(X) - ball(X)) * (ball(VX) - robot(VX)) + (robot(Y) - ball(Y)) * (ball(VY) - robot(VY))) > 0)
            nx = (ball(X) - robot(X)) / (BALL_RADIUS + ROBOT_RADIUS);
            ny = (ball(Y) - robot(Y)) / (BALL_RADIUS + ROBOT_RADIUS); 
            a1 = ball(VX) * nx + ball(VY) * ny;
            a2 = robot(VX) * nx + robot(VY) * ny;
            p = 2 * (a1 - a2) / (2 * pi * BALL_RADIUS + 2 * pi * ROBOT_RADIUS);

            ball(VX) = ball(VX) - p * nx * 2 * pi * ROBOT_RADIUS;
            ball(VY) = ball(VY) - p * ny * 2 * pi * ROBOT_RADIUS;   
            
            % it doesn't make sense to change the robot velocity because
            % (a) it has a much higher mass and (b) the robot has actuators and controller
        end
    end
    
    % RENDER SCENE
    subplot(1, 1, 1, 'replace');    
    hold on;    
    % draw field
    rectangle('Position', [0, 0, FIELD_SIZE_X, FIELD_SIZE_Y], 'FaceColor', 'g');
    % draw robot
    rectangle('Position', [robot(X) - ROBOT_RADIUS, robot(Y) - ROBOT_RADIUS, ROBOT_RADIUS * 2, ROBOT_RADIUS * 2], 'Curvature', [1 1], 'FaceColor', 'k');
    % draw ball
    rectangle('Position', [ball(X) - BALL_RADIUS, ball(Y) - BALL_RADIUS, BALL_RADIUS * 2, BALL_RADIUS * 2], 'Curvature', [1 1], 'FaceColor', [1 0.6 0]);

    axis([-0.2, FIELD_SIZE_X + 0.2, -0.2, FIELD_SIZE_Y + 0.2]);
    hold off;
    drawnow;   
    
    pause(TIME_STEP);   
   
    time = time + TIME_STEP;
end