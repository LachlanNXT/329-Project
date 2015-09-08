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

global CONSTANTS;
CONSTANTS.FIELD_SIZE_X = 2.0;
CONSTANTS.FIELD_SIZE_Y = 2.0;
CONSTANTS.GOAL_MIN_Y = 0.65;
CONSTANTS.GOAL_MAX_Y = 1.35;
CONSTANTS.BALL_RADIUS = 0.043 / 2.0;
CONSTANTS.ROBOT_RADIUS = 0.180 / 2.0;
CONSTANTS.BALL_SLOW = 1.0 - 0.005;
CONSTANTS.KICKER_VEL = 3.0;
CONSTANTS.TIME_STEP = 0.02;
CONSTANTS.WHEEL_BASE = 0.1;

fig_handle = figure('Position', [0 0 600 600]);
finished = 0;

global robot;
global obstacles;
global ball;
% start positions and velocities
ball.px = CONSTANTS.FIELD_SIZE_X/2.0+0.1;
ball.py = CONSTANTS.FIELD_SIZE_Y/2.0;
ball.vx = 4;
ball.vy = 1;
robot.px = CONSTANTS.FIELD_SIZE_X/2.0;
robot.py = 3.0 * CONSTANTS.FIELD_SIZE_Y/4.0;
robot.pt = 0;
robot.vx = 0;
robot.vy = 0;
robot.vt = 0;
obstacles(1).px = 0.1;
obstacles(1).py = 0.2; 
obstacles(2).px = 0.75; 
obstacles(2).py = 0.4;
obstacles(3).px = 1.2;
obstacles(3).py = 1.5;
obstacles(4).px = 1;
obstacles(4).py = 1;
obstacles(5).px = 0.5;
obstacles(5).py = 1.5;
time = 0;

rf = 0.5;
rt = 2.0;


vr = 0;
vl = 0;
kicker = 1;

MAX_ROBOT_VEL = 0.5;
MAX_ROBOT_ROT = 2.0;

navigation.goal_x = CONSTANTS.FIELD_SIZE_X / 2.0;
navigation.goal_y = CONSTANTS.FIELD_SIZE_Y / 2.0;

GOAL_P = 100.0;

DIFF = 1;

goal_map = zeros(360);
obs_map = zeros(360);
res_map = zeros(360);

while (finished == 0)
    try test = get(fig_handle); catch ME, break; end%#ok<NASGU>
    
    % this simulates a perfect global position sensor
    [r_px, r_py, r_pt, r_vx, r_vy, r_vt] = Get_Robot();
        
     if mod(time, 5) < CONSTANTS.TIME_STEP
        navigation.goal_x = rand * CONSTANTS.FIELD_SIZE_X;
        navigation.goal_y = rand * CONSTANTS.FIELD_SIZE_Y;
     end
    
    goal_rad = atan2(navigation.goal_y - r_py, navigation.goal_x - r_px);
   
    goal_deg = int64(Clip_Deg_360(radtodeg(goal_rad)));
    goal_map(goal_deg) = 1;
    for angle = 1:179
       goal_map(int64(Clip_Deg_360(goal_deg - angle))) = 1 - angle/200;
       goal_map(int64(Clip_Deg_360(goal_deg + angle))) = 1 - angle/200;       
    end
    
    for angle = 1:360
        obs_map(angle) = 0;
    end
    for obs_id=1:size(obstacles, 2)
        obs_x = obstacles(obs_id).px;
        obs_y = obstacles(obs_id).py;

        obs_dist = max(CONSTANTS.ROBOT_RADIUS * 2 + 0.02, sqrt((obs_x-r_px)^2 + (obs_y-r_py)^2));
        obs_rad = atan2(obs_y-r_py, obs_x-r_px);
        obs_width = CONSTANTS.ROBOT_RADIUS + CONSTANTS.ROBOT_RADIUS + 0.02; % their width plus our width plus a margin
        obs_width_rad = asin(obs_width/obs_dist);

        obs_deg = int64(Clip_Deg_360(radtodeg(obs_rad)));
        obs_width_deg = int64(Clip_Deg_360(radtodeg(obs_width_rad)));
        obs_effect = max(0, 1 - min(1, (obs_dist - CONSTANTS.ROBOT_RADIUS*2)));
        obs_map(obs_deg) = obs_effect;
        for angle = 1:obs_width_deg
           obs_map(int64(Clip_Deg_360(obs_deg - angle))) = max(obs_map(int64(Clip_Deg_360(obs_deg - angle))), obs_effect);
           obs_map(int64(Clip_Deg_360(obs_deg + angle))) = max(obs_map(int64(Clip_Deg_360(obs_deg + angle))), obs_effect);;       
        end
    end
    
    max_value = -1;
    for angle = 1:360
       res_map(angle) = max(0, goal_map(angle) - obs_map(angle)); 
       if (res_map(angle) > max_value)
           heading_rad = degtorad(angle);
           max_value = res_map(angle);
       end
    end
    
    goal_error = Get_Signed_Delta_Rad(r_pt, heading_rad);
    
    d_rt = min(MAX_ROBOT_ROT, max(-MAX_ROBOT_ROT, goal_error * GOAL_P * CONSTANTS.TIME_STEP));
    d_rf = MAX_ROBOT_VEL * (1.0 - 0.8*abs(d_rt)/MAX_ROBOT_ROT);

    if DIFF
        [vr, vl] = Diff_Controller(d_rf, d_rt, sqrt(r_vx^2 + r_vy^2), r_vt);     
        Simulate(vr, vl, kicker);
    end
    
    Render(navigation);
    map_size = 0.5;
    inc = 10;
    for angle = 1:inc:(360-1)
        line([robot.px + goal_map(angle) * map_size * cos(degtorad(angle)), robot.px + goal_map(Clip_Deg_360(angle+inc)) * map_size * cos(degtorad(angle+inc))], [robot.py + goal_map(angle) * map_size * sin(degtorad(angle)), robot.py + goal_map(Clip_Deg_360(angle+inc)) * map_size * sin(degtorad(angle+inc))], 'Color', [0 0 0], 'LineWidth', 2);
        line([robot.px + obs_map(angle) * map_size * cos(degtorad(angle)), robot.px + obs_map(Clip_Deg_360(angle+inc)) * map_size * cos(degtorad(angle+inc))], [robot.py + obs_map(angle) * map_size * sin(degtorad(angle)), robot.py + obs_map(Clip_Deg_360(angle+inc)) * map_size * sin(degtorad(angle+inc))], 'Color', [1 0 0], 'LineWidth', 2);
        line([robot.px + res_map(angle) * map_size * cos(degtorad(angle)), robot.px + res_map(Clip_Deg_360(angle+inc)) * map_size * cos(degtorad(angle+inc))], [robot.py + res_map(angle) * map_size * sin(degtorad(angle)), robot.py + res_map(Clip_Deg_360(angle+inc)) * map_size * sin(degtorad(angle+inc))], 'Color', [1 1 1], 'LineWidth', 2);
    end
    line([robot.px, robot.px + map_size / 2 * cos(heading_rad)], [robot.py, robot.py + map_size / 2 * sin(heading_rad)], 'Color', [0 0 1], 'LineWidth', 5);
    hold off;
    drawnow; 
    
   % pause(CONSTANTS.TIME_STEP);   
   
    time = time + CONSTANTS.TIME_STEP
    

end

end


function [vr, vl] = Diff_Controller(d_rf, d_rt, current_rf, current_rt)
global CONSTANTS;

    r_acc = 3.0 * CONSTANTS.TIME_STEP;

    if d_rf < r_acc && current_rf < r_acc
        rf = d_rf;    
    elseif d_rf > current_rf
        rf = current_rf + r_acc;
    else
        rf = current_rf - r_acc;
    end

    
    if abs(d_rt) < r_acc && abs(current_rt) < r_acc
        rt = d_rt;    
    elseif d_rt > 0 && d_rt >= current_rt
        rt = current_rt + r_acc;
    elseif d_rt > 0 && d_rt < current_rt
        rt = current_rt - r_acc;
    elseif d_rt < 0 && d_rt <= current_rt
        rt = current_rt - r_acc;
    elseif d_rt < 0 && d_rt > current_rt
        rt = current_rt + r_acc;
    end 
   
    vr = rf + rt * CONSTANTS.WHEEL_BASE / 2.0;
    vl = rf - rt * CONSTANTS.WHEEL_BASE / 2.0;
end

function [px, py, pt, vx, vy, vt] = Get_Robot()
    
    global robot;

    px = robot.px;
    py = robot.py;
    pt = robot.pt;
    vx = robot.vx;
    vy = robot.vy;
    vt = robot.vt;
end

function Simulate(vr, vl, kicker)

global CONSTANTS;
global robot;
global obstacles;
global ball;
    vf = (vr + vl)/2;
    robot.vt = (vr - vl)/CONSTANTS.WHEEL_BASE;

    
    robot.vx = vf * cos(robot.pt);
    robot.vy = vf * sin(robot.pt);
    
    % update the position of the robot
    robot.px = robot.px + robot.vx * CONSTANTS.TIME_STEP;
    robot.py = robot.py + robot.vy * CONSTANTS.TIME_STEP;
    robot.pt = robot.pt + robot.vt * CONSTANTS.TIME_STEP;

    % update the ball by the time step
    ball.vx = ball.vx * CONSTANTS.BALL_SLOW;
    ball.vy = ball.vy * CONSTANTS.BALL_SLOW;
    ball.px = ball.px + ball.vx * CONSTANTS.TIME_STEP;
    ball.py = ball.py + ball.vy * CONSTANTS.TIME_STEP;

    % collision detection
    % makes many assumptions like
    %   no energy lost on collisions
    %   no deformation of wall, robot or ball
    %   no slip during collision
    %   ball has zero mass, robot has infinte ball
    if (ball.px < CONSTANTS.BALL_RADIUS)
       ball.vx = -ball.vx;
    end
    if (ball.py < CONSTANTS.BALL_RADIUS)
       ball.vy = -ball.vy;
    end
    if (ball.px > CONSTANTS.FIELD_SIZE_X - CONSTANTS.BALL_RADIUS)
       ball.vx = -ball.vx;
    end
    if (ball.py > CONSTANTS.FIELD_SIZE_Y - CONSTANTS.BALL_RADIUS)
       ball.vy = -ball.vy;
    end    
    
    for robot_i = 0:size(obstacles, 2)
        

        if robot_i == 0
            rx = robot.px;
            ry = robot.py;
            rvx = robot.vx;
            rvy = robot.vy;
        else
            rx = obstacles(1, robot_i).px;
            ry = obstacles(1, robot_i).py;
            rvx = 0;
            rvy = 0;
        end

        robot_ball_dist = sqrt((ball.px-rx)^2 + (ball.py-ry)^2);

        % kicker
        if robot_i == 0 && kicker == 1
            if (robot_ball_dist < CONSTANTS.ROBOT_RADIUS + CONSTANTS.BALL_RADIUS)
                ball.vx = CONSTANTS.KICKER_VEL * cos(atan2(ball.py-ry, ball.px-rx));
                ball.vy = CONSTANTS.KICKER_VEL * sin(atan2(ball.py-ry, ball.px-rx));
            end
        else
            
            if (robot_ball_dist < CONSTANTS.ROBOT_RADIUS + CONSTANTS.BALL_RADIUS)
                % from 'zylum' here: http://compsci.ca/v3/viewtopic.php?t=14897
                if ( ((rx - ball.px) * (ball.vx - rvx) + (ry - ball.py) * (ball.vy - rvy)) > 0)
                    nx = (ball.px - rx) / (CONSTANTS.BALL_RADIUS + CONSTANTS.ROBOT_RADIUS);
                    ny = (ball.py - ry) / (CONSTANTS.BALL_RADIUS + CONSTANTS.ROBOT_RADIUS); 
                    a1 = ball.vx * nx + ball.vy * ny;
                    a2 = rvx * nx + rvy * ny;
                    p = 2 * (a1 - a2) / (2 * pi * CONSTANTS.BALL_RADIUS + 2 * pi * CONSTANTS.ROBOT_RADIUS);

                    ball.vx = ball.vx - p * nx * 2 * pi * CONSTANTS.ROBOT_RADIUS;
                    ball.vy = ball.vy - p * ny * 2 * pi * CONSTANTS.ROBOT_RADIUS;   

                    % it doesn't make sense to change the robot velocity because
                    % (a) it has a much higher mass and (b) the robot has actuators and controller
                end
            end
     
        end
    end

end

function Render(navigation)

global CONSTANTS;
global robot;
global obstacles;
global ball;

    % RENDER SCENE
    subplot(1, 1, 1, 'replace');    
    hold on;    
    % draw field
    rectangle('Position', [0, 0, CONSTANTS.FIELD_SIZE_X, CONSTANTS.FIELD_SIZE_Y], 'FaceColor', 'g');
    % draw the obstacles
    for robot_i = 1:size(obstacles, 2)
        rectangle('Position', [obstacles(1, robot_i).px - CONSTANTS.ROBOT_RADIUS, obstacles(1, robot_i).py - CONSTANTS.ROBOT_RADIUS, CONSTANTS.ROBOT_RADIUS * 2, CONSTANTS.ROBOT_RADIUS * 2], 'Curvature', [1 1], 'FaceColor', [0 0 0]);       
    end
   % draw robot
    rectangle('Position', [robot.px - CONSTANTS.ROBOT_RADIUS, robot.py - CONSTANTS.ROBOT_RADIUS, CONSTANTS.ROBOT_RADIUS * 2, CONSTANTS.ROBOT_RADIUS * 2], 'Curvature', [1 1], 'FaceColor', [0 0 1]);    
    line([robot.px robot.px + CONSTANTS.ROBOT_RADIUS * cos(robot.pt)], [robot.py robot.py + CONSTANTS.ROBOT_RADIUS * sin(robot.pt)], 'Color', [1 1 1]);
    % draw ball
    rectangle('Position', [ball.px - CONSTANTS.BALL_RADIUS, ball.py - CONSTANTS.BALL_RADIUS, CONSTANTS.BALL_RADIUS * 2, CONSTANTS.BALL_RADIUS * 2], 'Curvature', [1 1], 'FaceColor', [1 0.6 0]);
    % draw goal
    rectangle('Position', [navigation.goal_x - 0.01, navigation.goal_y - 0.01, 0.01 * 2, 0.01 * 2], 'Curvature', [1 1], 'FaceColor', [1 0 0]);
    
    axis([-0.2, CONSTANTS.FIELD_SIZE_X + 0.2, -0.2, CONSTANTS.FIELD_SIZE_Y + 0.2]);

  
end

function [delta]=Get_Min_Delta(d1, d2, max)
% Get the minimum delta distance between two values assuming a wrap to zero
% at max
    delta = min([abs(d1 - d2), max - abs(d1 - d2)]);
end

function [angle]=Clip_Rad_360(angle)
% Clip the input angle to between 0 and 2pi radians

    while angle < 0
        angle = angle + 2*pi;
    end
    while angle >= 2*pi
        angle = angle - 2*pi;
    end
end

function [angle]=Clip_Deg_360(angle)
% Clip the input angle to between 0 and 360 radians

    while angle < 1
        angle = angle + 360;
    end
    while angle >= 361
        angle = angle - 360;
    end
end

function [angle]=Clip_Rad_180(angle)
% Clip the input angle to between -pi and pi radians

    while angle > pi
        angle = angle - 2*pi;
    end
    while angle <= -pi
        angle = angle + 2*pi;
    end
end




function [angle]=Get_Signed_Delta_Rad(angle1, angle2)
% Get the signed delta angle from angle1 to angle2 handling the wrap from 2pi
% to 0.

    dir = Clip_Rad_180(angle2 - angle1);

    delta_angle = abs(Clip_Rad_360(angle1) - Clip_Rad_360(angle2));

    if (delta_angle) < (2*pi - delta_angle)
        if (dir > 0)
            angle = delta_angle;
        else
            angle = -delta_angle;
        end
    else
        if (dir > 0)
            angle = 2*pi - delta_angle;
        else
            angle = -(2*pi - delta_angle);
        end
    end

end