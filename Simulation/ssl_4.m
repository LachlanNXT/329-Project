% ENB329 RoboCup F180 Vicion
% David Ball
% Queensland University of Technology, Australia

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

% greyscale
clear all; close all;
im = imread('ball_obstacle_goal_low.jpg');
im_grey = rgb2gray(im);
idisp(im_grey);

pause;

% threshold 
im_wall = im_grey > 230;
idisp(im_wall);

pause;

% threshold 
im_obstacle = im_grey < 40;
idisp(im_obstacle);

pause;

% rotate image and have a look across ball and goal
im_rot = imrotate(im, 90);
idisp(im_rot);

pause;

im_yuv = rgb2ycbcr(im);
idisp(im_yuv);

pause;

idisp(imrotate(im_yuv, 90));

pause;

im_yuv_seg = im_yuv(:,:,3) > 135 & im_yuv(:,:,3) < 190; 
idisp(im_yuv_seg);

pause;

im_yuv_seg = im_yuv(:,:,2) > 80 & im_yuv(:,:,2) < 125 & im_yuv(:,:,3) > 135 & im_yuv(:,:,3) < 190; 
idisp(im_yuv_seg);

pause;

for i=1:10
    im_yuv_seg = ierode(im_yuv_seg, ones(3,3)); 
end

for i=1:40
     im_yuv_seg = idilate(im_yuv_seg, ones(3,3));    
end

idisp(im_yuv_seg);

pause;

im_yuv_seg = im_yuv(:,:,3) > 135 & im_yuv(:,:,3) < 190; 
idisp(im_yuv_seg);
iblobs(im_yuv_seg, 'area', [5000, 1000000000], 'boundary', 'touch', 0)

