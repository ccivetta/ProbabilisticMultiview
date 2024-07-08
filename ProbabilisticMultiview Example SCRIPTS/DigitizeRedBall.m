%% Digitize Red Ball
% Script used to digitze the red ball center relative to the fixed red ball
% MoCap fiducial
%
% Calibration wand is used to capture points on the surfase of the sphere.
% Points are saved and fit to a sphere in order to calculate the center of
% the ball's position relative to the MoCap fiducial. 
%
%   C. A. Civetta, C. Doherty, M. Kutzer, 22Aug2023, USNA


close all
clear all
clc

load 'RodCalibration 07 Feb 2023.mat'

%% Connect to Optitrack
v = initOptitrack({'Wand', 'DigitizingWand'}, {'RedBall', 'RedBall'});
%% Take data
index = 1;
while(1)
    disp('Press any key to define a point on the red ball rigid body')
    pause()

    % Recieve the motion capture poses   
    H_r2w = v.Wand.pose;
    H_rb2w = v.RedBall.pose;

    H_r2w(1:3,4) = H_r2w(1:3,4) * 1000;
    H_rb2w(1:3,4) = H_rb2w(1:3,4) * 1000;

    H_w2rb = invSE(H_rb2w);


    % Calculate position of the tip relative to world 
    p_w = H_r2w * p_r;

    % Calculate position of the tip relative to red ball rigid body
    p_rb(1:4,index) = H_w2rb * p_w;

    disp(index)
    index = index + 1;
end

%% Save points relative to red ball 
save(['Digitized Points ' datestr(datetime('now'),'dd mmm yyyy') '.mat'], 'p_rb')
save(['Digitized Points Wand Pose ' datestr(datetime('now'),'dd mmm yyyy') '.mat'], 'H_r2w')


%% Fit red ball points to a sphere
X_rb(1:3,:) = p_rb(1:3,:);
sfit = fitSphere(X_rb);

% plot sphere
[X,Y,Z] = sphere;
hold on
r = sfit.Radius;
X2 = X * r;
Y2 = Y * r;
Z2 = Z * r;
surf(X2+sfit.Center(1),Y2+sfit.Center(2),Z2+sfit.Center(3)) %plor red ball with measured radius and center relative to red ball rigid body
plot3(sfit.Center(1),sfit.Center(2),sfit.Center(3),'r.')
plot3(X_rb(1,:),X_rb(2,:),X_rb(3,:),'r.')
axis equal
title('Sphere Fit to Motion Tracked Points')

p_rb_center = [sfit.Center(1);sfit.Center(2);sfit.Center(3)];
save(['Red Ball Center ' datestr(datetime('now'),'dd mmm yyyy') '.mat'], 'p_rb_center')



