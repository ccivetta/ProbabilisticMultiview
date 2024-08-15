%% SCRIPT_DISTANCE_FROM_BALL 
% Calculate the distance from the base of the robot to the red ball'
% Used for calibrating motion stage ranges 
load('Red Ball Center 24 Jul 2024.mat')

p_rb_center = p_rb_center ./ 1000; % conver to m

p_rb_center(4) = 1;

v = initOptitrack({'RedBall', 'RedBall'},{'Base', 'UR10Base'});

H_rb_c2o = (v.Base.pose)^(-1) * v.RedBall.pose * p_rb_center;
distance = sqrt(H_rb_c2o(1)^2 + H_rb_c2o(2)^(2) + H_rb_c2o(3)^(2));
fprintf('%f m\n', distance);