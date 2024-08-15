%% Reproject Red ball with the addition of probability rings
%
% Reprojecting the red balls center onto an image and adding probability
% information based on what we know about the system uncertainty.
%
% Reprojection points are also added for
%       a single sample based on calibration values - Black X
%       the pixel location of the ball from computer vision - Red X
%       Motion Capture predicted location - Green X
%
% Run this script immediately after the monte carlo simulation or ensure
% you import values for: 
%                   meanx - the mean value for x in all MonteCarlo points 
%                   meany - the mean value for y in all MonteCarlo points
%                   meanz - the mean value for z in all MonteCarlo points
%                   cov3D - the covariance value for x, y, and z in all MonteCarlo points
%
% Created: 22Feb2024
% Edited: 02Jul2024
%
% C. A. Civetta, M. Kutzer, 02Jul2024, USNA

clear undistorted_img
clear img
clear imgObj
clc

index = '1';
%% Initiate Camera
SCRIPT_initializeBinocularCameras;
%% Load necessary matrices
for i = 1:2
    device = imaqhwinfo('winvideo',cam(i).DeviceID);
    camFormat{i} = [num2str(resy(i)) 'x' num2str(resx(i))];
    camName{i} = device.DeviceName;
    camSaveName{i} = [camFormat{i} '_' camName{i} index];

    load(['SavedMatrices\' camSaveName{i} '\CameraParams.mat'])
    bubble{i} = params;
end
clear params
params = bubble;
clear bubble
load('SavedMatrices\H_b2c.mat')
load('SavedMatrices\H_f2g.mat')
load('SavedMatrices\H_c12c2_MoCap.mat')
load('SavedMatrices\H_c12c2_mean.mat')
load('Red Ball Center 09 Aug 2024.mat')
for i=1:2
    A_c2m{i} = params{i}.Intrinsics.IntrinsicMatrix';
end

p_rb_center(4,1) = 1;
%% Connect to Optitrack
v = initOptitrack({'Bino', 'BinocularCameras'}, {'RedBall', 'RedBall'});
%% Reprojection visualization stream - Only Camera 1
fig = figure('Name','ReprojectBall.m');

% Create axes
axs = axes;
for i = 1:2
    % Gather initial image
    img{i} = prv(i).CData;

    % Undistort Image
    undistorted_img{i} = undistortImage(img{i},params{i});

    % Locate Ball
    [p_rb_center_m_cv{i}] = detectBall(undistorted_img{i});

    % Set axes properties
    set(axs,'NextPlot','Add');
end
imObj = imshow(undistorted_img{1},'Parent',axs);
%% Plot 3D Object Reprojection
colors = 'rygcmykrgbcmyk';
intervals = [.95 .50 .05]; % adjust to change what confidence intervals are plotted 
axsHold = axes;
set(axsHold, 'Visible', 'off');

for i = 1:length(intervals)

    CI = intervals(i);
    % Plot ellipsoid for visualization


    X_o3D = confIntervalPoints3D([meanx meany meanz], cov3D, CI);

    P_c12m = A_c2m{1}*[1 0 0 0; 0 1 0 0; 0 0 1 0];
    tildeX_o_rep3D = P_c12m*X_o3D;
    X_o_rep3D = tildeX_o_rep3D./tildeX_o_rep3D(3,:);

    ellipsoidPoints(i) = plot(axs,0, 0, ['-',colors(i)],'LineWidth',1);
    set(ellipsoidPoints(i), 'XData', X_o_rep3D(1,:), 'YData', X_o_rep3D(2,:));
end

%% Plot Monte Carlo Points
% ballRep = plot3(axs,nan, nan, nan, 'bo','MarkerSize',1,'LineWidth',1);
% for i=1:numel(p_c_mean)
%     p_c_mean{i}(4) = 1;
%     p_c1_rep_mean{i} = (1/p_c_mean{i}(3))*P_c12m*p_c_mean{i};
%     appendLine(ballRep,p_c1_rep_mean{i}(1:3,1));
% end
%% Calculate Red Ball Position Relative to Camera 1: NO MOCAP

%Save pixel location of the red ball in each image
p_m1 = [p_rb_center_m_cv{1} 1]';
p_m2 = [p_rb_center_m_cv{2} 1]';

[zc1, zc2] = recoverDepth(p_m1, p_m2, A_c2m{1}, A_c2m{2}, invSE(H_c12c2_mean));

%Define projection matrices for each camera to itself
P_c12m = A_c2m{1}*[1 0 0 0; 0 1 0 0; 0 0 1 0];
P_c22m = A_c2m{2}*[1 0 0 0; 0 1 0 0; 0 0 1 0];

% Recover 3D point of the red ball relative to each camera
p_c1 = recoverPoints(p_m1, zc1, P_c12m);
p_c2 = recoverPoints(p_m2, zc2, P_c22m);

%Calculate pixel location of generated 3D point
p_m1_rep = (1/zc1)*P_c12m*p_c1;
p_m2_rep = (1/zc2)*P_c22m*p_c2;

% Convert point relative to c2 to c1 coordinate frame for plotting
p_c2_relc1 = invSE(H_c12c2_MoCap) * p_c2; %MOCAP is used here to ensure coordinate frame switch is good for plotting


%% Plot reprojected ball - Camera 1 only: NO MOCAP

%Calculate average of both recovered positions
p_c_mean_single = [mean([p_c1(1),p_c2_relc1(1)]); mean([p_c1(2),p_c2_relc1(2)]); mean([p_c1(3),p_c2_relc1(3)]); 1];

%Convert position to pixel location
p_m1_rep_mean = (1/p_c_mean_single(3))*P_c12m*p_c_mean_single;

%Plot
center = plot(axs,0, 0, 'kx','MarkerSize',15,'LineWidth',3);
center_cv = plot(axs,0, 0, 'bx','MarkerSize',15,'LineWidth',3);
set(center_cv,'XData',p_rb_center_m_cv{1}(1),'YData',p_rb_center_m_cv{1}(2),'Visible','on');
set(center,'XData',p_m1_rep_mean(1),'YData',p_m1_rep_mean(2),'Visible','on');

%% Calculate Red Ball Position Relative to Camera 1: MOCAP

%Get necessary values from MoCap
H_rb2w = v.RedBall.pose;
H_b2w = v.Bino.pose;

H_rb2w(1:3,4) = H_rb2w(1:3,4)*1000; % Convert from m to mm
H_b2w(1:3,4) = H_b2w(1:3,4)*1000;   % Convert from m to mm

H_w2b = invSE(H_b2w);

H_rb2c{1} = H_b2c{1} * H_w2b * H_rb2w;

% Projection matrix = intrinsics * (R_rb2c d_rb2c)
Proj_rb2m = A_c2m{1} * H_rb2c{1}(1:3,1:4);

% Calculate scaled pixel coordinates of red ball per camera
p_tilde_rb_center_mocap(:,1) = Proj_rb2m*p_rb_center;

% Save the pixel coordinates as rbCenter - overwriting computer
% vision determined pixel location.
p_rb_center_mocap = [p_tilde_rb_center_mocap(1,1)./p_tilde_rb_center_mocap(3,1) p_tilde_rb_center_mocap(2,1)./p_tilde_rb_center_mocap(3,1)];

center_mocap_m = plot(axs,0, 0, 'xm','MarkerSize',15,'LineWidth',3);
set(center_mocap_m, 'XData', p_rb_center_mocap(1), 'YData', p_rb_center_mocap(2), 'Visible', 'on')

%% Add Legend
legend([ellipsoidPoints(1), ellipsoidPoints(2), ellipsoidPoints(3), center_mocap_m, center_cv, center], ...
         {'95% Confidence Interval', '50% Confidence Interval', '5% Confidence Interval', 'MoCap Ground Truth', 'Computer Vision Estimate', 'Sample Estimate'});

%legend([ellipsoidPoints(1), ellipsoidPoints(2), ellipsoidPoints(3), center_mocap_m], ...
         %{'95% Confidence Interval', '50% Confidence Interval', '5% Confidence Interval', 'MoCap Ground Truth'});