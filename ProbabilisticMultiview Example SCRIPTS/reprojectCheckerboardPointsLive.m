%% Reproject Checkerboard Points Live
%Reprojecting the checkerboard pattern onto an image. Used to visualize accuracy of
%calibration. 

% While stream is projecting, statistics are saved relating reprojected
% points to computer vision detected points. At the ned of the script,
% these statistics are presented in a bar graph to display how accurate the
% reprojection is, a metric that can be used to analyze the overall
% accuracy of calibration 

% Created: 21AUG2023
% Last Edited: 11OCT2023

% Christopher A. Civetta

close all
clear all
clc

index = '1';

%% Initiate Camera
SCRIPT_initializeBinocularCameras
%% Load necessary variables
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

load('RodCalibration 07 Feb 2023.mat')
load('Red Ball Center 18 Oct 2023.mat') 

for i=1:2
    A_c2m{i} = params{i}.Intrinsics.IntrinsicMatrix';
end

rb_len = 1;
p_rb_center(4,1) = 1;
%% Connect to Optitrack
v = initOptitrack({'Bino', 'BinocularCameras'},{'Check', 'NiceCheckerboard'});
%% Reprojection visualization stream
fig = figure('Name','ReprojectBall.m');
% initialize points
for i = 1:2
    % Create axes
    axs(i) = subplot(1,2,i);

    % Gather initial image
    img{i} = prv(i).CData;

    % Undistort Image
    undistorted_img = undistortImage(img{i},params{i});
    imObj(i) = imshow(undistorted_img,'Parent',axs(i));

    % Set axes properties
    set(axs(i),'NextPlot','Add');
    
    % Make axes title
    ttl(i) = title(axs(i),camSaveName{i});

    % Define check points
    checkPoints(i) = plot(axs(i),0,0,'ro','MarkerSize',3,'LineWidth',1);
end

%% Run loop
n=1;
while (true) %(n<=10)
    % gather images
    for i = 1:2
        img{i} = prv(i).CData;
    end

    % gather pose information
    H_b2w = v.Bino.pose;
    H_f2w = v.Check.pose;

    detector = vision.calibration.monocular.CheckerboardDetector();
    squareSizeMM = 30;
    p_g = params{1}.WorldPoints';
    p_g(3,:) = 0;
    p_g(4,:) = 1;

    % Switch to mm
    H_b2w(1:3,4) = H_b2w(1:3,4)*1000;
    H_f2w(1:3,4) = H_f2w(1:3,4)*1000;

    H_w2b = invSE(H_b2w);


    for i=1:2
        H_g2f{i} = invSE(H_f2g{i});
    end 


    for i = 1:2
        %Calculate Checkerboard grid to camera
        H_g2c{i} = H_b2c{i} * H_w2b * H_f2w * H_g2f{i};
        X_m_rep_tilde{i} = A_c2m{i} * H_g2c{i}(1:3,:) * p_g;
        X_m_rep{i} = (X_m_rep_tilde{i}./X_m_rep_tilde{i}(3,:));
        
        % Undistort image
        undistorted_img = undistortImage(img{i},params{i});
        % Update image in plot
        set(imObj(i),'CData',undistorted_img);

        % Check if point if in FOV and plot MOCAP overlay
        inFrameCheck = inFOV(X_m_rep_tilde{i}, params{1});

        if ~inFrameCheck
            set(checkPoints(i),'Visible','off');
            fprintf(['PIXEL COORDINATES ARE OUT OF ' char(camSaveName(i)) ' FRAME\n'])
            disp(inFrame)
            continue
        else
            set(checkPoints(i),'XData',X_m_rep{i}(1,:),'YData',X_m_rep{i}(2,:),'Visible','on');
        end

        % Calc reprojection accuracy stats 
        detector = vision.calibration.monocular.CheckerboardDetector();
        imagePoints{i} = detectPatternPoints(detector, img{i}, "PartialDetections", false);
        %Save statistics in struct d as nx2 matrices, where n is the number
        %of images and i is the number of cameras
        [d.mean(n,i),d.median(n,i), d.max(n,i), d.min(n,i), d.std(n,i), d.cov(n,i)] = calcReprojectionStats(imagePoints{i},X_m_rep{i}') 
    end
    n = n + 1;
end

%% Create Figure 
figBar = figure('Name', 'Reprojection Error');
baraxs = axes;
set(baraxs, 'NextPlot', 'add');
bars = bar(1:length(d.mean), d.mean, 'Parent', baraxs);
ylabel('Mean Pixel Error');
xlabel('Image');
c1mean_str = mean(d.mean(:,1));
c2mean_str = mean(d.mean(:,2));
c1mean = yline(mean(d.mean(:,1)), '-', 'Color', "#0072BD", 'LineWidth', 3, 'Alpha', 1, 'Label', c1mean_str, LabelHorizontalAlignment="left");
c2mean = yline(mean(d.mean(:,2)), '-', 'Color', "#D95319", 'LineWidth', 3, 'Alpha',1, 'Label', c2mean_str, LabelHorizontalAlignment="left", LabelVerticalAlignment="bottom");
legend('Camera 1', 'Camera 2', 'Camera 1 Mean', 'Camera 2 Mean');
title('Reprojection Error');