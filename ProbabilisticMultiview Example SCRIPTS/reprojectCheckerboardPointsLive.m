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
live = false;
%% Initiate Camera + UR
SCRIPT_initializeBinocularCameras;
ur = urRTDEClient('10.0.0.100');
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

for i=1:2
    A_c2m{i} = params{i}.Intrinsics.IntrinsicMatrix';
end

rb_len = 1;
p_rb_center(4,1) = 1;

load('URJoingAngles_dataCapture10.mat');
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
    %ttl(i) = title(axs(i),camSaveName{i});

    % Define check points
    checkPoints(i) = plot(axs(i),0,0,'ro','MarkerSize',7,'LineWidth',1);
    detectedCheckPoints(i) = plot(axs(i), 0, 0, 'g*', 'MarkerSize', 7, 'LineWidth',1);
end

%% Run loop
n=1;
if live
    p = 1000000;
else
    p = 10;
end
while (n<=p) %true

    % Move robot to next position
    if ~live
        ur.sendJointConfigurationAndWait(q(n,:),...
            'EndTime',3.0,'Velocity',2*pi,'Acceleration',4*pi);
        pause(.5);
    end
    % gather images
    for i = 1:2
        img{i} = prv(i).CData;
    end

    % gather pose information
    H_b2w = v.Bino.pose;
    H_f2w = v.Check.pose;

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

        % Find detected points
        detector = vision.calibration.monocular.CheckerboardDetector();
        imagePoints{i} = detectPatternPoints(detector, undistorted_img, "PartialDetections", false);
        if size(imagePoints{i},1) == 56
            set(detectedCheckPoints(i),'XData', imagePoints{i}(:,1),'YData',imagePoints{i}(:,2),'Visible','on');
            %Save statistics in struct d as nx2 matrices, where n is the number
            %of images and i is the number of cameras
            [d.mean(n,i),d.median(n,i), d.max(n,i), d.min(n,i), d.std(n,i), d.cov(n,i)] = calcReprojectionStats(imagePoints{i},X_m_rep{i}');
        else
            disp('Full checkerboard not detected');
            set(detectedCheckPoints(i),'XData', 0,'YData',0,'Visible','off');
        end

        % Add Legend
        legend('Reprojected Points', 'Detected Points');


    end
    n = n + 1;
end


%% Create Figure - Run seperately because loop above in infinite
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