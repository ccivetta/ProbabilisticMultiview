%% Reproject Red ball points in Viper Lab With 2 Cameras
%
%Reprojecting the red ball's center and checkerboard pattern onto an image and comparing it with the
%computer vision observed ball center. Used to visualize accuracy of
%calibration. 
%
% Created: 21AUG2023
% Last Edited: 01Jul2024
%
% Christopher A. Civetta, M.Kutzer, USNA

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
v = initOptitrack({'Bino', 'BinocularCameras'}, {'RedBall', 'RedBall'});
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

    % Define ball center point for MoCap
    centerMOCAP(i) = plot(axs(i),0, 0, 'k*','MarkerSize',5,'LineWidth',3);
    % Define ball center point CV
    centerThold(i) = plot(axs(i),0,0,'g*','MarkerSize',5,'LineWidth',3);
end

%% Run loop
n=1;
while (true) %(n<=10)
    % gather images
    for i = 1:2
        img{i} = prv(i).CData;
    end

    % gather pose information
    H_rb2w = v.RedBall.pose;
    H_b2w = v.Bino.pose;

    % Switch to mm
    H_rb2w(1:3,4) = H_rb2w(1:3,4)*1000;
    H_b2w(1:3,4) = H_b2w(1:3,4)*1000;

    H_w2b = invSE(H_b2w);


    for i=1:2
        H_g2f{i} = invSE(H_f2g{i});
    end 


    for i = 1:2
        % Calculate red ball to camera
        H_rb2c{i} = H_b2c{i} * H_w2b * H_rb2w;

        % Projection matrix = intrinsics * (R_rb2c d_rb2c)
        Proj_rb2m = A_c2m{i} * H_rb2c{i}(1:3,1:4);

        % Calculate scaled pixel coordinates
        p_tildem(:,i) = Proj_rb2m*p_rb_center;
        
        % Undistort image
        undistorted_img = undistortImage(img{i},params{i});
        % Update image in plot
        set(imObj(i),'CData',undistorted_img);

        % Check if point if in FOV and plot MOCAP overlay
        inFrame = inFOV(p_tildem(:,i),params{i});
        if ~inFrame
            set(centerMOCAP(i),'Visible','off');

            fprintf(['PIXEL COORDINATES ARE OUT OF ' char(camSaveName(i)) ' FRAME\n'])
            disp(inFrame)
            continue
        else
            % Calculate pixel coordinates
            p_m(:,i) = p_tildem(:,i)./p_tildem(3,i);
            
            set(centerMOCAP(i),'XData',p_m(1,i),'YData',p_m(2,i),'Visible','on');
        end
        
        % Threshold and plot center
        [cvCenter] = detectBall(undistorted_img);
        if ~isempty(cvCenter)
            set(centerThold(i),'XData',cvCenter(1),'YData',cvCenter(2),'Visible','off');
        else
            set(centerThold(i),'Visible','off');
        end
    end
    n = n + 1;
end