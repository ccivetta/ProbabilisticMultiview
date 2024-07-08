%% Reprojecting CheckerBoard Points
%
%The function of this code is to reproject points onto the checkerboard
%images from calibration using different calculated extrinsics. Good way to
%visually evaluate different values for H_c12c2.
%
%The points are calculated and plotted using
% - single camera extrinsics
% - Motion Capture correspondance extrinsics from c1 to b to c2
% - Motion Capture correspondance extrinsics camera->binocular->
%   fiducial->checkerboard fiducial->checkerboard grid
% - combination of both cameras' single image extrinsics
%
% Points are overalyed on the images over the original detected points for
% comparison
% 
%   Legend: 
%       Green Star     - computer vision detected points 
%       Green Circle   - single camera extrinsics for each given image from
%                        calibration 
%       Blue Circle    - go from camera, to binocular fiducial, to other
%                        camera, to grid using calibration extrinsics -
%                        tells us accuracy of known transforms from each
%                        camera to the binocular MoCap fiducial
%       Red Circle     - go from one camera to the other using H_c12c2_mean
%                        from calibration - tells us the accuracy of our
%                        mean trnasform from camera 1 to camera 2 from
%                        calibration
%       Magenta Circle - go from camera, to binocular fiducial, to
%                        checkerboard fiducial, to checkerboard grid -
%                        tells us accuracy of camera to motion capture
%                        fiducial and checkerboard grid to motion capture
%                        fiducial
%
%   C. A. Civetta, M. Kutzer, 02Jul2024, USNA

close all;
clear all;
clc;
%% Initiate Camera
index = '1';
SCRIPT_initializeBinocularCameras;
%% Load Variables and Images
squareSizeMM = 30;
index = '1';

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

load('SavedMatrices\H_f2w.mat')
load('SavedMatrices\H_f2g.mat')
load('SavedMatrices\H_b2w.mat')
load('SavedMatrices\H_b2c.mat')


%Genreate extrinsics
for i=1:2
    H_g2c(i,:) = generateExtrinsics(params{i});
end

load('SavedMatrices\H_c12c2_MoCap.mat');
load('SavedMatrices\H_c12c2_mean.mat');

%Save the cameras' intrinsic matrices 
for i=1:2
    A_c2m{i} = params{i}.Intrinsics.IntrinsicMatrix';
end

for i=1:2
    images{i} = imageSet(['CorrelationImages\' camSaveName{i}]);
    imageFileNames = images{i}.ImageLocation;
    detector = vision.calibration.monocular.CheckerboardDetector();
    [imagePoints{i}, boardSize] = detectPatternPoints(detector, imageFileNames, "PartialDetections", false);
end
%% Plot Images Side by Side
fig = figure('Name','ReprojectPoints.m');

%Convert to mm
for j = 1:length(H_f2w)
    H_f2w{j}(1:3,4) = H_f2w{j}(1:3,4)*1000;
    H_b2w{j}(1:3,4) = H_b2w{j}(1:3,4)*1000;
end

for n = 1:params{1}.NumPatterns
    for i=1:2
        axs(i) = subplot(1,2,i);
        title(axs(i),camSaveName{i});

        % Define world points on the checkerboard fiducial
        p_g{i} = params{i}.WorldPoints.'; %% World points in mm
        p_g{i}(3,:) = 0;
        p_g{i}(4,:) = 1;

        % Define segmented image points from checkerboard detections
        X_m{i} = undistortPoints(imagePoints{i}(:,:,n), params{i});
        X_m{i} = X_m{i}.';
        X_m{i}(3,:) = 1;

        % Define reproject image points using single camera extrinsics
        X_m_rep_tilde{i} = A_c2m{i} * H_g2c{i,n}(1:3,:) * p_g{i};
        X_m_rep{i} = (X_m_rep_tilde{i}./X_m_rep_tilde{i}(3,:));

        % Define reproject image points using MoCap H_c12c2
        H_c22c1_MoCap = invSE(H_c12c2_MoCap);
        if i == 1
            H_g2c1_mocap = H_c22c1_MoCap * H_g2c{2,n};
            X_m_rep_tilde_mocap{i} = A_c2m{i} * H_g2c1_mocap(1:3,:) * p_g{i};
            X_m_rep_mocap{i} = (X_m_rep_tilde_mocap{i}./X_m_rep_tilde_mocap{i}(3,:));
        else
            H_g2c2_mocap = H_c12c2_MoCap* H_g2c{1,n};
            X_m_rep_tilde_mocap{i} = A_c2m{i} * H_g2c2_mocap(1:3,:) * p_g{i};
            X_m_rep_mocap{i} = (X_m_rep_tilde_mocap{i}./X_m_rep_tilde_mocap{i}(3,:));
        end

        % Define reproject image points using mean from calibration
        H_c12c2_combo = H_c12c2_mean;
        if i == 1
            H_c22c1_combo = invSE(H_c12c2_combo);
            H_g2c1_combo =  H_c22c1_combo * H_g2c{2,n};
            X_m_rep_tilde_combo{i} = A_c2m{i} * H_g2c1_combo(1:3,:) * p_g{i};
            X_m_rep_combo{i} = (X_m_rep_tilde_combo{i}./X_m_rep_tilde_combo{i}(3,:));
        else
            H_g2c2_combo = H_c12c2_combo* H_g2c{1,n};
            X_m_rep_tilde_combo{i} = A_c2m{i} * H_g2c2_combo(1:3,:) * p_g{i};
            X_m_rep_combo{i} = (X_m_rep_tilde_combo{i}./X_m_rep_tilde_combo{i}(3,:));
        end

        % Define reproject image points using MoCap camera->binocular
        % fiducial->checkerboard fiducial->checkerboard grid
        if i == 1
            H_g2c1_mocap_long = H_b2c{1} * invSE(H_b2w{n}) * H_f2w{n} * invSE(H_f2g{i});
            X_m_rep_tilde_mocap_long{i} = A_c2m{i} * H_g2c1_mocap_long(1:3,:) * p_g{i};
            X_m_rep_mocap_long{i} = (X_m_rep_tilde_mocap_long{i}./X_m_rep_tilde_mocap_long{i}(3,:));
        else
            H_g2c2_mocap_long = H_b2c{2} * invSE(H_b2w{n}) * H_f2w{n} * invSE(H_f2g{i});
            X_m_rep_tilde_mocap_long{i} = A_c2m{i} * H_g2c2_mocap_long(1:3,:) * p_g{i};
            X_m_rep_mocap_long{i} = (X_m_rep_tilde_mocap_long{i}./X_m_rep_tilde_mocap_long{i}(3,:));
        end

        %Plot all points on image
        img{n} = imread(images{i}.ImageLocation{n});
        undistorted_img = undistortImage(img{n},params{i});
        imObj(n) = imshow(undistorted_img,'Parent',axs(i));
        set(axs(i),'NextPlot','Add');
        plot(axs(i),X_m{i}(1,:), X_m{i}(2,:), 'g*','MarkerSize',4,'LineWidth',1);
        plot(axs(i),X_m_rep{i}(1,:), X_m_rep{i}(2,:), 'go','MarkerSize',3,'LineWidth',1);
        plot(axs(i),X_m_rep_mocap{i}(1,:), X_m_rep_mocap{i}(2,:), 'bo','MarkerSize',3,'LineWidth',1);
        plot(axs(i),X_m_rep_combo{i}(1,:), X_m_rep_combo{i}(2,:), 'ro','MarkerSize',3,'LineWidth',1);
        plot(axs(i),X_m_rep_mocap_long{i}(1,:), X_m_rep_mocap_long{i}(2,:), 'mo','MarkerSize',3,'LineWidth',1);
        legend('Detected Checkerboard Points', 'Reprojection with self Extrinsics','Reprojection with MoCap Extrinsics','Reprojection with Combined Extrinsics', 'Reprojection with MoCap Extrinsics Long Way');
    end
    fprintf('Image %.0f, click to display next image\n', n);
    pause(); %Wait for click to display next image
end