%% Uncertainty Images + Data Collection
% A script used to collect the images and corresponding data for VIPER lab
% uncertainty data collection. 

% This script can be run instead of collecting data for each type of
% uncertainty when the checkerboard AND red ball are visible in all
% calibration images (this should be the case when using automated data
% capture in VIPER)

close all
clear all
clc


% The index will be added to the end of every save name. Change this value
% if you have multiple trials of the same data
index = '1';

% Square size of the checkerboard you are using
squareSizeMM = 30;
boardSize = [8,9];

%% Initiate OptiTrack + UR
v = initOptitrack({'Bino', 'BinocularCameras'}, {'RedBall', 'RedBall'}, {'Check', 'NiceCheckerboard'}, {'Base', 'UR10Base'});
ur = urRTDEClient('10.0.0.100');
%ura = URx_ROS('ura');

%% Initiate Cameras
SCRIPT_initializeBinocularCameras;

%% Load variables
load('SavedMatrices\H_b2c.mat');
load('SavedMatrices\H_f2g.mat');
for i = 1:2
    device = imaqhwinfo('winvideo',cam(i).DeviceID);
    camFormat{i} = [num2str(resy(i)) 'x' num2str(resx(i))];
    camName{i} = device.DeviceName;
    camSaveName{i} = [camFormat{i} '_' camName{i} index];
    mkdir(['ExtrinsicImages\' camSaveName{i}])
    mkdir(['IntrinsicImages\' camSaveName{i}])
    mkdir(['SegmentationImages\' camSaveName{i}])
    mkdir(['ExtrinsicDataMatrices\' camSaveName{i}])
    mkdir(['IntrinsicDataMatrices\' camSaveName{i}])
    mkdir(['SegmentationDataMatrices\' camSaveName{i}])

    load(['SavedMatrices\' camSaveName{i} '\CameraParams.mat'])
    bubble{i} = params;
end
clear params
params = bubble;
clear bubble

load('SavedMatrices\H_c12c2_mean.mat')
load("URJoingAngles_Planar.mat");

%% Begin Capturing Data
disp("Begin Taking Images/MoCap Data")
%binocularDisplay = figure;
for k=1:size(q,1)
    %img = binocularCaptureDisplay(prv, binocularDisplay); USE THIS IF
    %TAKING IMAGES  MANUALLY


    if mod(k-1,7) == 0
    ur.sendJointConfigurationAndWait(q(k,:),...
        'EndTime',3.0,'Velocity',2*pi,'Acceleration',4*pi);

    else
        ur.sendJointConfigurationAndWait(q(k,:),...
        'EndTime',1.0,'Velocity',2*pi,'Acceleration',4*pi);
    end
    pause(.5);

    %Save Images
    for i=1:2
        img{i} = prv(i).CData;
        imwrite(img{i}, ['ExtrinsicImages\' camSaveName{i} '\ExImage' num2str(k,'%03.f') '.jpg']);
        imwrite(img{i}, ['IntrinsicImages\' camSaveName{i} '\InImage' num2str(k,'%03.f') '.jpg']);
        imwrite(img{i}, ['SegmentationImages\' camSaveName{i} '\SegImage' num2str(k,'%03.f') '.jpg'])
        undistorted_img = undistortImage(img{i},params{i});
        [p_detected{i,k}] = detectBall(undistorted_img);
    end
    fprintf("Saved Image %d of %d \n",k, size(q,1))

    % Save MoCap + UR Data
    H_b2w_ex{k} = v.Bino.pose; %Saving Bino Fiducial in m
    H_o2w_ex{k} = v.Base.pose; %Saving base frame fidicual in m
    H_rb2w_ex{k} = v.RedBall.pose; %Saving base frame fidicual in m
    H_f2w_ex{k} = v.Check.pose;
    urJointAngles_ex{k} = ur.readJointConfiguration;
    H_b2w_in{k} = v.Bino.pose; %Saving Bino Fiducial in m
    H_o2w_in{k} = v.Base.pose; %Saving base frame fidicual in m
    H_rb2w_in{k} = v.RedBall.pose; %Saving base frame fidicual in m
    H_f2w_in{k} = v.Check.pose;
    urJointAngles_in{k} = ur.readJointConfiguration;
    H_b2w_seg{k} = v.Bino.pose; %Saving Bino Fiducial in m
    H_o2w_seg{k} = v.Base.pose; %Saving base frame fidicual in m
    H_rb2w_seg{k} = v.RedBall.pose; %Saving base frame fidicual in m
    urJointAngles_seg{k} = ur.readJointConfiguration;
end
disp("End of Image Gathering")
%close(binocularDisplay);

%% Remove bad data
%Check all images for imagepoints and save the image number for any images
%that do not contain detectable points. These images and all corresponding
%data will be removed before further processing

bad = [];

for i=1:2
    images = imageSet(['ExtrinsicImages\' camSaveName{i}]);
    imageFileNames = images.ImageLocation;

    % Detect Calibration Pattern for the purpose of identifying bad images
    detector = vision.calibration.monocular.CheckerboardDetector();
    [imagePoints{i}, imagesUsed{i}] = detectPatternPoints(detector, imageFileNames, "PartialDetections", false);

    for j=1:images.Count
        if imagesUsed{i}(j) == 0 % Add the index of any bad images to an array
            bad = [bad j];
        end
    end
end
bad = unique(bad);%Remove any duplicate values
bad = sort(bad); %Put values in ascending order


%Parse through Correlation Images and delete any images at indices not used
%for image point detection
for i=1:2
    for j=bad
        delete(['ExtrinsicImages\' camSaveName{i} '\ExImage' num2str(j,'%03.f') '.jpg']);
        delete(['IntrinsicImages\' camSaveName{i} '\InImage' num2str(j,'%03.f') '.jpg']);
    end
end

% Delete corresponding data for removed images
for j=length(bad):-1:1
    H_b2w_ex(bad(j)) = [];
    H_o2w_ex(bad(j)) = [];
    H_rb2w_ex(bad(j)) = [];
    H_f2w_ex(bad(j)) = [];
    urJointAngles_ex(bad(j)) = [];
    H_b2w_in(bad(j)) = [];
    H_o2w_in(bad(j)) = [];
    H_rb2w_in(bad(j)) = [];
    H_f2w_in(bad(j)) = [];
    urJointAngles_in(bad(j)) = [];
end

disp('Bad data removed:');
disp(bad);
%% Save data
for i=1:numel(H_f2w_ex)
    H_f2w_ex{i}(1:3,4) = H_f2w_ex{i}(1:3,4)*1000; % Convert from m to mm
    H_rb2w_ex{i}(1:3,4) = H_rb2w_ex{i}(1:3,4)*1000;   % Convert from m to mm
    H_o2w_ex{i}(1:3,4) = H_o2w_ex{i}(1:3,4)*1000; % Convert from m to mm
    H_b2w_ex{i}(1:3,4) = H_b2w_ex{i}(1:3,4)*1000;   % Convert from m to mm
    H_f2w_in{i}(1:3,4) = H_f2w_in{i}(1:3,4)*1000; % Convert from m to mm
    H_rb2w_in{i}(1:3,4) = H_rb2w_in{i}(1:3,4)*1000;   % Convert from m to mm
    H_o2w_in{i}(1:3,4) = H_o2w_in{i}(1:3,4)*1000; % Convert from m to mm
    H_b2w_in{i}(1:3,4) = H_b2w_in{i}(1:3,4)*1000;   % Convert from m to mm
end
for i=1:numel(H_rb2w_seg)
    H_rb2w_seg{i}(1:3,4) = H_rb2w_seg{i}(1:3,4)*1000;   % Convert from m to mm
    H_o2w_seg{i}(1:3,4) = H_o2w_seg{i}(1:3,4)*1000; % Convert from m to mm
    H_b2w_seg{i}(1:3,4) = H_b2w_seg{i}(1:3,4)*1000;   % Convert from m to mm
end
% Saving the same data in different folders for clarity
save('ExtrinsicDataMatrices\H_f2w_ex.mat',"H_f2w_ex");
save('ExtrinsicDataMatrices\H_rb2w_ex.mat',"H_rb2w_ex");
save('ExtrinsicDataMatrices\H_o2w_ex.mat',"H_o2w_ex");
save('ExtrinsicDataMatrices\H_b2w_ex.mat',"H_b2w_ex");
save('ExtrinsicDataMatrices\urJointAngles_ex.mat',"urJointAngles_ex");
save('IntrinsicDataMatrices\H_f2w_in.mat',"H_f2w_in");
save('IntrinsicDataMatrices\H_rb2w_in.mat',"H_rb2w_in");
save('IntrinsicDataMatrices\H_o2w_in.mat',"H_o2w_in");
save('IntrinsicDataMatrices\H_b2w_in.mat',"H_b2w_in");
save('IntrinsicDataMatrices\urJointAngles_in.mat',"urJointAngles_in");
save('SegmentationDataMatrices\H_rb2w_seg.mat',"H_rb2w_seg");
save('SegmentationDataMatrices\H_o2w_seg.mat',"H_o2w_seg");
save('SegmentationDataMatrices\H_b2w_seg.mat',"H_b2w_seg");
save('SegmentationDataMatrices\urJointAngles_seg.mat',"urJointAngles_seg");
save('SegmentationDataMatrices\p_detected.mat', "p_detected");

%% Generate Extrinsics per Image
% Generate points again after removing all bad image pairs
for i=1:2
    % Find image folder
    images = imageSet(['ExtrinsicImages\' camSaveName{i}]);
    imageFileNames = images.ImageLocation;

    % Detect Calibration Pattern (Long step)
    detector = vision.calibration.monocular.CheckerboardDetector();
    imagePoints{i} = detectPatternPoints(detector, imageFileNames,"PartialDetections", false);

    % Generate extrinsic H_g2c
    for n=1:length(imageFileNames)
        [RMatrix, TVector] = extrinsics(undistortPoints(imagePoints{i}(:,:,n),params{i}), params{i}.WorldPoints, params{i});
        bubble = [RMatrix', TVector'];
        bubble(4,1:4) = [0 0 0 1];
        H_g2c_ex{i,n} = bubble;
    end
end

% Define transform relating c1 to c2 for each image
for n=1:length(imageFileNames)
    H_c12c2_ex{n} = H_g2c_ex{2,n} * invSE(H_g2c_ex{1,n});
end

for i=1:numel(H_g2c_ex(1,:))
    H_g2c1_bar{i} = invSE(H_c12c2_mean) * H_g2c_ex{2,i};
    H_c12c1_bar{i} = H_g2c1_bar{i}*invSE(H_g2c_ex{1,i});
end

%Save the translation data for all transforms
for i = 1:numel(H_c12c1_bar)
    xpos(i) = H_c12c1_bar{i}(1,4);
    ypos(i) = H_c12c1_bar{i}(2,4);
    zpos(i) = H_c12c1_bar{i}(3,4);
end

%Remove outliers
[xpos_new,logicX] = rmoutliers(xpos);
[ypos_new,logicY] = rmoutliers(ypos);
[zpos_new,logicZ] = rmoutliers(zpos);

%Save all indicies at which an outlier was removed 
bad = [];
for i = 1:numel(H_c12c1_bar)
    if logicX(i) == 1 || logicY(i) == 1 || logicZ(i) == 1
        bad = [bad i];
    end
end
bad = unique(bad);

%Remove outlier data
for i=length(bad):-1:1
    H_c12c1_bar(bad(i)) = [];
end

% Save data 
save('ExtrinsicDataMatrices\H_c12c1_bar.mat',"H_c12c1_bar");
%% Save Extrinsic Matrices
tempH_g2c(:,:) = H_g2c_ex;
for i=1:2
    H_g2c_ex = tempH_g2c(i,:);
    save(['ExtrinsicDataMatrices\' camSaveName{i} '\H_g2c_ex.mat'],"H_g2c_ex");
end
H_g2c_ex = tempH_g2c;

save('ExtrinsicDataMatrices\H_c12c2_ex.mat',"H_c12c2_ex");
%% Generate Intrinsics
for i=1:2
    for n=1:length(H_b2w_in)
        H_g2c{i,n} = H_b2c{i} * invSE(H_b2w_in{n}) * H_f2w_in{n} * invSE(H_f2g{i});
    end
    A_c2m_in(i,:) = estimateIntrinsicMatrix(boardSize, squareSizeMM, ['IntrinsicImages\' camSaveName{i}], H_g2c(i,:), false);
end
%% Save Intrinsics
save('IntrinsicDataMatrices\A_c2m_in.mat',"A_c2m_in");
%% Collect and Save Hand Selected Pixel Coordinates
for i=1:2
    for k=1:size(q,1)
        p = selectPixelInImage(['SegmentationImages\' camSaveName{i} '\SegImage' num2str(k,'%03.f') '.jpg']);
        p_x_truth(i,k) = p(1);
        p_y_truth(i,k) = p(2);
    end
end
save('SegmentationDataMatrices\p_x_truth.mat', "p_x_truth");
save('SegmentationDataMatrices\p_y_truth.mat', "p_y_truth");
%% Remove Outliers From Segmentation Data
for i=1:2
    %Calculate difference between cv detected and hand selected pixel
    %location
    for k=1:size(q,1)
        deltaX{i}(k) = p_x_truth(i,k) - p_detected{i,k}(1);
        deltaY{i}(k) = p_y_truth(i,k) - p_detected{i,k}(2);
    end

    % Remove Outliers Idependently
    [dx{i},logicdx{i}] = rmoutliers(deltaX{i});
    [dy{i},logicdy{i}] = rmoutliers(deltaY{i});

    %Save all indicies at which an outlier was removed
    bad = [];
    for j = 1:size(q,1)
        if logicdx{i}(j) == 1 || logicdy{i}(j) == 1
            bad = [bad j];
        end
    end
    bad = unique(bad);

    %Remove outlier data together - if an image was an outlier in just one
    %dimension, remove the corresponding data in the other 
    for n=length(bad):-1:1
        deltaX{i}(bad(n)) = [];
        deltaY{i}(bad(n)) = [];
    end
end
%% Calculate Mean and Covariance For Segmentation
for i=1:2
    p_x_mean(i) = mean(deltaX{i}(:));
    p_y_mean(i) = mean(deltaY{i}(:));
    p_cov{i} = cov(deltaX{i}(:).', deltaY{i}(:).');
end
%% Save Segmentation Data
save('SegmentationDataMatrices\p_x_mean.mat', "p_x_mean");
save('SegmentationDataMatrices\p_y_mean.mat', "p_y_mean");
save('SegmentationDataMatrices\p_cov.mat', "p_cov");