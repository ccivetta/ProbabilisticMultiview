%% Segmentation Data Collection
% This script is used to quantify the uncertainty in segmentation for a red
% ball. 
% 
% N number of images are collected from each camera, and computer
% vision (detectBall() in this script) is used to estimate the object's
% pixel location. 
%
% Images are then displayed on the screen so the user can zoom and
% accurately hand select the true pixel location of the object 
%
% Satistical outliers, defined as data greater than three scaled medial 
% absolute deviations from the median, are removed 
%
% Mean and covariance of the resulting data are computed and saved
%
% C. A. Civetta, M. Kutzer, 28Jun2024, USNA

close all
clear all
clc


% The index will be added to the end of every save name. Change this value
% if you have multiple trials of the same data
index = '1';

% Number of correlation images to take
numImages = 35;
%% Initiate OptiTrack + UR
v = initOptitrack({'Bino', 'BinocularCameras'}, {'RedBall', 'RedBall'}, {'Base', 'UR10Base'});
%ura = URx_ROS('ura');
%% Initiate Camera
SCRIPT_initializeBinocularCameras;
%% Load necessary variables
for i = 1:2
    device = imaqhwinfo('winvideo',cam(i).DeviceID);
    camFormat{i} = [num2str(resy(i)) 'x' num2str(resx(i))];
    camName{i} = device.DeviceName;
    camSaveName{i} = [camFormat{i} '_' camName{i} index];
    mkdir(['SegmentationImages\' camSaveName{i}])
    mkdir(['SegmentationDataMatrices\' camSaveName{i}])

    load(['SavedMatrices\' camSaveName{i} '\CameraParams.mat'])
    bubble{i} = params;
end
clear params
params = bubble;
clear bubble
%% Begin Capturing Data
disp("Begin Taking Images/MoCap Data")
binocularDisplay = figure;
for k=1:numImages
    %Loop to display both images until a key is pressed, at which point the
    %loop breaks and the image data is saved for use in calibration
    img = binocularCaptureDisplay(prv, binocularDisplay);

    %Save Image
    for i=1:2
        imwrite(img{i}, ['SegmentationImages\' camSaveName{i} '\SegImage' num2str(k,'%03.f') '.jpg'])
        undistorted_img = undistortImage(img{i},params{i});
        [p_detected{i,k}] = detectBall(undistorted_img);
    end
    fprintf("Saved Image %d of %d \n",k, numImages)

    % Save MoCap + UR Data
    H_b2w_seg{k} = v.Bino.pose; %Saving Bino Fiducial in m
    H_o2w_seg{k} = v.Base.pose; %Saving base frame fidicual in m
    H_rb2w_seg{k} = v.RedBall.pose; %Saving base frame fidicual in m
    %urJointAngles_seg{k} = ura.jointAngles;

end
disp("End of Image Gathering")
close(binocularDisplay);
%% Save Data
for i=1:numel(H_rb2w_seg)
    H_rb2w_seg{i}(1:3,4) = H_rb2w_seg{i}(1:3,4)*1000;   % Convert from m to mm
    H_o2w_seg{i}(1:3,4) = H_o2w_seg{i}(1:3,4)*1000; % Convert from m to mm
    H_b2w_seg{i}(1:3,4) = H_b2w_seg{i}(1:3,4)*1000;   % Convert from m to mm
end

save('SegmentationDataMatrices\H_rb2w_seg.mat',"H_rb2w_seg");
save('SegmentationDataMatrices\H_o2w_seg.mat',"H_o2w_seg");
save('SegmentationDataMatrices\H_b2w_seg.mat',"H_b2w_seg");
%save('SegmentationDataMatrices\urJointAngles_seg.mat',"urJointAngles_seg");
save('SegmentationDataMatrices\p_detected.mat', "p_detected");
%% Collect and Save Hand Selected Pixel Coordinates
for i=1:2
    for k=1:numImages
        p = selectPixelInImage(['SegmentationImages\' camSaveName{i} '\SegImage' num2str(k,'%03.f') '.jpg']);
        p_x_truth(i,k) = p(1);
        p_y_truth(i,k) = p(2);
    end
end
save('SegmentationDataMatrices\p_x_truth.mat', "p_x_truth");
save('SegmentationDataMatrices\p_y_truth.mat', "p_y_truth");
%% Remove Outliers
for i=1:2
    %Calculate difference between cv detected and hand selected pixel
    %location
    for k=1:numImages
        deltaX{i}(k) = p_x_truth(i,k) - p_detected{i,k}(1);
        deltaY{i}(k) = p_y_truth(i,k) - p_detected{i,k}(2);
    end

    % Remove Outliers Idependently
    [dx{i},logicdx{i}] = rmoutliers(deltaX{i});
    [dy{i},logicdy{i}] = rmoutliers(deltaY{i});

    %Save all indicies at which an outlier was removed
    bad = [];
    for j = 1:numImages
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
%% Calculate Mean and Covariance 
for i=1:2
    p_x_mean(i) = mean(deltaX{i}(:));
    p_y_mean(i) = mean(deltaY{i}(:));
    p_cov{i} = cov(deltaX{i}(:).', deltaY{i}(:).');
end
%% Save 
save('SegmentationDataMatrices\p_x_mean.mat', "p_x_mean");
save('SegmentationDataMatrices\p_y_mean.mat', "p_y_mean");
save('SegmentationDataMatrices\p_cov.mat', "p_cov");