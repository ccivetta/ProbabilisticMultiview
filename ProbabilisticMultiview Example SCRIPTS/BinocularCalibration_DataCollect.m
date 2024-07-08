%% Binocular Calibration
% Code intended to calibrate binocular cameras and find the H transform between the Motion Capture checkerboard
% fiducial (f) and the checkerboard/grid (g). This transform will be used to find the
% transform of the cameras (c1 & c2) relative to the Binocular Camera rigid body (b)

%Use images that capture the checkerboard fiducial in the frame of both
%cameras at the same time in order to yield extrinsics relating the
%position of the cameras relative to eachother. 

%This script is intended to calibrate both cameras, collect n extrinsic
%matrices relating c1 to c2, and 1 instrinsic matrix with uncertainty per
%camera

% Results: 
%   H_f2w         - 1xN cell array of checkerboard MoCap fiducial relative
%                   to MoCap world corresponding with images 
%   H_b2w         - 1xN cell array of binocular cameras MoCap fiducial
%                   relative to MoCap world corresponding with images 
%   H_o2w         - 1xN cell array of UR base MoCap fiducial
%                   relative to MoCap world corresponding with images 
%   urJointAngles - 1xN cell array of UR joint angles corresponding with
%                   images
%   params        - cameraParameters object for each camera saved under the
%                   corresponding camera's name 
%   H_g2c         - 1xN cell array of checkerboard grid relative to each
%                   camera corresponding with images - one saved for each camera under the
%                   camera's name 
%   H_c12c2       - 1xN cell array of camera 1 relative to camera 2
%                   corresponding to each image 
%   H_c12c2_mean  - the meanSE transform from the data set of camera 1
%                   relative to camera 2 -> this mean is calculated after
%                   removing outlier data by translation only. An outlier
%                   is defined as any transform with either x, y, or z translation
%                   greater than three scaled medial absolute deviations from the median
%   H_f2g         - 1x2 cell array of solved transforms for the MoCap
%                   checkerboard fiducial relative to the checkerboard grid
%                   (two values because data from two cameras are used, but
%                   they would be identical in a perfect calibration) 
%   H_b2c         - 1x2 cell array containing the MoCap binocular camera
%                   fiducial fram relative to each camera frame 
%   H_c12c2_MoCap - a transform relating camera 1 to camera 2 calculated
%                   using the reltionships between each camera and the motions capture
%                   binocular fiducial - used to be called H_c12c2_truth
    

% Legend:
% g = checkerboard matrix (grid)
% rb - red ball 
% f = checkerboard motion capture fiducial
% c = camera
% c1 = camera 1
% c2 = camera 2
% w = motion capture world frame 
% o = UR10 base frame fiducial 
% b = binocular cameras fiducial 

% Authors: Charles Doherty, Chris Civetta
% created: 24Aug2023
% edited: 17Oct2023

close all
clear all
clc

% The index will be added to the end of every save name. Change this value
% if you have multiple trials of the same data
index = '1';

% Square size of the checkerboard you are using
squareSizeMM = 30;

% Number of correlation images to take
numImages = 30;
%% Initiate OptiTrack + UR
v = initOptitrack({'Bino', 'BinocularCameras'}, {'RedBall', 'RedBall'}, {'Check', 'NiceCheckerboard'});
%ura = URx_ROS('ura');
%% Initiate Camera
SCRIPT_initializeBinocularCameras
for i=1:2
    device = imaqhwinfo('winvideo',cam(i).DeviceID);
    camFormat{i} = [num2str(resy(i)) 'x' num2str(resx(i))];
    camName{i} = device.DeviceName;
    camSaveName{i} = [camFormat{i} '_' camName{i} index];
    mkdir(['CorrelationImages\' camSaveName{i}])
    mkdir(['SavedMatrices\' camSaveName{i}])
end

%% Gather images/mocap data
disp("Begin Taking Images/MoCap Data")
binocularDisplay = figure;
for k=1:numImages
    %Loop to display both images until a key is pressed, at which point the
    %loop breaks and the image data is saved for use in calibration
    img = binocularCaptureDisplay(prv, binocularDisplay);

    % Save image
    for i=1:2
        imwrite(img{i}, ['CorrelationImages\' camSaveName{i} '\CorImage' num2str(k,'%03.f') '.jpg'])
    end
    fprintf("Saved Image %d of %d \n",k, numImages)

    % Save MoCap + UR Data
    H_f2w{k} = v.Check.pose; %Saving Checkerboard Fiducial in m
    H_b2w{k} = v.Bino.pose; %Saving Bino Fiducial in m
    H_o2w{k} = v.Base.pose; %Saving base frame fidicual in m
    %urJointAngles{k} = ura.jointAngles;
end
disp("End of Image Gathering")
close(binocularDisplay);

%% Remove bad data
%Check all images for imagepoints and save the image number for any images
%that do not contain detectable points. These images and all corresponding
%data will be removed before further processing

bad = [];

for i=1:2
    images = imageSet(['CorrelationImages\' camSaveName{i}]);
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
        delete(['CorrelationImages\' camSaveName{i} '\CorImage' num2str(j,'%03.f') '.jpg']);
    end
end


%Remove any MoCap data corresponding to removed images and save
for j=length(bad):-1:1
    H_b2w(bad(j)) = [];
    H_f2w(bad(j)) = [];
    H_o2w(bad(j)) = [];
    %urJointAngles(bad(j)) = [];
end

%Save adjusted data -> this is saves in m 
save('SavedMatrices\H_f2w.mat',"H_f2w");
save('SavedMatrices\H_b2w.mat',"H_b2w");
save('SavedMatrices\H_o2w.mat',"H_o2w");
%save('SavedMatrices\urJointAngles.mat',"urJointAngles");
disp('Bad data removed');
disp(bad);

%% Checkerboard pose finder
% Generate points again after removing all bad image pairs 
% Find image folder
for i=1:2
    images = imageSet(['CorrelationImages\' camSaveName{i}]);
    imageFileNames = images.ImageLocation;

    % Detect Calibration Pattern (Long step)
    detector = vision.calibration.monocular.CheckerboardDetector();
    imagePoints{i} = detectPatternPoints(detector, imageFileNames);
    disp("Checkerboards Detected")

    % Generate world coordinates of the squares
    worldPoints{i} = generateWorldPoints(detector, 'SquareSize', squareSizeMM);
    disp("World Coordinates Generated")

    % Calibrate the camera
    I = readimage(images, 1);
    imageSize = [size(I,1), size(I,2)];
    [params{i}, imagesUsed{i}, estimationErrors{i}] = estimateCameraParameters(imagePoints{i}, worldPoints{i}, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], 'ImageSize', imageSize);
end 
disp("Cameras Calibrated")

%% Visualize calibration results
for i=1:2
    figure()
    title(['Reprojection Error ' camSaveName{i}]);
    subplot(1,2,1)
    showReprojectionErrors(params{i});
    subplot(1,2,2)
    showReprojectionErrors(params{i},'ScatterPlot');
    figure()
    title(['Extrinsics ' camSaveName{i}]);
    showExtrinsics(params{i},'PatternCentric');
end

%% Save params fo reach camera
tempParams(:,:) = params;
for i=1:2
    params = tempParams{i};
    save(['SavedMatrices\' camSaveName{i} '\CameraParams.mat'],"params");
end
params = tempParams;
%% Get extrinsics matrices + Calculate H_c12c2
for i=1:2
    H_g2c(i,:) = generateExtrinsics(params{i});
end

% Calculate camera 1 relative to camera 2
for p=1:length(params{1}.RotationMatrices(1,1,:))
    H_c12c2{p} = H_g2c{2,p} * invSE(H_g2c{1,p}); 
end

%Calculate average of the dataset
[H_c12c2_mean, ~, ~] = extrinsicTranslationStats(H_g2c(1,:), H_g2c(2,:));

%save
tempH_g2c(:,:) = H_g2c;
for i=1:2
    H_g2c = tempH_g2c(i,:);
    save(['SavedMatrices\' camSaveName{i} '\H_g2c.mat'],"H_g2c");
end
H_g2c = tempH_g2c;
save('SavedMatrices\H_c12c2.mat',"H_c12c2");
save('SavedMatrices\H_c12c2_mean.mat', "H_c12c2_mean");
%% Solve Correspondence Problem
% solving for H_b2c b=b a=c   x=m y=f       H_x2a = H_g2c      H_y2b =
% H_f2b

%Convert to mm
for j = 1:length(H_f2w)
    H_f2w{j}(1:3,4) = H_f2w{j}(1:3,4)*1000;
    H_b2w{j}(1:3,4) = H_b2w{j}(1:3,4)*1000;
end

for i = 1:length(H_f2w)
    H_f2b{i} = invSE(H_b2w{i}) * H_f2w{i};
end

for i=1:2
    [H_b2c{i},H_f2g{i}] = solveFixedTransforms(H_g2c(i,:),H_f2b);
end

H_c12c2_MoCap = H_b2c{2} * invSE(H_b2c{1});


disp("Found Camera relative to MoCap camera fiducial")


%Save found matrices in mm
save('SavedMatrices\H_f2g.mat',"H_f2g");
save('SavedMatrices\H_b2c.mat',"H_b2c");
save('SavedMatrices\H_c12c2_MoCaP', "H_c12c2_MoCap"); %Old name was "truth" 

