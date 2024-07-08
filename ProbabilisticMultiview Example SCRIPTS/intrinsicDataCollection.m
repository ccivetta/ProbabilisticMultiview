%% Intrinsic Data Collection 
% This script is used to quantify the uncertainty in estimating camera
% intrinsics
% 
% N number of checkerboard images are collected from each camera, and computer
% vision is used to estimate the points on the checkerboard grid pixel location. 
% Corresponding MoCap data is collected for the position of the cameras and
% checkerboard

% Using MoCap data and known MoCap relationships, the position of the
% checkerboard grid relating to the camera is used to estimate camera
% intrinsics for each image, yielding a different estimate per image. 

% C. A. Civetta, M. Kutzer, 28Jun2024, USNA

close all
clear all
clc


% The index will be added to the end of every save name. Change this value
% if you have multiple trials of the same data
index = '1';

% Square size of the checkerboard you are using
squareSizeMM = 30;

% Dimensions of the checkerboard grid in [h,w]
boardSize = [8,9];

% Number of correlation images to take
numImages = 40;
%% Initiate OptiTrack + UR
v = initOptitrack({'Bino', 'BinocularCameras'}, {'RedBall', 'RedBall'}, {'Check', 'NiceCheckerboard'}, {'Base', 'UR10Base'});
%ura = URx_ROS('ura');
%% Initiate Camera
SCRIPT_initializeBinocularCameras;
%% Load variables 
load('SavedMatrices\H_b2c.mat');
load('SavedMatrices\H_f2g.mat');

for i = 1:2
    device = imaqhwinfo('winvideo',cam(i).DeviceID);
    camFormat{i} = [num2str(resy(i)) 'x' num2str(resx(i))];
    camName{i} = device.DeviceName;
    camSaveName{i} = [camFormat{i} '_' camName{i} index];
    mkdir(['IntrinsicImages\' camSaveName{i}])
    mkdir(['IntrinsicDataMatrices\' camSaveName{i}])

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

    %Save Images
    for i=1:2
        imwrite(img{i}, ['IntrinsicImages\' camSaveName{i} '\InImage' num2str(k,'%03.f') '.jpg'])
    end
    fprintf("Saved Image %d of %d \n",k, numImages)

    % Save MoCap + UR Data
    H_b2w_in{k} = v.Bino.pose; %Saving Bino Fiducial in m
    H_o2w_in{k} = v.Base.pose; %Saving base frame fidicual in m
    H_rb2w_in{k} = v.RedBall.pose; %Saving base frame fidicual in m
    H_f2w_in{k} = v.Check.pose;
    urJointAngles_in{k} = ura.jointAngles;

end
disp("End of Image Gathering")
close(binocularDisplay);

%% Remove bad data
%Check all images for imagepoints and save the image number for any images
%that do not contain detectable points. These images and all corresponding
%data will be removed before further processing

bad = [];

for i=1:2
    images = imageSet(['IntrinsicImages\' camSaveName{i}]);
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
        delete(['IntrinsicImages\' camSaveName{i} '\InImage' num2str(j,'%03.f') '.jpg']);
    end
end

% Remove corresponding data from discarded images
for j=length(bad):-1:1
    H_b2w_in(bad(j)) = [];
    H_o2w_in(bad(j)) = [];
    H_rb2w_in(bad(j)) = [];
    H_f2w_in(bad(j)) = [];
    urJointAngles_in(bad(j)) = [];
end

disp('Bad data removed:');
disp(bad);
%% Save Data
for i=1:numel(H_f2w_in)
    H_f2w_in{i}(1:3,4) = H_f2w_in{i}(1:3,4)*1000; % Convert from m to mm
    H_rb2w_in{i}(1:3,4) = H_rb2w_in{i}(1:3,4)*1000;   % Convert from m to mm
    H_o2w_in{i}(1:3,4) = H_o2w_in{i}(1:3,4)*1000; % Convert from m to mm
    H_b2w_in{i}(1:3,4) = H_b2w_in{i}(1:3,4)*1000;   % Convert from m to mm
end

save('IntrinsicDataMatrices\H_f2w_in.mat',"H_f2w_in");
save('IntrinsicDataMatrices\H_rb2w_in.mat',"H_rb2w_in");
save('IntrinsicDataMatrices\H_o2w_in.mat',"H_o2w_in");
save('IntrinsicDataMatrices\H_b2w_in.mat',"H_b2w_in");
save('IntrinsicDataMatrices\urJointAngles_in.mat',"urJointAngles_in");
%% Generate Intrinsics
for i=1:2
    for n=1:length(H_b2w_in)
        H_g2c{i,n} = H_b2c{i} * invSE(H_b2w_in{n}) * H_f2w_in{n} * invSE(H_f2g{i});
    end
    A_c2m_in_tst(i,:) = estimateIntrinsicMatrix(boardSize, squareSizeMM, ['IntrinsicImages\' camSaveName{i}], H_g2c(i,:));
end
%% Save
save('IntrinsicDataMatrices\A_c2m_in.mat',"A_c2m_in");