%% Extrinsic Data Collection
% This script is used to quantify the uncertainty in estimating the
% position of camera 1 relative to camera 2
% 
% N number of checkerboard images are collected from each camera, and computer
% vision is used to estimate the points on the checkerboard grid pixel location. 
% extrinsics() is used to estimate the checkerboard grid relative to the
% camera for each image, and correspondance allows forthe calculation of
% camera 1 relative to camera 2 for each image. 

% Result is a cell array H_c12c2_ex containing a rigid body transform
% relating camera 1 to camera 2 for each image. 

% C. A. Civetta, M. Kutzer, 28Jun2024, USNA

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
v = initOptitrack({'Bino', 'BinocularCameras'}, {'RedBall', 'RedBall'}, {'Check', 'NiceCheckerboard'}, {'Base', 'UR10Base'});
ura = URx_ROS('ura');
%% Initiate Cameras
SCRIPT_initializeBinocularCameras;
%% Load variables
for i = 1:2
    device = imaqhwinfo('winvideo',cam(i).DeviceID);
    camFormat{i} = [num2str(resy(i)) 'x' num2str(resx(i))];
    camName{i} = device.DeviceName;
    camSaveName{i} = [camFormat{i} '_' camName{i} index];
    mkdir(['ExtrinsicImages\' camSaveName{i}])
    mkdir(['ExtrinsicDataMatrices\' camSaveName{i}])

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

    % Save Images
    for i=1:2
        imwrite(img{i}, ['ExtrinsicImages\' camSaveName{i} '\ExImage' num2str(k,'%03.f') '.jpg'])
    end
    fprintf("Saved Image %d of %d \n",k, numImages)

    % Save MoCap + UR Data
    H_b2w_ex{k} = v.Bino.pose; %Saving Bino Fiducial in m
    H_o2w_ex{k} = v.Base.pose; %Saving base frame fidicual in m
    H_rb2w_ex{k} = v.RedBall.pose; %Saving base frame fidicual in m
    H_f2w_ex{k} = v.Check.pose;
    urJointAngles_ex{k} = ura.jointAngles;

end
disp("End of Image Gathering")
close(binocularDisplay);

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
    end
end

% Delete corresponding data for removed images
for j=length(bad):-1:1
    H_b2w_ex(bad(j)) = [];
    H_o2w_ex(bad(j)) = [];
    H_rb2w_ex(bad(j)) = [];
    H_f2w_ex(bad(j)) = [];
    urJointAngles_ex(bad(j)) = [];
end

disp('Bad data removed:');
disp(bad);
%% Save data
for i=1:numel(H_f2w_ex)
    H_f2w_ex{i}(1:3,4) = H_f2w_ex{i}(1:3,4)*1000; % Convert from m to mm
    H_rb2w_ex{i}(1:3,4) = H_rb2w_ex{i}(1:3,4)*1000;   % Convert from m to mm
    H_o2w_ex{i}(1:3,4) = H_o2w_ex{i}(1:3,4)*1000; % Convert from m to mm
    H_b2w_ex{i}(1:3,4) = H_b2w_ex{i}(1:3,4)*1000;   % Convert from m to mm
end

save('ExtrinsicDataMatrices\H_f2w_ex.mat',"H_f2w_ex");
save('ExtrinsicDataMatrices\H_rb2w_ex.mat',"H_rb2w_ex");
save('ExtrinsicDataMatrices\H_o2w_ex.mat',"H_o2w_ex");
save('ExtrinsicDataMatrices\H_b2w_ex.mat',"H_b2w_ex");
save('ExtrinsicDataMatrices\urJointAngles_ex.mat',"urJointAngles_ex");
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

% Define transform relating c1 ot c2 for each image
for n=1:length(imageFileNames)
    H_c12c2_ex{n} = H_g2c_ex{2,n} * invSE(H_g2c_ex{1,n});
end
%% Save Matrices
tempH_g2c(:,:) = H_g2c_ex;
for i=1:2
    H_g2c_ex = tempH_g2c(i,:);
    save(['ExtrinsicDataMatrices\' camSaveName{i} '\H_g2c_ex.mat'],"H_g2c_ex");
end
H_g2c_ex = tempH_g2c;

save('ExtrinsicDataMatrices\H_c12c2_ex.mat',"H_c12c2_ex");