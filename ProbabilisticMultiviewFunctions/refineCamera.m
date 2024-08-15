function refinedParams = refineCamera(imagesPath, params1, params2)
%REFINECAMERA - This function refines one camera in a binocular pair based
% on the calculated mean transform relating camera 1 to camera 2
%
%  The function yields a refined cameraParameters object for one camera
%  based on corrresponding data from both cameras.
%
% Input(s): 
%   imagesPath - a string containing the file path with callibration images
%                for the camera associated with params2 - the camera being refined
%   params1    - cameraParameters object for the unchanging camera
%   params2    - cameraParameters object for the camera being refined
%
% Output(s): 
%   refinedParams - cameraParameters object containing refined parameters
%                   for camera 2
%
% C. A. Civetta 16OCT2023

%% Save params as cell array
params{1} = params1;
params{2} = params2;
%% Detect image points
images = imageSet(imagesPath);
imageFileNames = images.ImageLocation;
detector = vision.calibration.monocular.CheckerboardDetector();
[imagePoints, ~] = detectPatternPoints(detector, imageFileNames, "PartialDetections", false);

%% Define extrsinsic matrices

for i=1:2
    H_g2c(i,:) = generateExtrinsics(params{i});
end

[H_cA2cB, ~, ~] = extrinsicTranslationStats(H_g2c(1,:), H_g2c(2,:));
%% Refine Camera Intrinsics
%Define World Points
p_g = params{1}.WorldPoints.'; %% World points in mm
p_g(3,:) = 0;
p_g(4,:) = 1;

%Define matrices containing the necessary x and y values 
%from the detected points in each image
p_m2_1(1,:,:) = imagePoints(:,1,:);
p_m2_1_all = p_m2_1(:)';
p_m2_2(1,:,:) = imagePoints(:,2,:);
p_m2_2_all = p_m2_2(:)';
p_c2_all_1 = [];
p_c2_all_2 = [];
for n=1:params{1}.NumPatterns
    %Calculate points relative to camera 2
    p_c2_tilde{n} = H_cA2cB * H_g2c{1,n} * p_g;  % Grid points relative to camera 2 (B) using camera A to B extrinsics and camera A extrinsics
    p_c2{n} = p_c2_tilde{n}./p_c2_tilde{n}(3,:); % Scaled grid points relative to camera 2 (B)
    p_c2_all_1 = [p_c2_all_1 p_c2{n}(1:3,:)];    % rows 1-3
    p_c2_all_2 = [p_c2_all_2 p_c2{n}(2:3,:)];    % rows 2-3 only
end

%Calculate the first row of the intrinsic matric
A_row1 = p_m2_1_all * pinv(p_c2_all_1);
%Calculate the last 2 values in the second row of the intrinsic matrix -
%this is done to preserve the 0
A_row2 = p_m2_2_all * pinv(p_c2_all_2);
%Combine all values for a new Intrinsic Matrix
A_c2m_new = [];
A_c2m_new(1,:) = A_row1;
A_c2m_new(2,1) = 0;
A_c2m_new(2,2:3) = A_row2;
A_c2m_new(3,:) = [0 0 1];

%Convert params to struct to allow for changes
holder = toStruct(params{2});
holder = rmfield(holder, 'K');
holder.IntrinsicMatrix = A_c2m_new.';

%Convert struct back to cameraParameters with new Intrinsics
params{2} = cameraParameters(holder);

%% Refine Camera Extrinsics
%Estimate extrinsics based on newly refined intrinsics, image points, and
%world points
for i=1:params{2}.NumPatterns
    [refinedRMatrix(:,:,i), refinedT(i,:)] = extrinsics(undistortPoints(imagePoints(:,:,i),params{2}), params{2}.WorldPoints, params{2});
    refinedRVector(i,:) = rotationMatrixToVector(refinedRMatrix(:,:,i)); %Convert rotation matrix to vector - necessary format for wrapping into struct
end
holder.RotationVectors = refinedRVector;
holder.TranslationVectors = refinedT;

% Convert struct back to cameraParameters and save refined data replacing
% old data
refinedParams = cameraParameters(holder);
end