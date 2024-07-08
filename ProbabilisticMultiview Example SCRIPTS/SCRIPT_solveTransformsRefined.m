%% SCRIPT_solveTransformsRefined
% This script runs through the full refinement process for the system. 
% After running this script, both camera's parameters are refined, as well
% as H_b2c, H_f2g, H_c12c2_MoCap, and H_c12c2_mean. Data is saved with the
% ending '_refined'. 
%
% Note that other scripts will by default load saved variables with the original
% names, not '_refined'. When working with refined data, ensure you are
% loading the correct files. 
%
%   C. A. Civetta, M. Kutzer, 02Jul2024, USNA

clear all; 
close all; 
clc;

%% Initiate Camera 
index = '1';
SCRIPT_initializeBinocularCameras;
%% Refine Camera Parameters 
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

params{2} = refineCamera(['CorrelationImages\' camSaveName{2}], params{1}, params{2});
params{1} = refineCamera(['CorrelationImages\' camSaveName{1}], params{2}, params{1});

%% Generate Extrinsics
for i=1:2
    H_g2c(i,:) = generateExtrinsics(params{i});
end
[H_c12c2_mean, ~, ~] = extrinsicTranslationStats(H_g2c(1,:), H_g2c(2,:));
%% Solve Fixed Transforms 

load("SavedMatrices\H_f2w.mat");
load("SavedMatrices\H_b2w.mat");

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

%% Save refined values
save('SavedMatrices\H_c12c2_MoCaP_refined', "H_c12c2_MoCap");
save('SavedMatrices\H_c12c2_mean_refined', "H_c12c2_mean");
save('SavedMatrices\H_b2c_refined', "H_b2c");
save('SavedMatrices\H_f2g_refined', "H_f2g");

tempParams(:,:) = params;
for i=1:2
    params = tempParams{i};
    save(['SavedMatrices\' camSaveName{i} '\CameraParams_refined.mat'],"params");
end
params = tempParams;