%% SCRIPT_RedefineExtrinsicUncertainty
% This script is used to evaluate extrinsic data in the form H_c12c1_bar,
% which is a transform relating camera 1 to the mean position of camera 1.
% This allows us to define the data as a data set of camera 1's deviation
% from its the mean.
%
%   C. A. Civetta, M. Kutzer, 02Jul2024, USNA

clear all
close all
clc

%% Define Matrices
load('G:\Shared drives\VIPER Lab\Code, Trident Civetta\SavedMatrices\H_c12c2_mean.mat')
load('G:\Shared drives\VIPER Lab\Code, Trident Civetta\ExtrinsicDataMatrices\H_c12c2_ex.mat')
load('G:\Shared drives\VIPER Lab\Code, Trident Civetta\ExtrinsicDataMatrices\960x1280_DFK 23UM0211\H_g2c1_ex.mat')
load('G:\Shared drives\VIPER Lab\Code, Trident Civetta\ExtrinsicDataMatrices\480x640_DFK 23U6181\H_g2c2_ex.mat')

for i=1:numel(H_g2c2_ex)
    H_g2c1_bar{i} = invSE(H_c12c2_mean) * H_g2c2_ex{i};
    H_c12c1_bar{i} = H_g2c1_bar{i}*invSE(H_g2c1_ex{i});
end
%% Remove Outliers By Position
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

%% Save data 
save('ExtrinsicDataMatrices\H_c12c1_bar.mat',"H_c12c1_bar");
