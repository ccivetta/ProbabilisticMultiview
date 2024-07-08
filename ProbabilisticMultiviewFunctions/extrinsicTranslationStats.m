function [H_cA2cB,X_cA2cB,Sigma_X_cA2cB] = extrinsicTranslationStats(H_g2c1, H_g2c2, tfPlot)
%EXTRINSICTRANSLATIONSTATS- Yields average transform between two rigidly mounted
%cameras along with the standard deviation and covariance of the translation data
%
%   [H_cA2cB,X_cA2cB,Sigma_X_cA2cB] = extrinsicTranslationStats(H_g2c1, H_g2c2) takes two datasets 
%   of corresponding extrinsics for two cameras and yields the mean SE transform relating c1 to
%   c2 after outlier data is removed. It also returns the mean x, y, and z
%   translation values and associated covariance
%
%   [H_cA2cB,X_cA2cB,Sigma_X_cA2cB] = extrinsicStats(__, __, tfPlot)
%
% Optional boolean argument to enable plotting the data
%
% Outliers are defined as any transform with either x, y, or z translation
% greater than three scaled medial absolute deviations from the median
%
% Input(s): 
%   H_g2c1        - a 1xN cell array containing H_g2c1 (checkerboard grid (g) to camera 1 (c1))
%   H_g2c2        - a 1xN cell array containing H_g2c2 (checkerboard grid (g) to camera 2 (c2))
%   tfPlot        - [Optional] boolean argument to control plotting feature. True enables
%                   plotting
%
% Output(s): 
%   H_cA2cB       - 4x4 mean SE matrix relating camera 1 to camera 2
%   X_cA2cB       - 3x1 array containing the mean values for the x, y, and z
%                   translations in the form [x; y; z]
%   Sigma_X_cA2cB - 3x3 array containing the covariance for the x, y, and z
%                   translation data 
%
% C. A. Civetta 16OCT2023

% If the number of input arguments is less than 3
if nargin < 3
    tfPlot = false;
end

%% Check valid correspondence
ZERO = 1e-8;
[H_g2cA,H_g2cB,~] = validCorrespondenceSE(H_g2c1,H_g2c2,ZERO);

%% Define relative transform
n = numel(H_g2cA);
for i = 1:n
    H_cA2cB_ALL{i} = H_g2cB{i} * invSE(H_g2cA{i},ZERO);
end

%% Remove outliers by position
%Save the translation data for all transforms
for i = 1:n
    xpos(i) = H_cA2cB_ALL{i}(1,4);
    ypos(i) = H_cA2cB_ALL{i}(2,4);
    zpos(i) = H_cA2cB_ALL{i}(3,4);
end

%Remove outliers
[xpos_new,logicX] = rmoutliers(xpos);
[ypos_new,logicY] = rmoutliers(ypos);
[zpos_new,logicZ] = rmoutliers(zpos);

%Save all indicies at which an outlier was removed
bad = [];
for i = 1:n
    if logicX(i) == 1 || logicY(i) == 1 || logicZ(i) == 1
        bad = [bad i];
    end
end
bad = unique(bad);

%Remove outlier data
for i=length(bad):-1:1
    H_cA2cB_ALL(bad(i)) = [];
end
%% Calculate mean transformation
H_cA2cB = meanSE(H_cA2cB_ALL,ZERO);

%% Calculate origin statistics
for i = 1:numel(H_cA2cB_ALL)
    X_cA2cB_ALL(:,i) = H_cA2cB_ALL{i}(1:3,4);
end
X_cA2cB = mean(X_cA2cB_ALL.').';
Sigma_X_cA2cB = cov(X_cA2cB_ALL.');

%% Plot error
if tfPlot
    fig = figure('Name','Extrinsic Mean');
    axs = axes('Parent',fig,'DataAspectRatio',[1 1 1],'NextPlot','add');
    view(3);
    for i = 1:numel(H_cA2cB_ALL)
        h_cA2cB(i) = triad('Parent',axs,'Matrix',H_cA2cB_ALL{i});
    end
    h_cA2cB_star = triad('Parent',axs,'Matrix',H_cA2cB,'LineWidth',2, 'Scale', 3);
end