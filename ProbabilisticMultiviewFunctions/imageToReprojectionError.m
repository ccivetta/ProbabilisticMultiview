function [err,p_m,dp_m] = imageToReprojectionError(im,cameraParams,squareSize,varargin)
% IMAGETOREPROJECTIONERROR calculates the reprojection error associated
% with an image of a checkerboard fiducial given camera parameters, square
% size, and (optionally) an intrinsic matrix.
%
%   [err,p_m,dp_m] = imageToReprojectionError(im,cameraParams,squareSize)
%   ___ = imageToReprojectionError(im,cameraParams,squareSize,boardSize)
%   ___ = imageToReprojectionError(im,cameraParams,squareSize,boardSize,A_c2m)
%
%   Input(s)
%       im - image of checkerboard fiducial
%       cameraParams - MATLAB camera parameters object
%         squareSize - scalar defining the square size of the checkerboard in
%                      linear units
%         boardSize - [OPTIONAL] 1x2 array defining the board size (to 
%                     ignore board size and use intrinsics use 
%                     boardSize = [])
%             A_c2m - [OPTIONAL] 3x3 intrinsic matrix
%
%   Output(s)
%        err - scalar defining average RMS reprojetion error
%        p_m - 2xN array defining x/y pixel locations
%       dp_m - 2xN array defining individual pixel error
%
%   M. Kutzer, 02Jun2025, USNA

%% Check input(s)
narginchk(4,5);

if ~isa(cameraParams, 'cameraParameters')
    error('Camera parameters must be provided.');
end

if nargin < 5
    boardSize = [];
    A_c2m = varargin{1};
else
    boardSize = varargin{1};
    A_c2m = varargin{2};
end

%% Calculate extrinsics from image

% Define p_m from image of checkerboard
% NOTE: MATLAB's definition of "imagePoints" relates to p_m as follows:
%       % Define "imagePoints" from p_m
%       imagePoints = p_m(1:2,:).'; % <-- Note the transpose
%       % Define p_m from "imagePoints"
%       p_m(1:2,:) = imagePoints.'; % <-- Note the transpose
%       p_m(3,:) = 1;               % <-- Convert to homogeneous, 2D
%                                         pixel position
[imagePoints,boardSize_i] = detectCheckerboardPoints(im);

% Keep only finite points
tfIsFinite = isfinite(imagePoints(:,1));
imagePoints = imagePoints(tfIsFinite,:);

% Return if no finite points
if isempty(imagePoints)
    warning('No image points detected.');
    A_c2m_i = [];
    return
end

% Undistort image points
imagePoints = undistortPoints(imagePoints,cameraParams);

% Check for partial detection
if ~isempty(boardSize)
    if ~all(boardSize == boardSize_i,'all')
        warning('Partial detection: [%d,%d] detected, [%d,%d] expected.',...
            boardSize_i,boardSize);
        A_c2m_i = [];
        return
    end
else
    boardSize = boardSize_i;
end

% Define p_f given "boardSize" and "squareSize"
% NOTE: MATLAB's definition of "worldPoints" relates to p_f as follows:
%       % Define "worldPoints" from p_f
%       worldPoints = p_f(1:2,:).'; % <-- Note the transpose
%       % Define p_f from "worldPoints"
%       p_f(1:2,:) = worldPoints.'; % <-- Note the transpose
%       p_f(3,:) = 0;               % <-- Define z-coordinate
%       p_f(4,:) = 1;               % <-- Convert to homogeneous, 3D
%                                   %     coordinate relative to the
%                                   %     fiducial frame
[worldPoints] = generateCheckerboardPoints(boardSize,squareSize);

% Keep corresponding finite points
worldPoints = worldPoints(tfIsFinite,:);

% Recover the checkerboard pose relative to the camera frame (H_f2c)
[R_c2f, tpose_d_f2c] = extrinsics(...
    imagePoints,worldPoints,cameraParams);
R_f2c = R_c2f.';
d_f2c = tpose_d_f2c.';
H_f2c = [R_f2c, d_f2c; 0,0,0,1];

%% Define point correspondence
% Define p_m from "imagePoints"
p_m(1:2,:) = imagePoints.'; % <-- Note the transpose
p_m(3,:) = 1;               % <-- Convert to homogeneous, 2D

% Define p_f from "worldPoints"
p_f(1:2,:) = worldPoints.'; % <-- Note the transpose
p_f(3,:) = 0;               % <-- Define z-coordinate
p_f(4,:) = 1;               % <-- Convert to homogeneous, 3D

% Define p_c
%H_c2f = invSE(H_f2c);
p_c = H_f2c*p_f;

%% Reproject points
tilde_p_m = A_c2m*p_c(1:3,:);
p_m = tilde_p_m(1:2,:)./tilde_p_m(3,:);

%% Calculate error
% Segmented image points
p_m_star = imagePoints.';
% Signed pixel error
dp_m = p_m(1:2,:) - p_m_star(1:2,:);
% SSD
ssdp_m = sqrt( sum(dp_m.^2,1) );
% Average SSD
err = mean(ssdp_m);

%% Package output(s)
p_m = p_m_star(1:2,:);