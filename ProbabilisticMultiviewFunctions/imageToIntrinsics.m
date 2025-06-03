function A_c2m_i = imageToIntrinsics(im,cameraParams,squareSize,boardSize)
% IMAGETOINTRINSICS estimates the intrinsic matrix from a single image.
%
%   A_c2m_i = imageToIntrinsics(im,camaraParams,squareSize)
%
%   Input(s)
%       im - MxNxK array defining image
%       cameraParams - MATLAB camera parameters object
%       squareSize - scalar defining known square size
%       boardSize - 1x2 array defining expected board size
%
%   Output(s)
%       A_c2m_i - 3x3 array defining estimated intrinsic matrix
%
%   See also estimateIntrinsicMatrix
%
%   C. Civetta & M. Kutzer, 08Jul2024, USNA

%% Check input(s)
narginchk(3,4);

if ~isa(cameraParams, 'cameraParameters')
    error('Camera parameters must be provided.');
end

if nargin < 4
    boardSize = [];
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

% Undistort image points
imagePoints = undistortPoints(imagePoints,cameraParams);

% Check for partial detection
if ~isempty(boardSize)
    if ~all(boardSize == boardSize_i,'all')
        warning('Partial detection: [%d,%d] detected, [%d,%d] expected.',...
            boardSize_i,boardSize);
        A_c2m = [];
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
H_c2f = invSE(H_f2c);
p_c = H_c2f*p_f;

%% Define scaled camera refereced points
tilde_p_c = p_c(1:3,:) ./ p_c(3,:);

%% Calculate Intrinsics
A_row1 = p_m(1,:) * pinv(tilde_p_c(1:3,:));
A_row2 = p_m(2,:) * pinv(tilde_p_c(2:3,:));

% Format properly
A_c2m(1,:) = A_row1;
A_c2m(2,1) = 0;
A_c2m(2,2:3) = A_row2;
A_c2m(3,:) = [0 0 1];