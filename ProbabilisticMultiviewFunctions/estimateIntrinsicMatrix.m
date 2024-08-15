function [A_c2m] = estimateIntrinsicMatrix(boardSize, squareSize, imageFolder, varargin)
%ESTIMATEINTRINSICMATRIX Takes the board size and square size of the checkerboard grid, rigid body
%transform defining the checkerboard grid relative to the camera, and an
%image in order to estimate an intrinsic matrix 
%   [A_c2m] = estimateIntrinsicMatrix(board, squareSize, imageFolder, params) estimates an
%   intrinsic matrix for a camera given cameraParameters, square size, and images
%   of the checkerboard. 
%
%   [A_c2m] = estimateIntrinsicMatrix(board, squareSize, imageFolder, H_g2c) allows the user
%   to provide a cell array of rigid body transforms of the checkerboard
%   grid relative to the camera. Values must correspond to the images in
%   imageFolder. 
%
%   Input(s)
%       boardSize   - 1x2 array containing the size of the board in the form [height, weight]
%       squareSize  - the length of the size of a single square in mm 
%       H_g2c       - 1xN cell array of rigid body transforms of the
%                     checkerboard grid relative to the camera where N is the
%                     number of images. Values must correspond to the images
%                     given in imageFolder 
%       imageFolder - path of a folder containing images from which to
%                     estimate intrinsics 
%       params      - cameraParameters object
%
%   Output(s)
%       A_c2m - 1xN cell array of 3x3 intrinsic matrices 
%
%
%   C. A. Civetta, M. Kutzer, 28Jun2024, USNA
warning off; 

if nargin < 4
    disp('Incorrect number of arguments, 4 minimum');
    return;
end 

if isa(varargin{1}, 'cameraParameters')
    H_g2c = generateExtrinsics(varargin);
else 
    H_g2c = cell(1, length(varargin{1}));
    H_g2c = varargin{1};
end 

if nargin == 5 && varargin{2}
    plot = true; 
else 
    plot = false;
end 

images = imageSet(imageFolder);
imageFileNames = images.ImageLocation;

% Detect Calibration Pattern (Long step)
detector = vision.calibration.monocular.CheckerboardDetector();
imagePoints = detectPatternPoints(detector, imageFileNames,"PartialDetections", false);

% Establish grid coordinates
p_g = generateCheckerboardPoints(boardSize, squareSize).';
p_g(3,:) = 0;
p_g(4,:) = 1;


for n=1:length(imageFileNames)
    %Calculate Checkerboard grid
    p_c_tilde = H_g2c{n}(1:3,:) * p_g;
    p_c = p_c_tilde./p_c_tilde(3,:);

    %Define matrix points - pixel location of checkerboard points detected in image
    p_m_tilde = imagePoints(:,:,n);
    p_m_tilde(:,3) = 1;
    p_m_tilde = p_m_tilde.';

    %Calculate Intrinsics
    A_row1 = p_m_tilde(1,:) * pinv(p_c);
    A_row2 = p_m_tilde(2,:) * pinv(p_c(2:3,:));

    % Format properly
    A_c2m{n} = [];
    A_c2m{n}(1,:) = A_row1;
    A_c2m{n}(2,1) = 0;
    A_c2m{n}(2,2:3) = A_row2;
    A_c2m{n}(3,:) = [0 0 1];
end

%% Plot images to check correspondance 
if plot
fig = figure('Name','ReprojectPoints.m');
axs = axes;
for i=1:length(H_g2c)
        % Define reproject image points using single camera extrinsics
        X_m_rep_tilde{i} = A_c2m{i} * H_g2c{i}(1:3,:) * p_g;
        X_m_rep{i} = (X_m_rep_tilde{i}./X_m_rep_tilde{i}(3,:));
        im = read(images, i);
        imObj(n) = imshow(im,'Parent',axs);
        set(axs,'NextPlot','Add');
        plot(axs,X_m_rep{i}(1,:), X_m_rep{i}(2,:), 'go','MarkerSize',3,'LineWidth',1);
        disp('Press spacebar for next image');
        pause();
end
end
end