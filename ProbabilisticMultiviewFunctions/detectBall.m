function [centers] = detectBall(img)
%DETECTBALL returns the location of red balls in an image
%
%   [centers] = detectBall(img) detects red balls in a given image by
%   color and share, and returns the pixel location of the ball(s) center
%
% Input(s) 
%       img - image file 
%
% Output(s)
%       centers - Nx2 array containing the x and y pixel coordinates of the
%       red ball object(s) in an image 
%
% C. A. Civetta, C. Doherty, M. Kutzer, 22Aug2023, USNA

%% Detect Circle
[BW,~] = YCbCrredball(img);

% fill holes
BW = imfill(BW,"holes");

% dilation and erosion
SE = strel('disk',3);
BW = imdilate(BW,SE);
BW = imerode(BW,SE);

% get blob information
stats = regionprops(BW,'Area','Circularity', 'Centroid');
% number blobs
L = bwlabel(BW);
length1 = length(stats);

for i = 1:length1
    areaArray(i) = stats(i).Area;
    circArray(i) = stats(i).Circularity;
end

avg1 = mean(areaArray);
std1 = std(areaArray);

keepBlob1 = find(areaArray>=max(areaArray));
keepBlob2 = find(circArray>=.7);

BW = zeros(size(BW));
keep = intersect(keepBlob1,keepBlob2);
for i = 1:length(keep)
    BW = BW + (L == keep(i));
    %     BW = BW + (L == keepBlob1(i));
end

% remask
maskedRGBImage = bsxfun(@times, img, cast(BW,'like',img));

centers = stats(keepBlob1).Centroid; 
end

