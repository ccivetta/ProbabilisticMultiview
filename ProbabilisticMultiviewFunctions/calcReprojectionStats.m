function [d_mean,d_median, d_max, d_min, d_std, d_cov] = calcReprojectionStats(detectedPoints,repPoints)
%CALCREPROJECTIONSTATS - This function yields a number of statistics
% comparing repojected checkerboard points to detected points for a single
% image
%
%   [d_mean,d_median, d_max, d_min, d_std, d_cov] = calcReprojectionStats(detectedPoints,repPoints) 
%   takes the detected points and reprojection points of an image and calculates useful statistics 
%   based on the distance between corresponding points
%
%   Input(s)
%       detectedPoints - Nx2 array containing the x and y pixel coordinates
%                        of detected checkerboard points where N = number of checkerboard
%                        vertices
%       repPoints      - Nx2 array containing the x and y pixel coordinates
%                        of reprojected checkerboard points where N = number of checkerboard
%                        vertices
%
%   Output(s)
%       d_mean           - the mean euclidean distance between detected and
%                          reprojected points 
%       d_median         - the median euclidean distance between detected and
%                          reprojected points 
%       d_max            - the max euclidean distance between detected and
%                          reprojected points 
%       d_min            - the min euclidean distance between detected and
%                          reprojected points 
%       d_std            - the standard deviation of euclidean distance between detected and
%                          reprojected points 
%       d_cov            - the covariance of euclidean distance between detected and
%                          reprojected points 
%
% TODO: 
%       - Update to display error message when full checkerboard not
%         detected for other sizes 
%
%
% C. A. Civetta, M. Kutzer, 16OCT2023, USNA

if length(detectedPoints(:,1)) ~= 56
    disp('Full checkerboard not detected');
    return;
end

for i=1:length(detectedPoints(:,1))
    d(i) = sqrt(((detectedPoints(i,1)-repPoints(i,1))^2)+((detectedPoints(i,2)-repPoints(i,2))^2));
end

d_mean = mean(d);
d_median = median(d);
d_max = max(d);
d_min = min(d);
d_std = std(d);
d_cov = cov(d);

end
