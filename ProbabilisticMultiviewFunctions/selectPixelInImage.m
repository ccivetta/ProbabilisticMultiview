function [p] = selectPixelInImage(imageName)
%SELECTPIXELINIMAGE Displays in image and allows the user to select a pixel
%to return its cooridinates
%   [p_x,p_y] = selectPixelInImage(imageName) displays the given image and
%   allows the user to zoom the image to desire fit. When finished, user
%   clicks the spacebar to finish fitting and then uses the crosshairs to
%   select a pixel and return its coordinates. 
%
%   Input(s)
%       imageName - image or file path containing an image 
%
%   Output(s)
%        p - 1x2 array containing the pixel coordinates in the format 
%            [x,y]
%
%   C. A. Civetta, M. Kutzer, 28Jun2024, USNA

imshow(imageName)
disp('Please zoom image to desired frame, and click the space bar when finished');
pause; %wait for user input before ginput to allow user to zoom in for accuracy
disp('Please use the crosshairs to select a pixel to recieve its coordinates');
[p(1), p(2)] = ginput(1);
close;
end