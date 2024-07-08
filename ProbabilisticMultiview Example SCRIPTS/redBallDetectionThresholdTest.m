%% Red Ball Detection Threshold Test 
% This script is meant as an initial trial for how to detect the red ball in
% an image.

% Script loads a saved image ('redball.jpg'), applies thresholding
% techniques, and displays the resulting images 

% C. A. Civetta, C. Doherty, M. Kutzer, 17Aug2023, USNA



%% Initial setup
close all
clear all
clc

% load image
img = imread('redball.jpg');

% display image
imshow(img)
title('Red Ball Original')

%% Color Thresholding
% color threshold for rgb
[BW1,maskedRGBImage1] = rgbredball(img);

% color threshold for hsv
[BW2,maskedRGBImage2] = hsvredball(img);

% display images
figure(2)
subplot(121)
imshow(BW1)
title('1) RGB')

subplot(122)
imshow(BW2)
title('2) HSV')

%% Fill holes and other cleaning
% fill holes
BW1 = imfill(BW1,"holes");
BW2 = imfill(BW2,"holes");

% dilation
SE = strel('disk',3);
BW1 = imdilate(BW1,SE);
BW2 = imdilate(BW2,SE);

% erosion
BW1 = imerode(BW1,SE);
BW2 = imerode(BW2,SE);

% display images
figure(3)
subplot(121)
imshow(BW1)
title('1) RGB')

subplot(122)
imshow(BW2)
title('2) HSV')

%% Get information about masked images
% get blob information
stats1 = regionprops(BW1,'all');
stats2 = regionprops(BW2,'all');

% number blobs
L1 = bwlabel(BW1);
L2 = bwlabel(BW2);

%% Remove regions that are too small

%turn stats struct into usable arrays
length1 = length(stats1);
length2 = length(stats2);

if length1 > 2

    for i = 1:length1
        areaArray1(i) = stats1(i).Area;
    end

    avg1 = mean(areaArray1);
    std1 = std(areaArray1);

    keepBlob1 = find(areaArray1>=avg1+(2*std1));

    BW1 = zeros(size(BW1));
    for i = 1:length(keepBlob1)
        BW1 = BW1 + (L1 == keepBlob1(i));
    end
end

if length2 > 2

    for i = 1:length2
        areaArray2(i) = stats2(i).Area;
    end

    avg2 = mean(areaArray2);
    std2 = std(areaArray2);

    keepBlob2 = find(areaArray2>=avg2+(2*std2));

    BW2 = zeros(size(BW2));
    for i = 1:length(keepBlob2)
        BW2 = BW2 + (L2 == keepBlob2(i));
    end
end

figure(4)
subplot(121)
imshow(BW1)
title('1) RGB')
subplot(122)
imshow(BW2)
title('1) HSV')
