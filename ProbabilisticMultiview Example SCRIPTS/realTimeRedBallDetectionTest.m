%% Real Time Red Ball Detection
% This script applies thresholding and detection techniques to track the
% red ball in real time. Live images and overlapping detection region are displyed 

% C. A. Civetta, M. Kutzer, C. Doherty, 21Aug2023, USNA


%% Initial setup
close all
clear all
clc

% initiate camera
[cam, prv] = initCamera;
adjustCamera(cam);
pause()

% initiate img
img = prv.CData;
figure(2)
im = imshow(img);
hold on

circ = viscircles([10,10],10,'EdgeColor','b');
c = plot([0,0],'b+');
frame = 1;
t = text(10,10,num2str(frame),"FontSize",20,"Color",'y');

figure(3)
im2 = imshow(img);
hold on

%% Detect Circle
frame = 1;
while(1)
    figure(2)
    img = prv.CData;
    [BW,~] = hsvredball(img);
    set(im,'CData',img);

    % fill holes
    BW = imfill(BW,"holes");

    % dilation and erosion
    SE = strel('disk',3);
    BW = imdilate(BW,SE);
    BW = imerode(BW,SE);

    % get blob information
    stats = regionprops(BW,'Area','Circularity');
    % number blobs
    L = bwlabel(BW);
    length1 = length(stats);

    for i = 1:length1
        areaArray(i) = stats(i).Area;
        circArray(i) = stats(i).Circularity;
    end

    avg1 = mean(areaArray);
    std1 = std(areaArray);

    keepBlob1 = find(areaArray>=avg1);
    keepBlob2 = find(circArray>=.7);

    BW = zeros(size(BW));
    keep = intersect(keepBlob1,keepBlob2);
    for i = 1:length(keep)
        BW = BW + (L == keep(i));
%     BW = BW + (L == keepBlob1(i));
    end

    % remask
    maskedRGBImage = bsxfun(@times, img, cast(BW,'like',img));

    delete(t)
    frame = frame + 1;
    if frame > 60
        frame = 1;
    end

    [centers,radii] = imfindcircles(maskedRGBImage,[6,50],"EdgeThreshold",.2);
    if ~isempty(centers)
        delete(circ)
        delete(c)
        circ = viscircles(centers,radii,'EdgeColor','b');
        c = plot(centers(:,1),centers(:,2),'b+');
        t = text(10,10,num2str(frame),"FontSize",20,"Color",'g');
        drawnow
    else
        t = text(10,10,num2str(frame),"FontSize",20,"Color",'y');
    end

    figure(3)
    set(im2,'CData',maskedRGBImage);
    pause(.05)
end
