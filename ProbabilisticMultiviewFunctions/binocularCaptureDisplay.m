function img = binocularCaptureDisplay(prv, fig)
%BINOCULARCAPTUREDISPLAY displays real time feed from two cameras and
%captures the images simultaneously when the space bar is pressed
%   img = binocularCaptureDisplay(prv, fig) displays a real time feed from
%   two binocular cameras on a figure. When the space bar is pressed, both
%   images are captured and returned 
%
%   Input(s)
%       prv - 1x2 array of preview object's for the binocular cameras
%       fig - figure for display 
%
%   Output(s) 
%       img - 1x2 cell array of captured images from cameras 1 and 2
%             respectively, as ordered in the prv argument 
%
% C. A. Civetta, M. Kutzer, 26Jun2024, USNA
%% Display Images Until Any Key (spacebar) is Pressed
disp('Press any key to take image')
while true
    figure(fig);
    for i=1:2
        subplot(1,2,i);
        imshow(prv(i).CData);
        drawnow
    end
    if fig.CurrentCharacter ~= '1'
        set(fig,'currentch','1');
        break;
    end
end
%% Capture Image
for i=1:2
    img{i} = prv(i).CData;
end
end