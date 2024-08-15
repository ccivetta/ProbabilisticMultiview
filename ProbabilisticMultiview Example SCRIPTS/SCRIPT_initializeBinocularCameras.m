%% SCRIPT_initializeBinocularCameras
% This script easily initializes and adjusts settings for the VIPER Lab binocular
% cameras 
%
% A pop up prompting the user to choose the cameras is displyed. First
% choose DFK 23UM021, second choose DFK 23U618. It should be the top choice
% first and then the one below it. 
%
% Output(s) (to workspace) 
%   cam   - 1x2 videoinput objects
%   camU6 - videosource object for DFK 23U618
%   camUM - videosource object for DFK 23UM021
%   prv   - 1x2 image previews 
%   resx  - 1x2 array containing the resolution on the x axis for camera 1
%           and camera 2 respectively 
%   resy  - 1x2 array containing the resolution on the y axis for camera 1
%           and camera 2 respectively 
%
% C. A.Civetta, M. Kutzer, 26Jun2024, USNA

for i = 1:2
    [cam(i), prv(i)] = initCamera;
    if strcmp(imaqhwinfo('winvideo',cam(i).DeviceID).DeviceName,'DFK 23U618')
        stop(cam(i));
        camU6 = getselectedsource(cam(i));
        camU6.Brightness = 0;
        camU6.Contrast = 1;
        camU6.Exposure = -5;
        camU6.ExposureMode = "manual";
        camU6.FrameRate = "30.0000";
        camU6.Gain = 219;
        camU6.GainMode = "manual";
        camU6.Gamma = 100;
        camU6.Hue = 0;
        camU6.Saturation = 72;
        camU6.Sharpness = 8;
        camU6.WhiteBalance = 6500;
        camU6.WhiteBalanceMode = "auto";
        start(cam(i));
    elseif strcmp(imaqhwinfo('winvideo',cam(i).DeviceID).DeviceName,'DFK 23UM021')
        stop(cam(i));
        camUM = getselectedsource(cam(i));
        camUM.Brightness = 0;
        camUM.Contrast = 0;
        camUM.Exposure = -3;
        camUM.ExposureMode = "manual";
        camUM.FrameRate = "30.0000";
        camUM.Gain = 32;
        camUM.GainMode = "manual";
        camUM.Gamma = 65;
        camUM.Hue = 0;
        camUM.Saturation = 64;
        camUM.Sharpness = 10;
        camUM.WhiteBalance = 6500;
        camUM.WhiteBalanceMode = "auto";
        start(cam(i));
    end
    [resy(i), resx(i), ~] = size(prv(i).CData); % x is from left to right y is top to bottom
    disp(['Setting up Camera ' num2str(i)]);
end
clear i;