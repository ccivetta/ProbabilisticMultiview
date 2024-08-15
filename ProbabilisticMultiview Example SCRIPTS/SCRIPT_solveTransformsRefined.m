%% SCRIPT_solveTransformsRefined
% This script runs through the full refinement process for the system. 
% After running this script, both camera's parameters are refined, as well
% as H_b2c, H_f2g, H_c12c2_MoCap, and H_c12c2_mean. Data is saved with the
% ending '_refined' and the number iteration it was. 
%
% After each cycle, reprojection is run by moving the UR to 10 positions
% and measuring mean reprojection error
%
% The number that produced the lowest reprojection error is displayed at
% the end of the cycle
%
% Note that other scripts will by default load saved variables with the original
% names, not '_refined'. When working with refined data, ensure you are
% loading the correct files. 
%
%   C. A. Civetta, M. Kutzer, 08Aug2024, USNA

clear all; 
close all; 
clc;

%% Initiate Camera + Optitrack + UR
index = '1';
SCRIPT_initializeBinocularCameras;
v = initOptitrack({'Bino', 'BinocularCameras'},{'Check', 'NiceCheckerboard'});
ur = urRTDEClient('10.0.0.100');
%% Refine Camera Parameters 
for i = 1:2
    device = imaqhwinfo('winvideo',cam(i).DeviceID);
    camFormat{i} = [num2str(resy(i)) 'x' num2str(resx(i))];
    camName{i} = device.DeviceName;
    camSaveName{i} = [camFormat{i} '_' camName{i} index];

    load(['SavedMatrices\' camSaveName{i} '\CameraParams.mat'])
    bubble{i} = params;
end
clear params
params = bubble;
clear bubble
load('URJoingAngles_dataCapture10.mat');
%% Loop
for p=1:5
params{2} = refineCamera(['CorrelationImages\' camSaveName{2}], params{1}, params{2});
params{1} = refineCamera(['CorrelationImages\' camSaveName{1}], params{2}, params{1});

%% Generate Extrinsics
for i=1:2
    H_g2c(i,:) = generateExtrinsics(params{i});
end
[H_c12c2_mean, ~, ~] = extrinsicTranslationStats(H_g2c(1,:), H_g2c(2,:));
%% Solve Fixed Transforms 

load("SavedMatrices\H_f2w.mat");
load("SavedMatrices\H_b2w.mat");

%Convert to mm
for j = 1:length(H_f2w)
    H_f2w{j}(1:3,4) = H_f2w{j}(1:3,4)*1000;
    H_b2w{j}(1:3,4) = H_b2w{j}(1:3,4)*1000;
end

for i = 1:length(H_f2w)
    H_f2b{i} = invSE(H_b2w{i}) * H_f2w{i};
end

for i=1:2
    [H_b2c{i},H_f2g{i}] = solveFixedTransforms(H_g2c(i,:),H_f2b);
end

H_c12c2_MoCap = H_b2c{2} * invSE(H_b2c{1});

%% Save refined values
save(['SavedMatrices\H_c12c2_MoCaP_refined_' num2str(p)], "H_c12c2_MoCap");
save(['SavedMatrices\H_c12c2_mean_refined_' num2str(p)], "H_c12c2_mean");
save(['SavedMatrices\H_b2c_refined_' num2str(p)], "H_b2c");
save(['SavedMatrices\H_f2g_refined_' num2str(p)], "H_f2g");

tempParams(:,:) = params;
for i=1:2
    params = tempParams{i};
    save(['SavedMatrices\' camSaveName{i} '\CameraParams_refined_' num2str(p)],"params");
end
params = tempParams;

%% Reprojection Test
for i=1:2
    A_c2m{i} = params{i}.Intrinsics.IntrinsicMatrix';
end
% Reprojection visualization stream
fig = figure('Name','ReprojectBall.m');
% initialize points
for i = 1:2
    % Create axes
    axs(i) = subplot(1,2,i);

    % Gather initial image
    img{i} = prv(i).CData;

    % Undistort Image
    undistorted_img = undistortImage(img{i},params{i});
    imObj(i) = imshow(undistorted_img,'Parent',axs(i));

    % Set axes properties
    set(axs(i),'NextPlot','Add');

    % Make axes title
    %ttl(i) = title(axs(i),camSaveName{i});

    % Define check points
    checkPoints(i) = plot(axs(i),0,0,'ro','MarkerSize',7,'LineWidth',1);
    detectedCheckPoints(i) = plot(axs(i), 0, 0, 'g*', 'MarkerSize', 7, 'LineWidth',1);
end

% Run loop
n=1;
while (n<=10) %true

    % Move robot to next position
        ur.sendJointConfigurationAndWait(q(n,:),...
            'EndTime',3.0,'Velocity',2*pi,'Acceleration',4*pi);
        pause(.5);
    % gather images
    for i = 1:2
        img{i} = prv(i).CData;
    end

    % gather pose information
    H_b2w = v.Bino.pose;
    H_f2w = v.Check.pose;

    squareSizeMM = 30;
    p_g = params{1}.WorldPoints';
    p_g(3,:) = 0;
    p_g(4,:) = 1;

    % Switch to mm
    H_b2w(1:3,4) = H_b2w(1:3,4)*1000;
    H_f2w(1:3,4) = H_f2w(1:3,4)*1000;

    H_w2b = invSE(H_b2w);


    for i=1:2
        H_g2f{i} = invSE(H_f2g{i});
    end


    for i = 1:2
        %Calculate Checkerboard grid to camera
        H_g2c{i} = H_b2c{i} * H_w2b * H_f2w * H_g2f{i};
        X_m_rep_tilde{i} = A_c2m{i} * H_g2c{i}(1:3,:) * p_g;
        X_m_rep{i} = (X_m_rep_tilde{i}./X_m_rep_tilde{i}(3,:));

        % Undistort image
        undistorted_img = undistortImage(img{i},params{i});
        % Update image in plot
        set(imObj(i),'CData',undistorted_img);

        % Check if point if in FOV and plot MOCAP overlay
        inFrameCheck = inFOV(X_m_rep_tilde{i}, params{1});

        if ~inFrameCheck
            set(checkPoints(i),'Visible','off');
            fprintf(['PIXEL COORDINATES ARE OUT OF ' char(camSaveName(i)) ' FRAME\n'])
            disp(inFrame)
            continue
        else
            set(checkPoints(i),'XData',X_m_rep{i}(1,:),'YData',X_m_rep{i}(2,:),'Visible','on');
        end

        % Find detected points
        detector = vision.calibration.monocular.CheckerboardDetector();
        imagePoints{i} = detectPatternPoints(detector, undistorted_img, "PartialDetections", false);
        if size(imagePoints{i},1) == 56
            set(detectedCheckPoints(i),'XData', imagePoints{i}(:,1),'YData',imagePoints{i}(:,2),'Visible','on');
            %Save statistics in struct d as nx2 matrices, where n is the number
            %of images and i is the number of cameras
            [d.mean(n,i),d.median(n,i), d.max(n,i), d.min(n,i), d.std(n,i), d.cov(n,i)] = calcReprojectionStats(imagePoints{i},X_m_rep{i}');
        else
            disp('Full checkerboard not detected');
            set(detectedCheckPoints(i),'XData', 0,'YData',0,'Visible','off');
        end

        % Add Legend
        legend('Reprojected Points', 'Detected Points');


    end
    n = n + 1;
end
c1mean_str = mean(d.mean(:,1));
c2mean_str = mean(d.mean(:,2));
meanSum(p) = c1mean_str + c2mean_str; 
end
[~,m] = min(meanSum);
disp(["Refinement with the lowest reprojection error is # " num2str(m)]);