%% Monte Carlo Simulation for Extrinsic, Intrinsic, and Segmentation Uncertainty
%
% This script runs a Monte Carlo simulation N times and uses different
% values for extrinsic, instrinsic, and segmentation parameters each
% iteration. Values are pulled from normal distributions created from the
% covariance of the gathered data, and the mean from calibration. 
%
% After plotting the simulation this script also calculates relevant
% statistics and confidence intervals 
%
% Boolean statements at the begining of the code allow for the control of
% each type of uncertainty independently. If switched to false, MoCap data
% will be used as ground truth to eliminate uncertainty from the specified
% source. 
%
%   C. A. Civetta, M. Kutzer, 02Jul2024, USNA


close all;
clear all;
clc;

index = '1';
n = 10000; % Number of Monte Carlo samples

%% Booleans for what uncertainty to include
seg = true;
ext = true;
intr = true;
plotCameras = true;
distance = '_6.5m';
%% Initiate Camera
SCRIPT_initializeBinocularCameras;
%% Connect to Optitrack + UR
v = initOptitrack({'Bino', 'BinocularCameras'}, {'RedBall', 'RedBall'});
ur = urRTDEClient('10.0.0.100');
%% Load necessary matrices
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
load('SavedMatrices\H_b2c.mat')
load('SavedMatrices\H_f2g.mat')
load('SavedMatrices\H_c12c2_MoCap.mat')
load(['IntrinsicDataMatrices' distance '\A_c2m_in.mat'])
load(['SegmentationDataMatrices' distance '\p_cov.mat'])
load(['SegmentationDataMatrices' distance '\p_x_mean.mat'])
load(['SegmentationDataMatrices' distance '\p_y_mean.mat'])
load('SavedMatrices\H_c12c2_mean.mat');
load(['ExtrinsicDataMatrices' distance '\H_c12c1_bar.mat']);
load('URJoingAngles_dataCapture10.mat')
%load('SavedMatrices\A_c2m_in_all.mat')
%load('SavedMatrices\H_c12c1_bar_all.mat')

load('Red Ball Center 09 Aug 2024.mat')

p_rb_center_rb = p_rb_center;

rb_len = 1;
p_rb_center_rb(4,1) = 1;

for i=1:2
    A_c2m_truth{i} = params{i}.Intrinsics.IntrinsicMatrix';
end

%% Move Robot
%This step is for consistency but not required 
ur.sendJointConfigurationAndWait(q(4,:),...
        'EndTime',5.0,'Velocity',2*pi,'Acceleration',4*pi);
pause(.5);

%% Segment red ball and find center
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

    % Locate Ball
    [p_rb_center_m_cv{i}] = detectBall(undistorted_img);

    % Set axes properties
    set(axs(i),'NextPlot','Add');

    % Make axes title
    ttl(i) = title(axs(i),camSaveName{i});

    % Define ball center point
    center(i) = plot(axs(i),0, 0, 'cx','MarkerSize',8,'LineWidth',1.5);
    set(center(i), 'XData', p_rb_center_m_cv{i}(1), 'YData', p_rb_center_m_cv{i}(2), 'Visible', 'on')
end

%% Reproject red ball using MoCap
%Get necessary values from MoCap
H_rb2w = v.RedBall.pose;
H_b2w = v.Bino.pose;

H_rb2w(1:3,4) = H_rb2w(1:3,4)*1000; % Convert from m to mm
H_b2w(1:3,4) = H_b2w(1:3,4)*1000;   % Convert from m to mm

H_w2b = invSE(H_b2w);

for i = 1:2
    % Calculate red ball to camera
    H_rb2c{i} = H_b2c{i} * H_w2b * H_rb2w;

    % Projection matrix = intrinsics * (R_rb2c d_rb2c)
    Proj_rb2m = A_c2m_truth{i} * H_rb2c{i}(1:3,1:4);

    % Calculate scaled pixel coordinates of red ball per camera
    p_tilde_rb_center_m(:,i) = Proj_rb2m*p_rb_center_rb;

    % Save the pixel coordinates
    p_rb_center_m{i} = [p_tilde_rb_center_m(1,i)./p_tilde_rb_center_m(3,i) p_tilde_rb_center_m(2,i)./p_tilde_rb_center_m(3,i)];

    center_mocap_m(i) = plot(axs(i),0, 0, 'go','MarkerSize',8,'LineWidth',1.5);
    set(center_mocap_m(i), 'XData', p_rb_center_m{i}(1), 'YData', p_rb_center_m{i}(2), 'Visible', 'on')
end

%% Use motion capture instead of segmentation data
% We do this when we do not want computer segmentation error to play any
% role
if ~seg
    p_rb_center_m_cv = p_rb_center_m;
end
%% Monte Carlo Simulation
%% Create Figure
fig3 = figure('Name', 'Multi-View');
for k=1:2
    axs3(k) = subplot(2,1,k);
    set(axs3(k), 'NextPlot', 'add', 'Parent', fig3,'DataAspectRatio', [1 1 1]);

    % For visualization, we are going to use an hgtransform to "flip" x/y
    %   data
    h_d2a(k) = triad('Parent',axs3(k),'Scale',500,'Matrix',Ry(-pi/2)*Rz(pi));
    set(axs3(k),'YDir','reverse');
    xlabel(axs3(k),"z (mm)");
    ylabel(axs3(k),"y (mm)");
    zlabel(axs3(k),"x (mm)");

    view(3);
end
%% Plot Cameras
H_c22c1_MoCap = invSE(H_c12c2_MoCap);

if plotCameras
    for k=1:2
        cam1(k) = plot(h_d2a(k), 0, 0,'b^');
        cam2(k) = plot(h_d2a(k), 0, 0,'r^');
        set(cam2(k),'XData',H_c22c1_MoCap(1,4),'YData',H_c22c1_MoCap(2,4),'ZData',H_c22c1_MoCap(3,4),'Visible','on');
    end
end
%% Initialize MonteCarlo Values
%Save pixel location of the red ball in each image
p_m1 = [p_rb_center_m_cv{1} 1]';
p_m2 = [p_rb_center_m_cv{2} 1]';

%Segmentation offset value
if seg
    for i=1:2
        offset{i} = mvnrnd([p_x_mean(i), p_y_mean(i)],p_cov{i},n); %% Gets n random offsets for Camera 1 x & y coordinates
    end
end

%Intrinsics Monte Carlo values
if intr
    for i=1:2
        [~, v_A_c2m_cov{i}] = intrinsicStats(A_c2m_in(i,:), A_c2m_truth{i});

        % Get mean and covariance parameters
        v_A_c2m_mu = veeIntrinsics(A_c2m_truth{i});

        % Create n-random samples of intrinsic parameters
        v_A_c2m_mc{i} = mvnrnd(veeIntrinsics(A_c2m_truth{i}),v_A_c2m_cov{i},n);

    end
end

%Extrinsic Monte Carlo values
if ext
    H_c12c2_mu = H_c12c2_mean;
    v_c12c2_cov = covSE(H_c12c1_bar, eye(4));
    %v_c12c2_cov = covSE(H_c12c2_ex,H_c12c2_mean);
    v_c12c2_mu = veeSE( logSE( H_c12c2_mu) );
    v_c12c2_mc = mvnrnd(zeros(1,6),v_c12c2_cov,n);%Generates n randomly selected values
end
%% Initialize plot objects
ball(1) = plot3(h_d2a(1),nan, nan, nan, 'bo','MarkerSize',1,'LineWidth',1); % From Camera 1
ball(2) = plot3(h_d2a(1),nan, nan, nan, 'ro','MarkerSize',1,'LineWidth',1); % From Camera 2 
ball(3) = plot3(h_d2a(2),nan, nan, nan, 'ko','MarkerSize',1,'LineWidth',1); % Average Cameras 1 and 2

%% Loop
for i = 1:n
    %% Pull Monte Calo Values for Current Iteration
    %Segmentation Monte Carlo Values
    if seg
        p_m1_mc{i} = [(p_m1(1) + offset{1}(i,1)); (p_m1(2) + offset{1}(i,2)); 1];
        p_m2_mc{i} = [(p_m2(1) + offset{2}(i,1)); (p_m2(2) + offset{2}(i,2)); 1];
    else
        p_m1_mc{i} = p_m1;
        p_m2_mc{i} = p_m2;
    end

    %Intrinsic Monte Carlo Values
    if intr
        for j=1:2
            A_c2m_mc{j,i} = wedgeIntrinsics(v_A_c2m_mc{j}(i,:)); % Put in randomly generated values for intrinsics
        end
    else
        for j=1:2
            A_c2m_mc{j,i} = A_c2m_truth{j};
        end
    end

    %Extrinsic Monte Carlo Values
    if ext %% && v_c12c2_mc(i,6) > 0
        H_c12c2_mc{i} = H_c12c2_mu*expSE( wedgeSE( v_c12c2_mc(i,:) ) ); % Put randomly selected values back into H_c12c2 form
    else
        H_c12c2_mc{i} = H_c12c2_MoCap;
    end
    %% Point Recovery
    %Depth Recovery
    [zc1, zc2] = recoverDepth(p_m1_mc{i}, p_m2_mc{i}, A_c2m_mc{1,i}, A_c2m_mc{2,i}, invSE(H_c12c2_mc{i}));% Recover depth using random transform above

    %Define projection matrices for each camera to itself
    P_c12m = A_c2m_mc{1,i}*[1 0 0 0; 0 1 0 0; 0 0 1 0];
    P_c22m = A_c2m_mc{2,i}*[1 0 0 0; 0 1 0 0; 0 0 1 0];

    % Recover 3D point of the red ball relative to each camera
    p_c1{i} = recoverPoints(p_m1_mc{i}, zc1, P_c12m);
    p_c2{i} = recoverPoints(p_m2_mc{i}, zc2, P_c22m);

    % Convert point relative to c2 to c1 coordinate frame for plotting
    p_c2_relc1{i} = H_c22c1_MoCap * p_c2{i}; %Use MOCAP truth transorm for coordiante fram shift only for plotting purposes

    %% Plot recovered points 
    %Plot the location of both recovered positions on the same axes
    if p_c1{i}(3) > 0 && p_c2{i}(3) > 0 
    appendLine(ball(1),p_c1{i}(1:3,1));
    appendLine(ball(2),p_c2_relc1{i}(1:3,1));

    % Plot the mean recovered position
    p_c_mean{i} = [mean([p_c1{i}(1),p_c2_relc1{i}(1)]); mean([p_c1{i}(2),p_c2_relc1{i}(2)]); mean([p_c1{i}(3),p_c2_relc1{i}(3)])];
    appendLine(ball(3),p_c_mean{i}(1:3,1));
    end
end
%% Plot mocap location of the red ball
H_rb2c1 =  H_b2c{1} * invSE(H_b2w) * H_rb2w * p_rb_center_rb;
for k=1:2
    trueball(k) = plot(h_d2a(k),0, 0, 'm*','MarkerSize',15,'LineWidth',3);
    set(trueball(k),'XData',H_rb2c1(1),'YData',H_rb2c1(2),'ZData',H_rb2c1(3),'Visible','on');
end

%% Stat Calculations
meanx = 0;
meany = 0;
meanz = 0;
for i=1:numel(p_c_mean)
    if p_c1{i}(3) > 0 && p_c2{i}(3) > 0
    dist(i) = sqrt((p_c_mean{i}(1) - H_rb2c1(1))^2 + (p_c_mean{i}(2) - H_rb2c1(2))^2 + (p_c_mean{i}(3) - H_rb2c1(3))^2);
    distx(i) = (p_c_mean{i}(1) - H_rb2c1(1));
    disty(i) = (p_c_mean{i}(2) - H_rb2c1(2));
    distz(i) = (p_c_mean{i}(3) - H_rb2c1(3));
    meanx = meanx + p_c_mean{i}(1);
    meany = meany + p_c_mean{i}(2);
    meanz = meanz + p_c_mean{i}(3);
    x_hold(i) = p_c_mean{i}(1);
    y_hold(i) = p_c_mean{i}(2);
    z_hold(i) = p_c_mean{i}(3);
    end
end
meanx = meanx/n;
meany = meany/n;
meanz = meanz/n;
covXY = cov(x_hold, y_hold);
covXZ = cov(x_hold, z_hold);
covYZ = cov(y_hold, z_hold);
meanDistx = mean(distx);
meanDisty = mean(disty);
meanDistz = mean(distz);
cov3D = cov([x_hold.', y_hold.', z_hold.']);

%% Plot 3D Ring - looped to create a ring at each 5% interval for comparison with data
% Define confidence interval (value > 0 and < 1)
% -> 95% confidence interval (i.e. 95% of data is contained)
%    ---> Use 0.95
dataPoints = [];
for j = 1:length(p_c_mean)
    dataPoints = [dataPoints, p_c_mean{j}];
end

ALL_prcConfInterval_Sample = [];
ALL_prcConfInterval        = [];

for i = linspace(.01,.99,1000)
    %create ellipsoid
    efit = mvnpdfConfInterval([meanx meany meanz],cov3D,i);
    % Find how many data points fall within the created ellipsoid
    tfIn = inEllipsoid(efit,dataPoints);
    % Calculate percent of data in ellipsoid
    prcSample = nnz(tfIn)/numel(tfIn);
    ALL_prcConfInterval(end+1)        = i;
    ALL_prcConfInterval_Sample(end+1) = prcSample;
end

%% Plot ellipsoid visual
CI = 0.50;

% Plot ellipsoid for visualization
efit = mvnpdfConfInterval([meanx meany meanz],cov3D,CI);
%ptcCOV = plotEllipsoid(h_d2a(2),efit);
ptcCOV = plotEllipsoidUpdated(efit);
ptcCOV.FaceColor = 'y';
ptcCOV.FaceAlpha = .15;
set(ptcCOV,'Parent',h_d2a(2));
ellipsoidVolume(efit)

CI = 0.95;

% Plot ellipsoid for visualization
efit = mvnpdfConfInterval([meanx meany meanz],cov3D,CI);
%ptcCOV = plotEllipsoid(h_d2a(2),efit);
ptcCOV = plotEllipsoidUpdated(efit);
ptcCOV.FaceColor = 'r';
ptcCOV.FaceAlpha = .05;
set(ptcCOV,'Parent',h_d2a(2));
ellipsoidVolume(efit)

CI = 0.05;

% Plot ellipsoid for visualization
efit = mvnpdfConfInterval([meanx meany meanz],cov3D,CI);
%ptcCOV = plotEllipsoid(h_d2a(2),efit);
ptcCOV = plotEllipsoidUpdated(efit);
ptcCOV.FaceColor = 'g';
ptcCOV.FaceAlpha = .25;
set(ptcCOV,'Parent',h_d2a(2));
ellipsoidVolume(efit)
%% Plot Confidence Intervals 
figure;
axs = axes;
set(axs, 'NextPlot', 'Add');
theory = plot(axs,0,0,'b-');
samp = plot(axs,0,0,'r-');
set(theory, 'XData', linspace(1,99,1000), 'YData', ALL_prcConfInterval.*100, 'Visible', 'on');
set(samp, 'XData', linspace(1,99,1000), 'YData', ALL_prcConfInterval_Sample.*100, 'Visible', 'on');
legend('Theoretical Expected Values', 'Samples');
xlabel('Expected Samples Inside Hyper-Ellipsoid (%)');
ylabel('Observed Samples Inside Hyper-Ellipsoid (%)');
axs.FontSize = 30;

%% Copy subplot data into larger figure 
fig4 = figure;
axs4 = axes; 
copyobj(allchild(axs3(2)), axs4);
set(axs4, 'NextPlot', 'add', 'Parent', fig4,'DataAspectRatio', [1 1 1]);
view(3);