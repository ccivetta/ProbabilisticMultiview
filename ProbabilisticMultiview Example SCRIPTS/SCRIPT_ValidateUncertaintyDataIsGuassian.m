%% SCRIPT_ValidateUncertaintyDataIsGuassian
% This script takes sample data from extrinsics, intrinsics, and
% segmentation and compares how much of the data falls within a certain
% theoretical confidence interval for a normal distribution. 
% 
% Each dataset is plotted against the theoretical for visualization 
%
%   C. A. Civetta, M. Kutzer, 02Jul2024, USNA
%% Extrinsics
clear all
load('ExtrinsicDataMatrices\H_c12c2_ex.mat');
load('SavedMatrices\H_c12c2_mean.mat');
load('ExtrinsicDataMatrices\H_c12c1_bar.mat');
%load('SavedMatrices\H_c12c1_bar_all.mat')
samples = numel(H_c12c1_bar);

H_c12c2_mu = H_c12c2_mean;

v_c12c2_cov = covSE(H_c12c1_bar,eye(4));
v_c12c2_mu = veeSE( logSE(meanSE(H_c12c1_bar)) );

for i=1:samples
    Xs_o(i,:) = veeSE(logSE(H_c12c1_bar{i}));
end

ALL_prcConfInterval_Sample = [];
ALL_prcConfInterval        = [];

for prcConfInterval = linspace(0.01,0.99,1000)

    % Define n-dimensional confidence interval
    efit = mvnpdfConfInterval(zeros(6,1),v_c12c2_cov,prcConfInterval);

    % Reference samples to mean and covariance
    Xs_e = invSO(efit.Rotation)*(Xs_o.' - efit.Center);

    % Find all points inside the confidence interval
    tfIn = ...
        sum(( Xs_e./repmat(efit.PrincipalRadii,1,samples) ).^2, 1) <= 1;

    prcConfInterval_Sample = nnz(tfIn)/numel(tfIn);

    fprintf('Confidence Interval: [USER DEFINED] %5.2f%% | %5.2f%% [OF SAMPLES]\n',...
        prcConfInterval*100,prcConfInterval_Sample*100);

    % Keep all of the info
    ALL_prcConfInterval(end+1)        = prcConfInterval;
    ALL_prcConfInterval_Sample(end+1) = prcConfInterval_Sample;
end
%% Calc Convex Hull Volume
efit = mvnpdfConfInterval(zeros(6,1),v_c12c2_cov,.99);
[K,vol_ex] = convhulln(Xs_o);
evol_ex = ellipsoidVolume(efit);
%% Fit ellipsoid
spherePoints = generateUnitHypersphere(1000, 3);
efitTest = mvnpdfConfInterval(zeros(3,1), v_c12c2_cov(4:6,4:6), .5);
%efitTest = mvnpdfConfInterval([mean(Xs_o(:,4)); mean(Xs_o(:,5)); mean(Xs_o(:,6))], v_c12c2_cov(4:6,4:6), .5);
Xs_o_3D = Xs_o.';
Xs_e = invSO(efitTest.Rotation)*(Xs_o_3D(4:6,:) - efitTest.Center);
[M,o] = hyperellipsoidfit(Xs_o_3D(4:6,:),[],[], 'forceOrigin', true);
points = bsxfun(@plus, (M(:,:,1)*spherePoints')', o(:,1)');
[~,fit_vol] = convhulln(points);
evol_ex = ellipsoidVolume(efitTest);
plotEllipsoid(efitTest);
hold on;
plot3(Xs_o(:,4), Xs_o(:,5), Xs_o(:,6), '*m');
plot3(points(:,1), points(:,2), points(:,3), 'ob');
%% Plot Extrinsics
figure;
axs = axes;
set(axs, 'NextPlot', 'Add');
theory = plot(axs,0,0,'b.');
samp = plot(axs,0,0,'r.');
set(theory, 'XData', linspace(1,99,1000), 'YData', ALL_prcConfInterval.*100, 'Visible', 'on');
set(samp, 'XData', linspace(1,99,1000), 'YData', ALL_prcConfInterval_Sample.*100, 'Visible', 'on');
legend('Theoretical Expected Values', 'Samples');
xlabel('Expected Samples Inside Hyper-Ellipsoid (%)');
ylabel('Observed Samples Inside Hyper-Ellipsoid (%)');
axs.FontSize = 30;
%% Intrinsics
clear all;
clc;
load('IntrinsicDataMatrices\A_c2m_in.mat')
load('SavedMatrices\960x1280_DFK 23UM0211\CameraParams.mat')
bubble{1} = params;
load('SavedMatrices\480x640_DFK 23U6181\CameraParams.mat')
bubble{2} = params;
clear params
params = bubble;
clear bubble
for i=1:2
    A_c2m_truth{i} = params{i}.Intrinsics.IntrinsicMatrix';
end

ALL_prcConfInterval_Sample = {[], []};
ALL_prcConfInterval       = {[], []};
for i=1:2
    [~, v_A_c2m_cov{i}, bad] = intrinsicStats(A_c2m_in(i,:),A_c2m_truth{i});
    A_c2m_mu{i} = A_c2m_truth{i};
    % Get mean and covariance parameters
    v_A_c2m_mu{i} = veeIntrinsics(A_c2m_mu{i});
    samples = numel(A_c2m_in(1,:));

    for j=1:samples
        if ~any(j == bad)
            Xs_o{i}(j,:) = veeIntrinsics(A_c2m_in{i,j});
        end
    end

    for j=length(bad):-1:1
        Xs_o{i}(bad(j),:) = [];
    end


    for prcConfInterval = linspace(0.01,0.99,1000)

        % Define n-dimensional confidence interval
        efit(i) = mvnpdfConfInterval(v_A_c2m_mu{i},v_A_c2m_cov{i},prcConfInterval);

        % Reference samples to mean and covariance
        Xs_e{i} = invSO(efit(i).Rotation)*(Xs_o{i}.' - efit(i).Center);

        % Find all points inside the confidence interval
        tfIn = ...
            sum(( Xs_e{i}./repmat(efit(i).PrincipalRadii,1,numel(Xs_o{i}(:,1))) ).^2, 1) <= 1;

        prcConfInterval_Sample = nnz(tfIn)/numel(tfIn);

        fprintf('Confidence Interval Camera %f: [USER DEFINED] %5.2f%% | %5.2f%% [OF SAMPLES]\n',...
            i,prcConfInterval*100,prcConfInterval_Sample*100);

        % Keep all of the info
        ALL_prcConfInterval{i}(end+1)        = prcConfInterval;
        ALL_prcConfInterval_Sample{i}(end+1) = prcConfInterval_Sample;
    end

end
%% Calc Convex Hull Volume
for i=1:2
    efit(i) = mvnpdfConfInterval(v_A_c2m_mu{i},v_A_c2m_cov{i},.99);
    [~,vol_in(i)] = convhulln(Xs_o{i});
    evol_in(i) = ellipsoidVolume(efit(i));
end
%% Plot Intrinsics
colors = 'rg';
figure;
axs = axes;
set(axs, 'NextPlot', 'Add');
theory = plot(axs,0,0,'b.');
set(theory, 'XData', linspace(1,99,1000), 'YData', ALL_prcConfInterval{1}.*100, 'Visible', 'on');
for i=1:2
    samp(i) = plot(axs,0,0,[colors(i),'.']);
    set(samp(i), 'XData', linspace(1,99,1000), 'YData', ALL_prcConfInterval_Sample{i}.*100, 'Visible', 'on');
end
legend('Theoretical Expected Values', 'Camera 1', 'Camera 2');
xlabel('Expected Samples Inside Hyper-Ellipsoid (%)');
ylabel('Observed Samples Inside Hyper-Ellipsoid (%)');
axs.FontSize = 30;
%% Segmentation
clear all;
clc;
load('SegmentationDataMatrices\p_cov.mat')
load('SegmentationDataMatrices\p_detected.mat')

ALL_prcConfInterval_Sample = {[], []};
ALL_prcConfInterval       = {[], []};
for i=1:2

    samples = numel(p_detected(1,:));

    for j=1:samples
        Xs_o{i}(j,:) = p_detected{i,j};
    end

    seg_cov{i} = cov(Xs_o{i});
    mean_x_seg(i) = mean(Xs_o{i}(:,1));
    mean_y_seg(i) = mean(Xs_o{i}(:,2));


    for prcConfInterval = linspace(0.01,0.99,1000)

        % Define n-dimensional confidence interval
        efit(i) = mvnpdfConfInterval([mean_x_seg(i) mean_y_seg(i)],seg_cov{i},prcConfInterval);

        % Reference samples to mean and covariance
        Xs_e{i} = invSO(efit(i).Rotation)*(Xs_o{i}.' - efit(i).Center);

        % Find all points inside the confidence interval
        tfIn = ...
            sum(( Xs_e{i}./repmat(efit(i).PrincipalRadii,1,samples) ).^2, 1) <= 1;

        prcConfInterval_Sample = nnz(tfIn)/numel(tfIn);

        fprintf('Confidence Interval Camera %f: [USER DEFINED] %5.2f%% | %5.2f%% [OF SAMPLES]\n',...
            i,prcConfInterval*100,prcConfInterval_Sample*100);

        % Keep all of the info
        ALL_prcConfInterval{i}(end+1)        = prcConfInterval;
        ALL_prcConfInterval_Sample{i}(end+1) = prcConfInterval_Sample;
    end

end
%% Calc Convex Hull Volume
for i=1:2
    efit(i) = mvnpdfConfInterval([mean_x_seg(i) mean_y_seg(i)],seg_cov{i},.99);
    [~,vol_seg(i)] = convhulln(Xs_o{i});
    evol_seg(i) = ellipsoidVolume(efit(i));
end

%% Fit ellipse
spherePoints = generateUnitHypersphere(1000, 2).';
for i=1:2
    [M{i},o{i}] = hyperellipsoidfit(Xs_e{i}.', 'auto');
    points{i} = (M{i}*spherePoints + o{i}).';
    [~,fit_vol(i)] = convhulln(points{i});
end

%% Plot Segmentation
colors = 'rg';
figure;
axs = axes;
set(axs, 'NextPlot', 'Add');
theory = plot(axs,0,0,'b.');
set(theory, 'XData', linspace(1,99,1000), 'YData', ALL_prcConfInterval{1}.*100, 'Visible', 'on');
for i=1:2
    samp(i) = plot(axs,0,0,[colors(i),'.']);
    set(samp(i), 'XData', linspace(1,99,1000), 'YData', ALL_prcConfInterval_Sample{i}.*100, 'Visible', 'on');
end
legend('Theoretical Expected Values', 'Camera 1', 'Camera 2');
xlabel('Expected Samples Inside Hyper-Ellipsoid (%)');
ylabel('Observed Samples Inside Hyper-Ellipsoid (%)');
axs.FontSize =30;