%% Test_randSE_mu_cov
%
% This script was used to test how to properly generate extrinsic samples using
% mvnrnd(). This script assumes the workspace contains a cell array of
% H_c12c2 samples (camera 1 relative to camera 2) named H_c12c2_im
%
% Two test cases are used:
%       1 - using the sample mean and covariance 
%       2 - using zero mean and covariance 
%
% The resulting random samples are plotted with the original data to see
% which samples best fit the data 
%
% It was concluded that generating random samples around a zero mean is the
% correct way to do it 
%
% C. A. Civetta, M. Kutzer, 26Jun2024, USNA

close all
%% Calculate statistics
H_c12c2_mu = meanSE(H_c12c2_im);
v_c12c2_cov = covSE(H_c12c2_im);

v_c12c2_mu = veeSE( logSE( H_c12c2_mu) );

%% Get data from samples
for i = 1:numel(H_c12c2_im)
    X_c12c2_tst(:,i) = H_c12c2_im{i}(1:3,4);                       % Position of origin (translation)
    r_c12c2_tst(:,i) = veeSO( logSO( H_c12c2_im{i}(1:3,1:3) ) );   % Orientation of frame using exponential
end
%% Test case 1 - Using the mean and covariance
n = 1000;
v_c12c2_smp1 = mvnrnd(v_c12c2_mu.',v_c12c2_cov,n);

for i = 1:n
    H_c12c2_smp1{i} = expSE( wedgeSE( v_c12c2_smp1(i,:)) );
    X_c12c2_smp1(:,i) = H_c12c2_smp1{i}(1:3,4);                       % Position of origin (translation)
    r_c12c2_smp1(:,i) = veeSO( logSO( H_c12c2_smp1{i}(1:3,1:3) ) );   % Orientation of frame using exponential
end

%% Test case 2 - Using zero mean and covariance
n = 1000;
v_c12c2_smp2 = mvnrnd(zeros(1,6),v_c12c2_cov,n);

for i = 1:n
    H_c12c2_smp2{i} = H_c12c2_mu*expSE( wedgeSE( v_c12c2_smp2(i,:) ) );
    X_c12c2_smp2(:,i) = H_c12c2_smp2{i}(1:3,4);                       % Position of origin (translation)
    r_c12c2_smp2(:,i) = veeSO( logSO( H_c12c2_smp2{i}(1:3,1:3) ) );   % Orientation of frame using exponential
end

%% Plot results
fig = figure('Name','Approximate Distribution');
axs(1) = subplot(1,2,1);
axs(2) = subplot(1,2,2);

xlabel(axs(1),'x (mm)');
ylabel(axs(1),'y (mm)');
zlabel(axs(1),'z (mm)');

xlabel(axs(2),'k_1 (rad)');
ylabel(axs(2),'k_2 (rad)');
zlabel(axs(2),'k_3 (rad)');

view(axs(1),3);
view(axs(2),3);
set(axs,'NextPlot','add');

plt_X = plot3(axs(1),X_c12c2_tst(1,:),X_c12c2_tst(2,:),X_c12c2_tst(3,:),'+g','MarkerSize',6,'LineWidth',1.5);
plt_X1 = plot3(axs(1),X_c12c2_smp1(1,:),X_c12c2_smp1(2,:),X_c12c2_smp1(3,:),'.r');
plt_X2 = plot3(axs(1),X_c12c2_smp2(1,:),X_c12c2_smp2(2,:),X_c12c2_smp2(3,:),'.b');

plt_r = plot3(axs(2),r_c12c2_tst(1,:),r_c12c2_tst(2,:),r_c12c2_tst(3,:),'+g','MarkerSize',6,'LineWidth',1.5);
plt_r1 = plot3(axs(2),r_c12c2_smp1(1,:),r_c12c2_smp1(2,:),r_c12c2_smp1(3,:),'.r');
plt_r2 = plot3(axs(2),r_c12c2_smp2(1,:),r_c12c2_smp2(2,:),r_c12c2_smp2(3,:),'.b');