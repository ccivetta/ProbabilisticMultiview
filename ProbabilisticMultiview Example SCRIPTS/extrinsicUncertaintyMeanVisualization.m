%% Plot Extrinsic Uncertainty Mean Visualization

load('ExtrinsicDataMatrices\H_c12c2_ex.mat');
load('SavedMatrices\H_c12c2_MoCap.mat');
load('SavedMatrices\H_c12c2_mean.mat')
load('ExtrinsicDataMatrices\H_c12c1_bar.mat')

H_c12c2_mu = meanSE(H_c12c2_ex);
v_c12c2_cov = covSE(H_c12c2_ex);
v_c12c2_mu = veeSE( logSE( H_c12c2_mu) );

axs = axes;
view(3);
daspect([1 1 1]);
for i=1:numel(H_c12c2_ex)
    h{i} = triad('Parent',axs,'Scale',2,'LineWidth',2,'Matrix', H_c12c2_ex{i}, 'linestyle', ':');
end


h_cal = triad('Parent',axs,'Scale',5,'LineWidth',2,'Matrix', H_c12c2_mean, 'axislabels', {"{x_{calibration}}", "{y_{calibration}}", "{z_{calibration}}"});
h_truth = triad('Parent',axs,'Scale',5,'LineWidth',2,'Matrix', H_c12c2_MoCap, 'axislabels', {"{x_{MoCap}}", "{y_{MoCap}}", "{z_{MoCap}}"});
h_mu = triad('Parent',axs,'Scale',5,'LineWidth',2,'Matrix', H_c12c2_mu, 'axislabels', {"{x_{mean}}", "{y_{mean}}", "{z_{mean}}"});

xlabel("x (mm)");
ylabel("y (mm)");
zlabel("z (mm)");
h_mu.Children(1).FontSize = 20;
h_mu.Children(2).FontSize = 20;
h_mu.Children(3).FontSize = 20;
h_cal.Children(1).FontSize = 20;
h_cal.Children(2).FontSize = 20;
h_cal.Children(3).FontSize = 20;
h_truth.Children(1).FontSize = 20;
h_truth.Children(2).FontSize = 20;
h_truth.Children(3).FontSize = 20;
axs.FontSize = 25;

%% H_c12c1_bar 

%load('G:\Shared drives\VIPER Lab\Code, Trident Civetta\ExtrinsicDataMatrices\H_c12c1_bar.mat')
axs = axes;
view(3);
daspect([1 1 1]);
for i=1:numel(H_c12c1_bar)
    h{i} = triad('Parent',axs,'Scale',2,'LineWidth',2,'Matrix', H_c12c1_bar{i}, 'linestyle', ':');
end
h_mu = triad('Parent',axs,'Scale',5,'LineWidth',2,'Matrix', eye(4), 'axislabels', {"{x_{0}}", "{y_{0}}", "{z_{0}}"}, 'FontSize', 20);