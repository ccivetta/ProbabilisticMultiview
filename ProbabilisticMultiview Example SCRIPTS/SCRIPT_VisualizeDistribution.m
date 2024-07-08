%% SCRIPT_VisualizeDistribution
% This is an old script used as a proof of concept for how to plot 
% elipses and ellipsoids from covaraince and mean 
%
%   C. A. Civetta, M. Kutzer, 02Jul2024

%% Get Eigenvalues and Eigenvectors
[V,D] = eig(p_cov{2});
x_hat = V(:,1)./norm(V(:,1));
y_hat = nCross(x_hat);

r1 = D(1,1); % this is a covariance-ish value
r2 = D(2,2); %

R_e2o = [x_hat,y_hat];

H_e2o = eye(3);
H_e2o(1:2,1:2) = R_e2o;

%% Plot the ellipse
s = linspace(0,2*pi,1000);
X_e(1,:) = r1*cos(s);
X_e(2,:) = r2*sin(s);
X_e(3,:) = 1;

X_o = H_e2o*X_e;

figure
plot(X_o(1,:),X_o(2,:));
daspect([1 1 1]);

%% APPLIED TO WORK
[V,D] = eig(cov3D(1:2, 1:2));
x_hat = V(:,1)./norm(V(:,1));
y_hat = nCross(x_hat);

r1 = D(1,1); % this is a covariance-ish value
r2 = D(2,2); %

R_e2o = [x_hat,y_hat];

H_e2o = eye(3);
H_e2o(1:2,1:2) = R_e2o;

s = linspace(0,2*pi,1000);
X_e(1,:) = r1*cos(s) - meanDistx;
X_e(2,:) = r2*sin(s) - meanDisty;
X_e(3,:) = 1;

X_o = H_e2o*X_e;

figure
plot(X_o(1,:),X_o(2,:));
daspect([1 1 1]);

%% 3D
% Calculate eigenvalues and eigenvectors of A
[V,D] = eig(cov3D);

r1 = D(1,1); % this is a covariance-ish value
r2 = D(2,2);
r3 = D(3,3);

x_hat = V(:,1)./norm(V(:,1));
y_hat = V(:,2)./norm(V(:,2));
z_hat = V(:,3)./norm(V(:,3));

R_e2o = [x_hat,y_hat, z_hat];

H_e2o = eye(4);
H_e2o(1:3,1:3) = R_e2o;

theta = linspace(0, 2*pi, 10);
phi = linspace(0, pi, 10);

% Calculate the points of the ellipsoid
[thetaGrid, phiGrid] = meshgrid(theta, phi);
x = r1 * sin(phiGrid) .* cos(thetaGrid);
y = r2 * sin(phiGrid) .* sin(thetaGrid);
z = r3 * cos(phiGrid);
X_e(1:3,:) = [reshape(x, 1, []); reshape(y, 1, []); reshape(z, 1, [])];
X_e(4,:) = 1;
X_o = H_e2o*X_e;
figure
plot3(X_o(1,:),X_o(2,:), X_o(3,:), 'k-');
daspect([1 1 1]);