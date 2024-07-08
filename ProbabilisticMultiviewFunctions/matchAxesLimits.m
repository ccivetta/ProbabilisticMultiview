function matchAxesLimits(axs)
% MATCHAXESLIMITS matches the limits on an array of axes handles
%
%   matchAxesLimits(axs) takes an array of axes handles and mathes all of
%   their limits
%
%   C. A. Civetta, M. Kutzer, 12Oct2023, USNA

%% Check input(s)
% TODO - check if handles are valid and axes

%% Get combine limits
n = numel(axs);
xx = zeros(n,2);
yy = zeros(n,2);
zz = zeros(n,2);

for i = 1:n
    xx(i,:) = xlim(axs(i));
    yy(i,:) = ylim(axs(i));
    zz(i,:) = zlim(axs(i));
end

% Define max limits
x_lim(1,1) = min( xx(:,1) );
x_lim(1,2) = max( xx(:,2) );

y_lim(1,1) = min( yy(:,1) );
y_lim(1,2) = max( yy(:,2) );

z_lim(1,1) = min( zz(:,1) );
z_lim(1,2) = max( zz(:,2) );

%% Set axes limits
for i = 1:n
    xlim(axs(i),x_lim);
    ylim(axs(i),y_lim);
    zlim(axs(i),z_lim);
end