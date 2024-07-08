function X_o3D = confIntervalPoints3D(meanx, meany, meanz, cov3D, CI)
%CONFINTERVALPOINTS3D Summary of this function goes here
%   Detailed explanation goes here
    efit3D = mvnpdfConfInterval([meanx meany meanz],cov3D,CI);

    % Calculate the points of the ellipsoid
    ellipsoidPatch = plotEllipsoid(~,efit3D);
    set(ellipsoidPatch, 'Visible', 'off');
    X_o3D(1,:) = ellipsoidPatch.XData(:).';
    X_o3D(2,:) = ellipsoidPatch.YData(:).';
    X_o3D(3,:) = ellipsoidPatch.ZData(:).';
    X_o3D(4,:) = 1;
end

