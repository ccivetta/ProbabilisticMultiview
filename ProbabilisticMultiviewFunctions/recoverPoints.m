function p_a = recoverPoints(p_m, z_c, P_a2m)
%RECOVERPOINTS returns the 3D position of an object relative to a camera
%frame
% 
% p_a = recoverPoints(p_m, z_c, P_a2m) calculated the homogeneous 3D
% coordinates of an object relative to a camera frame, given the objects 
% pixel coordinates, depth from the camera, and projection matrix
%
% Input(s)
%   p_m   - a 3x1 matrix containing the objects pixel coordinates in the form
%           [x; y; 1]
%   z_c   - the object's depth from the camera
%   P_a2m - 3x4 projection matrix
%
% Output(s)
%   p_a   - 4x1 homogeneous array of the object coordinates relative to the
%   camera [x; y; z; 1]
%
% C. A. Civetta, M. Kutzer, 25Jun24, USNA


%% Isolate "rotation" and "translation" portions of projection matrix
Q_a2m = P_a2m(1:3,1:3);
V_a2m = P_a2m(1:3,4);

%% Recover 3D point coordinate(s) & make homogeneous
p_a = (Q_a2m^(-1))*(z_c.*p_m - V_a2m);
p_a(4,:) = 1;
end