function [z_ca,z_cb] = recoverDepth(p_ma,p_mb,A_ca2ma,A_cb2mb,H_cb2ca)
%RECOVERDEPTH returns the depth of an object relative to both cameras
% 
% [z_ca,z_cb] = recoverDepth(p_ma,p_mb,A_ca2ma,A_cb2mb,H_cb2ca) calculates
% the depth of an object relative to each of two cameras, given the
% pixel coordinates of the object in corresponding images from each camera,
% the intrinsics of each camera, and the extrinsics relating one camera to
% the other
%
% Input(s)
%   p_ma    - a 3x1 matrix containing the objects pixel coordinates in the 
%             camera 1 matrix frame in the form [x; y; 1]
%   p_mb    - a 3x1 matrix containing the objects pixel coordinates in the 
%             camera 2 matrix frame in the form [x; y; 1]
%   A_ca2ma - 3x3 camera 1 intrinsic matrix
%   A_cb2mb - 3x3 camera 2 intrinsic matrix
%   H_cb2ca - 4x4 rigid body transform relating camera 2 to camera 1
%
%
%
% Output(s)
%   z_ca    - the object's depth from camera 1
%   z_cb    - the object's depth from camera 2
%
% C. A. Civetta, M. Kutzer, 25Jun24, USNA

%% Define scaled point coordinates relative to camera frame
tilde_p_ca = (A_ca2ma^(-1))*p_ma;
tilde_p_cb = (A_cb2mb^(-1))*p_mb;

%% Isolate rotation & translation of extrinsics
R_cb2ca = H_cb2ca(1:3,1:3);
d_cb2ca = H_cb2ca(1:3,4);

%% Calcualte depth
z_c_ALL = pinv( [tilde_p_ca, -R_cb2ca*tilde_p_cb] )*d_cb2ca;

%% Isolate depth parameters
z_ca = z_c_ALL(1,:);
z_cb = z_c_ALL(2,:);

end