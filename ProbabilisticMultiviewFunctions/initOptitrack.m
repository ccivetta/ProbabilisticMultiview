function [v] = initOptitrack(varargin)
%INITOPTITRACK connects to the OptiTrack system and returns a structure of
%VRPN_ROS objects given assigned motive names 
%   [v] = initOptitrack({variableName, motiveAssetName}, ...) connects Matlab to
%   the motioncapture system and populats a structure with VRPN_ROS objects
%   tracking given motion capture assets. Assets are given in 1x2 arrays
%   containing the desired varialbe name in the struct, and the assigned
%   asset name in Motive
%       Example: v = initOptitrack({'Bino', 'BinocularCameras'});
%
%   Input(s)
%       {variableName, motiveAssetName} - variableName is a character array
%                                           containing the chosen name for the
%                                           variable in the structure 
%                                       -  motiveAssetName is a character array
%                                           containing the exact name assigned to
%                                           the asset in Motive
%
%   Outpit(s)
%       v - struct containing VRPN_ROS objects under the name given as the
%       argument variableName
%
% C. A. Civetta, M. Kutzer, 26Jun2024, USNA


disp("Setting Up Motion Capture Assets")

rosshutdown
rosinit('10.0.0.31');
pause(1)
for i = 1:nargin
    v.(varargin{i}{1}) = VRPN_ROS(varargin{i}{2});  
end

end