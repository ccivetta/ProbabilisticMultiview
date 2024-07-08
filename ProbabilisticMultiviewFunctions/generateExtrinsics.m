function H_g2c = generateExtrinsics(params)
%GENERATEEXTRINSICS takes a cameraParameters object and returns a cell
%array containing the H_g2c (checkerboard grid (g) to camera (c))
%
%   H_g2c = generateExtrinsics(params) takes the RotationMatrices and
%   TranslationVectors properties in the cameraParameters object and
%   returns the values as a 1xN cell array of 4x4 rigid body transforms
%   relating checkerboard grid (g) to camera (c)
%
%   Input(s)
%       params - cameraParameters object
%
%   Output(s)
%       H_g2c - 1xN cell array where N = params.NumPatterns
%           H_g2c{i} - ith extrinsic matrix defining the fiducial
%                      (checkerboard) frame relative to the camera frame
%
%   C. A. Civetta, M. Kutzer, 19OCT2023, USNA

N = params.NumPatterns;
H_g2c = cell(1,N);
for p=1:N
    bubble = [params.RotationMatrices(:,:,p)', params.TranslationVectors(p,:)'];
    bubble(4,1:4) = [0 0 0 1];
    H_g2c{p} = bubble;
end

end