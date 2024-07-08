function [intrinsicParams] = veeIntrinsics(A_c2m)
%VEEINTRINSICS function to perform the vee operator on an intrinsic matrix
%   [intrinsicParams] = veeIntrinsics(A_c2m) performs the vee operator 
%   on an intrinsic matrix and returns a 1x5 array containing significant
%   intrinsic parameters
%
%  Input(s)
%       A_c2m           - 3x3 intrinsic array
%
%   Output(s)
%       intrinsicParams - 1x5 array of intrinsic parameters following the
%                         format: [fx fy s x0 y0] where
%                           fx - x component of the focal length
%                           fy - y component of the focal length
%                           s  - skew
%                           x0 - x component of the principal point
%                           y0 - y component of the principal point
%
%  See also wedgeIntrinsics
%
%  C. A. Civetta, 25Jun2024, USNA


fx = A_c2m(1,1);
fy = A_c2m(2,2);
s = A_c2m(1,2);
x0 = A_c2m(1,3);
y0 = A_c2m(2,3);
intrinsicParams = [fx fy s x0 y0].';

end