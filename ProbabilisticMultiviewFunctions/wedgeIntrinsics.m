function [A_c2m] = wedgeIntrinsics(intrinsicParams)
%WEDGEINTRINSICS function to perform the wedge operator on an intrinsic matrix
%   [A_c2m] = wedgeIntrinsics(intrinsicParams) performs the wedge operator 
%   on an intrinsic parameters and returns a properly formatted intrinsic
%   matrix
%
%   Input(s)
%       intrinsicParams - 1x5 array of intrinsic parameters following the
%                         format: [fx fy s x0 y0] where
%                           fx - x component of the focal length
%                           fy - y component of the focal length
%                           s  - skew
%                           x0 - x component of the principal point
%                           y0 - y component of the principal point
%
%  Output(s)
%       A_c2m           - 3x3 intrinsic array
%
%  See also veeIntrinsics
%
%  C. A. Civetta, 25Jun2024, USNA

fx = intrinsicParams(1);
fy = intrinsicParams(2);
s = intrinsicParams(3);
x0 = intrinsicParams(4);
y0 = intrinsicParams(5);

A_c2m = zeros(3,3);
A_c2m(3,3) = 1;
A_c2m(1,1) = fx; 
A_c2m(2,2) = fy; 
A_c2m(1,2) = s;
A_c2m(1,3) = x0;
A_c2m(2,3) = y0;
end