function X = confIntervalPoints3D(mean, cov3D, CI)
%CONFINTERVALPOINTS3D takes three dimensional mean and covariance, and a
%covidence interval and returns the 3D points associated with the surface
%of the fit ellipsoid
%   X = confIntervalPoints3D(mean, cov3D, CI) takes a three dimensional
%   mean, covariance, and a confidnce interval and returns the 3D points of
%   the ellipsoid's surfase that cointains the specificed confidence
%   interval. 
%
%   Input(s)
%       mean  - 1x3 array of mean values for x, y, and z [x y z] 
%       cov3D - 3x3 covariance matrix for the variables x, y, and z
%       CI    - a value between 0 and 1 noting the specified confidence
%               interval. ex. CI = .25 generates an ellipsoid that contains 25% of
%               the sample data
%
%   Output(s)
%       X - 4xN array containing points on the ellipsoid surface, where
%           each column is a point of the form |x|
%                                              |y|
%                                              |z|
%                                              |1|
%   C. A. Civetta, M. Kutzer, 02Jul2024, USNA

efit3D = mvnpdfConfInterval(mean,cov3D,CI);


[x,y,z] = ellipsoid(0,0,0,efit3D.PrincipalRadii(1),efit3D.PrincipalRadii(2),efit3D.PrincipalRadii(3),100);
X = [reshape(x,1,[]); reshape(y,1,[]); reshape(z,1,[])];
X(4,:) = 1;

H = eye(4);
H(1:3,1:3) = efit3D.Rotation;
H(1:3,4) = efit3D.Center;

X = H*X;
end

