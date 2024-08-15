function [V] = ellipsoidVolume(efit)
%ellipsoidVolume calculates the volume of an n-dimensional hyperellipsoid
%   V = ellipsoidVolume(efit) takes an efit and returns the volume of the
%   ellipse, ellipsoid, or hyperellipsoid. 
%
%   Input(s):
%       efit - efit object of an n-dimensional hyperellipsoid
%
%   Output(s):
%       V - hypervolume of the given hyperellipsoid
%
%   C. A. Civetta, 16Jul2024, USNA

n = length(efit.PrincipalRadii);
nHold = (n/2);

V = (2/n)*((pi)^(n/2))/(gamma(nHold)) * prod(efit.PrincipalRadii);

end