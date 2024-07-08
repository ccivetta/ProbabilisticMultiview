function sigmaX = covGivenMean(X,muX)
% COVGIVENMEAN calculates the covariance of a sample given 
% known sample mean.
%   sigmaX = covGivenMean(X,muX)
%
%   Input(s)
%         X - MxN array defining N-dimensional samples
%       muX - 1xN array defining known, N-dimensional mean
%
%   Output(s)
%       sigmaX - NxN array defining covariance of samples 
%                about the given mean
%
%   References
%       [1] Covariance, Wolfram MathWorld, 22Mar2024
%           https://mathworld.wolfram.com/Covariance.html
%
%   M. Kutzer & C. Civetta, 22Mar2024, USNA

%% Check input(s)
narginchk(2,2);

if ~ismatrix(X) || ~ismatrix(muX)
    error('Inputs must be 2-dimensional arrays.');
end

[M,N] = size(X);
if size(muX,1) ~= 1 || size(muX,2) ~= N
    error('Mean must be a 1x%d array.',N);
end

%% Calculate covariance
sigmaX = zeros(N,N);
for i = 1:N
    for j = 1:N
        sigmaX(i,j) = ...
            sum( (X(:,i) - muX(i)).*(X(:,j) - muX(j)) )./(M-1);
    end
end