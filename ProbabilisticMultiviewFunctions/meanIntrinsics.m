function [A_c2m_mean, v_A_c2m_cov, bad] = meanIntrinsics(A_c2m,A_c2m_mean)
%MEANINTRINSICS takes a cell array of intrinsic matrices and returns a
%single mean matrix and associated covariance after removing outliers
%
%   This function has been renamed instrinsicStats
%
%   [A_c2m_mean, v_A_c2m_cov, bad] = meanIntrinsics(A_c2m,A_c2m_mean) calculates a
%   mean intrinsic matrix and associated covariance from the given 1xN cell
%   array of intrinsic matrices. Any matrix deemed to be an outlier is 
%   removed from the set before statistics are calculated. Indices containing
%   the removed matrices are returned as an array.
%
%   Outliers are defined as any matrix with either fx, fy, s, x0, or y0 
%   greater than three scaled medial absolute deviations from the median
%
%   [A_c2m_mean, v_A_c2m_cov, bad] = intrinsicStats(A_c2m,A_c2m_mean)
%
%   Input(s)
%       A_c2m       - 1xN cell array of 3x3 intrinsic matrices 
%       A_c2m_mean  - [Optional] a 3x3 intrinsic matrix defining the mean of
%                     the data set 
%
%   Output(s) 
%       A_c2m_mean  - 3x3 mean intrinsic matrix of the data set, calculated if not given 
%       v_A_c2m_cov - 5x5 covariance matrix for intrinsic parameters 
%       bad         - 1xN array containing the indices of intrinsic
%                     matrices deemed to be outliers 
%
%   C. A. Civetta, M. Kutzer, 25Jun2024, USNA
for i=1:numel(A_c2m)
    v_A_c2m{i} = veeIntrinsics(A_c2m{i});
    fx(i) = v_A_c2m{i}(1);
    fy(i) = v_A_c2m{i}(2);
    s(i) = v_A_c2m{i}(3);
    x0(i) = v_A_c2m{i}(4);
    y0(i) = v_A_c2m{i}(5);
end


% Remove Outliers
[fx_new,logicfx] = rmoutliers(fx);
[fy_new,logicfy] = rmoutliers(fy);
[s_new,logics] = rmoutliers(s);
[x0_new,logicx0] = rmoutliers(x0);
[y0_new,logicy0] = rmoutliers(y0);

%Save all indicies at which an outlier was removed
bad = [];
for i = 1:numel(A_c2m)
    if logicfx(i) == 1 || logicfy(i) == 1 || logics(i) == 1 || logicx0(i) == 1 || logicy0(i) == 1
        bad = [bad i];
    end
end
bad = unique(bad);

%Remove outlier data
for i=length(bad):-1:1
    fx(bad(i)) = [];
    fy(bad(i)) = [];
    s(bad(i)) = [];
    x0(bad(i)) = [];
    y0(bad(i)) = [];
end

if nargin < 2
    v_A_c2m_mean = [mean(fx) mean(fy) mean(s) mean(x0) mean(y0)].';
    A_c2m_mean = wedgeIntrinsics(v_A_c2m_mean);
else
    v_A_c2m_mean = veeIntrinsics(A_c2m_mean);
end

v_A_c2m_matrix = [fx.' fy.' s.' x0.' y0.'];
v_A_c2m_cov = covGivenMean(v_A_c2m_matrix,v_A_c2m_mean.');
end