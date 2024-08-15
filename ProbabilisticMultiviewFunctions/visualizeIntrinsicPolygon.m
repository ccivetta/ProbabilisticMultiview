function [intrinsicPoly,axs] = visualizeIntrinsicPolygon(varargin)
%VISUALIZEINTRINSICPOLYGON is used to create a visualization for a set of
%intrinsic matrices 
%   visualizeIntrinsicPolygon(A_c2m)
%   visualizeIntrinsicPolygon(axs,A_c2m)
%
%   axs - can be any parent (e.g. axis or hgtransform)
%
%   Each intrinsic matrix is visualized as a polygon with vertices at
%   (x0,y0), (s+x0, fy+y0), (fx+s+x0, fy+y0), (fx+x0, y0)
%
%   The resulting polygon allows principle point to be visualized as
%   location on the axis, focal length to be visualized as height and
%   width, and skew to be visualized as angle between the otherwise
%   perpendicular polygon sides. 
%
%   Input(s): 
%       A_c2m: a cell array of intrinsic matrices 
%
%   Output(s): 
%       intrinsicPoly: a cell array of Polygon objects for each given array
%
%   C. Civetta, M. Kutzer, 18Jan2024, USNA


%% Parse input(s)
axs = [];
narginchk(1,2);
if nargin == 1
    A_c2m = varargin{1};
else
    for i = 1:nargin
        if ishandle(varargin{i})
            axs = varargin{i};
        else
            A_c2m = varargin{i};
        end
    end
end

if isempty(axs)
    axs = gca;
end
hold(axs,'on');
daspect(axs,[1 1 1]);

% Account for non-cell A_c2m
if ~iscell(A_c2m)
    A_c2m = {A_c2m};
end

%% Create Polygons
for i = 1:numel(A_c2m)
    fx = A_c2m{i}(1,1);
    sx = A_c2m{i}(1,2);
    fy = A_c2m{i}(2,2);
    x0 = A_c2m{i}(1,3);
    y0 = A_c2m{i}(2,3);

    xx{i} = [x0, x0+sx, x0+fx+sx, x0+fx].';
    yy{i} = [y0, y0+fy, y0+fy, y0].';
    a(i) = polyarea(xx{i},yy{i});
end



for i = 1:numel(A_c2m)
    [~,indexOfMin] = min(a);
    vertices{i} = [xx{indexOfMin},yy{indexOfMin}];
    intrinsicPoly{i} = plot(polyshape(vertices{i}),...
        'Parent',axs,'FaceAlpha',0.75,'EdgeColor','k');
    a(indexOfMin) = [];
end

end