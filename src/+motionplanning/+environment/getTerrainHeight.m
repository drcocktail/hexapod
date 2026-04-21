function z = getTerrainHeight(x, y, terrain, boundsMode)
%GETTERRAINHEIGHT Query terrain height with explicit boundary behavior.

if nargin < 4
    boundsMode = 'clamp';
end

inside = x >= terrain.minX & x <= terrain.maxX & y >= terrain.minY & y <= terrain.maxY;
xQuery = motionplanning.utils.clamp(x, terrain.minX, terrain.maxX);
yQuery = motionplanning.utils.clamp(y, terrain.minY, terrain.maxY);

z = interp2(terrain.X, terrain.Y, terrain.Z, xQuery, yQuery, 'nearest', 0);
if strcmpi(boundsMode, 'nan')
    z(~inside) = NaN;
elseif ~strcmpi(boundsMode, 'clamp')
    error('motionplanning:environment:InvalidBoundsMode', ...
        'Bounds mode must be either "clamp" or "nan".');
end
end
