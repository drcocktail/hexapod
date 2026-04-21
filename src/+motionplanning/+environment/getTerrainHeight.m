function z = getTerrainHeight(x, y, terrain, boundsMode)
%GETTERRAINHEIGHT Query terrain height with explicit boundary behavior.

if nargin < 4
    boundsMode = 'clamp';
end

inside = x >= terrain.minX & x <= terrain.maxX & y >= terrain.minY & y <= terrain.maxY;

% Direct nearest-neighbor index arithmetic — equivalent to interp2(...,'nearest')
% on a uniform grid but avoids all interp2 overhead for scalar and array inputs.
nRows = size(terrain.Z, 1);
nCols = size(terrain.Z, 2);
gs    = terrain.gridSpacing;
colIdx = min(max(round((x - terrain.minX) / gs) + 1, 1), nCols);
rowIdx = min(max(round((y - terrain.minY) / gs) + 1, 1), nRows);

if isscalar(x)
    z = terrain.Z(rowIdx, colIdx);
else
    z = terrain.Z(sub2ind([nRows, nCols], rowIdx(:), colIdx(:)));
    z = reshape(z, size(x));
end

if strcmpi(boundsMode, 'nan')
    z(~inside) = NaN;
elseif ~strcmpi(boundsMode, 'clamp')
    error('motionplanning:environment:InvalidBoundsMode', ...
        'Bounds mode must be either "clamp" or "nan".');
end
end
