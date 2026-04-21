function [idxX, idxY] = localGridWindow(x, y, radius, terrain)
%LOCALGRIDWINDOW Return numeric grid index ranges around a query circle.

if isfield(terrain, 'xVec')
    xCount = numel(terrain.xVec);
else
    xCount = size(terrain.X, 2);
end
if isfield(terrain, 'yVec')
    yCount = numel(terrain.yVec);
else
    yCount = size(terrain.Y, 1);
end

spacing = terrain.gridSpacing;
colMin = max(1, floor((x - radius - terrain.minX) / spacing) + 1);
colMax = min(xCount, ceil((x + radius - terrain.minX) / spacing) + 1);
rowMin = max(1, floor((y - radius - terrain.minY) / spacing) + 1);
rowMax = min(yCount, ceil((y + radius - terrain.minY) / spacing) + 1);

idxX = colMin:colMax;
idxY = rowMin:rowMax;
end
