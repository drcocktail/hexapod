function inside = isInsideDomain(x, y, terrain)
%ISINSIDEDOMAIN Return true when every queried point is inside the map.

inside = all(x(:) >= terrain.minX & x(:) <= terrain.maxX & ...
    y(:) >= terrain.minY & y(:) <= terrain.maxY);
end
