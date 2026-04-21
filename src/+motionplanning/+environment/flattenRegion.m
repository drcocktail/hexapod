function Z = flattenRegion(Z, X, Y, center, radius)
%FLATTENREGION Force a circular region to its minimum local height.

distSq = (X - center(1)) .^ 2 + (Y - center(2)) .^ 2;
mask = distSq < radius ^ 2;
if any(mask(:))
    flatHeight = min(Z(mask));
    Z(mask) = flatHeight;
end
end
