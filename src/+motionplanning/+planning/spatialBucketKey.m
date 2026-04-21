function [key, bucketX, bucketY] = spatialBucketKey(spatialIndex, state)
%SPATIALBUCKETKEY Convert an XY state to a spatial bucket key.

bucketX = floor((state(1) - spatialIndex.minX) / spatialIndex.cellSize);
bucketY = floor((state(2) - spatialIndex.minY) / spatialIndex.cellSize);
% Encode as a single double — avoids sprintf overhead with no collision risk
% for bucket coordinates in the expected [-50, 500] range.
key = bucketX * 10000 + bucketY;
end
