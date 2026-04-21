function [key, bucketX, bucketY] = spatialBucketKey(spatialIndex, state)
%SPATIALBUCKETKEY Convert an XY state to a stable spatial bucket key.

bucketX = floor((state(1) - spatialIndex.minX) / spatialIndex.cellSize);
bucketY = floor((state(2) - spatialIndex.minY) / spatialIndex.cellSize);
key = sprintf('%d:%d', bucketX, bucketY);
end
