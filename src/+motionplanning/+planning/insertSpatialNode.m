function spatialIndex = insertSpatialNode(spatialIndex, nodeIdx, nodeState)
%INSERTSPATIALNODE Add a node index to the spatial bucket map.

[key, ~, ~] = motionplanning.planning.spatialBucketKey(spatialIndex, nodeState);
if isKey(spatialIndex.buckets, key)
    bucket = spatialIndex.buckets(key);
    bucket(end + 1) = nodeIdx;
else
    bucket = nodeIdx;
end
spatialIndex.buckets(key) = bucket;
end
