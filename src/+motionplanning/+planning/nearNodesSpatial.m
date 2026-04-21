function nodeIdx = nearNodesSpatial(spatialIndex, nodes, nodeCount, queryState, radius)
%NEARNODESSPATIAL Exact radius query using spatial buckets.

[~, baseX, baseY] = motionplanning.planning.spatialBucketKey(spatialIndex, queryState);
ring = ceil(radius / spatialIndex.cellSize) + 1;
candidateIdx = [];

for bx = (baseX - ring):(baseX + ring)
    for by = (baseY - ring):(baseY + ring)
        key = bx * 10000 + by;
        if isKey(spatialIndex.buckets, key)
            candidateIdx = [candidateIdx, spatialIndex.buckets(key)]; %#ok<AGROW>
        end
    end
end

candidateIdx = unique(candidateIdx);
candidateIdx = candidateIdx(candidateIdx <= nodeCount);
if isempty(candidateIdx)
    nodeIdx = [];
    return;
end

distSq   = sum((nodes(candidateIdx, :) - queryState) .^ 2, 2);
nodeIdx  = candidateIdx(distSq <= radius ^ 2);
nodeIdx  = nodeIdx(:);
end
