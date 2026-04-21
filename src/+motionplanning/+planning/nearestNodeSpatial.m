function nearestIdx = nearestNodeSpatial(spatialIndex, nodes, nodeCount, queryState)
%NEARESTNODESPATIAL Find the exact nearest node by expanding spatial buckets.

if spatialIndex.buckets.Count == 0
    nearestIdx = motionplanning.planning.nearestNode(nodes, nodeCount, queryState);
    return;
end

[~, baseX, baseY] = motionplanning.planning.spatialBucketKey(spatialIndex, queryState);
bestDistSq = Inf;
nearestIdx = 1;
foundCandidate = false;

for ring = 0:spatialIndex.maxRing
    candidateIdx = ringCandidates(spatialIndex, baseX, baseY, ring);
    for idx = candidateIdx
        if idx > nodeCount
            continue;
        end
        distSq = sum((nodes(idx, :) - queryState) .^ 2);
        if distSq < bestDistSq
            bestDistSq = distSq;
            nearestIdx = idx;
            foundCandidate = true;
        end
    end

    if foundCandidate
        lowerBound = distanceToOutsideSearchWindow(spatialIndex, queryState, baseX, baseY, ring);
        if sqrt(bestDistSq) <= lowerBound
            return;
        end
    end
end

nearestIdx = motionplanning.planning.nearestNode(nodes, nodeCount, queryState);
end

function candidateIdx = ringCandidates(spatialIndex, baseX, baseY, ring)
candidateIdx = [];
if ring == 0
    candidateIdx = bucketValues(spatialIndex, baseX, baseY);
    return;
end

for bx = (baseX - ring):(baseX + ring)
    candidateIdx = [candidateIdx, bucketValues(spatialIndex, bx, baseY - ring)]; %#ok<AGROW>
    candidateIdx = [candidateIdx, bucketValues(spatialIndex, bx, baseY + ring)]; %#ok<AGROW>
end
for by = (baseY - ring + 1):(baseY + ring - 1)
    candidateIdx = [candidateIdx, bucketValues(spatialIndex, baseX - ring, by)]; %#ok<AGROW>
    candidateIdx = [candidateIdx, bucketValues(spatialIndex, baseX + ring, by)]; %#ok<AGROW>
end
end

function values = bucketValues(spatialIndex, bx, by)
key = sprintf('%d:%d', bx, by);
if isKey(spatialIndex.buckets, key)
    values = spatialIndex.buckets(key);
else
    values = [];
end
end

function lowerBound = distanceToOutsideSearchWindow(spatialIndex, queryState, baseX, baseY, ring)
cellSize = spatialIndex.cellSize;
xMin = spatialIndex.minX + (baseX - ring) * cellSize;
xMax = spatialIndex.minX + (baseX + ring + 1) * cellSize;
yMin = spatialIndex.minY + (baseY - ring) * cellSize;
yMax = spatialIndex.minY + (baseY + ring + 1) * cellSize;

lowerBound = min([ ...
    queryState(1) - xMin, ...
    xMax - queryState(1), ...
    queryState(2) - yMin, ...
    yMax - queryState(2)]);
lowerBound = max(lowerBound, 0);
end
