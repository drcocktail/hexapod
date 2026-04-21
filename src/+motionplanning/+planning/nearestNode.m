function nearestIdx = nearestNode(nodes, nodeCount, queryState)
%NEARESTNODE Return the index of the nearest node in squared Euclidean XY.

distances = sum((nodes(1:nodeCount, :) - queryState) .^ 2, 2);
[~, nearestIdx] = min(distances);
end
