function nodeIdx = nearNodes(nodes, nodeCount, queryState, radius)
%NEARNODES Brute-force radius query over active nodes.

distSq = sum((nodes(1:nodeCount, :) - queryState) .^ 2, 2);
nodeIdx = find(distSq <= radius ^ 2);
end
