function costs = propagateCosts(parents, edgeCosts, costs, rootIdx, nodeCount)
%PROPAGATECOSTS Refresh descendants after a tree rewire.

queue = rootIdx;
while ~isempty(queue)
    current = queue(1);
    queue(1) = [];

    children = find(parents(1:nodeCount) == current);
    for child = children(:)'
        costs(child) = costs(current) + edgeCosts(child);
        queue(end + 1) = child; %#ok<AGROW>
    end
end
end
