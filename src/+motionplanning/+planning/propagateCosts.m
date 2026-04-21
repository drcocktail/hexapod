function costs = propagateCosts(parents, edgeCosts, costs, rootIdx, nodeCount)
%PROPAGATECOSTS Refresh descendants after a tree rewire.

% Pre-allocate a fixed queue with a head pointer so neither pop (queue(1)=[])
% nor push (end+1) resizes the underlying array during the BFS.
queue    = zeros(nodeCount, 1);
queue(1) = rootIdx;
head     = 1;
tail     = 1;

pSubset = parents(1:nodeCount);

while head <= tail
    current = queue(head);
    head    = head + 1;
    children = find(pSubset == current);
    for child = children(:)'
        costs(child) = costs(current) + edgeCosts(child);
        tail         = tail + 1;
        queue(tail)  = child;
    end
end
end
