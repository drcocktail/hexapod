function [path, pathZ] = reconstructPath(nodes, nodesZ, parents, goalIdx)
%RECONSTRUCTPATH Backtrack parent indices into an ordered path.

% Count path length first (O(n) pass 1)
count = 0;
current = goalIdx;
while current > 0
    count = count + 1;
    current = parents(current);
end

% Fill backward (O(n) pass 2) — no array resizing
indices = zeros(count, 1);
current = goalIdx;
for k = count:-1:1
    indices(k) = current;
    current = parents(current);
end

path = nodes(indices, :);
pathZ = nodesZ(indices);
end

