function [path, pathZ] = reconstructPath(nodes, nodesZ, parents, goalIdx)
%RECONSTRUCTPATH Backtrack parent indices into an ordered path.

indices = goalIdx;
current = goalIdx;
while parents(current) > 0
    current = parents(current);
    indices = [current; indices]; %#ok<AGROW>
end

path = nodes(indices, :);
pathZ = nodesZ(indices);
end
