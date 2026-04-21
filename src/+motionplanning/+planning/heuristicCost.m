function cost = heuristicCost(state, goalState)
%HEURISTICCOST Admissible Euclidean lower bound in XY.

cost = norm(goalState - state);
end
