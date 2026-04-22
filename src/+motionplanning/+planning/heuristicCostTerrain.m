function cost = heuristicCostTerrain(state, goalState, epsilon)
%HEURISTICCOSTTERRAIN Inflated Euclidean heuristic for Weighted A*.
%
%   Multiplies the Euclidean lower bound by epsilon to produce a
%   bounded-suboptimal heuristic.  With epsilon=1 this is admissible
%   (identical to heuristicCost).  With epsilon>1 the search becomes
%   greedy, expanding fewer nodes at the cost of path quality.

if nargin < 3
    epsilon = 1.0;
end

cost = epsilon * norm(goalState - state);
end
