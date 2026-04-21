function [cost, zEnd, isValid, failReason] = edgeCost(fromState, toState, terrain, robot, options)
%EDGECOST Validate an edge and compute weighted traversal cost.

[isValid, zSamples, ~, failReason] = motionplanning.planning.isEdgeValid( ...
    fromState, toState, terrain, robot, options);
if ~isValid
    cost = Inf;
    zEnd = NaN;
    return;
end

distanceCost = norm(toState - fromState);
verticalCost = sum(abs(diff(zSamples)));
roughnessCost = std(zSamples);

distanceWeight = getOption(options, 'distanceWeight', 1.0);
verticalWeight = getOption(options, 'verticalWeight', 0.0);
roughnessWeight = getOption(options, 'roughnessWeight', 0.0);

cost = distanceWeight * distanceCost + ...
    verticalWeight * verticalCost + ...
    roughnessWeight * roughnessCost;
zEnd = zSamples(end);
end

function value = getOption(options, name, defaultValue)
if isfield(options, name)
    value = options.(name);
else
    value = defaultValue;
end
end
