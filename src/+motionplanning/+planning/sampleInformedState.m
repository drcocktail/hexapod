function state = sampleInformedState(startState, goalState, bestCost, terrain, options)
%SAMPLEINFORMEDSTATE Sample uniformly from the informed ellipse when possible.

if ~isfinite(bestCost)
    state = motionplanning.planning.sampleState(goalState, terrain, options);
    return;
end

cMin = norm(goalState - startState);
if bestCost <= cMin + eps
    state = goalState;
    return;
end

center = (startState + goalState) / 2;
theta = atan2(goalState(2) - startState(2), goalState(1) - startState(1));
rotation = [cos(theta), -sin(theta); sin(theta), cos(theta)];
majorAxis = bestCost / 2;
minorAxis = sqrt(bestCost ^ 2 - cMin ^ 2) / 2;

for attempt = 1:100 %#ok<NASGU>
    angle = 2 * pi * rand();
    radius = sqrt(rand());
    localSample = [majorAxis * radius * cos(angle), minorAxis * radius * sin(angle)];
    candidate = center + (rotation * localSample')';
    if motionplanning.environment.isInsideDomain(candidate(1), candidate(2), terrain)
        state = candidate;
        return;
    end
end

state = motionplanning.planning.sampleState(goalState, terrain, options);
end
