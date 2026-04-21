function state = sampleState(goalState, terrain, options)
%SAMPLESTATE Draw a goal-biased random XY state.

if rand() < options.goalBias
    state = goalState;
else
    state = [ ...
        terrain.minX + rand() * (terrain.maxX - terrain.minX), ...
        terrain.minY + rand() * (terrain.maxY - terrain.minY)];
end
end
