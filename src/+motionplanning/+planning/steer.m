function newState = steer(fromState, towardState, stepSize)
%STEER Move from one state toward another by at most stepSize.

direction = towardState - fromState;
distance = norm(direction);
if distance < eps
    newState = fromState;
else
    newState = fromState + min(stepSize, distance) * (direction / distance);
end
end
