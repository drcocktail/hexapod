function [isValid, zSamples, sampledStates, failReason] = isEdgeValid(fromState, toState, terrain, robot, options)
%ISEDGEVALID Validate an edge by sampling intermediate body states.

if ~isfield(options, 'edgeCheckResolution')
    options.edgeCheckResolution = 1.0;
end

delta = toState - fromState;
distance = norm(delta);
if distance < eps
    sampleCount = 1;
    yaw = 0;
else
    sampleCount = max(2, ceil(distance / options.edgeCheckResolution) + 1);
    yaw = atan2(delta(2), delta(1));
end

zSamples = NaN(sampleCount, 1);
sampledStates = zeros(sampleCount, 2);
failReason = 'valid';
previousZ = NaN;

for idx = 1:sampleCount
    if sampleCount == 1
        alpha = 0;
    else
        alpha = (idx - 1) / (sampleCount - 1);
    end

    state = fromState + alpha * delta;
    sampledStates(idx, :) = state;

    [stateValid, bodyZ, stateInfo] = motionplanning.planning.isStateValid(state, terrain, robot, yaw);
    if ~stateValid
        isValid = false;
        failReason = stateInfo.reason;
        return;
    end

    if idx > 1 && abs(bodyZ - previousZ) > robot.maxStepZ
        isValid = false;
        failReason = 'excessive_body_z_step';
        return;
    end

    zSamples(idx) = bodyZ;
    previousZ = bodyZ;
end

isValid = true;
end
