function [isStable, info] = checkStaticStability(feet, comXY, margin)
%CHECKSTATICSTABILITY Require COM projection inside both tripod supports.

supportSets = [1 3 5; 2 4 6];
phaseStable = false(2, 1);
phaseMargins = NaN(2, 1);

for phase = 1:2
    polygon = feet(supportSets(phase, :), 1:2);
    if any(isnan(polygon(:)))
        continue;
    end

    [inside, onBoundary] = inpolygon(comXY(1), comXY(2), polygon(:, 1), polygon(:, 2));
    phaseMargins(phase) = pointPolygonDistance(comXY, polygon);
    phaseStable(phase) = (inside || onBoundary) && phaseMargins(phase) >= margin;
end

isStable = all(phaseStable);
info = struct( ...
    'supportSets', supportSets, ...
    'phaseStable', phaseStable, ...
    'phaseMargins', phaseMargins, ...
    'requiredMargin', margin);
end

function distance = pointPolygonDistance(point, polygon)
distance = Inf;
for idx = 1:size(polygon, 1)
    nextIdx = idx + 1;
    if nextIdx > size(polygon, 1)
        nextIdx = 1;
    end
    distance = min(distance, pointSegmentDistance(point, polygon(idx, :), polygon(nextIdx, :)));
end
end

function distance = pointSegmentDistance(point, segA, segB)
segment = segB - segA;
segmentLengthSq = sum(segment .^ 2);
if segmentLengthSq < eps
    distance = norm(point - segA);
    return;
end

t = dot(point - segA, segment) / segmentLengthSq;
t = motionplanning.utils.clamp(t, 0, 1);
projection = segA + t * segment;
distance = norm(point - projection);
end
