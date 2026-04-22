function [traversable, costMap] = buildTraversabilityMap(terrain, robot, options)
%BUILDTRAVERSABILITYMAP Vectorized precomputation of grid traversability.
%
%   Sweeps the entire terrain grid once (O(n^2)) to produce:
%     traversable  - logical matrix: true where the hexapod body can occupy
%     costMap      - double matrix: base traversal cost per cell
%
%   This intentionally skips IK and tripod stability checks.  Those are
%   deferred to the post-planning validation pass (validateAndRepairPath).
%   The traversability map is a conservative fast filter based on:
%     1. Body boundary margin (feet must stay inside domain)
%     2. Maximum terrain slope (pitch / roll proxy)
%     3. Body clearance feasibility

[nRows, nCols] = size(terrain.Z);
gs = terrain.gridSpacing;

% --- 1. Boundary margin: body center must be far enough from edges
%     so that all foot placements remain inside the domain.
margin = robot.nominalFootRadius;
cellX = terrain.X(1, :);    % 1 x nCols
cellY = terrain.Y(:, 1);    % nRows x 1

insideX = cellX >= (terrain.minX + margin) & cellX <= (terrain.maxX - margin);
insideY = cellY >= (terrain.minY + margin) & cellY <= (terrain.maxY - margin);
boundaryOK = insideY & insideX;  % nRows x nCols via implicit expansion

% --- 2. Slope-based orientation limits (vectorized finite differences)
%     Compute forward-difference slopes; pad edges to keep matrix size.
slopeWeight = getOpt(options, 'astarSlopeWeight', 3.0);
maxSlope    = getOpt(options, 'astarMaxSlope', 0.8);

% dZ/dx  (along columns)
dZdx = zeros(nRows, nCols);
dZdx(:, 1:end-1) = diff(terrain.Z, 1, 2) / gs;
dZdx(:, end) = dZdx(:, end-1);

% dZ/dy  (along rows)
dZdy = zeros(nRows, nCols);
dZdy(1:end-1, :) = diff(terrain.Z, 1, 1) / gs;
dZdy(end, :) = dZdy(end-1, :);

slopeMag = sqrt(dZdx .^ 2 + dZdy .^ 2);

% Pitch/roll proxies — conservative: use the max slope component.
pitchProxy = abs(atan2(dZdx, 1));   % rise/run in X
rollProxy  = abs(atan2(dZdy, 1));   % rise/run in Y

slopeOK = slopeMag <= maxSlope & ...
          pitchProxy <= robot.maxPitch & ...
          rollProxy  <= robot.maxRoll;

% --- 3. Combine into traversability mask
traversable = boundaryOK & slopeOK;

% --- 4. Cost map: base cost = 1 + slope penalty
%     Cells that are not traversable get Inf cost.
costMap = ones(nRows, nCols) + slopeWeight * slopeMag;
costMap(~traversable) = Inf;

end

function value = getOpt(options, name, default)
if isfield(options, name)
    value = options.(name);
else
    value = default;
end
end
