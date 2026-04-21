function [feet, inside] = footTargets(centerXY, yaw, terrain, robot, targetMode)
%FOOTTARGETS Compute nominal foot placements and terrain contact heights.

if nargin < 5
    targetMode = 'planning';
end

if strcmpi(targetMode, 'visual')
    targetRadius = robot.visualFootRadius;
else
    targetRadius = robot.nominalFootRadius;
end

% Vectorised: compute all six foot positions in one shot then do a single
% batched terrain query instead of six individual getTerrainHeight calls.
headings = yaw + robot.baseAngles(:);          % 6×1
footX    = centerXY(1) + targetRadius * cos(headings);
footY    = centerXY(2) + targetRadius * sin(headings);
footZ    = motionplanning.environment.getTerrainHeight(footX, footY, terrain, 'nan');

feet   = [footX(:), footY(:), footZ(:)];      % 6×3
inside = ~isnan(footZ(:));                    % 6×1
end
