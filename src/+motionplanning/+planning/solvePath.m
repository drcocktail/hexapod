function [path, pathZ, info] = solvePath(startState, goalState, terrain, robot, options)
%SOLVEPATH Dispatch to the configured motion planner.

if ~isfield(options, 'algorithm')
    algorithm = 'rrt';
else
    algorithm = lower(char(options.algorithm));
end

switch algorithm
    case 'rrt'
        [path, pathZ, info] = motionplanning.planning.planRRT(startState, goalState, terrain, robot, options);
    case {'informed_rrt_star', 'informedrrtstar', 'irrtstar'}
        [path, pathZ, info] = motionplanning.planning.planInformedRRTStar(startState, goalState, terrain, robot, options);
    case {'bit_star', 'bitstar'}
        [path, pathZ, info] = motionplanning.planning.planBITStar(startState, goalState, terrain, robot, options);
    otherwise
        error('motionplanning:planning:UnknownPlanner', ...
            'Unknown planning algorithm: %s', algorithm);
end

info.algorithm = algorithm;
end
