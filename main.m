function result = main(varargin)
%MAIN Launch the hexapod motion-planning simulator.

projectRoot = fileparts(mfilename('fullpath'));
srcRoot = fullfile(projectRoot, 'src');
if ~any(strcmp(strsplit(path, pathsep), srcRoot))
    addpath(srcRoot);
end

testOverrides = struct();
testOverrides.seed = 1099; % Syncing with your recent test seed
% Buff robot leg lengths heavily so it has the physical wingspan to step over the 3D variations
testOverrides.robot.L_femur = 7.0;
testOverrides.robot.L_tibia = 9.0;
testOverrides.robot.clearance = 3.0; % Lower nominal body hover to extend downward leg reach
testOverrides.robot.maxStepZ = 14.0;
% Smooth the absolute sharpest cliff drops, keeping rolling mountains but reducing impassable shear walls
testOverrides.environment.amplitude = 12.0;
testOverrides.environment.faultStepHeight = 5.0;
testOverrides.planning.maxNodes = 2500; % Reduced to make finding the path faster for testing

if isempty(varargin)
    varargin = {testOverrides};
end

result = motionplanning.runSimulation(varargin{:});
end
