function result = main(varargin)
%MAIN Launch the hexapod motion-planning simulator.

projectRoot = fileparts(mfilename('fullpath'));
srcRoot = fullfile(projectRoot, 'src');
if ~any(strcmp(strsplit(path, pathsep), srcRoot))
    addpath(srcRoot);
end

testOverrides = struct();

if numel(varargin) == 1 && isnumeric(varargin{1})
    testOverrides.seed = varargin{1};
    varargin = {testOverrides};
elseif isempty(varargin)
    testOverrides.seed = 1099; % Syncing with your recent test seed
    varargin = {testOverrides};
end

result = motionplanning.runSimulation(varargin{:});
end
