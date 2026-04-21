function result = main(varargin)
%MAIN Launch the hexapod motion-planning simulator.

projectRoot = fileparts(mfilename('fullpath'));
srcRoot = fullfile(projectRoot, 'src');
if ~any(strcmp(strsplit(path, pathsep), srcRoot))
    addpath(srcRoot);
end

result = motionplanning.runSimulation(varargin{:});
end
