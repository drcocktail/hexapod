function saveSimulation(result, filename)
%SAVESIMULATION Save the computed simulation result to a .mat file
%   Drops graphical handles from the struct before saving to ensure clean data storage.

if nargin < 2
    filename = 'simulation_result.mat';
end

saveStruct = result;
if isfield(saveStruct, 'figure')
    saveStruct = rmfield(saveStruct, 'figure');
end
if isfield(saveStruct, 'axes')
    saveStruct = rmfield(saveStruct, 'axes');
end

save(filename, 'saveStruct');
fprintf('Simulation successfully saved to %s\n', filename);
end
