function root = startup_paths()
%STARTUP_PATHS  Add the project's MATLAB folders to the path and return the matlab/ folder.
root = fileparts(mfilename('fullpath'));
subs = {'params', 'model', 'analysis', 'sim', 'control', 'estimator', 'util'};
for k = 1:numel(subs)
    addpath(fullfile(root, subs{k}));
end
end
