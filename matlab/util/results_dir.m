function d = results_dir(sub)
%RESULTS_DIR  Output folder <repo>/results[/sub] (gitignored), created if needed.
repo = fileparts(fileparts(fileparts(mfilename('fullpath'))));
d = fullfile(repo, 'results');
if nargin > 0
    d = fullfile(d, sub);
end
if ~exist(d, 'dir')
    mkdir(d);
end
end
