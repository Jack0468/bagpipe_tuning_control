function save_fig(fig, name)
%SAVE_FIG  Save a figure as results/figures/<name>.png and close it.
exportgraphics(fig, fullfile(results_dir('figures'), [name '.png']), 'Resolution', 150);
close(fig);
end
