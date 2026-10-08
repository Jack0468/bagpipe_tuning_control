function shade_gaps(ax, gaps)
%SHADE_GAPS  Shade breath-gap intervals [t_start t_end] on an axes (behind the data).
yl = ylim(ax);
hold(ax, 'on');
for k = 1:size(gaps, 1)
    patch(ax, gaps(k, [1 2 2 1]), yl([1 1 2 2]), [0.85 0.85 0.85], ...
        'EdgeColor', 'none', 'FaceAlpha', 0.6, 'HandleVisibility', 'off');
end
ylim(ax, yl);
ax.Children = flipud(ax.Children);   % keep the patches behind the lines
end
