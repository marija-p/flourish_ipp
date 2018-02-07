plot_path(logger.trial2.cmaes.path_travelled, planning_params);

ax2 = axes;
h_img = imagesc(ax2, [-15, 15], [-15, 15], ground_truth_map);
axis([-20, 20, -20, 20, 0 30]);
set(ax2, 'Visible', 'off')
set(ax2, 'XTick', [])
set(ax2, 'YTick', [])
colormap(cmap)
caxis([0 1])

change_views
freezeColors
colormap(jet)