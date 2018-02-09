plot_path(logger.trial3.cmaes_adaptive.path_travelled(1:end-5, :), planning_params);
ax1 = gca;
set(ax1, 'Visible', 'off')
axis([-15, 15, -15, 15, 0 30]);
legend off

ax2 = axes;
h_img = imagesc(ax2, [-15, 15], [-15, 15], grid_map.m);
axis([-15, 15, -15, 15, 0 30]);
set(ax2, 'Visible', 'off')
set(ax2, 'XTick', [])
set(ax2, 'YTick', [])
load weed_cm.mat
colormap(cmap)
caxis([0 1])
uistack(gca, 'bottom')

freezeColors
colormap(jet)

ax3 = axes;
axis([-15, 15, -15, 15, 0 30]);
set(ax3, 'XTick', [-15 0 15])
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
set(ax3, 'YTick', [-15 0 15])
set(ax3, 'ZTick', [0 10 20 30])
set(gca, 'FontSize', 12)
set(gca, 'FontName', 'Times')
uistack(gca, 'bottom')
grid on

change_views