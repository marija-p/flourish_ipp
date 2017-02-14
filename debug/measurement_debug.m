% Debugging taking measurements

close all
load init.mat

point = [6 10 25];
grid_map1 = take_measurement_at_point(point, grid_map, ...
    ground_truth_map, map_parameters, planning_parameters);

figure;
subplot(1,3,1)
imagesc(grid_map.m)
caxis([0, 1])
title('Mean - init.')
set(gca,'Ydir','Normal');
colorbar;
subplot(1,3,2)
imagesc(grid_map1.m)
caxis([0, 1])
title('Mean - final')
set(gca,'Ydir','Normal');
colorbar;
subplot(1,3,3)
imagesc(ground_truth_map)
caxis([0, 1])
title('Ground truth map')
set(gca,'Ydir','Normal');
colorbar;

set(gcf, 'Position',  [2818, 848, 1477, 376])