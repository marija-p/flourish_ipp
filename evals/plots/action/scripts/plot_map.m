figure;

shade = 0;

hold on

I = checkerboard(size(grid_map.m,1)/8);
I(I > 0.5) = 0.7;
I(I < 0.5) = 0.3;
imagesc(I, 'AlphaData', 0.2);
colormap gray

freezeColors
Y_sigma = sqrt(diag(grid_map.P)');
P_post = reshape(2*Y_sigma,40,40);
P_post = (P_post - min(min(P_post))) / (max(max(P_post)) - min(min(P_post)));
imagesc(grid_map.m, 'AlphaData', (1-P_post));
%imagesc(ground_truth_map)
A = (grid_map.m < 0.4);
%A = (ground_truth_map < 0.4);
set(gca,'Ydir','Normal');

if (shade)
    
    stats = regionprops(A,'all');
    for i = 1:length(stats)
        hull = stats(i).ConvexHull;
        h_patch = patch(hull(:,1), hull(:,2),'w', 'FaceAlpha', 0);
        hatchfill2(h_patch, 'HatchSpacing', 8, 'HatchLineWidth', 0.5);
    end
    
end

colormap(cmap)
caxis([0 1])

set(gca, 'Visible', 'Off')
pbaspect([1 1 1])

hold off

%figure;imagesc(P_post)