figure;

hold on
I = checkerboard(size(colormap_image,1)/8);
I(I > 0.5) = 0.7;
I(I < 0.5) = 0.3;
imagesc(I, 'AlphaData', 0.2);
colormap gray

freezeColors
imagesc(colormap_image, 'AlphaData', repmat(linspace(1,0.7,size(colormap_image,1)),64,1)');

set(gca, 'Visible', 'Off')
set(gca,'Ydir','Normal');
pbaspect([4 1 1])