figure;

colormap_image = zeros(256,256,3);
colormap_image(:,:,1) = repmat(cmap(:,1)',256,1);
colormap_image(:,:,2) = repmat(cmap(:,2)',256,1);
colormap_image(:,:,3) = repmat(cmap(:,3)',256,1);
colormap_image = imresize(colormap_image, [256, 256*4]);

hold on
I = checkerboard(size(colormap_image,1)/4, 2, ...
    16);
I(I > 0.5) = 0.7;
I(I < 0.5) = 0.3;
imagesc(I, 'AlphaData', 0.2);
colormap gray

freezeColors
imagesc(colormap_image, 'AlphaData', ...
    repmat(linspace(1,0.7,size(colormap_image,1)), ...
    size(colormap_image,2),1)');
set(gca, 'Visible', 'Off')
set(gca,'Ydir','Normal');
pbaspect([4 1 1])