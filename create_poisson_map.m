function [ground_truth_map] = create_poisson_map(num_weeds, map_dim_x, map_dim_y)
% Creates 2D 'farmland' environment with Poisson distributed weeds.
%
% Coordinate system:
%
%  ^ y (rows)
%  |
%  |
%  -----> x (cols)

% Set free space.
ground_truth_map = zeros(map_dim_y, map_dim_x);

% Obtain weed coordinates.
% x-dir
weed_coords_x = 1 + (map_dim_x-1).*rand(num_weeds, 1);
% y-dir
weed_coords_y = 1 + (map_dim_y-1).*rand(num_weeds, 1);
weed_coords = round([weed_coords_y, weed_coords_x]);
weeds_ind = sub2ind(size(ground_truth_map), weed_coords(:,1), weed_coords(:,2));
ground_truth_map(weeds_ind) = 1;

% Visualize.
%plot(weed_coords(:,1), weed_coords(:,2), 'x', 'Color', [0 0.5 0]);
%imagesc(ground_truth_map);
%set(gca,'Ydir','Normal');

end

