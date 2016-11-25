% Script to generate coeffs for probabilistic sensor curves
% (altitude-dependent).

figure;

height1 = 0:0.1:30;

% Heights to fix [m]
x = [1, 10, 30];
% Probabilities to fix
y_weed = [0.96, 0.90, 0.6];         % True positives - P(w|w)
y_nonweed = [0.05, 0.08, 0.35];     % False positives - P(w|nw)

% Get the coefficients
weed_coeffs = polyfit(x,y_weed,2);
nonweed_coeffs = polyfit(x,y_nonweed,2);

p_weed = polyval(weed_coeffs, height1);
p_nonweed = polyval(nonweed_coeffs, height1);

height2 = 30:0.1:50;
p_weed = [p_weed, 0.5*ones(1,length(height2))];
p_nonweed = [p_nonweed, 0.5*ones(1,length(height2))];

height = [height1, height2];

hold on
plot(height, p_weed, 'Color', [0, 0.4470, 0.7410])
plot(height, p_nonweed, 'Color', [0.8500, 0.3250, 0.0980]);
hold off
h_legend = legend('W', 'NW');
set(h_legend, 'Position', [0.7333 0.7873 0.1386 0.0948]);
xlabel('Altitude (m)')
ylabel('Weed classif. probability')
axis([0 40 0 1])
set(gca, ...
    'Box'         , 'off'     , ...
    'TickDir'     , 'out'     , ...
    'TickLength'  , [.02 .02] , ...
    'XMinorTick'  , 'on'      , ...
    'YMinorTick'  , 'on'      , ...
    'YGrid'       , 'on'      , ...
    'XColor'      , [.3 .3 .3], ...
    'YColor'      , [.3 .3 .3], ...
    'YTick'       , 0:0.2:1, ...
    'LineWidth'   , 1         , ...
    'FontSize'    , 10.5);

grid minor

%planning_parameters.weed_coeffs = weed_coeffs;
%planning_parameters.nonweed_coeffs = nonweed_coeffs;