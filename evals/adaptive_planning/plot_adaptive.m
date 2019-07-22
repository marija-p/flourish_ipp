rescale_factor = 15;
text_size = 11.5;
plot_aspect_ratio = [1 1 1];
line_width = 0.7;

do_plot = 1;
do_print = 0;
show_legend = 0;

paper_pos = [0, 0, 6, 4];

trials = fieldnames(logger);
methods = {'cmaes_adaptive', 'cmaes_nonadaptive'};

time_vector = 0:0.1:200;

P_traces = zeros(length(methods)-1,length(time_vector));
P_traces_interesting = zeros(length(methods)-1,length(time_vector));
P_traces_uninteresting = zeros(length(methods)-1,length(time_vector));
rmses = zeros(length(methods)-1,length(time_vector));
wrmses = zeros(length(methods)-1,length(time_vector));
mlls = zeros(length(methods)-1,length(time_vector));
wmlls = zeros(length(methods)-1,length(time_vector));
uncertainty_diffs = zeros(length(methods)-1,length(time_vector));

for i = 1:length(trials)
    
    for j = 1:length(methods)
       
        try
           time = logger.(trials{i}).(methods{j}).times;
        catch
           disp(['Cant find ', trials{i}, ' ' methods{j}])
           break;
        end
            
        P_trace = logger.(trials{i}).(methods{j}).P_traces;
        P_trace_interesting = ...
            logger.(trials{i}).(methods{j}).P_traces_interesting;
        P_trace_uninteresting = ...
            logger.(trials{i}).(methods{j}).P_traces_uninteresting;
        rmse = logger.(trials{i}).(methods{j}).rmses;
        wrmse = logger.(trials{i}).(methods{j}).wrmses;
        mll = logger.(trials{i}).(methods{j}).mlls;
        wmll = logger.(trials{i}).(methods{j}).wmlls;
        uncertainty_diff = logger.(trials{i}).(methods{j}).uncertainty_diff;
        
        ts = timeseries(P_trace, time);
        ts_resampled = resample(ts, time_vector, 'zoh');
        P_traces(j,:,i) = ts_resampled.data';

        ts = timeseries(P_trace_interesting, time);
        ts_resampled = resample(ts, time_vector, 'zoh');
        P_traces_interesting(j,:,i) = ts_resampled.data';
     
        ts = timeseries(P_trace_uninteresting, time);
        ts_resampled = resample(ts, time_vector, 'zoh');
        P_traces_uninteresting(j,:,i) = ts_resampled.data';
        
        ts = timeseries(rmse, time);
        ts_resampled = resample(ts, time_vector, 'zoh');
        rmses(j,:,i) = ts_resampled.data';
     
        ts = timeseries(wrmse, time);
        ts_resampled = resample(ts, time_vector, 'zoh');
        wrmses(j,:,i) = ts_resampled.data';
 
        ts = timeseries(mll, time);
        ts_resampled = resample(ts, time_vector, 'zoh');
        mlls(j,:,i) = ts_resampled.data';

        ts = timeseries(wmll, time);
        ts_resampled = resample(ts, time_vector, 'zoh');
        wmlls(j,:,i) = ts_resampled.data';
        
        ts = timeseries(uncertainty_diff, time);
        ts_resampled = resample(ts, time_vector, 'zoh');
        uncertainty_diffs(j,:,i) = ts_resampled.data;
        
    end
    
end

% Find means and medians.
mean_P_traces = sum(P_traces,3)./length(trials);
mean_P_traces_interesting = sum(P_traces_interesting,3)./length(trials);
mean_P_traces_uninteresting = sum(P_traces_uninteresting,3)./length(trials);
mean_rmses = sum(rmses,3)./length(trials);
mean_wrmses = sum(wrmses,3)./length(trials);
mean_mlls = sum(mlls,3)./length(trials);
mean_wmlls = sum(wmlls,3)./length(trials);
mean_uncertainty_diffs = sum(uncertainty_diffs,3)./length(trials);
median_P_traces = median(P_traces,3);
median_P_traces_interesting = median(P_traces_interesting,3);
median_P_traces_uninteresting = median(P_traces_uninteresting,3);
median_rmses = median(rmses,3);
median_wrmses = median(wrmses,3);
median_mlls = median(mlls,3);
median_uncertainty_diffs = median(uncertainty_diffs,3);

% Print average values.
disp(methods(2:end))
disp('Mean P traces: ')
disp(sum(mean_P_traces, 2, 'omitnan')./size(mean_P_traces,2));
disp('Mean P traces interesting: ')
disp(sum(mean_P_traces_interesting, 2, 'omitnan')./ ...
    size(mean_P_traces_interesting,2));
disp('Mean P traces uninteresting: ')
disp(sum(mean_P_traces_uninteresting, 2, 'omitnan')./ ...
    size(mean_P_traces_uninteresting,2));
disp('Mean RMSEs: ')
disp(sum(mean_rmses, 2, 'omitnan')./size(mean_rmses,2));
disp('Mean WRMSEs: ')
disp(sum(mean_wrmses, 2, 'omitnan')./size(mean_wrmses,2));
disp('Mean MLLs: ')
disp(sum(mean_mlls, 2, 'omitnan')./size(mean_mlls,2));
disp('Mean WMLLs: ')
disp(sum(mean_wmlls, 2, 'omitnan')./size(mean_wmlls,2));
disp('Mean Uncertainty Diffs: ')
disp(sum(mean_uncertainty_diffs, 2, 'omitnan')./ ...
    size(mean_uncertainty_diffs,2))

% Find confidence intervals
% http://ch.mathworks.com/matlabcentral/answers/159417-how-to-calculate-the-confidence-interva
SEM_P_traces = [];
SEM_P_traces_interesting = [];
SEM_P_traces_uninteresting = [];
SEM_rmses = [];
SEM_wrmses = [];
SEM_mlls = [];
SEM_wmlls = [];
SEM_uncertainty_diffs = [];

for j = 1:length(methods)

    SEM_P_traces(j,:) = std(squeeze(P_traces(j,:,:))', 'omitnan')/...
        sqrt(length(trials));
    SEM_P_traces_interesting(j,:) = ...
        std(squeeze(P_traces_interesting(j,:,:))', 'omitnan')/...
        sqrt(length(trials));
    SEM_P_traces_uninteresting(j,:) = ...
        std(squeeze(P_traces_uninteresting(j,:,:))', 'omitnan')/...
        sqrt(length(trials));
    SEM_rmses(j,:) = (std(squeeze(rmses(j,:,:))', 'omitnan')/...
        sqrt(length(trials)));
    SEM_wrmses(j,:) = (std(squeeze(wrmses(j,:,:))', 'omitnan')/...
        sqrt(length(trials)));
    SEM_mlls(j,:) = (std(squeeze(mlls(j,:,:))', 'omitnan')/...
        sqrt(length(trials)));
    SEM_wmlls(j,:) = (std(squeeze(wmlls(j,:,:))', 'omitnan')/...
        sqrt(length(trials)));
    SEM_uncertainty_diffs(j,:) = ...
        (std(squeeze(uncertainty_diffs(j,:,:))', 'omitnan')/...
        sqrt(length(trials)));
    
end

% Symmetric
ts = tinv(0.97, length(trials)-1);

colours = [0.8500    0.3250    0.0980;
    0    0.4470    0.7410;
  %  0.9290    0.6940    0.1250;
  %  0.4940    0.1840    0.5560;
    0.4660    0.6740    0.1880;
     0.6350    0.0780    0.1840];
    % 0.3010    0.7450    0.9330];
    % 0.1379    0.1379    0.0345]
 transparency = 0.3;
 

%% PLOTTING %%

if (do_plot)
        
    figure;
    
    %% Trace of P %%
    %subplot(1,3,1)
%     hold on
%     h = zeros(2,1);
%     boundedline( ...
%         time_vector, mean_P_traces(1,:), SEM_P_traces(1,:)*ts, ...
%         time_vector, mean_P_traces(2,:), SEM_P_traces(2,:)*ts, ...
%         'alpha', 'cmap', colours, 'transparency', transparency);
%      
%     for i = 1:length(methods)
%         P_trace = mean_P_traces(i,:);
%         h(i) = plot(time_vector, P_trace, 'LineWidth', line_width, ...
%             'Color', colours(i,:));
%     end
%     
%     h_xlabel = xlabel('Time (s)');
%     h_ylabel = ylabel('Tr(\itP\rm)');
%     set([h_xlabel, h_ylabel], ...
%         'FontName'   , 'Helvetica');
%     
%     set(gca, ...
%         'Box'         , 'off'     , ...
%         'TickDir'     , 'out'     , ...
%         'TickLength'  , [.02 .02] , ...
%         'XMinorTick'  , 'on'      , ...
%         'YMinorTick'  , 'on'      , ...
%         'YGrid'       , 'on'      , ...
%         'XColor'      , [.3 .3 .3], ...
%         'YColor'      , [.3 .3 .3], ...
%         'YScale'      , 'log'     , ...
%         'YGrid'       , 'on'      , ...
%         'LineWidth'   , line_width         , ...
%         'FontSize'    , text_size, ...
%         'FontName'    , 'Times', ...
%         'LooseInset', max(get(gca,'TightInset'), 0.02));
%     set(gca, 'YTick', [0, 10.^1, 10.^2]);
%     axis([0 time_vector(end) 0 400])
%     %rescale_axes(rescale_factor);
%     pbaspect(gca, plot_aspect_ratio)
%     hold off
    
    %% Trace of P - Interesting %%
    figure;
    hold on
    h = zeros(2,1);
    boundedline(...
        time_vector, mean_P_traces_interesting(1,:), SEM_P_traces_interesting(1,:)*ts, ...
        time_vector, mean_P_traces_interesting(2,:), SEM_P_traces_interesting(2,:)*ts, ...
        'alpha', 'cmap', colours, 'transparency', transparency);
     
    for i = 1:length(methods)
        rmse = mean_P_traces_interesting(i,:);
        h(i) = plot(time_vector, rmse, 'LineWidth', line_width, ...
            'Color', colours(i,:));
    end
    
    h_xlabel = xlabel('Time (s)');
    h_ylabel = ylabel('Tr(\itP\rm)');
    set([h_xlabel, h_ylabel], ...
        'FontName'   , 'Helvetica');
    
    set(gca, ...
        'Box'         , 'off'     , ...
        'TickDir'     , 'out'     , ...
        'TickLength'  , [.02 .02] , ...
        'XMinorTick'  , 'on'      , ...
        'YMinorTick'  , 'on'      , ...
        'YGrid'       , 'on'      , ...
        'XColor'      , [.3 .3 .3], ...
        'YColor'      , [.3 .3 .3], ...
        'YScale'      , 'log'     , ...
        'YGrid'       , 'on'      , ...
        'LineWidth'   , line_width         , ...
        'FontSize'    , text_size, ...
        'FontName'    , 'Times', ...
        'LooseInset', max(get(gca,'TightInset'), 0.02));
    set(gca, 'YTick', [0, 10.^1, 10.^2]);
    axis([0 time_vector(end) 0 400])
    %rescale_axes(rescale_factor);
    pbaspect(gca, plot_aspect_ratio)
    set(gcf,'color','w');
    hold off
    
    %% RMSE %%
    %subplot(1,2,1)
    figure;
    hold on
    boundedline(...
        time_vector, mean_wrmses(1,:), SEM_wrmses(1,:)*ts, ...
        time_vector, mean_wrmses(2,:), SEM_wrmses(2,:)*ts, ...
        'alpha', 'cmap', colours, 'transparency', transparency);
     
    for i = 1:length(methods)
        rmse = mean_wrmses(i,:);
        h(i) = plot(time_vector, rmse, 'LineWidth', line_width, ...
            'Color', colours(i,:));
    end
    
    h_xlabel = xlabel('Time (s)');
    h_ylabel = ylabel('WRMSE');
    set([h_xlabel, h_ylabel], ...
        'FontName'   , 'Helvetica');
    set(gca, ...
        'Box'         , 'off'     , ...
        'TickDir'     , 'out'     , ...
        'TickLength'  , [.02 .02] , ...
        'XMinorTick'  , 'on'      , ...
        'YMinorTick'  , 'on'      , ...
        'YGrid'       , 'on'      , ...
        'XColor'      , [.3 .3 .3], ...
        'YColor'      , [.3 .3 .3], ...
        'YTick'       , 0:0.05:0.2, ...
        'LineWidth'   , line_width         , ...
        'FontSize'    , text_size, ...
        'FontName'    , 'Times', ...
        'LooseInset', max(get(gca,'TightInset'), 0.02));
    %rescale_axes(rescale_factor);
    axis([0 time_vector(end) 0 0.2])
    pbaspect(gca, plot_aspect_ratio)
    set(gcf,'color','w');
    hold off
  
    %% Uncertainty diff %%
    figure;
    %subplot(1,2,2)
    hold on
    boundedline( ...
        time_vector, mean_uncertainty_diffs(1,:), ...
        SEM_uncertainty_diffs(1,:)*ts, ...
        time_vector, mean_uncertainty_diffs(2,:), ...
        SEM_uncertainty_diffs(2,:)*ts, ...
        'alpha', 'cmap', colours, 'transparency', transparency);
     
    for i = 1:length(methods)
        uncertainty_diff = mean_uncertainty_diffs(i,:);
        h(i) = plot(time_vector, uncertainty_diff, ...
            'LineWidth', line_width, 'Color', colours(i,:));
    end
    
    h_xlabel = xlabel('Time (s)');
    h_ylabel = ylabel('Uncertainty difference \Delta \sigma^2');
    set([h_xlabel, h_ylabel], ...
        'FontName'   , 'Helvetica');
    set(gca, ...
        'Box'         , 'off'     , ...
        'TickDir'     , 'out'     , ...
        'TickLength'  , [.02 .02] , ...
        'XMinorTick'  , 'on'      , ...
        'YMinorTick'  , 'on'      , ...
        'YGrid'       , 'on'      , ...
        'XColor'      , [.3 .3 .3], ...
        'YColor'      , [.3 .3 .3], ...
        'YTick'       , -1:0.5:1      , ...
        'LineWidth'   , line_width         , ...
        'FontSize'    , text_size, ...
        'FontName'    , 'Times', ...
        'LooseInset', max(get(gca,'TightInset'), 0.02));
    %rescale_axes(rescale_factor);
    axis([0 time_vector(end) -1 1])
    pbaspect(gca, plot_aspect_ratio)
    hold off
    set(gcf,'color','w');
    
    if (do_print)
        fig = gcf;
        fig.PaperUnits = 'inches';
        fig.PaperPosition = paper_pos;
        fig.PaperPositionMode = 'manual';
        export_fig -depsc2 adaptive_planning_evaluation.eps -opengl
    end
    
        
    if (show_legend)
        %h_legend = legend(h, 'CMA-ES', 'RIG-tree', 'Coverage', 'Random');
        h_legend = legend(h, 'Adaptive', 'Non-adaptive');
        set(h_legend, 'Location', 'SouthOutside');
        set(h_legend, 'orientation', 'horizontal')
        set(h_legend, 'box', 'off');
        set(h_legend, 'FontName', 'Times');
    end
  
end

%close all;