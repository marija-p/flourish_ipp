trials = 1;
method = 'coverage';

time_vector = 0:0.1:600;
P_traces = [];

for i = 1:length(trials)
    
    try
        time = logger.(['trial', num2str(i)]).(method).times;
    catch
        disp(['Cant find trial', num2str(i), ' ' method])
        break;
    end
    P_trace = logger.(['trial', num2str(i)]).(method).P_traces;
    ts = timeseries(P_trace, time);
    ts_resampled = resample(ts, time_vector, 'zoh');
    P_traces(:,i) = ts_resampled.data';
end

P_traces_ave = sum(P_traces,1,'omitnan') ./ size(P_traces,1)