function submap = add_sensor_noise(altitude, submap, planning_parameters)
% Simulates Gaussian noise in measurements taken with height-dependent sensor.

var = sensor_model(altitude, planning_parameters);
submap = normrnd(submap, var);

end

