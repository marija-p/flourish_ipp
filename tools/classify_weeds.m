function [prob] = classify_weeds(altitude, num_weeds, planning_parameters)
% Classifier sensor model for detecting weeds:
% Outputs probability of finding a weed at a given point, for current
% altitude.

if (altitude > 30)
    prob = 0.5;
else
    prob = polyval(planning_parameters.weed_coeffs, altitude);
end

prob = prob*ones(num_weeds,1);

end

