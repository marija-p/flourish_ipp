function [prob] = classify_nonweeds(altitude, num_nonweeds, planning_parameters)
% Classifier sensor model for detecting nonweeds:
% Outputs probability of finding a nonweed at a given point, for current
% altitude.

if (altitude > 30)
    prob = 0.5;
else
    prob = polyval(planning_parameters.nonweed_coeffs, altitude);
end

prob = prob*ones(num_nonweeds,1);

end
