function [var] = sensor_model(altitude)
% Inverse model of sensor to detect % vegetation cover in environment.
% (Height-dependant)
%
% Input:
% altitude = current UAV altitude
% ---
% Output:
% var = variance associated with measurement

% TODO(M):- Parametrise this.
%var = altitude/64;
var = 5 .* (1 - exp(-0.08 .* altitude));

end

