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

var = 0.05 .* (1 - exp(-0.2 .* altitude));

%if (altitude > 15)
%    var = 4 * 0.05 .* (1 - exp(-0.2 .* altitude));
%end

end

