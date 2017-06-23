function factor = get_resolution_from_height(altitude)
% Obtains resolution diffusion factor for sensor model for a particular UAV
% altitude.

%if altitude > 2
%    factor = 2;
%elseif altitude < 1
%    factor = 1;
%else
%    factor = altitude/2;
%end

if altitude > 10
    factor = 4;
else
    factor = 1;
end


