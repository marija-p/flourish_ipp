function factor = get_resolution_from_height(height)

if height > 5
    factor = 2;
elseif height < 1
     factor = 1;
else
    factor = height/2;
end
