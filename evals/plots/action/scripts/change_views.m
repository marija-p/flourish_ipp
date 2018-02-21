hfigs = get(0,'children');
haxes = findall(hfigs,'type','axes');

for i = 1:length(haxes)
%    haxes(i).View = [45 30];
    haxes(i).View = [-45 30];
    haxes(i).Position = [0.1598    0.1065    0.6867    0.7599];
end