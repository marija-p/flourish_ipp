altitude = 1:0.01:26;
var = 0.5 .* (1 - exp(-0.01 .* altitude));
plot(altitude, var)

title('Sensor model')
xlabel('Altitude (m)')
ylabel('Var.')
grid minor