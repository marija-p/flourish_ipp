altitude = 1:0.01:26;
var = 5 .* (1 - exp(-0.08 .* altitude));
plot(altitude, var)

title('Sensor model')
xlabel('Altitude (m)')
ylabel('Var.')
grid minor