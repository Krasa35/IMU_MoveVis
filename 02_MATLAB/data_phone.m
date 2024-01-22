x = zeros([100 1]);
y = zeros([100 1]);
z = zeros([100 1]);
for i = 1:100
    x(i) = m.Latitude;
    y(i) = m.Longitude;
    z(i) = m.Altitude;
    waitfor m.Latitude ~= x(i);
end