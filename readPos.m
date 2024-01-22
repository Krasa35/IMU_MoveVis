function [x_, y_, z_] = readPos(Position, i)
s_x = (Position.latitude(i));%-floor(Position.latitude(1)*100)/100)*100;
s_y = (Position.longitude(i));%-floor(Position.longitude(1)*100)/100)*100;
s_z = (Position.altitude(i));%-floor(Position.altitude(1)*100)/100)*100;
x_ = s_x*ones(1,6);
x_(2) = x_(2)+1;
y_ = s_y*ones(1,6);
y_(4) = y_(4)+1;
z_ = s_z*ones(1,6);
z_(6) = z_(6)+1;
end