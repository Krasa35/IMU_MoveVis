function L = co_sys_vis(x, y, z, obj)
%obj - matrix 3 by X
%x, y, z in radians
%plot3([0 0 0 1 0 0],[0 1 0 0 0 0],[0 0 0 0 0 1]); uklad wsp
%-----Functions-----
Rx = [1 0 0; 0 cos(x) -sin(x); 0 sin(x) cos(x)];
Ry = [cos(y) 0 sin(y); 0 1 0; -sin(y) 0 cos(y)];
Rz = [cos(z) -sin(z) 0; sin(z) cos(z) 0; 0 0 1];

%-----Compute-------
rotated = Rx*Ry*Rz*obj;

x_ = rotated(1,:);
y_ = rotated(2,:);
z_ = rotated(3,:);
%-------------------
field1 = 'x';  value1 = x_;
field2 = 'y';  value2 = y_;
field3 = 'z';  value3 = pi/2-z_;
%field4 = 'O';  value4 = [Start.x Start.y Start.z];
%-------------------
%L = struct(field1, value1, field2, value2, field3, value3, field4, value4);
L = struct(field1, value1, field2, value2, field3, value3);% field4, value4);

end