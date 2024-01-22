function p = point(x_,y_,z_)
%-------------------
field1 = 'x';  value1 = x_;
field2 = 'y';  value2 = y_;
field3 = 'z';  value3 = z_;
%-------------------
p = struct(field1, value1, field2, value2, field3, value3);
end