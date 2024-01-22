%plot3(AngularVelocity.X, AngularVelocity.Y, AngularVelocity.Z);
close all
load('plasko.mat');
waitforbuttonpress;

data = dlmread('dane.csv', ';');
data = data(:,1:6);

field1 = 'x';
field2 = 'y';
field3 = 'z';

value1 = data(1:10:length(data),1);
value2 = data(1:10:length(data),2);
value3 = data(1:10:length(data),3);

Rot = struct(field1, value1, field2, value2, field3, value3);

value1 = data(1:10:length(data),4);
value2 = data(1:10:length(data),5);
value3 = data(1:10:length(data),6);

Pos = struct(field1, value1, field2, value2, field3, value3);
clear field1 value1 field2 value2 field3 value3;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

cosys = [[0 0.1 0 0 0 0];
         [0 0 0 0.1 0 0];
         [0 0 0 0 0 0.1]];
smuga = 10;
plthdl(length(Rot.x)) = line();
txhdl(length(Rot.x)) = text();

for i = 1:length(Orientation.X)
    plthdl(i) = plot3(cosys(1,:), cosys(2 ,:), cosys(3,:));  hold on;
    plthdl(i).Visible = "off";
    %txhdl = text(cosys(1,:)+x_, cosys(2 ,:)+y_, cosys(3,:)+z_,{'','X','','Y','','Z'});% cosys(2 ,4)+y_, cosys(3,6)+z_)
    xlim([min(Pos.x)-1 max(Pos.x)+1]);
    ylim([min(Pos.y)-1 max(Pos.y)+1 ]);   
    zlim([min(Pos.z)-1 max(Pos.z)+1]); axis tight square equal
    rotate(plthdl(i),[1 0 0], Rot.x(i), [Pos.x,Pos.y,Pos.z])
    rotate(plthdl(i),[0 1 0], Rot.y(i), [Pos.x,Pos.y,Pos.z])
    rotate(plthdl(i),[0 0 1], Rot.z(i), [Pos.x,Pos.y,Pos.z])
    plthdl(i) = plot3(plthdl(i).XData+Pos.x(i), plthdl(i).YData+Pos.y(i), plthdl(i).ZData+Pos.z(i))
    plthdl(i).Visible = "on";
    txhdl = text(plthdl(i).XData, plthdl(i).YData, plthdl(i).ZData,{'','X','','Y','','Z'});% c
    plot3(Pos.x(1:i), Pos.y(1:i), Pos.z(1:i), 'Color', 'Blue');
    view(-75, 45);
    grid on;

    if i > smuga  
        plthdl(i-smuga).reset;
    end
    pause(1e-15);
    delete(txhdl);
    %disp(sqrt((a.x).^2+(a.y).^2+(a.z).^2));
end