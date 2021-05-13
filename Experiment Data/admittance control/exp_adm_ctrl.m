clear; clc;
close all;
%% -------------------------- Read file ------------------------
filename = 'horizentalnew051002.xlsx';
data = readtable(filename);
data = table2array(data);
data=data(50:10000,:);
lenth = size(data(:,1));
row_robotpos_PS = data(:,1:3);    
row_led_1 = data(:,4:6);
row_led_2 = data(:,7:9);
row_led_3 = data(:,10:12);
center = (row_led_1 + row_led_2 + row_led_3 )/3;
impulse = sqrt(data(:,13).^2+data(:,14).^2+data(:,15).^2);
row_forces_0 = data(:,16:18);
row_sensor_t = data(:,19:24);
row_armangle = data(:,25:30);
row_robotpos_0 = data(:,31:33);
row_roboteuler_0 = data(:,34:36); 
%% -------------------------------Fitting--------------------------------
param1= polyfit(center(:,1),center(:,2),1);
param2= polyfit(center(:,1),center(:,3),1);
Zax = cross(mean(row_led_1(1:50,:))-mean(row_led_2(1:50,:)),mean(row_led_1(1:50,:))-mean(row_led_3(1:50,:)));
Zax = Zax/norm(Zax);
tempX = [Zax(2),-Zax(1),0];
tempX = tempX/norm(tempX); 
tempY = cross( Zax , tempX );
tempY = tempY/norm(tempY); 
trans1 = [tempX ; tempY ; Zax].';
new_center  = trans1 * center.' ;
new_robot = trans1 *row_robotpos_PS.' ;
param3= polyfit(new_center(1,:),new_center(2,:),1);
calib_X = [1,param3(1),0] ;
calib_X = calib_X/norm(calib_X); 
calib_Y = cross([0 0 1] ,calib_X);
calib_Y = calib_Y/norm(calib_Y); 
trans2 = [calib_X; calib_Y ;[0 0 1]];
calib_center = trans2 * new_center ; 
calib_robot = trans2 *new_robot ; 
figure(3)
scatter3(new_center(1,:),new_center(2,:) ,new_center(3,:));
  
%% ----------------------------- Rotation matrix {0}  -> {PS} --------------------------
Rp_0 = row_robotpos_PS.' * pinv(row_robotpos_0.');   % RA = B --> R = Apinv(B)

%% ------------------------------Plot (Platform position after calibration)-----------------------------------------------------------
xlabel('Xaxis','Interpreter','Latex');
ylabel('Yaxis','Interpreter','Latex');
zlabel('Zaxis','Interpreter','Latex');
title('Platform position after calibration','Interpreter','Latex');
set(findall(gcf,'type','line'),'linewidth',1);  %line width
set(findall(gcf,'type','text'),'FontSize',10);    %text size
ax=gca;
ax.FontSize = 10;
ax.TickLabelInterpreter = 'latex';

figure(4)
scatter3(calib_center(1,:),calib_center(2,:) ,calib_center(3,:));
xlabel('Xaxis','Interpreter','Latex');
ylabel('Yaxis','Interpreter','Latex');
zlabel('Zaxis','Interpreter','Latex');
title('Platform position after calibration','Interpreter','Latex');
set(findall(gcf,'type','line'),'linewidth',1);  %line width
set(findall(gcf,'type','text'),'FontSize',10);    %text size
ax=gca;
ax.FontSize = 10;
ax.TickLabelInterpreter = 'latex';
%% --------------------------------------Figure Data -------------------------------------------------
row_forces_PS = Rp_0 * row_forces_0;
error_x = calib_robot(1,:) - calib_center(1,:);
error_y = calib_robot(2,:) - calib_center(2,:);
error_z = calib_robot(3,:) - calib_center(3,:);
templen = size(error_x);
robotdis = sqrt(calib_robot(1,:).^2+calib_robot(2,:).^2+calib_robot(3,:).^2);
centerdis = sqrt(calib_center(1,:).^2+calib_center(2,:).^2+calib_center(3,:).^2);
in = 2001+1693;
fin = in+5999;
row_forces_PS = row_forces_PS(in:fin,:);
velocity = row_robotpos_PS(in+1:fin+1,:) - row_robotpos_PS(in:fin,:);
error_x = error_x(in:fin);
error_y = error_y(in:fin);
error_z = error_z(in:fin);
robotdis = robotdis(in:fin);
impulse =impulse(in:fin);
centerdis = centerdis(in:fin);
len = size(error_x);
T = linspace (0,len(2),len(2)) ; 
T=T*0.01;
%%  ----------------------Plot (Positions of Robot and root canal) -------------------
figure(1), subplot(3,1,1)
plot(T,calib_robot(1,:),'color',[0 0 1]);hold on;
plot(T,calib_center(1,:),'color',[1 0 0]);hold on;
ylabel('X axis [mm]','Interpreter','Latex');
xlabel('Time [s]','Interpreter','Latex');
L0=legend('Robot','Root','Interpreter','Latex');
set(L0,'FontSize',16);                            %legend size

figure(1), subplot(3,1,2)
plot(T,calib_robot(1,:),'color',[0 0 1]);hold on;
plot(T,calib_center(1,:),'color',[1 0 0]);hold on;
ylabel('Y axis [mm]','Interpreter','Latex');
xlabel('Time [s]','Interpreter','Latex');

figure(1), subplot(3,1,3)
plot(T,calib_robot(1,:),'color',[0 0 1]);hold on;
plot(T,calib_center(1,:),'color',[1 0 0]);hold on;
ylabel('Z axis [mm]','Interpreter','Latex');
xlabel('Time [s]','Interpreter','Latex');

set(findall(gcf,'type','line'),'linewidth',1);  %line width
set(findall(gcf,'type','text'),'FontSize',10);    %text size
ax=gca;
ax.FontSize = 10;
ax.TickLabelInterpreter = 'latex';
set(gcf,'position',[0 0 1900 4000]); %Figure Size
sgtitle('Horizontal','Interpreter','Latex')
grid on;
%%  ----------------------Plot (Forces vs velocity(position difference) relative to PhaseSpace frame ) -------------------
figure(5), subplot(3,1,1)
title('Force vs Velocity','Interpreter','Latex');
yyaxis left
plot(T,row_forces_PS(:,1),'color',[0 0 1]);hold on;
ylabel('Force_x [mm]','Interpreter','Latex');
yyaxis right 
plot(T,velocity(:,1),'color',[1 0 0]);hold off;
ylabel('Velocity_x [mm]','Interpreter','Latex');
xlabel('Time [s]','Interpreter','Latex');
set(findall(gcf,'type','line'),'linewidth',1);  %line width
set(findall(gcf,'type','text'),'FontSize',10);    %text size
ax=gca;
ax.FontSize = 10;
ax.TickLabelInterpreter = 'latex';

figure(5), subplot(3,1,2)
yyaxis left
plot(T,row_forces_PS(:,2),'color',[0 0 1]);hold on;
ylabel('Force_y [mm]','Interpreter','Latex');
yyaxis right 
plot(T,velocity(:,2),'color',[1 0 0]);hold off;
ylabel('Velocity_y [mm]','Interpreter','Latex');
xlabel('Time [s]','Interpreter','Latex');
set(findall(gcf,'type','line'),'linewidth',1);  %line width
set(findall(gcf,'type','text'),'FontSize',10);    %text size
ax=gca;
ax.FontSize = 10;
ax.TickLabelInterpreter = 'latex';

figure(5), subplot(3,1,3)
yyaxis left
plot(T,row_forces_PS(:,3),'color',[0 0 1]);hold on;
ylabel('Force_z [mm]','Interpreter','Latex');
yyaxis right 
plot(T,velocity(:,3),'color',[1 0 0]);hold off;
ylabel('Velocity_z [mm]','Interpreter','Latex');
xlabel('Time [s]','Interpreter','Latex');
set(findall(gcf,'type','line'),'linewidth',1);  %line width
set(findall(gcf,'type','text'),'FontSize',10);    %text size
ax=gca;
ax.FontSize = 10;
ax.TickLabelInterpreter = 'latex';

set(gcf,'position',[0 0 1900 4000]); %Figure Size
grid on;


% %%  ----------------------Plot (Force integration, Moving distance of robot, Moving distance of platform  ) -------------------
% figure(2), subplot(3,1,2)
% plot(T,robotdis-robotdis(1),'color',[0 0 1]);hold on;
% %plot(T,ith,'color',[1 0 0]);hold off;
% %xlim([0 len(1)]);
% %ylim([-6 6]);                                   
% %xticks(0:1000:len(1));
% %yticks(-6:1:6);
% %L0=legend('','Interpreter','Latex');
% %set(L0,'FontSize',16);                            %legend size
% xlabel('Time [s]','Interpreter','Latex');
% ylabel('distance [mm]','Interpreter','Latex');
% title('Moving distance of robot','Interpreter','Latex');
% set(findall(gcf,'type','line'),'linewidth',1);  %line width
% set(findall(gcf,'type','text'),'FontSize',10);    %text size
% ax=gca;
% ax.FontSize = 10;
% ax.TickLabelInterpreter = 'latex';
% set(gcf,'position',[0 0 1900 4000]); %Figure Size
% grid on;
% 
% figure(2), subplot(3,1,1)
% plot(T,(impulse-impulse(1)),'color',[0 0 1]);hold on;
% %plot(T,ith,'color',[1 0 0]);hold off;
% %xlim([0 12000]);
% %ylim([-10 10]);                                   
% %xticks(0:1000:len);
% %yticks(-10:1:10);
% %L0=legend('','Interpreter','Latex');
% %set(L0,'FontSize',16);                            %legend size
% xlabel('Time [s]','Interpreter','Latex');
% ylabel('J [N*m]','Interpreter','Latex');
% title('Force integration','Interpreter','Latex');
% set(findall(gcf,'type','line'),'linewidth',1);  %line width
% set(findall(gcf,'type','text'),'FontSize',10);    %text size
% ax=gca;
% ax.FontSize = 10;
% ax.TickLabelInterpreter = 'latex';
% set(gcf,'position',[0 0 1900 4000]); %Figure Size
% grid on;
% 
% figure(2), subplot(3,1,3)
% plot(T,(centerdis-centerdis(1)),'color',[0 0 1]);hold on;
% %plot(T,ith,'color',[1 0 0]);hold off;
% %xlim([0 12000]);
% %ylim([-10 10]);                                   
% %xticks(0:1000:len);
% %yticks(-10:1:10);
% %L0=legend('','Interpreter','Latex');
% %set(L0,'FontSize',16);                            %legend size
% xlabel('Time [s]','Interpreter','Latex');
% ylabel('distance [mm]','Interpreter','Latex');
% title('Moving distance of platform','Interpreter','Latex');
% set(findall(gcf,'type','line'),'linewidth',1);  %line width
% set(findall(gcf,'type','text'),'FontSize',10);    %text size
% ax=gca;
% ax.FontSize = 10;
% ax.TickLabelInterpreter = 'latex';
% set(gcf,'position',[0 0 1900 4000]); %Figure Size
% grid on;