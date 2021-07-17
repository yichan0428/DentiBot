clear; clc;
close all;
%% -------------------------- Read file ------------------------
filename = '1_3_500_20.xlsx';  K = 500;

data = readtable(filename);
data = table2array(data);
data=data(:,:);
lenth = size(data(:,1));
row_robotpos_PS = data(:,1:3); 
row_led_1 = data(:,4:6);
row_led_2 = data(:,7:9);
row_led_3 = data(:,10:12);
center = (row_led_1 + row_led_2 + row_led_3 )/3;
impulse = sqrt(data(:,13).^2+data(:,14).^2+data(:,15).^2);
row_forces_PS = data(:,16:18);
row_sensor_t = data(:,19:24);
row_robotpos_0 = data(:,25:27);
row_robotang_0 = data(:,28:30);
row_armangle_0 = data(:,31:36);
row_command_velocity_PS = data(:,37:39);
row_command_pos_PS = data(:,40:42);
row_sensor_m = data(:,43:48);
row_filetorque = data(:,49);
row_Fz_des = data(:,50);
row_time=data(:,51);

%% -------------------------------Fitting--------------------------------
Zax = -cross(mean(row_led_1(1:50,:))-mean(row_led_3(1:50,:)),mean(row_led_1(1:50,:))-mean(row_led_2(1:50,:)));
Zax = Zax/norm(Zax);
tempX = [Zax(2),-Zax(1),0];     
tempX = tempX/norm(tempX); 
tempY = cross( Zax , tempX );
tempY = tempY/norm(tempY); 
trans1 = [tempX ; tempY ; Zax];%從phasespace看平台
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

robotdis = sqrt(calib_robot(1,:).^2+calib_robot(2,:).^2+calib_robot(3,:).^2);
centerdis = sqrt(calib_center(1,:).^2+calib_center(2,:).^2+calib_center(3,:).^2);
in = 1;
[r c] = size(data);
fin = in+ (r - mod(r,100)-1);
%fin = 12000;
row_forces_PS = row_forces_PS(in:fin,:);
row_forces_PS = trans2*trans1*row_forces_PS.'; 
row_forces_PS = row_forces_PS.';
%{stew} 
diff = row_robotpos_PS(in+3:fin+3,:) - row_robotpos_PS(in:fin,:);   %{PS}
velocity = diff ./ 10e-3;   %/ sampling time
velocity = trans2*trans1*velocity.';      %{stew} 
velocity = velocity.';

row_command_velocity_PS = row_command_velocity_PS(in:fin,:);
row_command_velocity_PS = trans2*trans1*row_command_velocity_PS.'; 
row_command_velocity_PS = row_command_velocity_PS.';
robotdis = robotdis(in:fin);
impulse =impulse(in:fin);
calib_robot = calib_robot(:,in:fin);
centerdis = centerdis(in:fin);
calib_center = calib_center(:,in:fin);
relative_dis = calib_robot - calib_center;
len = fin-in+1;
T = linspace (0,len,len) ; 
T=T*0.01;


fc =3;                                  %% filter 3hz
Ts = 10e-3;
fs = 1/Ts;
[b,a] = butter(1,fc/(fs/2));
velocity_filt = filtfilt(b,a,velocity);

axis_size = 12;
%%  ----------------------Plot (Positions of Robot and root canal) -------------------
yticks_position_low = -15;                  % Set moving range
yticks_position = 15;
figure(1), subplot(3,1,1)
plot(T,calib_robot(1,:) - mean(calib_center(1,:)),'color',[0 0 1]);hold on;      % minusing center's mean is correct
plot(T,calib_center(1,:)- mean(calib_center(1,:)),'color',[1 0 0]);              
plot(T,relative_dis(1,:),'color','k');hold off;
ylabel('X axis (mm)','Interpreter','Latex');
ylim([yticks_position_low yticks_position]);
yticks(linspace(yticks_position_low,yticks_position,11));
xlabel('Time (s)','Interpreter','Latex');
set(gca,'fontSize',axis_size,'TickLabelInterpreter','latex');
%title('Position','Interpreter','Latex');
L0=legend('Robot','Root','Relative distance','Interpreter','Latex');
set(L0,'FontSize',12);                           %legend size
grid on;

figure(1), subplot(3,1,2)
plot(T,calib_robot(2,:) - mean(calib_center(2,:)),'color',[0 0 1]);hold on;      % minusing center's mean is correct
plot(T,calib_center(2,:)- mean(calib_center(2,:)),'color',[1 0 0]);          
plot(T,relative_dis(2,:),'color','k');hold off;
ylabel('Y axis (mm)','Interpreter','Latex');
ylim([yticks_position_low yticks_position]);
yticks(linspace(yticks_position_low,yticks_position,11));
xlabel('Time (s)','Interpreter','Latex');
set(gca,'fontSize',axis_size,'TickLabelInterpreter','latex');
grid on;

figure(1), subplot(3,1,3)
plot(T,calib_robot(3,:) - mean(calib_center(3,:)),'color',[0 0 1]);hold on;      % minusing center's mean is correct
plot(T,calib_center(3,:)- mean(calib_center(3,:)),'color',[1 0 0]);          
plot(T,relative_dis(3,:),'color','k');hold off;
ylabel('Z axis (mm)','Interpreter','Latex');
ylim([-10 70]);
yticks(linspace(-10,70,9));
xlabel('Time  (s)','Interpreter','Latex');
set(gca,'fontSize',axis_size,'TickLabelInterpreter','latex');
grid on;

set(gcf,'position',[0 0 1900 1000]); %Figure Size
sgtitle('\ \ \ \ \ \ \ \ \ Position','Interpreter','Latex','fontsize',24);
%%  ----------------------Plot (Forces vs velocity(position difference) relative to PhaseSpace frame ) -------------------

B = 80;
gain = (K * 0.002 / B)*1000;
row_command_velocity_PS = row_command_velocity_PS./500;   
yticks_force_low = -5;                                          
yticks_force_high = 5;
interval = (yticks_force_high - yticks_force_low)+1;
yticks_velocity_low = yticks_force_low*gain;                  %force : velocity = 1 : gain
yticks_velocity_high = yticks_force_high*gain;   

figure(5), subplot(3,1,1)
title('X axis','Interpreter','Latex');
yyaxis left
plot(T,row_forces_PS(:,1),'color','b');hold on;
ylabel('Force (N)','Interpreter','Latex');
ylim([yticks_force_low yticks_force_high]);
yticks(linspace(yticks_force_low,yticks_force_high,interval));
set(gca,'fontSize',axis_size,'TickLabelInterpreter','latex','ycolor','k');
yyaxis right 
plot(T,velocity_filt(:,1),'color','r');hold on;
plot(T,row_command_velocity_PS(:,1),'color','k');hold off;
ylabel('Velocity (mm/s)','Interpreter','Latex');
ylim([yticks_velocity_low yticks_velocity_high]);
yticks(linspace(yticks_velocity_low,yticks_velocity_high,interval));
% xlabel('Time (s)','Interpreter','Latex');
set(gca,'fontSize',axis_size,'TickLabelInterpreter','latex','ycolor','k');
L0=legend('Force','Velocity','Velocity command','Interpreter','Latex');
set(L0,'FontSize',12);                           %legend size
grid on;

figure(5), subplot(3,1,2)
title('Y axis','Interpreter','Latex');
yyaxis left
plot(T,row_forces_PS(:,2),'color','b');hold on;
ylabel('Force (N)','Interpreter','Latex');
ylim([yticks_force_low yticks_force_high]);
yticks(linspace(yticks_force_low,yticks_force_high,interval));
set(gca,'fontSize',axis_size,'TickLabelInterpreter','latex','ycolor','k');
yyaxis right 
plot(T,velocity_filt(:,2),'color','r');
plot(T,row_command_velocity_PS(:,2),'color','k');hold off;
ylabel('Velocity (mm/s)','Interpreter','Latex');
ylim([yticks_velocity_low yticks_velocity_high]);
yticks(linspace(yticks_velocity_low,yticks_velocity_high,interval));
% xlabel('Time (s)','Interpreter','Latex');
set(gca,'fontSize',axis_size,'TickLabelInterpreter','latex','ycolor','k');
grid on;

figure(5), subplot(3,1,3)
title('Z axis','Interpreter','Latex');
yyaxis left
plot(T,row_forces_PS(:,3),'color','b');hold on;
ylabel('Force (N)','Interpreter','Latex');
ylim([yticks_force_low yticks_force_high]);
yticks(linspace(yticks_force_low,yticks_force_high,interval));
set(gca,'fontSize',axis_size,'TickLabelInterpreter','latex','ycolor','k');
yyaxis right 
plot(T,velocity_filt(:,3),'color','r');
plot(T,row_command_velocity_PS(:,3),'color','k');hold off;
ylabel('Velocity (mm/s)','Interpreter','Latex');
ylim([yticks_velocity_low yticks_velocity_high]);
yticks(linspace(yticks_velocity_low,yticks_velocity_high,interval));
xlabel('Time (s)','Interpreter','Latex');
set(gca,'fontSize',axis_size,'TickLabelInterpreter','latex','ycolor','k');
grid on;

set(gcf,'position',[0 0 1900 1000]); %Figure Size
sgtitle('\ \ \ \ \ \ \ \ \ Force vs Velocity','Interpreter','Latex','fontsize',24);
%% plot force and position

figure(6), subplot(3,1,1)
title('X axis','Interpreter','Latex');
yyaxis left
plot(T,row_forces_PS(:,1),'color','b');hold on;
ylabel('Force (N)','Interpreter','Latex');
ylim([yticks_force_low yticks_force_high]);
yticks(linspace(yticks_force_low,yticks_force_high,interval));
set(gca,'fontSize',axis_size,'TickLabelInterpreter','latex','ycolor','k');
yyaxis right 
plot(T,calib_robot(1,:) - mean(calib_center(1,:)),'color','r');
hold on;
plot(T,calib_center(1,:) - mean(calib_center(1,:)),'color','k');hold off;
ylabel('Position (mm)','Interpreter','Latex');
ylim([yticks_position_low yticks_position]);
yticks(linspace(yticks_position_low,yticks_position,interval));
% xlabel('Time (s)','Interpreter','Latex');
set(gca,'fontSize',axis_size,'TickLabelInterpreter','latex','ycolor','k');
L0=legend('Force','Position of Robot','Position of Root','Latex');
set(L0,'FontSize',12);                           %legend size
grid on;

figure(6), subplot(3,1,2)
title('Y axis','Interpreter','Latex');
yyaxis left
plot(T,row_forces_PS(:,2),'color','b');hold on;
ylabel('Force (N)','Interpreter','Latex');
ylim([yticks_force_low yticks_force_high]);
yticks(linspace(yticks_force_low,yticks_force_high,interval));
set(gca,'fontSize',axis_size,'TickLabelInterpreter','latex','ycolor','k');
yyaxis right 
plot(T,calib_robot(2,:) - mean(calib_center(2,:)),'color','r');
hold on;
plot(T,calib_center(2,:) - mean(calib_center(2,:)),'color','k');hold off;
ylabel('Position (mm)','Interpreter','Latex');
ylim([yticks_position_low yticks_position]);
yticks(linspace(yticks_position_low,yticks_position,interval));
% xlabel('Time (s)','Interpreter','Latex');
set(gca,'fontSize',axis_size,'TickLabelInterpreter','latex','ycolor','k');
grid on;

figure(6), subplot(3,1,3)
title('Z axis','Interpreter','Latex');
yyaxis left
plot(T,row_forces_PS(:,3),'color','b');hold on;
ylabel('Force (N)','Interpreter','Latex');
ylim([yticks_force_low yticks_force_high]);
yticks(linspace(yticks_force_low,yticks_force_high,interval));
set(gca,'fontSize',axis_size,'TickLabelInterpreter','latex','ycolor','k');
yyaxis right 
plot(T,calib_robot(3,:) - mean(calib_center(3,:)),'color','r');
hold on;
plot(T,calib_center(3,:) - mean(calib_center(3,:)),'color','k');hold off;
ylabel('Position (mm)','Interpreter','Latex');
ylim([yticks_position_low yticks_position]);
yticks(linspace(yticks_position_low,yticks_position,interval));
xlabel('Time (s)','Interpreter','Latex');
set(gca,'fontSize',axis_size,'TickLabelInterpreter','latex','ycolor','k');
grid on;

set(gcf,'position',[0 0 1900 1000]); %Figure Size
sgtitle('\ \ \ \ \ \ \ \ \ Force vs Position','Interpreter','Latex','fontsize',24);
%%----------------------Plot (Torque vs Fz_des) -------------------
% figure(2)
% title('Torque vs Fz_des','Interpreter','Latex');
% yyaxis left
% plot(T,row_filetorque,'color','b');hold on;
% ylabel('Torque (mNm)','Interpreter','Latex');
% ylim([0 30]);
% yticks(linspace(0,30,7));
% set(gca,'fontSize',axis_size,'TickLabelInterpreter','latex','ycolor','k');
% yyaxis right 
% plot(T,row_Fz_des,'color','r');hold off;
% ylabel('Fz_des (Nm)','Interpreter','Latex');
% ylim([-0.2 1]);
% yticks(linspace(-0.2,1,13));
% set(gca,'fontSize',axis_size,'TickLabelInterpreter','latex','ycolor','k');
% xlabel('Time (s)','Interpreter','Latex');
% set(gca,'fontSize',axis_size,'TickLabelInterpreter','latex','ycolor','k');
% 
% L0=legend('Torque','Fz_des','Interpreter','Latex');
% set(L0,'FontSize',12);                           %legend size
% 
% 
% set(gcf,'position',[0 700 1900 300]);
% grid on;