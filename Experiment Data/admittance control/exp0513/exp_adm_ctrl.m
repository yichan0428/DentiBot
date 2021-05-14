clear; clc;
close all;
%% -------------------------- Read file ------------------------
 ccfilename = 'vertical0514_2.xlsx';         
%filename = 'horizontal0514_2.xlsx';
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
%% -------------------------------Fitting--------------------------------
Zax = cross(mean(row_led_1(1:50,:))-mean(row_led_3(1:50,:)),mean(row_led_1(1:50,:))-mean(row_led_2(1:50,:)));
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

%% ----------------------------- Rotation matrix {0}  -> {PS} --------------------------
%Rp_0 = row_robotpos_PS.' * pinv(row_robotpos_0.');   % RA = B --> R = Apinv(B)

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
in = 1500;
fin = in+9999;
row_forces_PS = row_forces_PS(in:fin,:);
row_forces_PS = trans2*trans1*row_forces_PS.'; 
row_forces_PS = row_forces_PS.';
%{stew} 
velocity = row_robotpos_PS(in+3:fin+3,:) - row_robotpos_PS(in:fin,:);   %{PS}
velocity = trans2*trans1*velocity.';      %{stew} 
velocity = velocity.';
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
figure(1), subplot(3,1,1)
plot(T,calib_robot(1,:) - mean(calib_center(1,:)),'color',[0 0 1]);hold on;      % minusing center's mean is correct
plot(T,calib_center(1,:)- mean(calib_center(1,:)),'color',[1 0 0]);              
plot(T,relative_dis(1,:),'color','k');hold off;
ylabel('X axis (mm)','Interpreter','Latex');
ylim([-25 25]);
yticks(linspace(-25,25,11));
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
ylim([-25 25]);
yticks(linspace(-25,25,50/5+1));
xlabel('Time (s)','Interpreter','Latex');
set(gca,'fontSize',axis_size,'TickLabelInterpreter','latex');
grid on;

figure(1), subplot(3,1,3)
plot(T,calib_robot(3,:) - mean(calib_center(3,:)),'color',[0 0 1]);hold on;      % minusing center's mean is correct
plot(T,calib_center(3,:)- mean(calib_center(3,:)),'color',[1 0 0]);          
plot(T,relative_dis(3,:),'color','k');hold off;
ylabel('Z axis (mm)','Interpreter','Latex');
ylim([-25 85]);
yticks(linspace(-25,85,110/10+1));
xlabel('Time  (s)','Interpreter','Latex');
set(gca,'fontSize',axis_size,'TickLabelInterpreter','latex');
grid on;

set(gcf,'position',[0 0 1900 1000]); %Figure Size
sgtitle('\ \ \ \ \ \ \ \ \ Position','Interpreter','Latex','fontsize',24);
%%  ----------------------Plot (Forces vs velocity(position difference) relative to PhaseSpace frame ) -------------------
figure(5), subplot(3,1,1)
title('X axis','Interpreter','Latex');
yyaxis left
plot(T,row_forces_PS(:,1),'color',[0 0 1]);hold on;
ylabel('Force (N)','Interpreter','Latex');
ylim([-5 5]);
yticks(linspace(-5,5,10/1+1));
set(gca,'fontSize',axis_size,'TickLabelInterpreter','latex','ycolor','k');
yyaxis right 
plot(T,velocity_filt(:,1),'color',[1 0 0]);hold off;
ylabel('Velocity (mm/s)','Interpreter','Latex');
ylim([-0.5 0.5]);
yticks(linspace(-0.5,0.5,1/0.1+1));
% xlabel('Time (s)','Interpreter','Latex');
set(gca,'fontSize',axis_size,'TickLabelInterpreter','latex','ycolor','k');
L0=legend('Force','Velocity','Interpreter','Latex');
set(L0,'FontSize',12);                           %legend size
grid on;

figure(5), subplot(3,1,2)
title('Y axis','Interpreter','Latex');
yyaxis left
plot(T,row_forces_PS(:,2),'color',[0 0 1]);hold on;
ylabel('Force (N)','Interpreter','Latex');
ylim([-5 5]);
yticks(linspace(-5,5,10/1+1));
set(gca,'fontSize',axis_size,'TickLabelInterpreter','latex','ycolor','k');
yyaxis right 
plot(T,velocity_filt(:,2),'color',[1 0 0]);hold off;
ylabel('Velocity (mm/s)','Interpreter','Latex');
ylim([-0.5 0.5]);
yticks(linspace(-0.5,0.5,1/0.1+1));
% xlabel('Time (s)','Interpreter','Latex');
set(gca,'fontSize',axis_size,'TickLabelInterpreter','latex','ycolor','k');
grid on;

figure(5), subplot(3,1,3)
title('Z axis','Interpreter','Latex');
yyaxis left
plot(T,row_forces_PS(:,3),'color',[0 0 1]);hold on;
ylabel('Force (N)','Interpreter','Latex');
ylim([-5 5]);
yticks(linspace(-5,5,10/1+1));
set(gca,'fontSize',axis_size,'TickLabelInterpreter','latex','ycolor','k');
yyaxis right 
plot(T,velocity_filt(:,3),'color',[1 0 0]);hold off;
ylabel('Velocity (mm/s)','Interpreter','Latex');
ylim([-0.5 0.5]);
yticks(linspace(-0.5,0.5,1/0.1+1));
xlabel('Time (s)','Interpreter','Latex');
set(gca,'fontSize',axis_size,'TickLabelInterpreter','latex','ycolor','k');
grid on;

set(gcf,'position',[0 0 1900 1000]); %Figure Size
sgtitle('\ \ \ \ \ \ \ \ \ Force vs Velocity','Interpreter','Latex','fontsize',24);

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