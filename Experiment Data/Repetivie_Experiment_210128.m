%% Exp2
dt=0.01;
N=length(Repetitiveexp2);
T=dt.*(0:1:N-1);
ith=400.*ones(1,N);

figure(1), subplot(3,1,1)
plot(T,Repetitiveexp2(:,1),'color',[0 0 1]);hold on;
plot(T,ith,'color',[1 0 0]);hold off;
xlim([0 N*dt]);
ylim([0 2000]);                                   
xticks(0:10:N*dt);
yticks(0:200:2000);
L0=legend('$i_{a}$(t)','$i_{th}$','Interpreter','Latex');
set(L0,'FontSize',16);                            %legend size
xlabel('Time [s]','Interpreter','Latex');
ylabel('Current [mA]','Interpreter','Latex');
title('Motor Current','Interpreter','Latex');
set(findall(gcf,'type','line'),'linewidth',1);  %line width
set(findall(gcf,'type','text'),'FontSize',10);    %text size
ax=gca;
ax.FontSize = 10;
ax.TickLabelInterpreter = 'latex';
set(gcf,'position',[0 0 1900 4000]); %Figure Size
grid on;



figure(1), subplot(3,1,2)
plot(T,Repetitiveexp2(:,2),'color',[1 0 0]); hold on;
plot(T,Repetitiveexp2(:,3),'color',[0 1 0]);
plot(T,Repetitiveexp2(:,4),'color',[0 0 1]);hold off;
xlim([0 N*dt]);
ylim([-2.5 0.5]);                                   
xticks(0:10:N*dt);
yticks(-2.5:0.5:0.5);
L0=legend('$F_{x}$','$F_{y}$','$F_{z}$','Interpreter','Latex');
set(L0,'FontSize',16);                            %legend size
xlabel('Time [s]','Interpreter','Latex');
ylabel('Forces [N]','Interpreter','Latex');
title('Forces','Interpreter','Latex');
set(findall(gcf,'type','line'),'linewidth',1);  %line width
set(findall(gcf,'type','text'),'FontSize',12);    %text size
ax=gca;
ax.FontSize = 12;
ax.TickLabelInterpreter = 'latex';
%set(gcf,'position',[500 500 1000 350]); %Figure Size
grid on;
% grid minor;


figure(1), subplot(3,1,3)
plot(T,Repetitiveexp2(:,5),'color',[1 0 0]); hold on;
plot(T,Repetitiveexp2(:,6),'color',[0 1 0]);
plot(T,Repetitiveexp2(:,7),'color',[0 0 1]);hold off;
xlim([0 N*dt]);
ylim([-0.1 0.15]);                                   
xticks(0:10:N*dt);
yticks(-0.1:0.05:0.15);
L0=legend('$\tau_{x}$','$\tau_{y}$','$\tau_{z}$','Interpreter','Latex');
set(L0,'FontSize',16);                            %legend size
xlabel('Time [s]','Interpreter','Latex');
ylabel('Torques [N $\cdot$ m]','Interpreter','Latex');
title('Torques','Interpreter','Latex');
set(findall(gcf,'type','line'),'linewidth',1);  %line width
set(findall(gcf,'type','text'),'FontSize',12);    %text size
ax=gca;
ax.FontSize = 12;
ax.TickLabelInterpreter = 'latex';
%set(gcf,'position',[500 100 1000 350]); %Figure Size
grid on;

%% Exp3
dt=0.01;
N=length(Repetitiveexp3);
T=dt.*(0:1:N-1);
ith=400.*ones(1,N);

figure(2),subplot(3,1,1),
plot(T,Repetitiveexp3(:,1),'color',[0 0 1]);hold on;
plot(T,ith,'color',[1 0 0]);hold off;
xlim([0 N*dt]);
ylim([0 2000]);                                   
xticks(0:10:N*dt);
yticks(0:200:2000);
L0=legend('$i_{a}$(t)','$i_{th}$','Interpreter','Latex');
set(L0,'FontSize',16);                            %legend size
xlabel('Time [s]','Interpreter','Latex');
ylabel('Current [mA]','Interpreter','Latex');
title('Motor Current','Interpreter','Latex');
set(findall(gcf,'type','line'),'linewidth',1);  %line width
set(findall(gcf,'type','text'),'FontSize',10);    %text size
ax=gca;
ax.FontSize = 10;
ax.TickLabelInterpreter = 'latex';
set(gcf,'position',[0 0 1900 4000]); %Figure Size
grid on;



figure(2), subplot(3,1,2)
plot(T,Repetitiveexp3(:,2),'color',[1 0 0]); hold on;
plot(T,Repetitiveexp3(:,3),'color',[0 1 0]);
plot(T,Repetitiveexp3(:,4),'color',[0 0 1]);hold off;
xlim([0 N*dt]);
ylim([-3.5 0.5]);                                   
xticks(0:10:N*dt);
yticks(-3.5:0.5:0.5);
L0=legend('$F_{x}$','$F_{y}$','$F_{z}$','Interpreter','Latex');
set(L0,'FontSize',16);                            %legend size
xlabel('Time [s]','Interpreter','Latex');
ylabel('Forces [N]','Interpreter','Latex');
title('Forces','Interpreter','Latex');
set(findall(gcf,'type','line'),'linewidth',1);  %line width
set(findall(gcf,'type','text'),'FontSize',12);    %text size
ax=gca;
ax.FontSize = 12;
ax.TickLabelInterpreter = 'latex';
%set(gcf,'position',[500 500 1000 350]); %Figure Size
grid on;
% grid minor;


figure(2), subplot(3,1,3)
plot(T,Repetitiveexp3(:,5),'color',[1 0 0]); hold on;
plot(T,Repetitiveexp3(:,6),'color',[0 1 0]);
plot(T,Repetitiveexp3(:,7),'color',[0 0 1]);hold off;
xlim([0 N*dt]);
ylim([-0.1 0.3]);                                   
xticks(0:10:N*dt);
yticks(-0.1:0.05:0.3);
L0=legend('$\tau_{x}$','$\tau_{y}$','$\tau_{z}$','Interpreter','Latex');
set(L0,'FontSize',16);                            %legend size
xlabel('Time [s]','Interpreter','Latex');
ylabel('Torques [N $\cdot$ m]','Interpreter','Latex');
title('Torques','Interpreter','Latex');
set(findall(gcf,'type','line'),'linewidth',1);  %line width
set(findall(gcf,'type','text'),'FontSize',12);    %text size
ax=gca;
ax.FontSize = 12;
ax.TickLabelInterpreter = 'latex';
%set(gcf,'position',[500 100 1000 350]); %Figure Size
grid on;