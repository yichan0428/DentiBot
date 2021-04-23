dt=0.01;
N=length(Repetitiveexp2);
T=dt.*(0:1:N-1);
ith=400.*ones(1,N);

figure(1);
plot(T,Repetitiveexp2(:,1),'color',[0.3 0.3 0.3]);hold on;
plot(T,ith,'color',[0.5 0.5 0.5]);hold off;
xlim([0 N*dt]);
ylim([0 450]);                                   
xticks(0:10:N*dt);
yticks(0:50:450);
L0=legend('$i_{a}$(t)','$i_{th}$','Interpreter','Latex');
set(L0,'FontSize',16);                            %legend size
xlabel('Time [s]','Interpreter','Latex');
ylabel('Current [mA]','Interpreter','Latex');
title('Motor Current','Interpreter','Latex');
set(findall(gcf,'type','line'),'linewidth',1.5);  %line width
set(findall(gcf,'type','text'),'FontSize',10);    %text size
ax=gca;
ax.FontSize = 10;
ax.TickLabelInterpreter = 'latex';
set(gcf,'position',[500 500 1000 350]); %Figure Size




figure(2);
plot(T,Repetitiveexp2(:,2)); hold on;
plot(T,Repetitiveexp2(:,3));
plot(T,Repetitiveexp2(:,4));hold off;
xlim([0 N*dt]);
ylim([-1 1]);                                   
xticks(0:10:N*dt);
yticks(-1:0.2:1);
L0=legend('$F_{x}$','$F_{y}$','$F_{z}$','Interpreter','Latex');
set(L0,'FontSize',16);                            %legend size
xlabel('Time [s]','Interpreter','Latex');
ylabel('Forces [N]','Interpreter','Latex');
title('Forces','Interpreter','Latex');
set(findall(gcf,'type','line'),'linewidth',1.5);  %line width
set(findall(gcf,'type','text'),'FontSize',12);    %text size
ax=gca;
ax.FontSize = 12;
ax.TickLabelInterpreter = 'latex';
set(gcf,'position',[500 500 1000 350]); %Figure Size


figure(3);
plot(T,Repetitiveexp2(:,5)); hold on;
plot(T,Repetitiveexp2(:,6));
plot(T,Repetitiveexp2(:,7));hold off;
xlim([0 N*dt]);
ylim([-0.2 0.2]);                                   
xticks(0:10:N*dt);
yticks(-0.2:0.05:0.2);
L0=legend('$\tau_{x}$','$\tau_{y}$','$\tau_{z}$','Interpreter','Latex');
set(L0,'FontSize',16);                            %legend size
xlabel('Time [s]','Interpreter','Latex');
ylabel('Torques [N $\cdot$ m]','Interpreter','Latex');
title('Torques','Interpreter','Latex');
set(findall(gcf,'type','line'),'linewidth',1.5);  %line width
set(findall(gcf,'type','text'),'FontSize',12);    %text size
ax=gca;
ax.FontSize = 12;
ax.TickLabelInterpreter = 'latex';
set(gcf,'position',[500 500 1000 350]); %Figure Size
