color_m=[[0.4660, 0.6740, 0.1880]; [0, 0.4470, 0.7410]; [0.9290, 0.6940, 0.1250]; [0.25, 0.25, 0.25]; [0.4940, 0.1840, 0.5560]; [0.6350, 0.0780, 0.1840]];
load("FTEBE.mat");

w_tilde_ftebe = dilute_data(sqrt(out.esti_norm_1(:,2).^2 + out.esti_norm_2(:,2).^2 + out.esti_norm_3(:,2).^2 + out.esti_norm_4(:,2).^2 + out.esti_norm_5(:,2).^2 + out.esti_norm_6(:,2).^2), 10);
s_tilde_ftebe = dilute_data(sqrt(sum(out.esti_s_tilde_1(:,2:4).^2 + out.esti_s_tilde_2(:,2:4).^2 + out.esti_s_tilde_3(:,2:4).^2 + out.esti_s_tilde_4(:,2:4).^2 + out.esti_s_tilde_5(:,2:4).^2 + out.esti_s_tilde_6(:,2:4).^2,2)), 10);
x_tilde_ftebe = dilute_data(sqrt(sum(out.esti_x_tilde_1(:,2:4).^2 + out.esti_x_tilde_2(:,2:4).^2 + out.esti_x_tilde_3(:,2:4).^2 + out.esti_x_tilde_4(:,2:4).^2 + out.esti_x_tilde_5(:,2:4).^2 + out.esti_x_tilde_6(:,2:4).^2,2)), 10);
time = dilute_data(out.tout, 10);

load("EBE.mat");
w_tilde_ebe = dilute_data(sqrt(out.esti_norm_1(:,2).^2 + out.esti_norm_2(:,2).^2 + out.esti_norm_3(:,2).^2 + out.esti_norm_4(:,2).^2 + out.esti_norm_5(:,2).^2 + out.esti_norm_6(:,2).^2), 10);
s_tilde_ebe = dilute_data(sqrt(sum(out.esti_s_tilde_1(:,2:4).^2 + out.esti_s_tilde_2(:,2:4).^2 + out.esti_s_tilde_3(:,2:4).^2 + out.esti_s_tilde_4(:,2:4).^2 + out.esti_s_tilde_5(:,2:4).^2 + out.esti_s_tilde_6(:,2:4).^2,2)), 10);
x_tilde_ebe = dilute_data(sqrt(sum(out.esti_x_tilde_1(:,2:4).^2 + out.esti_x_tilde_2(:,2:4).^2 + out.esti_x_tilde_3(:,2:4).^2 + out.esti_x_tilde_4(:,2:4).^2 + out.esti_x_tilde_5(:,2:4).^2 + out.esti_x_tilde_6(:,2:4).^2,2)), 10);

load("CTE.mat");
w_tilde_cte = dilute_data(sqrt(out.esti_norm_1(:,2).^2 + out.esti_norm_2(:,2).^2 + out.esti_norm_3(:,2).^2 + out.esti_norm_4(:,2).^2 + out.esti_norm_5(:,2).^2 + out.esti_norm_6(:,2).^2), 10);
% x_tilde_cte = dilute_data(sqrt(sum(out.esti_x_tilde_1(:,2:4).^2 + out.esti_x_tilde_2(:,2:4).^2 + out.esti_x_tilde_3(:,2:4).^2 + out.esti_x_tilde_4(:,2:4).^2 + out.esti_x_tilde_5(:,2:4).^2 + out.esti_x_tilde_6(:,2:4).^2,2)), 10);

load("FTESO.mat");
w_tilde_eso = dilute_data(sqrt(out.esti_norm_1(:,2).^2 + out.esti_norm_2(:,2).^2 + out.esti_norm_3(:,2).^2 + out.esti_norm_4(:,2).^2 + out.esti_norm_5(:,2).^2 + out.esti_norm_6(:,2).^2), 10);
x_tilde_eso = dilute_data(sqrt(sum(out.esti_x_tilde_1(:,2:4).^2 + out.esti_x_tilde_2(:,2:4).^2 + out.esti_x_tilde_3(:,2:4).^2 + out.esti_x_tilde_4(:,2:4).^2 + out.esti_x_tilde_5(:,2:4).^2 + out.esti_x_tilde_6(:,2:4).^2,2)), 10);


load("NBO.mat");
w_tilde_nbo = dilute_data(sqrt(out.esti_norm_1(:,2).^2 + out.esti_norm_2(:,2).^2 + out.esti_norm_3(:,2).^2 + out.esti_norm_4(:,2).^2 + out.esti_norm_5(:,2).^2 + out.esti_norm_6(:,2).^2), 10);
x_tilde_nbo = dilute_data(sqrt(sum(out.esti_x_tilde_1.signals.values.^2 + out.esti_x_tilde_2.signals.values.^2 + out.esti_x_tilde_3.signals.values.^2 + out.esti_x_tilde_4.signals.values.^2 + out.esti_x_tilde_5.signals.values.^2 + out.esti_x_tilde_6.signals.values.^2,2)), 10);

%%

figure(1)
hold on;
grid on;
box on;
set(gcf,'Position',[140,200,1500,600]);
plot(time,w_tilde_cte,'-','color',color_m(1,:), 'linewidth', 2);
plot(time,w_tilde_ebe,'--','color',color_m(2,:), 'linewidth', 2);
plot(time,w_tilde_ftebe,'-.','color',color_m(3,:), 'linewidth', 2);
plot(time,w_tilde_eso,'--','color',color_m(4,:), 'linewidth', 2);
plot(time,w_tilde_nbo,'-.','color',color_m(5,:), 'linewidth', 2);
set(gca,'FontSize',18);
set(gca,'fontname','times');
xlabel('Time (Second)','FontSize',18);
ylabel('Norm of estimation error','FontSize',18);
ylim([0 0.5]);

legend('CTE ($\widetilde{w}^N$)','EBE ($\widetilde{w}^E$)','FTEBE ($\widetilde{w}^E$)','FTESO ($\widetilde{w}^E$)','NBO ($\widetilde{w}^E$)','fontsize',18,'Orientation','vertical', 'interpreter', 'latex', 'Position', [0.182179097493489,0.683074822955663,0.128487569173177,0.152480732599894]);

axes('Position', [0.38785376344086,0.524444444444444,0.221035125448029,0.276424676209954]);
set(gca,'FontSize',14);
set(gca,'fontname','times');
hold on;
grid on;
box on;
plot(time,w_tilde_cte,'color',color_m(1,:), 'linewidth', 2);
plot(time,w_tilde_ebe,'color',color_m(2,:), 'linewidth', 2);
plot(time,w_tilde_ftebe,'color',color_m(3,:), 'linewidth', 2);
ylim([0 4500]);
yticks(0:500:4500);

axes('Position', [0.6464,0.524444444444444,0.221035125448029,0.276424676209954]);
set(gca,'FontSize',14);
set(gca,'fontname','times');
xlim([15 30]);
ylim([0 0.005]);
hold on;
grid on;
box on;
plot(time,w_tilde_cte,'color',color_m(1,:), 'linewidth', 2);
plot(time,w_tilde_ebe,'color',color_m(2,:), 'linewidth', 2);
plot(time,w_tilde_ftebe,'color',color_m(3,:), 'linewidth', 2);

%%

figure(2)
hold on;
grid on;
box on;
set(gcf,'Position',[140,200,1500,600]);
% plot(time,x_tilde_cte,'-','color',color_m(1,:), 'linewidth', 2);
plot(time,x_tilde_ebe,'-','color',color_m(2,:), 'linewidth', 2);
plot(time,x_tilde_ftebe,'-.','color',color_m(3,:), 'linewidth', 2);
plot(time,x_tilde_eso,'--','color',color_m(4,:), 'linewidth', 2);
plot(time,x_tilde_nbo,'-.','color',color_m(5,:), 'linewidth', 2);
set(gca,'FontSize',18);
set(gca,'fontname','times');
xlabel('Time (Second)','FontSize',18);
ylabel('$\|\widetilde{x}\|$','FontSize',18,'interpreter', 'latex');
% ylim([0 0.5]);

legend('EBE','FTEBE','FTESO','NBO','fontsize',18,'Orientation','vertical', 'interpreter', 'latex', 'Position', [0.319512430826822,0.698731858147517,0.128487569173177,0.201166662216187]);

axes('Position', [0.6464,0.524444444444444,0.221035125448029,0.276424676209954]);
set(gca,'FontSize',14);
set(gca,'fontname','times');
xlim([15 30]);
ylim([0 0.0006]);
hold on;
grid on;
box on;
plot(time,x_tilde_ebe,'-','color',color_m(2,:), 'linewidth', 2);
plot(time,x_tilde_ftebe,'-.','color',color_m(3,:), 'linewidth', 2);
plot(time,x_tilde_eso,'--','color',color_m(4,:), 'linewidth', 2);
% plot(time,x_tilde_nbo,'-.','color',color_m(5,:), 'linewidth', 2);



%%
figure(3)
hold on;
grid on;
box on;
set(gcf,'Position',[140,200,1500,600]);
plot(time,s_tilde_ebe,'-','color',color_m(2,:), 'linewidth', 2);
plot(time,s_tilde_ftebe,'-.','color',color_m(3,:), 'linewidth', 2);
set(gca,'FontSize',18);
set(gca,'fontname','times');
xlabel('Time (Second)','FontSize',18);
ylabel('$\|\widetilde{x}\|$','FontSize',18,'interpreter', 'latex');
legend('EBE','FTEBE','fontsize',18,'Orientation','vertical');

%% Convergence time for x
max(x_tilde_ebe(100:301))

% for i = 50 : 301
%     if x_tilde_ebe(i) < max(x_tilde_ebe(50:301))
%         i/10
%         break
%     end
% end

%%
max(x_tilde_ftebe(100:301))

% for i = 1 : 301
%     if x_tilde_ftebe(i) < max(x_tilde_ftebe(100:301))
%         i/10
%         break
%     end
% end

%%
max(x_tilde_eso(100:301))
% 
% for i = 1 : 301
%     if x_tilde_ftebe(i) < max(x_tilde_eso(100:301))
%         i/10
%         break
%     end
% end

%%
max(x_tilde_nbo(100:301))

% for i = 1 : 301
%     if x_tilde_ftebe(i) < max(x_tilde_nbo(100:301))
%         i/10
%         break
%     end
% end


%% Convergence time for s

max(s_tilde_ebe(100:301))
max(s_tilde_ftebe(100:301))

for i = 50 : 301
    if s_tilde_ebe(i) < max(s_tilde_ebe(50:301))
        i/10
        break
    end
end

for i = 1 : 301
    if s_tilde_ftebe(i) < max(s_tilde_ftebe(100:301))
        i/10
        break
    end
end

%%
max(w_tilde_ebe(100:301))
max(w_tilde_ftebe(100:301))
max(w_tilde_eso(100:301))
max(w_tilde_nbo(100:301))

%% 
for i = 50 : 301
    if w_tilde_ebe(i) < max(w_tilde_ebe(50:301))
        i/10
        break
    end
end

for i = 1 : 301
    if w_tilde_ftebe(i) < max(w_tilde_ftebe(100:301))
        i/10
        break
    end
end

for i = 1 : 301
    if w_tilde_eso(i) < max(w_tilde_eso(100:301))
        i/10
        break
    end
end

for i = 50 : 301
    if w_tilde_nbo(i) < max(w_tilde_nbo(100:301))
        i/10
        break
    end
end

