run("HITL_data_collection.m");
color_m=[[0.4660, 0.6740, 0.1880]; [0, 0.4470, 0.7410]; [0.9290, 0.6940, 0.1250]; [0.25, 0.25, 0.25]; [0.4940, 0.1840, 0.5560]; [0.6350, 0.0780, 0.1840]];

%% Precision comparison

f = figure(1);
set(gcf,'Position',[100,100,1200,400]);
hold on;
grid on;
box on;
% set(gcf, 'Color', 'none');  % 设置图形背景透明
% set(gca, 'Color', 'none');  % 设置坐标轴区域透明

subplot(1,3,1)
hold on;
grid on;
box on;
set(gca,'FontSize',16);
set(gca,'FontName','times');
plot(time_all, track_error_hybrid_dual, '-', 'Color', color_m(1,:), 'LineWidth', 1.5); 
plot(time_all, track_error_hybrid_dual_compare, '--', 'Color', color_m(2,:), 'LineWidth', 1.5);
plot(time_all, track_error_Ackerman_dual, '-.', 'Color', color_m(3,:), 'LineWidth', 1.5); 
xlim([0, 40]);
xlabel('Time (Second)','FontSize',16);
ylabel('$\sqrt{x_e^2 + y_e^2}$','FontSize',16,'Interpreter','latex');
title("Dual circles")

legend('HCE', 'HC', 'ASE', 'Orientation','vertical');

subplot(1,3,2)
hold on;
grid on;
box on;
set(gca,'FontSize',16);
set(gca,'FontName','times');
plot(time_all, track_error_hybrid_fixed, '-', 'Color', color_m(1,:), 'LineWidth', 1.5); 
plot(time_all, track_error_hybrid_fixed_compare, '--', 'Color', color_m(2,:), 'LineWidth', 1.5); 
plot(time_all, track_error_Ackerman_fixed, '-.', 'Color', color_m(3,:), 'LineWidth', 1.5); 
xlim([0, 40]);
xlabel('Time (Second)','FontSize',16);
ylabel('$\sqrt{x_e^2 + y_e^2}$','FontSize',16,'Interpreter','latex');
title("Fixed points")

subplot(1,3,3)
hold on;
grid on;
box on;
set(gca,'FontSize',16);
set(gca,'FontName','times');
plot(time_all, track_error_hybrid_dynamic, '-', 'Color', color_m(1,:), 'LineWidth', 1.5); 
plot(time_all, track_error_hybrid_dynamic_compare, '--', 'Color', color_m(2,:), 'LineWidth', 1.5); 
plot(time_all, track_error_Ackerman_dynamic, '-.', 'Color', color_m(3,:), 'LineWidth', 1.5); 
xlim([0, 40]);
xlabel('Time (Second)','FontSize',16);
ylabel('$\sqrt{x_e^2 + y_e^2}$','FontSize',16,'Interpreter','latex');
title("Nonlinear trajectory")

% exportgraphics(gcf, 'D:/Paper 12/Data_202504/Figure_simulation/tracking_performance.pdf', 'ContentType', 'vector', 'BackgroundColor', 'none');
% exportgraphics(gcf, 'D:/Paper 12/Data_202504/Figure_simulation/tracking_performance.svg', 'ContentType', 'vector', 'BackgroundColor', 'none');

%% Performance of estimator
figure(2)
set(gcf,'Position',[100,100,1000,500]);
hold on;
grid on;
box on;
% set(gcf, 'Color', 'none');  % 设置图形背景透明
% set(gca, 'Color', 'none');  % 设置坐标轴区域透明
plot(time_all, pos_tilde_hybrid_dual, '-', 'Color', color_m(1,:), 'linewidth', 1.5);
plot(time_all, pos_tilde_hybrid_fixed, '-.', 'Color', color_m(2,:), 'linewidth', 1.5);
plot(time_all, pos_tilde_hybrid_dynamic, '--', 'Color', color_m(3,:), 'linewidth', 1.5);
set(gca,'FontSize',16);
set(gca,'FontName','times');
xlim([0, 40]);
xlabel('Time (Second)','FontSize',16);
ylabel('$\sqrt{\widetilde{X}^2 + \widetilde{Y}^2}$','FontSize',16,'Interpreter','latex');

legend('Dual circles', 'Fixed points', 'Nonlinear trajectory', 'Orientation','horizontal',...
    'Location', [0.162933321851094,0.93706666908355,0.685600011482239,0.051200001029968]);

%% Singular issue
figure(3)
set(gcf,'Position',[100,100,800,500]);
hold on;
grid on;
box on;
% set(gcf, 'Color', 'none');  % 设置图形背景透明
% set(gca, 'Color', 'none');  % 设置坐标轴区域透明

set(gca,'FontSize',16);
set(gca,'FontName','times');
plot(time_all, e_Phi_hybrid_fixed, '-.', 'color', color_m(1,:), 'linewidth', 1.5);
plot(time_all, F_sing_hybrid_fixed, '-', 'color', color_m(2,:), 'linewidth', 1.5);
plot(time_all, F_mode_hybrid_fixed, '-', 'color', color_m(3,:), 'linewidth', 1.5);
plot([0,40], [e_Phi_max, e_Phi_max], '--', 'Color', color_m(4,:), 'LineWidth', 0.5);
plot([0,40], [-e_Phi_max, -e_Phi_max], '--', 'Color', color_m(4,:), 'LineWidth', 0.5);
plot([0,40], [e_Phi_min, e_Phi_min], '--', 'Color', color_m(5,:), 'LineWidth', 0.5);
plot([0,40], [-e_Phi_min, -e_Phi_min], '--', 'Color', color_m(5,:), 'LineWidth', 0.5);
xlim([0, 40]);
ylim([-2 2]);
xlabel('Time (Second)','FontSize',16);
title("Fixed points")
legend('$e_{\Phi}$', '$F_{\rm sing}$', '$F_{\rm mode}$', 'Orientation','horizontal', 'interpreter', 'latex');

%% Convergence of compensators
figure(4)
set(gcf,'Position',[100,100,1200,700]);
% set(gcf, 'Color', 'none');  % 设置图形背景透明
% set(gca, 'Color', 'none');  % 设置坐标轴区域透明

subplot(2,1,1)
hold on;
grid on;
box on;
plot(time_all, xi_v_hybrid_dual, '-.', 'Color', color_m(2,:), 'LineWidth', 1.5);
plot(time_all, xi_v_hybrid_fixed, '--', 'Color', color_m(3,:), 'LineWidth', 1.5);
plot(time_all, xi_v_hybrid_dynamic, '-', 'Color', color_m(4,:), 'LineWidth', 1.5);
set(gca,'FontSize',16);
set(gca,'FontName','times');
xlim([0 40]);
xlabel('Time (Second)','FontSize',16);
ylabel('$\xi_v$','FontSize',16,'Interpreter','latex');

subplot(2,1,2)
hold on;
grid on;
box on;
plot(time_all, xi_u_hybrid_dual, '-.', 'Color', color_m(2,:), 'LineWidth', 1.5);
plot(time_all, xi_u_hybrid_fixed, '--', 'Color', color_m(3,:), 'LineWidth', 1.5);
plot(time_all, xi_u_hybrid_dynamic, '-', 'Color', color_m(4,:), 'LineWidth', 1.5);
set(gca,'FontSize',16);
set(gca,'FontName','times');
xlim([0 40]);
xlabel('Time (Second)','FontSize',16);
ylabel('$\xi_u$','FontSize',16,'Interpreter','latex');

%% Control inputs
figure(5)

subplot(2,2,1)
hold on;
grid on;
box on;


%% Draw the trajectory--Dual circles
figure(5)
set(gcf,'Position',[100,100,1300,550]);
set(gcf, 'Color', 'none');  % 设置图形背景透明
set(gca, 'Color', 'none');  % 设置坐标轴区域透明

subplot(1,2,1)
hold on;
grid on;
box on;
plot(pos_all_hybrid_dual(:, 1), pos_all_hybrid_dual(:,2), '-.', 'Color', color_m(1,:), 'LineWidth', 1.5);
plot(pos_ref_dual(:, 1), pos_ref_dual(:, 2), '--', 'Color', color_m(2,:), 'LineWidth', 1.5);
set(gca,'FontSize',16);
set(gca,'FontName','times');
xlabel('X (m)','FontSize',16);
ylabel('Y (m)','FontSize',16);
title("Dual circles");

legend('Tracking performance', 'References', 'Orientation','vertical')

subplot(1,2,2)
hold on;
grid on;
box on;
plot(pos_all_hybrid_dynamic(:, 1), pos_all_hybrid_dynamic(:,2), '-.', 'Color', color_m(1,:), 'LineWidth', 1.5);
plot(pos_ref_dynamic(:, 1), pos_ref_dynamic(:, 2), '--', 'Color', color_m(2,:), 'LineWidth', 1.5);
% plot(pos_all_hybrid_dynamic(1:100, 1), pos_all_hybrid_dynamic(1:100,2), '-.', 'Color', color_m(1,:), 'LineWidth', 1.5);
% plot(pos_ref_dynamic(1:100, 1), pos_ref_dynamic(1:100, 2), '--', 'Color', color_m(2,:), 'LineWidth', 1.5);
set(gca,'FontSize',16);
set(gca,'FontName','times');
xlabel('X (m)','FontSize',16);
ylabel('Y (m)','FontSize',16);
title("Nonlinear trajectory");
