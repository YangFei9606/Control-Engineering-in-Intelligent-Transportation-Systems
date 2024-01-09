close all
clc
clear

%% Data collection
load("Ob.mat");

% 1-norm
% s_norm_nfc = sum(abs(out.s_1(:,2:4)) + abs(out.s_2(:,2:4)) + abs(out.s_3(:,2:4)) + abs(out.s_4(:,2:4)) + abs(out.s_5(:,2:4)) + abs(out.s_6(:,2:4)), 2);
% 2-norm
% s_norm_nfc = sqrt(sum(out.s_1(:,2:4).^2 + out.s_2(:,2:4).^2 + out.s_3(:,2:4).^2 + out.s_4(:,2:4).^2 + out.s_5(:,2:4).^2 + out.s_6(:,2:4).^2, 2));

% Integration
time_step = 0.01;
s_nfc = [out.s_1(:,2:4), out.s_2(:,2:4), out.s_3(:,2:4), out.s_4(:,2:4), out.s_5(:,2:4), out.s_6(:,2:4)];
s_nfc_abs = abs(s_nfc);
s_nfc_int = zeros(size(s_nfc,1),1);
for i = 1 : size(s_nfc,1)
    if i == 1
        s_nfc_int(i) = time_step * sum(s_nfc_abs(i,:));
    else
        s_nfc_int(i) = time_step * sum(s_nfc_abs(i,:)) + s_nfc_int(i-1);
    end
end

e_nfc = [out.e_x_1(:,2:4), out.e_x_2(:,2:4), out.e_x_3(:,2:4), out.e_x_4(:,2:4), out.e_x_5(:,2:4), out.e_x_6(:,2:4)];
e_nfc_abs = abs(e_nfc);
e_nfc_int = zeros(size(e_nfc,1),1);
for i = 1 : size(e_nfc,1)
    if i == 1
        e_nfc_int(i) = time_step * sum(e_nfc_abs(i,:));
    else
        e_nfc_int(i) = time_step * sum(e_nfc_abs(i,:)) + e_nfc_int(i-1);
    end
end

ref_x = out.ref_x1(:, 2:19);
x_nfc = [out.x_1(:,2:4), out.x_2(:,2:4), out.x_3(:,2:4), out.x_4(:,2:4), out.x_5(:,2:4), out.x_6(:,2:4)];
delta_nfc = ref_x - x_nfc;
delta_nfc_abs = abs(delta_nfc);
delta_nfc_int = zeros(size(delta_nfc,1),1);
for i = 1 : size(delta_nfc,1)
    if i == 1
        delta_nfc_int(i) = time_step * sum(delta_nfc_abs(i,:));
    else
        delta_nfc_int(i) = time_step * sum(delta_nfc_abs(i,:)) + delta_nfc_int(i-1);
    end
end

u_nfc = [out.control_1(:,2:4), out.control_2(:,2:4), out.control_3(:,2:4), out.control_4(:,2:4), out.control_5(:,2:4), out.control_6(:,2:4)];
u_nfc_abs = abs(u_nfc);
u_nfc_int = zeros(size(u_nfc_abs,1),1);
for i = 1 : size(u_nfc_int,1)
    if i == 1
        u_nfc_int(i) = time_step * sum(u_nfc_abs(i,:));
    else
        u_nfc_int(i) = time_step * sum(u_nfc_abs(i,:)) + u_nfc_int(i-1);
    end
end


time = out.tout;
time_dilute = dilute_data(time, 5);
s_1_nfc = dilute_data(out.s_1(:,2:4), 5);
s_2_nfc = dilute_data(out.s_2(:,2:4), 5);
s_3_nfc = dilute_data(out.s_3(:,2:4), 5);
s_4_nfc = dilute_data(out.s_4(:,2:4), 5);
s_5_nfc = dilute_data(out.s_5(:,2:4), 5);
s_6_nfc = dilute_data(out.s_6(:,2:4), 5);

%%
load("Ob_Aux.mat");

xi_norm_cfc = sum(abs(out.xi_1(:,2:4)) + abs(out.xi_2(:,2:4)) + abs(out.xi_3(:,2:4)) + abs(out.xi_4(:,2:4)) + abs(out.xi_5(:,2:4)) + abs(out.xi_6(:,2:4)), 2);
s_norm_cfc = sum(abs(out.s_1(:,2:4)) + abs(out.s_2(:,2:4)) + abs(out.s_3(:,2:4)) + abs(out.s_4(:,2:4)) + abs(out.s_5(:,2:4)) + abs(out.s_6(:,2:4)), 2);
e_norm_cfc = sum(abs(out.e_x_1(:,2:4)) + abs(out.e_x_2(:,2:4)) + abs(out.e_x_3(:,2:4)) + abs(out.e_x_4(:,2:4)) + abs(out.e_x_5(:,2:4)) + abs(out.e_x_6(:,2:4)), 2);
x_cfc = [out.x_1(:,2:4), out.x_2(:,2:4), out.x_3(:,2:4), out.x_4(:,2:4), out.x_5(:,2:4), out.x_6(:,2:4)];
delta_norm_cfc = sum(abs(ref_x - x_cfc),2);
s_1_cfc = dilute_data(out.s_1(:,2:4), 5);
s_2_cfc = dilute_data(out.s_2(:,2:4), 5);
s_3_cfc = dilute_data(out.s_3(:,2:4), 5);
s_4_cfc = dilute_data(out.s_4(:,2:4), 5);
s_5_cfc = dilute_data(out.s_5(:,2:4), 5);
s_6_cfc = dilute_data(out.s_6(:,2:4), 5);

s_cfc = [out.s_1(:,2:4), out.s_2(:,2:4), out.s_3(:,2:4), out.s_4(:,2:4), out.s_5(:,2:4), out.s_6(:,2:4)];
s_cfc_abs = abs(s_cfc);
s_cfc_int = zeros(size(s_cfc,1),1);
for i = 1 : size(s_cfc,1)
    if i == 1
        s_cfc_int(i) = time_step * sum(s_cfc_abs(i,:));
    else
        s_cfc_int(i) = time_step * sum(s_cfc_abs(i,:)) + s_cfc_int(i-1);
    end
end

e_cfc = [out.e_x_1(:,2:4), out.e_x_2(:,2:4), out.e_x_3(:,2:4), out.e_x_4(:,2:4), out.e_x_5(:,2:4), out.e_x_6(:,2:4)];
e_cfc_abs = abs(e_cfc);
e_cfc_int = zeros(size(e_cfc,1),1);
for i = 1 : size(e_cfc,1)
    if i == 1
        e_cfc_int(i) = time_step * sum(e_cfc_abs(i,:));
    else
        e_cfc_int(i) = time_step * sum(e_cfc_abs(i,:)) + e_cfc_int(i-1);
    end
end

delta_cfc = ref_x - x_cfc;
delta_cfc_abs = abs(delta_cfc);
delta_cfc_int = zeros(size(delta_cfc,1),1);
for i = 1 : size(delta_cfc,1)
    if i == 1
        delta_cfc_int(i) = time_step * sum(delta_cfc_abs(i,:));
    else
        delta_cfc_int(i) = time_step * sum(delta_cfc_abs(i,:)) + delta_cfc_int(i-1);
    end
end

u_cfc = [out.control_1(:,2:4), out.control_2(:,2:4), out.control_3(:,2:4), out.control_4(:,2:4), out.control_5(:,2:4), out.control_6(:,2:4)];
u_cfc_abs = abs(u_cfc);
u_cfc_int = zeros(size(u_cfc_abs,1),1);
for i = 1 : size(u_cfc_int,1)
    if i == 1
        u_cfc_int(i) = time_step * sum(u_cfc_abs(i,:));
    else
        u_cfc_int(i) = time_step * sum(u_cfc_abs(i,:)) + u_cfc_int(i-1);
    end
end

%%
load("Ob_Aux_LP_test.mat");

xi_norm_lpbcfc = sum(abs(out.xi_1(:,2:4)) + abs(out.xi_2(:,2:4)) + abs(out.xi_3(:,2:4)) + abs(out.xi_4(:,2:4)) + abs(out.xi_5(:,2:4)) + abs(out.xi_6(:,2:4)), 2);
s_norm_lpbcfc = sum(abs(out.s_1(:,2:4)) + abs(out.s_2(:,2:4)) + abs(out.s_3(:,2:4)) + abs(out.s_4(:,2:4)) + abs(out.s_5(:,2:4)) + abs(out.s_6(:,2:4)), 2);
e_norm_lpbcfc = sum(abs(out.e_x_1(:,2:4)) + abs(out.e_x_2(:,2:4)) + abs(out.e_x_3(:,2:4)) + abs(out.e_x_4(:,2:4)) + abs(out.e_x_5(:,2:4)) + abs(out.e_x_6(:,2:4)), 2);
x_lpbcfc = [out.x_1(:,2:4), out.x_2(:,2:4), out.x_3(:,2:4), out.x_4(:,2:4), out.x_5(:,2:4), out.x_6(:,2:4)];
s_1_lpbcfc = dilute_data(out.s_1(:,2:4), 5);
s_2_lpbcfc = dilute_data(out.s_2(:,2:4), 5);
s_3_lpbcfc = dilute_data(out.s_3(:,2:4), 5);
s_4_lpbcfc = dilute_data(out.s_4(:,2:4), 5);
s_5_lpbcfc = dilute_data(out.s_5(:,2:4), 5);
s_6_lpbcfc = dilute_data(out.s_6(:,2:4), 5);
delta_norm_lpbcfc = sum(abs(ref_x - x_lpbcfc),2);

s_lpbcfc = [out.s_1(:,2:4), out.s_2(:,2:4), out.s_3(:,2:4), out.s_4(:,2:4), out.s_5(:,2:4), out.s_6(:,2:4)];
s_lpbcfc_abs = abs(s_lpbcfc);
s_lpbcfc_int = zeros(size(s_lpbcfc,1),1);
for i = 1 : size(s_lpbcfc,1)
    if i == 1
        s_lpbcfc_int(i) = time_step * sum(s_lpbcfc_abs(i,:));
    else
        s_lpbcfc_int(i) = time_step * sum(s_lpbcfc_abs(i,:)) + s_lpbcfc_int(i-1);
    end
    
end

e_lpbcfc = [out.e_x_1(:,2:4), out.e_x_2(:,2:4), out.e_x_3(:,2:4), out.e_x_4(:,2:4), out.e_x_5(:,2:4), out.e_x_6(:,2:4)];
e_lpbcfc_abs = abs(e_lpbcfc);
e_lpbcfc_int = zeros(size(e_lpbcfc,1),1);
for i = 1 : size(e_lpbcfc,1)
    if i == 1
        e_lpbcfc_int(i) = time_step * sum(e_lpbcfc_abs(i,:));
    else
        e_lpbcfc_int(i) = time_step * sum(e_lpbcfc_abs(i,:)) + e_lpbcfc_int(i-1);
    end
end

delta_lpbcfc = ref_x - x_lpbcfc;
delta_lpbcfc_abs = abs(delta_lpbcfc);
delta_lpbcfc_int = zeros(size(delta_lpbcfc,1),1);
for i = 1 : size(delta_lpbcfc,1)
    if i == 1
        delta_lpbcfc_int(i) = time_step * sum(delta_lpbcfc_abs(i,:));
    else
        delta_lpbcfc_int(i) = time_step * sum(delta_lpbcfc_abs(i,:)) + delta_lpbcfc_int(i-1);
    end
end

u_lpbcfc = [out.control_1(:,2:4), out.control_2(:,2:4), out.control_3(:,2:4), out.control_4(:,2:4), out.control_5(:,2:4), out.control_6(:,2:4)];
u_lpbcfc_abs = abs(u_lpbcfc);
u_lpbcfc_int = zeros(size(u_lpbcfc_abs,1),1);
for i = 1 : size(u_lpbcfc_int,1)
    if i == 1
        u_lpbcfc_int(i) = time_step * sum(u_lpbcfc_abs(i,:));
    else
        u_lpbcfc_int(i) = time_step * sum(u_lpbcfc_abs(i,:)) + u_lpbcfc_int(i-1);
    end
end




cut = 15/0.01/5 + 1;

%% Plotting

figure(1)
hold on;
grid on;
box on;
set(gcf,'Position',[540,-100,1500,800]);
subplot(2,3,1)
hold on;
grid on;
box on;
plot(time_dilute(1:cut),s_1_nfc(1:cut, 1),'color',[0.9290, 0.6940, 0.1250], 'linewidth', 2);
plot(time_dilute(1:cut),s_1_nfc(1:cut, 2),'--','color',[1, 0, 0], 'linewidth', 2);
plot(time_dilute(1:cut),s_1_nfc(1:cut, 3),'-.','color',[0, 0.4470, 0.7410], 'linewidth', 2);
set(gca,'FontSize',18);
set(gca,'fontname','times');
xlabel('Time (Second)','FontSize',18);
ylabel('Sliding variable','FontSize',18);
ylim([-10 30]);
title("ODR 1");

legend('$s_i^x$','$s_i^y$','$s_i^{\theta}$','fontsize',18,'Orientation','horizontal', 'interpreter', 'latex');

subplot(2,3,2)
hold on;
grid on;
box on;
plot(time_dilute(1:cut),s_2_nfc(1:cut, 1),'color',[0.9290, 0.6940, 0.1250], 'linewidth', 2);
plot(time_dilute(1:cut),s_2_nfc(1:cut, 2),'--','color',[1, 0, 0], 'linewidth', 2);
plot(time_dilute(1:cut),s_3_nfc(1:cut, 3),'-.','color',[0, 0.4470, 0.7410], 'linewidth', 2);
set(gca,'FontSize',18);
set(gca,'fontname','times');
xlabel('Time (Second)','FontSize',18);
ylabel('Sliding variable','FontSize',18);
ylim([-15 20]);
yticks(-15:5:20);
title("ODR 2");

subplot(2,3,3)
hold on;
grid on;
box on;
plot(time_dilute(1:cut),s_3_nfc(1:cut, 1),'color',[0.9290, 0.6940, 0.1250], 'linewidth', 2);
plot(time_dilute(1:cut),s_3_nfc(1:cut, 2),'--','color',[1, 0, 0], 'linewidth', 2);
plot(time_dilute(1:cut),s_3_nfc(1:cut, 3),'-.','color',[0, 0.4470, 0.7410], 'linewidth', 2);
set(gca,'FontSize',18);
set(gca,'fontname','times');
xlabel('Time (Second)','FontSize',18);
ylabel('Sliding variable','FontSize',18);
title("ODR 3");

subplot(2,3,4)
hold on;
grid on;
box on;
plot(time_dilute(1:cut),s_4_nfc(1:cut, 1),'color',[0.9290, 0.6940, 0.1250], 'linewidth', 2);
plot(time_dilute(1:cut),s_4_nfc(1:cut, 2),'--','color',[1, 0, 0], 'linewidth', 2);
plot(time_dilute(1:cut),s_4_nfc(1:cut, 3),'-.','color',[0, 0.4470, 0.7410], 'linewidth', 2);
set(gca,'FontSize',18);
set(gca,'fontname','times');
xlabel('Time (Second)','FontSize',18);
ylabel('Sliding variable','FontSize',18);
title("ODR 4");

subplot(2,3,5)
hold on;
grid on;
box on;
plot(time_dilute(1:cut),s_5_nfc(1:cut, 1),'color',[0.9290, 0.6940, 0.1250], 'linewidth', 2);
plot(time_dilute(1:cut),s_5_nfc(1:cut, 2),'--','color',[1, 0, 0], 'linewidth', 2);
plot(time_dilute(1:cut),s_5_nfc(1:cut, 3),'-.','color',[0, 0.4470, 0.7410], 'linewidth', 2);
set(gca,'FontSize',18);
set(gca,'fontname','times');
xlabel('Time (Second)','FontSize',18);
ylabel('Sliding variable','FontSize',18);
title("ODR 5");

subplot(2,3,6)
hold on;
grid on;
box on;
plot(time_dilute(1:cut),s_6_nfc(1:cut, 1),'color',[0.9290, 0.6940, 0.1250], 'linewidth', 2);
plot(time_dilute(1:cut),s_6_nfc(1:cut, 2),'--','color',[1, 0, 0], 'linewidth', 2);
plot(time_dilute(1:cut),s_6_nfc(1:cut, 3),'-.','color',[0, 0.4470, 0.7410], 'linewidth', 2);
set(gca,'FontSize',18);
set(gca,'fontname','times');
xlabel('Time (Second)','FontSize',18);
ylabel('Sliding variable','FontSize',18);
title("ODR 6");

%%

figure(2)
hold on;
grid on;
box on;
set(gcf,'Position',[540,-100,1500,800]);
subplot(2,3,1)
hold on;
grid on;
box on;
plot(time_dilute(1:cut),s_1_cfc(1:cut, 1),'color',[0.9290, 0.6940, 0.1250], 'linewidth', 2);
plot(time_dilute(1:cut),s_1_cfc(1:cut, 2),'--','color',[1, 0, 0], 'linewidth', 2);
plot(time_dilute(1:cut),s_1_cfc(1:cut, 3),'-.','color',[0, 0.4470, 0.7410], 'linewidth', 2);
set(gca,'FontSize',18);
set(gca,'fontname','times');
xlabel('Time (Second)','FontSize',18);
ylabel('Sliding variable','FontSize',18);
ylim([-10 30]);
title("ODR 1");

legend('$s_i^x$','$s_i^y$','$s_i^{\theta}$','fontsize',18,'Orientation','horizontal', 'interpreter', 'latex');

subplot(2,3,2)
hold on;
grid on;
box on;
plot(time_dilute(1:cut),s_2_cfc(1:cut, 1),'color',[0.9290, 0.6940, 0.1250], 'linewidth', 2);
plot(time_dilute(1:cut),s_2_cfc(1:cut, 2),'--','color',[1, 0, 0], 'linewidth', 2);
plot(time_dilute(1:cut),s_3_cfc(1:cut, 3),'-.','color',[0, 0.4470, 0.7410], 'linewidth', 2);
set(gca,'FontSize',18);
set(gca,'fontname','times');
xlabel('Time (Second)','FontSize',18);
ylabel('Sliding variable','FontSize',18);
title("ODR 2");
ylim([-12 6]);
yticks(-12:3:6);

subplot(2,3,3)
hold on;
grid on;
box on;
plot(time_dilute(1:cut),s_3_cfc(1:cut, 1),'color',[0.9290, 0.6940, 0.1250], 'linewidth', 2);
plot(time_dilute(1:cut),s_3_cfc(1:cut, 2),'--','color',[1, 0, 0], 'linewidth', 2);
plot(time_dilute(1:cut),s_3_cfc(1:cut, 3),'-.','color',[0, 0.4470, 0.7410], 'linewidth', 2);
set(gca,'FontSize',18);
set(gca,'fontname','times');
xlabel('Time (Second)','FontSize',18);
ylabel('Sliding variable','FontSize',18);
title("ODR 3");

subplot(2,3,4)
hold on;
grid on;
box on;
plot(time_dilute(1:cut),s_4_cfc(1:cut, 1),'color',[0.9290, 0.6940, 0.1250], 'linewidth', 2);
plot(time_dilute(1:cut),s_4_cfc(1:cut, 2),'--','color',[1, 0, 0], 'linewidth', 2);
plot(time_dilute(1:cut),s_4_cfc(1:cut, 3),'-.','color',[0, 0.4470, 0.7410], 'linewidth', 2);
set(gca,'FontSize',18);
set(gca,'fontname','times');
xlabel('Time (Second)','FontSize',18);
ylabel('Sliding variable','FontSize',18);
ylim([-6 9]);
yticks(-6:3:9);
title("ODR 4");

subplot(2,3,5)
hold on;
grid on;
box on;
plot(time_dilute(1:cut),s_5_cfc(1:cut, 1),'color',[0.9290, 0.6940, 0.1250], 'linewidth', 2);
plot(time_dilute(1:cut),s_5_cfc(1:cut, 2),'--','color',[1, 0, 0], 'linewidth', 2);
plot(time_dilute(1:cut),s_5_cfc(1:cut, 3),'-.','color',[0, 0.4470, 0.7410], 'linewidth', 2);
set(gca,'FontSize',18);
set(gca,'fontname','times');
xlabel('Time (Second)','FontSize',18);
ylabel('Sliding variable','FontSize',18);
title("ODR 5");

subplot(2,3,6)
hold on;
grid on;
box on;
plot(time_dilute(1:cut),s_6_cfc(1:cut, 1),'color',[0.9290, 0.6940, 0.1250], 'linewidth', 2);
plot(time_dilute(1:cut),s_6_cfc(1:cut, 2),'--','color',[1, 0, 0], 'linewidth', 2);
plot(time_dilute(1:cut),s_6_cfc(1:cut, 3),'-.','color',[0, 0.4470, 0.7410], 'linewidth', 2);
set(gca,'FontSize',18);
set(gca,'fontname','times');
xlabel('Time (Second)','FontSize',18);
ylabel('Sliding variable','FontSize',18);
ylim([-8 8]);
yticks(-8:4:8);
title("ODR 6");


%%

figure(3)
hold on;
grid on;
box on;
set(gcf,'Position',[540,-100,1500,800]);
subplot(2,3,1)
hold on;
grid on;
box on;
plot(time_dilute(1:cut),s_1_lpbcfc(1:cut, 1),'color',[0.9290, 0.6940, 0.1250], 'linewidth', 2);
plot(time_dilute(1:cut),s_1_lpbcfc(1:cut, 2),'--','color',[1, 0, 0], 'linewidth', 2);
plot(time_dilute(1:cut),s_1_lpbcfc(1:cut, 3),'-.','color',[0, 0.4470, 0.7410], 'linewidth', 2);
set(gca,'FontSize',18);
set(gca,'fontname','times');
xlabel('Time (Second)','FontSize',18);
ylabel('Sliding variable','FontSize',18);
ylim([-10 30]);
title("ODR 1");

legend('$s_i^x$','$s_i^y$','$s_i^{\theta}$','fontsize',18,'Orientation','horizontal', 'interpreter', 'latex');


subplot(2,3,2)
hold on;
grid on;
box on;
plot(time_dilute(1:cut),s_2_lpbcfc(1:cut, 1),'color',[0.9290, 0.6940, 0.1250], 'linewidth', 2);
plot(time_dilute(1:cut),s_2_lpbcfc(1:cut, 2),'--','color',[1, 0, 0], 'linewidth', 2);
plot(time_dilute(1:cut),s_3_lpbcfc(1:cut, 3),'-.','color',[0, 0.4470, 0.7410], 'linewidth', 2);
set(gca,'FontSize',18);
set(gca,'fontname','times');
xlabel('Time (Second)','FontSize',18);
ylabel('Sliding variable','FontSize',18);
title("ODR 2");
ylim([-12 6]);
yticks(-12:3:6);

subplot(2,3,3)
hold on;
grid on;
box on;
plot(time_dilute(1:cut),s_3_lpbcfc(1:cut, 1),'color',[0.9290, 0.6940, 0.1250], 'linewidth', 2);
plot(time_dilute(1:cut),s_3_lpbcfc(1:cut, 2),'--','color',[1, 0, 0], 'linewidth', 2);
plot(time_dilute(1:cut),s_3_lpbcfc(1:cut, 3),'-.','color',[0, 0.4470, 0.7410], 'linewidth', 2);
set(gca,'FontSize',18);
set(gca,'fontname','times');
xlabel('Time (Second)','FontSize',18);
ylabel('Sliding variable','FontSize',18);
ylim([-4, 10]);
yticks(-4:2:10);
title("ODR 3");

subplot(2,3,4)
hold on;
grid on;
box on;
plot(time_dilute(1:cut),s_4_lpbcfc(1:cut, 1),'color',[0.9290, 0.6940, 0.1250], 'linewidth', 2);
plot(time_dilute(1:cut),s_4_lpbcfc(1:cut, 2),'--','color',[1, 0, 0], 'linewidth', 2);
plot(time_dilute(1:cut),s_4_lpbcfc(1:cut, 3),'-.','color',[0, 0.4470, 0.7410], 'linewidth', 2);
set(gca,'FontSize',18);
set(gca,'fontname','times');
xlabel('Time (Second)','FontSize',18);
ylabel('Sliding variable','FontSize',18);
ylim([-8 8]);
yticks(-8:4:8);
title("ODR 4");

subplot(2,3,5)
hold on;
grid on;
box on;
plot(time_dilute(1:cut),s_5_lpbcfc(1:cut, 1),'color',[0.9290, 0.6940, 0.1250], 'linewidth', 2);
plot(time_dilute(1:cut),s_5_lpbcfc(1:cut, 2),'--','color',[1, 0, 0], 'linewidth', 2);
plot(time_dilute(1:cut),s_5_lpbcfc(1:cut, 3),'-.','color',[0, 0.4470, 0.7410], 'linewidth', 2);
set(gca,'FontSize',18);
set(gca,'fontname','times');
xlabel('Time (Second)','FontSize',18);
ylabel('Sliding variable','FontSize',18);
ylim([-15 5]);
title("ODR 5");

subplot(2,3,6)
hold on;
grid on;
box on;
plot(time_dilute(1:cut),s_6_lpbcfc(1:cut, 1),'color',[0.9290, 0.6940, 0.1250], 'linewidth', 2);
plot(time_dilute(1:cut),s_6_lpbcfc(1:cut, 2),'--','color',[1, 0, 0], 'linewidth', 2);
plot(time_dilute(1:cut),s_6_lpbcfc(1:cut, 3),'-.','color',[0, 0.4470, 0.7410], 'linewidth', 2);
set(gca,'FontSize',18);
set(gca,'fontname','times');
xlabel('Time (Second)','FontSize',18);
ylabel('Sliding variable','FontSize',18);
ylim([-9 6]);
yticks(-9:3:6);
title("ODR 6");

%%

choose = 1;
a = s_nfc(choose/0.01+1,2:18);
b = s_cfc(choose/0.01+1,2:18);
c = s_lpbcfc(choose/0.01+1,2:18);

norm(a)
norm(b)
norm(c)

%% 

figure(10)
hold on;
grid on;
box on;
set(gcf,'Position',[540,-100,1200,400]);
subplot(1,2,1)
hold on;
grid on;
box on;
plot(dilute_data(time, 10), dilute_data(e_nfc_int, 10), 'color',[0.9290, 0.6940, 0.1250], 'linewidth', 2);
plot(dilute_data(time, 10), dilute_data(e_cfc_int, 10),'--','color',[1, 0, 0], 'linewidth', 2);
plot(dilute_data(time, 10), dilute_data(e_lpbcfc_int, 10),'-.','color',[0, 0.4470, 0.7410], 'linewidth', 2);
set(gca,'FontSize',18);
set(gca,'fontname','times');
xlabel('Time (Second)','FontSize',18);
ylabel('$\Delta(e_x, t)$','FontSize',18,'interpreter', "latex");

subplot(1,2,2)
hold on;
grid on;
box on;
plot(dilute_data(time, 10), dilute_data(delta_nfc_int, 10), 'color',[0.9290, 0.6940, 0.1250], 'linewidth', 2);
plot(dilute_data(time, 10), dilute_data(delta_cfc_int, 10),'--','color',[1, 0, 0], 'linewidth', 2);
plot(dilute_data(time, 10), dilute_data(delta_lpbcfc_int, 10),'-.','color',[0, 0.4470, 0.7410], 'linewidth', 2);
set(gca,'FontSize',18);
set(gca,'fontname','times');
xlabel('Time (Second)','FontSize',18);
ylabel('$\Delta(\delta_x, t)$','FontSize',18,'interpreter', "latex");

legend('NFC','CFC','LPBCFC','fontsize',18,'Orientation','horizontal', 'interpreter', 'latex');

%%

figure(5)
hold on;
grid on;
box on;
set(gcf,'Position',[540,-100,1500,800]);

for i = 1 : 6
    subplot(2,3,i)
    hold on;
    grid on;
    box on;
    plot(dilute_data(time, 10), dilute_data(u_lpbcfc(:, (i-1)*3 + 1), 10), 'color',[0.9290, 0.6940, 0.1250], 'linewidth', 2);
    plot(dilute_data(time, 10), dilute_data(u_lpbcfc(:, (i-1)*3 + 2), 10), '--','color',[1, 0, 0], 'linewidth', 2);
    plot(dilute_data(time, 10), dilute_data(u_lpbcfc(:, (i-1)*3 + 3), 10), '-.','color',[0, 0.4470, 0.7410], 'linewidth', 2);
    plot([0 30], [0.2 0.2], '--', 'color', [0.25, 0.25, 0.25], 'linewidth', 2);
    plot([0 30], [-0.2 -0.2], '--', 'color', [0.25, 0.25, 0.25], 'linewidth', 2);
    set(gca,'FontSize',18);
    set(gca,'fontname','times');
    xlabel('Time (Second)','FontSize',18);
    ylabel('Control input','FontSize',18);
    title(append("ODR ", num2str(i)));
    ylim([-0.3 0.3]);
    yticks(-0.3:0.1:0.3);
end

legend('$a_i^1$','$a_i^2$','$a_i^3$', '$U_{Mi}$', 'fontsize', 18, 'Orientation', 'horizontal', 'interpreter', 'latex');


%%
color_m = [[0.9290, 0.6940, 0.1250]; [1, 0, 0]; [0, 0.4470, 0.7410]; [0, 0, 1]; [0.6350, 0.0780, 0.1840]; [0.75, 0.75, 0]]; 
R_c = 1;
w_c = -0.1;
Radius = [0.20,0.22,0.21,0.19,0.23,0.18];

figure(6)
hold on;
grid on;
box on;
set(gcf,'Position',[540,-100,800,1200]);
set(gca,'FontSize',16);
set(gca,'FontName','times');

subplot(2,1,1)
grid on;
hold on;
box on;
set(gca,'FontSize',16);
set(gca,'FontName','times');
% Draw current formation
t = 0;
x_c = R_c * cos(w_c * t);
y_c = R_c * sin(w_c * t);
cir = -pi:0.01:pi;
plot(x_c + 2 * cos(cir), y_c + 2 * sin(cir), '--', 'color',[0.75, 0, 0.75], 'linewidth', 2);

% Draw robots
for i = 1 : 6
    draw_the_robot(x_lpbcfc(1,(i-1)*3+1:(i-1)*3+3), Radius(i), i, color_m(i, :));
    hold on;
end
xlim([-4 4]);
ylim([-3 3]);

title("t = 0s");


subplot(2,1,2)
grid on;
hold on;
box on;
set(gca,'FontSize',16);
set(gca,'FontName','times');
% Draw current formation
t = 30;
x_c = R_c * cos(w_c * t);
y_c = R_c * sin(w_c * t);
plot(x_c + 2 * cos(cir), y_c + 2 * sin(cir), '--', 'color',[0.75, 0, 0.75], 'linewidth', 2);

% Draw actual trajectories
for i = 1 : 6
    plot(x_lpbcfc(:, (i-1) * 3 + 1),  x_lpbcfc(:, (i-1) * 3 + 2), '-.', 'color', color_m(i,:), 'linewidth', 1.5)
end

% Draw expectations
for i = 1 : 6
    plot(ref_x(:, (i-1) * 3 + 1),  ref_x(:, (i-1) * 3 + 2), '-', 'color', color_m(i,:), 'linewidth', 1.5)
end

% Draw robots
for i = 1 : 6
    draw_the_robot(x_lpbcfc(3001,(i-1)*3+1:(i-1)*3+3), Radius(i), i, color_m(i, :));
    hold on;
end
xlim([-4 4]);
ylim([-3 3]);

title("t = 30s");


%%
% 
% ref_x_dilute = dilute_data(ref_x, 10);
% x_lpbcfc_dilute = dilute_data(x_lpbcfc, 10);
% Radius = [0.20,0.22,0.21,0.19,0.23,0.18];
% R_c = 1;
% w_c = -0.1;
% 
% figure(7)
% myVideo=VideoWriter('LPBCFC','MPEG-4');
% myVideo.FrameRate=10;
% open(myVideo);
% axis_range=[-4 4 -4 3];
% z=0;
% marg=30;
% for i = 1 : 301
%     set(gcf,'Position',[10,20,800,700]);
%     xticks(-4:1:4);
%     yticks(-3:1:3);
%     hold off
%     ct=(i-1)*time_step;
%     % Reference states
%     cir=0:pi/100:2*pi;
%     
%     % Draw the references
%     for j = 1 : 6
%         plot(ref_x_dilute(1:i, (j-1)*3 + 1),ref_x_dilute(1:i, (j-1)*3 + 2),'-','color',color_m(j,:),'linewidth',1.5);
%         hold on;
%     end
% 
%     % Draw the formation
%     t = (i-1) * 0.1;
%     x_c = R_c * cos(w_c * t);
%     y_c = R_c * sin(w_c * t);
%     plot(x_c + 2 * cos(cir), y_c + 2 * sin(cir), '--', 'color',[0.75, 0, 0.75], 'linewidth', 2);
% 
%     % Draw the actual trajectories
%     for j = 1 : 6
%         plot(x_lpbcfc_dilute(1:i, (j-1)*3 + 1),x_lpbcfc_dilute(1:i, (j-1)*3 + 2),'-.','color',color_m(j,:),'linewidth',1.5);
%         hold on;
%     end
% 
%     % Draw the robots
%     for j = 1 : 6
%         draw_the_robot(x_lpbcfc_dilute(i,(j-1)*3+1:(j-1)*3+3), Radius(j), j, color_m(j, :));
%         hold on;
%     end
%     
%     set(gca,'FontSize',16);
%     set(gca,'FontName','times');
%     axis(axis_range);
%     xlabel('X(m)','FontSize',16);
%     ylabel('Y(m)','FontSize',16);
%     title(append("t = ", num2str((i-1) * 0.1), "s"), 'FontSize',16);
%     grid on;
% 
%     % include the axis information
%     ax = gca;
%     ax.Units = 'pixels';
%     pos = ax.Position;
%     rect = [-marg-25, -marg-25, pos(3)+2*marg + 10, pos(4)+3*marg];
%     frame=getframe(gca,rect);
%     writeVideo(myVideo,frame)
%     z=z+1;
% end
% 
% close(myVideo);
% 
% 
% %%
% 
% ref_x_dilute = dilute_data(ref_x, 10);
% x_nfc_dilute = dilute_data(x_nfc, 10);
% Radius = [0.20,0.22,0.21,0.19,0.23,0.18];
% R_c = 1;
% w_c = -0.1;
% 
% figure(8)
% myVideo=VideoWriter('NFC','MPEG-4');
% myVideo.FrameRate=10;
% open(myVideo);
% axis_range=[-4 4 -4 3];
% z=0;
% marg=30;
% for i = 1 : 301
%     set(gcf,'Position',[10,20,800,700]);
%     xticks(-4:1:4);
%     yticks(-3:1:3);
%     hold off
%     ct=(i-1)*time_step;
%     % Reference states
%     cir=0:pi/100:2*pi;
%     
%     % Draw the references
%     for j = 1 : 6
%         plot(ref_x_dilute(1:i, (j-1)*3 + 1),ref_x_dilute(1:i, (j-1)*3 + 2),'-','color',color_m(j,:),'linewidth',1.5);
%         hold on;
%     end
% 
%     % Draw the formation
%     t = (i-1) * 0.1;
%     x_c = R_c * cos(w_c * t);
%     y_c = R_c * sin(w_c * t);
%     plot(x_c + 2 * cos(cir), y_c + 2 * sin(cir), '--', 'color',[0.75, 0, 0.75], 'linewidth', 2);
% 
%     % Draw the actual trajectories
%     for j = 1 : 6
%         plot(x_nfc_dilute(1:i, (j-1)*3 + 1),x_nfc_dilute(1:i, (j-1)*3 + 2),'-.','color',color_m(j,:),'linewidth',1.5);
%         hold on;
%     end
% 
%     % Draw the robots
%     for j = 1 : 6
%         draw_the_robot(x_nfc_dilute(i,(j-1)*3+1:(j-1)*3+3), Radius(j), j, color_m(j, :));
%         hold on;
%     end
%     
%     set(gca,'FontSize',16);
%     set(gca,'FontName','times');
%     axis(axis_range);
%     xlabel('X(m)','FontSize',16);
%     ylabel('Y(m)','FontSize',16);
%     title(append("t = ", num2str((i-1) * 0.1), "s"), 'FontSize',16);
%     grid on;
% 
%     % include the axis information
%     ax = gca;
%     ax.Units = 'pixels';
%     pos = ax.Position;
%     rect = [-marg-25, -marg-25, pos(3)+2*marg + 10, pos(4)+3*marg];
%     frame=getframe(gca,rect);
%     writeVideo(myVideo,frame)
%     z=z+1;
% end
% 
% close(myVideo);
