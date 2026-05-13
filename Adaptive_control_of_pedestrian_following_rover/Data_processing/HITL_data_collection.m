
clc
clear
close all

%% Hybrid mode + compensator + estimator

data_dual = open('D:\Paper 12\Data_20250609\data_hybrid_dual_circle_esti_compen_0.01.mat');
data_fixed = open('D:\Paper 12\Data_20250609\data_hybrid_fixed_points_esti_compen_0.01.mat');
data_dynamic = open('D:\Paper 12\Data_20250609\data_hybrid_dynamic_sine_esti_compen_0.01.mat');

time_all = data_dual.time_all;
e_Phi_max = data_dual.e_Phi_max;
e_Phi_min = data_dual.e_Phi_min;
control_step = data_dual.control_step_size;

% Dual circles
control_actual_hybrid_dual = data_dual.control_actual_all;
control_nominal_hybrid_dual = data_dual.control_nominal_all;
xi_u_hybrid_dual = data_dual.xi_u_all;
xi_v_hybrid_dual = data_dual.xi_v_all;
pos_tilde_hybrid_dual = sqrt(sum(data_dual.pos_tilde_all(:,1:2).^2, 2));
track_error_hybrid_dual = sqrt(sum(data_dual.local_error_all.^2, 2));
pos_all_hybrid_dual = data_dual.pos_all_coppelia;
F_sing_hybrid_dual = data_dual.F_sing;
F_mode_hybrid_dual = data_dual.F_mode;
e_Phi_hybrid_dual = data_dual.e_Phi;
pos_ref_dual = data_dual.pos_ref_all;

control_actual_hybrid_dual(:, 3) = control_actual_hybrid_dual(:, 3)./pi .* 180;
control_actual_hybrid_dual(:, 4) = control_actual_hybrid_dual(:, 4)./pi .* 180;

% Fixed points
control_actual_hybrid_fixed = data_fixed.control_actual_all;
xi_u_hybrid_fixed = data_fixed.xi_u_all;
xi_v_hybrid_fixed = data_fixed.xi_v_all;
pos_tilde_hybrid_fixed = sqrt(sum(data_fixed.pos_tilde_all(:,1:2).^2, 2));
track_error_hybrid_fixed = sqrt(sum(data_fixed.local_error_all.^2, 2));
pos_all_hybrid_fixed = data_fixed.pos_all_coppelia;
F_sing_hybrid_fixed = data_fixed.F_sing;
F_mode_hybrid_fixed = data_fixed.F_mode;
e_Phi_hybrid_fixed = data_fixed.e_Phi;
pos_ref_fixed = data_fixed.pos_ref_all;

% Dynamic sine wave
control_actual_hybrid_dynamic = data_dynamic.control_actual_all;
control_nominal_hybrid_dynamic = data_dynamic.control_nominal_all;
xi_u_hybrid_dynamic = data_dynamic.xi_u_all;
xi_v_hybrid_dynamic = data_dynamic.xi_v_all;
pos_tilde_hybrid_dynamic = sqrt(sum(data_dynamic.pos_tilde_all(:,1:2).^2, 2));
track_error_hybrid_dynamic = sqrt(sum(data_dynamic.local_error_all.^2, 2));
pos_all_hybrid_dynamic = data_dynamic.pos_all_coppelia;
F_sing_hybrid_dynamic = data_dynamic.F_sing;
F_mode_hybrid_dynamic = data_dynamic.F_mode;
e_Phi_hybrid_dynamic = data_dynamic.e_Phi;
pos_ref_dynamic = data_dynamic.pos_ref_all;

control_nominal_hybrid_dynamic(:, 2) = control_nominal_hybrid_dynamic(:, 2)./pi .* 180;

%% Hybrid mode + compensator
data_dual = open('D:\Paper 12\Data_20250609\data_hybrid_dual_circle_compen_0.01.mat');
data_fixed = open('D:\Paper 12\Data_20250609\data_hybrid_fixed_points_compen_0.01.mat');
data_dynamic = open('D:\Paper 12\Data_20250609\data_hybrid_dynamic_sine_compen_0.01.mat');

% Dual circles
track_error_hybrid_dual_compare = sqrt(sum(data_dual.local_error_all.^2, 2));
pos_all_hybrid_dual_compare = data_dual.pos_all_coppelia;
F_sing_hybrid_dual_compare = data_dual.F_sing;
F_mode_hybrid_dual_compare = data_dual.F_mode;

% Fixed points
track_error_hybrid_fixed_compare = sqrt(sum(data_fixed.local_error_all.^2, 2));
pos_all_hybrid_fixed_compare = data_fixed.pos_all_coppelia;
F_sing_hybrid_fixed_compare = data_fixed.F_sing;
F_mode_hybrid_fixed_compare = data_fixed.F_mode;

% Dynamic sine wave
track_error_hybrid_dynamic_compare = sqrt(sum(data_dynamic.local_error_all.^2, 2));
pos_all_hybrid_dynamic_compare = data_dynamic.pos_all_coppelia;
F_sing_hybrid_dynamic_compare = data_dynamic.F_sing;
F_mode_hybrid_dynamic_compare = data_dynamic.F_mode;

%% Ackerman mode + compensator + estimator
data_dual = open('D:\Paper 12\Data_20250609\data_Ackerman_dual_circle_esti_compen_0.01.mat');
data_fixed = open('D:\Paper 12\Data_20250609\data_Ackerman_fixed_points_esti_compen_0.01.mat');
data_dynamic = open('D:\Paper 12\Data_20250609\data_Ackerman_dynamic_sine_esti_compen_0.01.mat');


% Dual circles
track_error_Ackerman_dual = sqrt(sum(data_dual.local_error_all.^2, 2));
pos_all_Ackerman_dual = data_dual.pos_all_coppelia;

% Fixed points
track_error_Ackerman_fixed = sqrt(sum(data_fixed.local_error_all.^2, 2));
pos_all_Ackerman_fixed = data_fixed.pos_all_coppelia;

% Dynamic sine wave
track_error_Ackerman_dynamic = sqrt(sum(data_dynamic.local_error_all.^2, 2));
pos_all_Ackerman_dynamic = data_dynamic.pos_all_coppelia;

