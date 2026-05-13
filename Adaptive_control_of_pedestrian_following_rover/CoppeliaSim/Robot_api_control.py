import socket
import time
import numpy as np
import struct
import pickle
import json
import math
import matplotlib.pyplot as plt
from scipy.io import savemat
import os

import computer_class_coppelia



import sys
import sim

##################################################################################### Python to CoppeliaSim connection ##################################################################################### 
# Basic parameters
model_parameter = np.matrix([0.145, 0.145, 0.29, 0.41, 0.29])     # l_r, l_f, l_1, l_2, l_3
task_time = 35
control_step_size = 0.05
pos_now = np.matrix([0,0,math.pi/2])

computer = computer_class_coppelia.computer(model_parameter, control_step_size, pos_now)
computer.python_to_coppelia_connection()
computer.acquire_coppelia_handle()

##################################################################################### Preparation for real-time loop ##################################################################################### 
# Control settings
control_mode = 2 # Mode = 1 indicates Arkerman steering control, Mode = 2 indicates hybrid mode control
control_style = 5 # 1: Full estimator; 2: Half estimator; 3: Valina 4: Compensator 5: Compensator + Estimator 
step_extend_index = 10 # Reduce the control frequency by this index
task_flag = 3 # 1: Fixed point 2: Dual circles 3: Dynamic sine waves
step = 0
step_max = int(task_time/control_step_size ) + 1

# Variables for control
# Referemce tracking
pos_prev = pos_now
local_error_now = np.matrix([0,0])
control_nominal = np.matrix([0,0])
control_temp = control_nominal
control_actual = np.matrix([0,0,0,0])
xi_v = 0
xi_u = 0

pos_all_psuedo = np.zeros((step_max,3))
pos_all_coppelia = np.zeros((step_max,3))
local_error_all = np.zeros((step_max,2))
control_nominal_all = np.zeros((step_max,2))
control_actual_all = np.zeros((step_max,4))
pos_ref_all = np.zeros((step_max,3))
vel_ref_all = np.zeros((step_max,2))
time_all = np.zeros((step_max,1))

xi_v_all = np.zeros((step_max,1))
xi_u_all = np.zeros((step_max,1))
mode_flag_all = np.zeros((step_max,1))
sing_flag_all = np.zeros((step_max,1))
e_Phi_all = np.zeros((step_max,1))

# State estimation
pos_hat_now = pos_now
pos_hat_next = pos_hat_now
pos_tilde_now = pos_now - pos_hat_now
W_hat_now = np.matrix([0,0,0])
W_hat_next = W_hat_now
W_now = np.matrix([0,0,0])
U_hat_now = np.matrix([0,0,0])

pos_hat_all = np.zeros((step_max,3))
pos_tilde_all = np.zeros((step_max,3))
U_hat_all = np.zeros((step_max,3))
W_tilde_all = np.zeros((step_max, 3))
e_Phi_max = computer.e_Phi_max
e_Phi_min = computer.e_Phi_min
e_Phi = 0

##################################################################################### Real-time loop starts from here##################################################################################### 

time_initial = time.time()  # Initial time
previous_task_time = time_initial

while step < step_max:
    # Record the current time (relative time)
    current_task_time = time.time() - time_initial
    # current_task_time = step * control_step_size

    # Time-triggered control 
    if current_task_time >= step * control_step_size:
        # Calculate current time
        control_temp = control_nominal
        task_actual_interval = current_task_time - previous_task_time # Use this to replace control_step_size for pseudo loop

        # Reference generation
        pos_d, vel_d = computer.reference_generator(current_task_time, task_flag)

        # Feedback
        pos_now_coppelia = computer.read_pos_from_coppelia()
        pos_now_psuedo = computer.read_pos_psuedo()
        # computer.robot_pos = pos_now_psuedo
        computer.robot_pos = pos_now_coppelia

        if control_mode == 1:
            # Adaptive controller
            U_hat_now, W_hat_now, pos_hat_now, pos_tilde_now = computer.adaptive_estimator()
            local_error_now = computer.error_calculation()

            if control_style == 1:
                control_nominal = computer.kinematic_controller_estimator_u()
            elif control_style == 2:
                control_nominal = computer.kinematic_controller_estimator()
            elif control_style == 3:
                control_nominal = computer.kinematic_controller_vanila()
            elif control_style == 4:
                control_nominal = computer.kinematic_controller_compensator()
            else:
                control_nominal = computer.kinematic_controller_compensator_estimator()

            control_actual = computer.nominal_to_actual()
            computer.send_control_command_to_coppelia()

        else:
            # Switch between two modes
            U_hat_now, W_hat_now, pos_hat_now, pos_tilde_now = computer.adaptive_estimator()
            local_error_now = computer.error_calculation()
            control_nominal = computer.kinematic_controller_compensator_estimator()
            e_Phi= computer.singular_issue_justification()
            computer.hybrid_control_mode_algorithm(e_Phi)

            if computer.F_mode == 1:
                control_nominal = computer.kinematic_controller_compensator_estimator()
                control_actual = computer.nominal_to_actual()
                computer.send_control_command_to_coppelia()
            else:
                U_hat_now, W_hat_now, pos_hat_now, pos_tilde_now, xi_v, xi_u, control_nominal = computer.reset()
                computer.control_actual = control_actual = np.matrix([-np.sign(computer.F_mode) * 0.4, np.sign(computer.F_mode) * 0.4, -np.arctan2(2 * computer.model_parameter[0,4],computer.model_parameter[0,2]), np.arctan2(2 * computer.model_parameter[0,4],computer.model_parameter[0,2])])
                computer.send_control_command_to_coppelia()
        
                
            

        # print(local_error_now)
        # print(pos_now_coppelia)
        # print(control_nominal)
        print(step)
        print(computer.F_mode)
        print(computer.F_sing)
        print(e_Phi)
        print(computer.control_actual)

        # Error-related vectors
        local_error_all[step] = local_error_now

        # Other vectors
        control_nominal_all[step] = control_nominal
        control_actual_all[step] = computer.control_actual
        time_all[step] = current_task_time
        pos_ref_all[step] = pos_d
        vel_ref_all[step] = vel_d
        pos_all_psuedo[step] = pos_now_psuedo
        pos_all_coppelia[step] = pos_now_coppelia

        pos_hat_all[step] = pos_hat_now
        pos_tilde_all[step] = pos_tilde_now
        U_hat_all[step] = U_hat_now

        xi_u_all[step] = computer.xi_u
        xi_v_all[step] = computer.xi_v

        mode_flag_all[step] = computer.F_mode
        sing_flag_all[step] = computer.F_sing
        e_Phi_all[step] = e_Phi

        computer.robot_pos_prev = computer.robot_pos
        step += 1

# Stop the simulation
computer.control_nominal = np.matrix([0, 0])
computer.nominal_to_actual()
computer.send_control_command_to_coppelia()
computer.stop_simulation()

        
# Data saving
data_to_save = {
    # 'pos_tilde_norm': pos_tilde_norm,
    # 'W_tilde_norm': W_tilde_norm,
    # 'U_tilde_norm': U_tilde_norm, 
    # 'pos_error_norm': pos_error_norm, 
    # 'local_error_norm': local_error_norm,
    'time_all': time_all,
    'control_nominal_all': control_nominal_all,
    'control_actual_all': control_actual_all, 
    'xi_u_all': xi_u_all,
    'xi_v_all': xi_v_all,
    # 'W_tilde_all': W_tilde_all,
    'pos_tilde_all': pos_tilde_all,
    'local_error_all': local_error_all,
    'task_time': task_time,
    'control_step_size': control_step_size,
    'model_parameter': model_parameter,
    'pos_ref_all': pos_ref_all,
    'vel_ref_all': vel_ref_all,
    'pos_all_psuedo': pos_all_psuedo,
    'pos_all_coppelia': pos_all_coppelia,
    'F_sing': sing_flag_all,
    'F_mode': mode_flag_all,
    'e_Phi': e_Phi_all,
    'e_Phi_max': e_Phi_max,
    'e_Phi_min': e_Phi_min
}


data_to_send = np.array([pos_now[0,0], pos_now[0,1], pos_now[0,2], pos_d[0,0], pos_d[0,1], vel_d[0,0], vel_d[0,1], 0.0, 0.0], dtype = np.float64)
data_to_send_bytes = data_to_send.tobytes()
# client_socket.sendall(data_to_send_bytes)
# client_socket.close()

data_store_name = 'D:/Paper 12/Data/data'

if task_flag == 1:
    data_store_name += '_linear_traject'
elif task_flag == 2:
    data_store_name += '_dual_circle'
elif task_flag == 3:
    data_store_name += '_fixed_points'
else:
    data_store_name += '_dynamic_sine'

if control_style == 1:
    data_store_name += '_esti_'
elif control_style == 2:
    data_store_name += '_half_esti_'
elif control_style == 3:
    data_store_name += '_vanila_'
elif control_style == 4:
    data_store_name += '_compen_'
else:
    data_store_name += '_esti_compen_'

data_store_name += str(control_step_size)
data_store_name +='.mat'

savemat(data_store_name, {'__globals': {}, **data_to_save})

print("Task completed")








































