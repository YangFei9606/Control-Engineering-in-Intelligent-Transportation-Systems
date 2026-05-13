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
from pynput import keyboard

import sys

import computer_class_hitl
import nokov_function

flag = 0

def on_press(key):
    global flag
    if key == keyboard.Key.esc:
        flag = 1
        print("ESC is pressed, program termiGnated!")
    else:
        print("Other keys are pressed, program continues.")

listener = keyboard.Listener(on_press = on_press)
listener.start()

##################################################################################### Python to CoppeliaSim connection ##################################################################################### 
# Basic parameters
model_parameter = np.matrix([0.145, 0.145, 0.29, 0.41, 0.29])     # l_r, l_f, l_1, l_2, l_3
task_time = 40
control_step_size = 0.01
pos_now = np.matrix([-3, -2.5, math.pi/2 * 0])

computer = computer_class_hitl.computer(model_parameter, control_step_size, pos_now)

##################################################################################### Preparation for real-time loop ##################################################################################### 
# Control settings
control_mode = 2 # Mode = 1 indicates Ackerman steering control, Mode = 2 indicates hybrid mode control
control_method = 5 # 1: Vanila; 2: Compensator; 3: Estimator 4: Special estimator 5: Compensator + Estimator 
task_flag = 4 # 1: Linear trajectories with singular 2: Dual circles (This is not very useful) 3: Four fixed points 4: Dynamic sine waves
step = 0
step_max = int(task_time/control_step_size ) + 1

# Variables for control
# Referemce tracking
pos_prev = pos_now
local_error_now = np.matrix([0,0])
global_error_now = np.matrix([0,0])
control_nominal = np.matrix([0,0])
control_temp = control_nominal
control_actual = np.matrix([0,0,0,0])
xi_v = 0
xi_u = 0

pos_all_psuedo = np.zeros((step_max,3))
pos_all_coppelia = np.zeros((step_max,3))
pos_all_coppelia[0] = pos_now
local_error_all = np.zeros((step_max,2))
global_error_all = np.zeros((step_max,2))
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

##################################################################################### Include NOKOV and vehicle into the loop ##################################################################################### 
# Global variable to store rigid body ID to name mapping

time.sleep(0.5)

client_socket = computer.wireless_communication()

# Connect Nokov
client = nokov_function.nokov_setup()
body_name = "Rover"

time.sleep(0.5)

# Initialize the position information
pos_now = nokov_function.nokov_feedback(client, body_name, pos_now)
computer.robot_pos = pos_now
computer.robot_pos_prev = pos_now
computer.robot_pos_psuedo = pos_now
computer.pos_hat_now = pos_now

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
W_hat_all = np.zeros((step_max, 3))
e_Phi_max = computer.e_Phi_max
e_Phi_min = computer.e_Phi_min
e_Phi = 0

angle = np.arctan2(2 * computer.model_parameter[0,4],computer.model_parameter[0,2])


##################################################################################### Real-time loop starts from here ##################################################################################### 

time_initial = time.time()  # Initial time
previous_task_time = time_initial

while step < step_max and flag == 0:
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
        pos_now = nokov_function.nokov_feedback(client, body_name, pos_now)
        computer.robot_pos = pos_now
        computer.adaptive_estimator()
        pos_now_psuedo = computer.read_pos_psuedo()

        global_error, local_error = computer.error_calculation()

        if control_method == 1:
            computer.control_nominal = control_nominal = computer.kinematic_controller_vanila()
        elif control_method == 2:
            computer.control_nominal = control_nominal = computer.kinematic_controller_compensator()
        elif control_method == 3:
            computer.control_nominal = control_nominal = computer.kinematic_controller_estimator()
        elif control_method == 4:
            # But this method's results are not good
            computer.control_nominal = control_nominal = computer.kinematic_controller_estimator_u()
        else:
            computer.control_nominal = control_nominal = computer.kinematic_controller_compensator_estimator()
        
        if control_mode == 1:
            control_actual = computer.nominal_to_actual()
        else:
            e_Phi= computer.singular_issue_justification()
            control_actual = computer.hybrid_control(e_Phi, angle)

        # Send control commends to the rover    
        computer.send_commend_to_rover(client_socket)
        ################################################################################################################################################################

        # Print something out
        if step % int(1/control_step_size) == 0:
            print("Task time: %s, Control mode: %s, Singular status: %s, Control stage: %s, Tracking error: %s." \
                % (round(step * control_step_size, 2), computer.F_mode, computer.F_sing, computer.F_stage, round(np.linalg.norm(local_error), 4))  )
            print(f"Local error: {local_error}, control inputs: {control_actual}")


        # Error-related vectors
        local_error_all[step] = local_error
        global_error_all[step] = global_error

        # Other vectors
        control_nominal_all[step] = computer.control_nominal
        control_actual_all[step] = computer.control_actual
        time_all[step] = current_task_time
        pos_ref_all[step] = pos_d
        vel_ref_all[step] = vel_d
        pos_all_psuedo[step] = pos_now_psuedo
        pos_all_coppelia[step] = pos_now

        pos_hat_all[step] = computer.pos_hat_now
        pos_tilde_all[step] = computer.pos_tilde_now
        U_hat_all[step] = computer.U_hat
        W_hat_all[step] = computer.W_hat_now

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
computer.send_commend_to_rover(client_socket)
client_socket.close()
        
# Data saving
data_to_save = {
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
    'e_Phi_min': e_Phi_min,
    'W_hat_all': W_hat_all
}

data_store_name = 'D:/Paper 12/Data_20250609/data'

if control_mode == 1:
    data_store_name += '_Ackerman'
else:
    data_store_name += '_hybrid'

if task_flag == 1:
    data_store_name += '_linear_traject'
elif task_flag == 2:
    data_store_name += '_dual_circle'
elif task_flag == 3:
    data_store_name += '_fixed_points'
else:
    data_store_name += '_dynamic_sine'

if control_method == 1:
    data_store_name += '_vanila_'
elif control_method == 2:
    data_store_name += '_compen_'
elif control_method == 3:
    data_store_name += '_esti_'
elif control_method == 4:
    data_store_name += '_special_esti_'
else:
    data_store_name += '_esti_compen_'

data_store_name += str(control_step_size)
data_store_name +='.mat'

savemat(data_store_name, {'__globals': {}, **data_to_save})

print("Task completed")

