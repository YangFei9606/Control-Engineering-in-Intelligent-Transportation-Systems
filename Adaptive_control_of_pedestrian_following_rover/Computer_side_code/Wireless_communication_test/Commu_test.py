import numpy as np
import time
import math
import computer_class_hitl
from pynput import keyboard

flag = 0

def on_press(key):
    global flag
    if key == keyboard.Key.esc:
        flag = 1
        print("ESC is pressed, program terminated!")
    else:
        print("Other keys are pressed, program continues.")

listener = keyboard.Listener(on_press = on_press)
listener.start()

# Basic parameters
model_parameter = np.matrix([0.145, 0.145, 0.29, 0.41, 0.29])     # l_r, l_f, l_1, l_2, l_3
task_time = 40
control_step_size = 0.05
pos_now = np.matrix([-2, -3.5, math.pi/2 * 0])

computer = computer_class_hitl.computer(model_parameter, control_step_size, pos_now)

rigid_body_id_name_map = {}
client_socket = computer.wireless_communication()

step = 0
step_max = int(task_time/control_step_size ) + 1

time_initial = time.time()
prev_time = time_initial

while step < step_max and flag == 0:

    cur_time = time.time() - time_initial

    if cur_time>= step * control_step_size:

        computer.control_nominal = np.matrix( [ math.pi/4 * math.sin(math.pi/5 * cur_time) , 0.8 + 0.2 * math.sin(math.pi/4 * cur_time)] )
        # computer.control_nominal = np.matrix([0.5, math.pi/4])
        # computer.control_nominal = np.matrix([0,0])

        control_actual = computer.nominal_to_actual()
        print(control_actual)

        computer.send_commend_to_rover(client_socket)

        step += 1

computer.control_nominal = np.matrix([0, 0])
computer.nominal_to_actual()
computer.send_commend_to_rover(client_socket)
client_socket.close()

