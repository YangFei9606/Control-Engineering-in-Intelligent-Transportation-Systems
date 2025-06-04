import nokov_function
import numpy as np
import time
import math
from scipy.io import savemat

task_time = 30
control_step_size = 0.05

step = 0
step_max = int(task_time/control_step_size ) + 1

# Global variable to store rigid body ID to name mapping
rigid_body_id_name_map = {}

pos_now = np.matrix([0,0,0])
pos_all = np.zeros((step_max, 3))
time_all = np.zeros((step_max, 1))


client = nokov_function.nokov_setup()
body_name = "test_rover"

# while True:

#     ans = input("Start to receive data now?(Y/N): ")

#     if ans == 'Y' or 'y':
#         break
#     else:
#         print("I will wait a bit longer.")

time.sleep(0.5)

pos_now = nokov_function.nokov_feedback(client, body_name, pos_now)
pos_hat_now = pos_now

time_initial = time.time()
prev_time = time_initial

while step < step_max:
    cur_time = time.time() - time_initial

    if cur_time >= step * control_step_size:
        task_actual_interval = cur_time - prev_time
        pos_now = nokov_function.nokov_feedback(client, body_name, pos_now)

        time_all[step] = cur_time
        pos_all[step] = pos_now

        if step % 10 == 0:
            print(f"Task time: {round(cur_time, 2)} s, Current position: {pos_now}.")

        step += 1
        prev_time = cur_time


data_to_save = {
    'pos_all': pos_all,
    'time_all': time_all
}

data_address = 'D:/Paper 12/Data_nokov_test.mat'

savemat(data_address, {'__globals': {}, **data_to_save})

print(data_address)

