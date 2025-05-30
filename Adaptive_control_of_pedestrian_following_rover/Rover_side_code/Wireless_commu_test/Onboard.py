import socket
import math
import numpy as np 
import motor
import time
from serial import Serial

left_id = 1
right_id = 2



def send_control_command(v_r_l, v_r_r, sigma_f_l, sigma_f_r, serial_port):

    # Why negative?
    motor.set_angle(left_id, sigma_f_l, 300, 10)
    motor.set_angle(right_id, sigma_f_r, 300, 10)
    arduino_data = f"{v_r_r} {v_r_l}\n"
    serial_port.write(arduino_data.encode('utf-8'))
    print("Data is sent:")

def vehicle_2_computer_commu():
    print("Trying to build v2c communication.")
    HOST = '192.168.31.62'   # 接收端IP地址
    # HOST = '192.168.3.6'   # 接收端IP地址 (The address of the onboard computer)
    PORT = 3000  # 端口号
    # serial_port = Serial('COM7', 115200)
    serial_port = Serial('/dev/ttyACM0', 115200)
    time.sleep(1)
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen()
    
    conn, addr = server_socket.accept()

    print("Wireless connection is established!")

    return serial_port, server_socket, conn, addr

def motor_initialise(left_id, right_id):

    motor.motor_estop(left_id)
    motor.motor_estop(right_id)
    time.sleep(0.1)

    while True:
        ans = input("是否已经初始化前轮位置？(Y/N)： ")

        if ans == 'Y' or 'y':
            break
        elif ans == 'N' or 'n':
            print("请初始化前轮位置。")
        else:
            print("输入错误，请调整。")

    motor.set_zero_position(left_id)
    motor.set_zero_position(right_id)
    time.sleep(0.1)

    print("前轮初始化已经完成。")


def main():
    # Real-time loop related
    task_time = 40
    control_step_size = 0.05
    step = 0
    step_max = int(task_time/control_step_size ) + 1
    # For wireless communication (alter if necessary)
    number_receive = 9
    number_length = 8
    # model_parameter = np.matrix([0.145, 0.145, 0.29, 0.41])

    motor_initialise(left_id, right_id)

    serial_port, server_socket, conn, _ = vehicle_2_computer_commu()

    while step < step_max:

        received = conn.recv(number_receive * number_length)

        if received:
            # Bytes to tuple
            received_now = np.frombuffer(received, dtype = np.float64)

            v_r_l = round(received_now[0], 3)
            v_r_r = round(received_now[1], 3)
            sigma_f_l = received_now[2]
            sigma_f_r = received_now[3]

            send_control_command(v_r_l, v_r_r, sigma_f_l, sigma_f_r, serial_port)

            print("Step number:", step)
            
            print(v_r_l, v_r_r, sigma_f_l, sigma_f_r)
            
            step += 1

    send_control_command(0,0,0,0, serial_port)
    
    conn.close()
    server_socket.close()

if __name__ == "__main__":

    main()

# # Transfer nominal control inputs into actual control inputs
# def actual_control_input_matrix(control_input, model_parameter):
    
#     v_r_l = (1 - model_parameter[0,3] * control_input[0,1]/ (2 * np.sum(model_parameter[0, 0:2]))) * control_input[0,0]
#     v_r_r = (1 + model_parameter[0,3] * control_input[0,1]/ (2 * np.sum(model_parameter[0, 0:2]))) * control_input[0,0]
#     sigma_f_l = math.atan(2 * control_input[0,1] * np.sum(model_parameter[0, 0:2]) / (2 * np.sum(model_parameter[0, 0:2]) - model_parameter[0,2] * control_input[0,1]) ) * 180/math.pi
#     sigma_f_r = math.atan(2 * control_input[0,1] * np.sum(model_parameter[0, 0:2]) / (2 * np.sum(model_parameter[0, 0:2]) + model_parameter[0,2] * control_input[0,1]) ) * 180/math.pi

#     return np.matrix([-v_r_l, v_r_r, sigma_f_l, sigma_f_r])

# def actual_control_input(control_input, model_parameter):
    
#     v_r_l = (1 - model_parameter[0,3] * control_input[0,1]/ (2 * np.sum(model_parameter[0, 0:2]))) * control_input[0,0]
#     v_r_r = (1 + model_parameter[0,3] * control_input[0,1]/ (2 * np.sum(model_parameter[0, 0:2]))) * control_input[0,0]
#     sigma_f_l = math.atan(2 * control_input[0,1] * np.sum(model_parameter[0, 0:2]) / (2 * np.sum(model_parameter[0, 0:2]) - model_parameter[0,2] * control_input[0,1]) ) * 180/math.pi
#     sigma_f_r = math.atan(2 * control_input[0,1] * np.sum(model_parameter[0, 0:2]) / (2 * np.sum(model_parameter[0, 0:2]) + model_parameter[0,2] * control_input[0,1]) ) * 180/math.pi

#     return -v_r_l, v_r_r, sigma_f_l, sigma_f_r
