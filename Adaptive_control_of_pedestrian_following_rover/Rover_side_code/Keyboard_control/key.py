from pynput import keyboard
import time
import math
from serial import Serial
import motor

# Note: Switch to Python 3.8.10 for smooth run

serial_port = Serial('/dev/ttyACM0', 115200)
time.sleep(2)

# Step 1: Reset front wheels' original positions

left_id = 1
right_id = 2

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

# Step 2: Monitor the keyboards inputs

flag = 0

# 加速度控制参数
MAX_ACCELERATION = 0.05  # 最大加速度
TARGET_V_r = 0
TARGET_V_l = 0
CURRENT_V_r = 0
CURRENT_V_l = 0

def update_velocity(current, target):
    """平滑更新速度"""
    if abs(target - current) <= MAX_ACCELERATION:
        return target
    elif target > current:
        return current + MAX_ACCELERATION
    else:
        return current - MAX_ACCELERATION

def on_press(key):
    global flag
    global TARGET_V_r
    global TARGET_V_l
    try:
        print('字母键： {} 被按下'.format(key.char))
        if key.char == 'w':
            if flag == 0:
                print('前进')
                TARGET_V_r = 0.3
                TARGET_V_l = 0.3
                flag = 1
        if key.char == 's':
            if flag == 0:
                print('后退')
                TARGET_V_r = -0.3
                TARGET_V_l = -0.3
                flag = 1
        if key.char == 'a':
            if flag == 0:
                print('左转')
                TARGET_V_r = 0.2
                TARGET_V_l = -0.2
                flag = 1
        if key.char == 'd':
            if flag == 0:
                print('右转')
                TARGET_V_r = -0.2
                TARGET_V_l = 0.2
                flag = 1
        if key == keyboard.Key.space:
            print('停止')
            TARGET_V_r = 0
            TARGET_V_l = 0
            send_to_arduino(0,0)
            send_to_arduino(0,0)
            time.sleep(0.1)
            flag = 0
    except AttributeError:
        print('特殊键： {} 被按下'.format(key))
        


def on_release(key):
    global flag
    global TARGET_V_r
    global TARGET_V_l
    print('{} 释放了'.format(key))
    print('停止')

    TARGET_V_r = 0
    TARGET_V_l = 0
    flag = 0
    if key == keyboard.Key.esc:
        # 释放了esc 键，停止监听
        exit()
        return False
    
# 计算应转角
def angle_caculate(v__left, v__right):
    d1 = 0.105
    d2 = 0.105
    d3 = 0.045
    d4 = 0.105
    a = 0.5
    angle_list = [0, 0]
    if abs(v__left - v__right) >= 0.1:
        x = ((2 * d3 - a) * v__right) / (v__right - v__left)
        if (x + a - d3 - d1) != 0 and (x + d1 - d3) != 0:
            angle_left = (math.atan((a - d2 - d4) / (x + a - d3 - d1))) * 180 / math.pi
            angle_right = (math.atan((a - d2 - d4) / (x + d1 - d3))) * 180 / math.pi
            angle_list[0] = angle_left
            angle_list[1] = angle_right
        elif (x + a - d3 - d1) == 0 and (x + d1 - d3) != 0:
            angle_right = (math.atan((a - d2 - d4) / (x + d1 - d3))) * 180 / math.pi
            angle_list[0] = 90
            angle_list[1] = angle_right
        elif (x + a - d3 - d1) != 0 and (x + d1 - d3) == 0:
            angle_left = (math.atan((a - d2 - d4) / (x + a - d3 - d1))) * 180 / math.pi
            angle_list[0] = angle_left
            angle_list[1] = 90
    else:
        angle_list[0] = 0
        angle_list[1] = 0
    return angle_list[:]

# 向arduino发送轮速信息
def send_to_arduino(v_right, v_left):
    arduino_data = f"{v_right} {v_left}\n"
    serial_port.write(arduino_data.encode('utf-8'))
    # print("发送完成！")
    
# 方式2：构造监听器对象listener
listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)

# 监听启动方式2：非阻断式
listener.start()

while True:

    CURRENT_V_r = round(update_velocity(CURRENT_V_r, TARGET_V_r), 3)
    CURRENT_V_l = round(update_velocity(CURRENT_V_l, TARGET_V_l), 3) 

    if flag == 1:
        # 平滑更新当前速度
        angle_left = angle_caculate(CURRENT_V_r, CURRENT_V_l)[0]        # 通过后轮速度计算前轮角度
        angle_right = angle_caculate(CURRENT_V_r, CURRENT_V_l)[1]
    else:
        angle_left = angle_right = 0

    print("V_1 = ", CURRENT_V_r, ", V_2 = ", CURRENT_V_l,"， angle_l = ", angle_left, ", angle_r = ", angle_right)

    motor.set_angle(left_id, -angle_left, 300, 10)        # 将前轮角度发给前轮
    motor.set_angle(right_id, -angle_right, 300, 10)
    send_to_arduino(CURRENT_V_r, CURRENT_V_l)      # 将轮速发给后轮
    
    time.sleep(0.05)


