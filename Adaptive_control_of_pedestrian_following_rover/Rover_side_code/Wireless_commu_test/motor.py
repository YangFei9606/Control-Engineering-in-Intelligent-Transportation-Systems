# -*- coding=utf-8 -*-

"""
大然机器人-小米CyberGear微电机python控制库

适用平台：windows或linux平台
库版本号：v1.0
测试主控版本：windows 10 python 3.7
测试人员：唐昭
测试时间：2023.08.31

备注：该库函数目前仅适用于大然科技的USB转CAN模块
淘宝链接： https://item.taobao.com/item.htm?spm=a1z10.5-c-s.w4002-22325405943.12.31aac3b4jKqqpc&id=705379289168
"""
import time
import serial
import math as cm
import struct
import numpy

P_MIN = -12.5
P_MAX = 12.5
V_MIN = -30.0
V_MAX = 30.0
KP_MIN = 0.0
KP_MAX = 500.0
KD_MIN = 0.0
KD_MAX = 5.0
T_MIN = -12.0
T_MAX = 12.0

I_MAX = 27
TORQUE_CONSTANT = T_MAX / I_MAX

RAD_DEG = 180 / cm.pi
DEG_RAD = cm.pi / 180
RAD_S_R_MIN = 30 / cm.pi
R_MIN_RAD_S = cm.pi / 30

# uart = serial.Serial('COM4', 115200)  # 在 windows 下控制电机测试，相应的输入连接的COM口和波特率

uart = serial.Serial('/dev/ttyACM1', 115200)

# linux系统下
# result = os.popen("sudo ls -l /dev/ttyACM*").read()  # 在 jetson nano（ubuntu）及树莓派（raspbian）下控制电机测试，相应的输入连接的串口和波特率
# com = result.split()[-1]  # sudo chmod 666 /dev/ttyACM* 有时会报错，显示无曲线打开串口，这时需运行左侧命令
# os.system("sudo chmod 777 " + com)
# # os.system("sudo fuser -k " + com)
# uart = serial.Serial(com, 115200, timeout=0.5)

ERROR_FLAG = "状态正常"
READ_FLAG = 0  # 读取结果标志位
MOTOR_NUM = 127
motor_state = numpy.zeros((MOTOR_NUM, 6))
MCU_ID = []  # 电机主控MCU芯片序列号
# 电机状态二维数组，通过motor_state[id_num-1]获取电机id_num的实时返回状态[angle,speed,torque,motor_temp,axis_error,mode_status]，单位分别为degree，r/min,Nm，三个变量值均指的是电机输出轴
# 其中motor_state[id_num-1][0]表示id_num号电机的角度，motor_state[id_num-1][1]表示id_num号电机的速度，motor_state[id_num-1][2]表示id_num号电机的输出扭矩

"""
内部辅助函数，用户无需使用
"""


# 串口发送函数
def write_data(data=[]):
    global uart
    if uart.inWaiting() > 0:
        uart.read(uart.inWaiting())
    try:
        result = uart.write(data)  # 写数据
        return result
    except Exception as e:
        print("---error in write_data--：", e)
        print("重启串口")
        uart.close()
        uart.open()
        result = uart.write(data)  # 写数据
        return result


# 串口接收函数
def read_data(num=16):
    global READ_FLAG
    READ_FLAG = -1
    byte_list = []
    i = 1000  # 经过测试，发现正常接收16位耗时大概为500
    # 该函数中的两处time.sleep(0.001)会影响CAN接收速度，如果要提高CAN读取速度，可以将这两行注释掉，并将上面的i改成一个较大的值，比如5000
    while uart.inWaiting() == 0 and i > 0:  # To do:
        i -= 1
    #     time.sleep(0.001)
    # time.sleep(0.001)
    while uart.inWaiting() > 0:
        byte_list.append(list(uart.read(1))[0])
    if len(byte_list) == num:
        READ_FLAG = 1
        return byte_list
    else:
        print("Received data error in read_data(): " + str(byte_list))
        READ_FLAG = -1
        return


# USB转CAN模块包模式：CAN报文->串行帧
def can_to_uart(data=[], rtr=0):
    udata = [0xAA, 1, 0, 0x08, 0, 0, 0, 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    if len(data) == 13 and data[0] == 0x08:
        for i in range(12):
            udata[4 + i] = data[i + 1]
        return udata
    else:
        return []


# USB转CAN模块包模式：串行帧->CAN报文
def uart_to_can(data=[]):
    global READ_FLAG
    cdata = [0x08, 0, 0, 0, 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    if len(data) == 16 and data[3] == 0x08:
        for i in range(12):
            cdata[1 + i] = data[i + 4]
        return cdata
    else:
        READ_FLAG = -1
        return []


# CAN发送函数
def send_command(id_num=127, cmd_mode=0, cmd_data=[], data=[], rtr=0):
    cdata = [0x08, 0, 0, 0, 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    cdata[1] = cmd_mode
    cdata[2] = cmd_data[1]
    cdata[3] = cmd_data[0]  # master_ID
    cdata[4] = int(id_num)
    for i in range(8):
        cdata[5 + i] = data[i]
    write_data(data=can_to_uart(data=cdata, rtr=rtr))


# CAN接收函数
def receive_data():
    udata = read_data(16)
    if READ_FLAG == 1:
        cdata = uart_to_can(data=udata)
        if cdata[1] == 21:
            dump_error(cdata, True)
        return cdata[:]


# 数据格式转换函数，decode是将二进制(bytes)转化成人看的懂得数据，encode反之
def format_data(data=[], format="f f", type='decode'):
    # print(data)
    format_list = format.split()
    rdata = []
    if type == 'decode':
        p = 0
        for f in format_list:
            s_f = []
            if f == 'f':
                s_f = [4, 'f']
            elif f == 'u16':
                s_f = [2, 'H']
            elif f == 's16':
                s_f = [2, 'h']
            elif f == 'u32':
                s_f = [4, 'I']
            elif f == 's32':
                s_f = [4, 'i']
            elif f == 'u8':
                s_f = [1, 'B']
            elif f == 's8':
                s_f = [1, 'b']
            ba = bytearray()
            if len(s_f) == 2:
                for i in range(s_f[0]):
                    ba.append(data[p])
                    p = p + 1
                rdata.append(struct.unpack(s_f[1], ba)[0])
            else:
                print('unkown format in format_data(): ' + f)
                return []
        return rdata
    elif type == 'encode' and len(format_list) == len(data):
        for i in range(len(format_list)):
            f = format_list[i]
            s_f = []
            if f == 'f':
                s_f = [4, 'f']
            elif f == 'u16':
                s_f = [2, 'H']
            elif f == 's16':
                s_f = [2, 'h']
            elif f == 'u32':
                s_f = [4, 'I']
            elif f == 's32':
                s_f = [4, 'i']
            elif f == 'u8':
                s_f = [1, 'B']
            elif f == 's8':
                s_f = [1, 'b']
            if f != 'f':
                data[i] = int(data[i])
            if len(s_f) == 2:
                bs = struct.pack(s_f[1], data[i])
                for j in range(s_f[0]):
                    rdata.append(bs[j])
            else:
                print('unkown format in format_data(): ' + f)
                return []
        if len(rdata) < 4:
            for i in range(4 - len(rdata)):
                rdata.append(0x00)
        return rdata


def float_to_uint(x, x_min, x_max, bits):
    span = x_max - x_min
    offset = x_min
    if x > x_max:
        x = x_max
    elif x < x_min:
        x = x_min
    return int(((x - offset) * ((1 << bits) - 1) / span))


def uint_to_float(x, x_min, x_max, bits):
    span = (1 << bits) - 1
    offset = x_max - x_min
    if x > span:
        x = span
    elif x < 0:
        x = 0
    return offset * x / span + x_min


# 故障反馈帧解析
def dump_error(rdata=[]):
    """打印电机错误编号(电机自动返回错误信息)

       读取电机错误信息编码，如果错误编码为0，表示无异常。如果错误编码不为0，则表示存在故障。

       Args:
           rdata: 故障反馈帧内容

       Returns:
           无

       Raises:
           无

       """

    # [0x08 mode cmd_data[1] cmd_data[0] id_num data0 data1 data2 data3 data4 data5 data6 data7]
    global ERROR_FLAG
    mode = rdata[1]
    cmd_data = rdata[2:4]
    id_num = rdata[4]
    data = rdata[5:]
    if READ_FLAG == 1 and mode == 21:
        print("电机CAN_ID" + str(id_num))
        print("主CAN_ID" + str(cmd_data[0]))
    if data[0] & 0x3F or data[5] & (0x01 << 0):
        ERROR_FLAG = "状态异常："
        if data[0] & (0x01 << 0):
            ERROR_FLAG = ERROR_FLAG + '\n' + "电机过温故障，默认80度"
        if data[0] & (0x01 << 1):
            ERROR_FLAG = ERROR_FLAG + '\n' + "驱动芯⽚故障"
        if data[0] & (0x01 << 2):
            ERROR_FLAG = ERROR_FLAG + '\n' + "⽋压故障"
        if data[0] & (0x01 << 3):
            ERROR_FLAG = ERROR_FLAG + '\n' + "过压故障"
        if data[0] & (0x01 << 4):
            ERROR_FLAG = ERROR_FLAG + '\n' + "B相电流采样过流"
        if data[0] & (0x01 << 5):
            ERROR_FLAG = ERROR_FLAG + '\n' + "C相电流采样过流"
        if data[0] & (0x01 << 7):
            ERROR_FLAG = ERROR_FLAG + '\n' + "编码器未标定"
        if data[1] & 0xFF:
            ERROR_FLAG = ERROR_FLAG + '\n' + "过载故障"
        if data[2] & (0x01 << 0):
            ERROR_FLAG = ERROR_FLAG + '\n' + "A相电流采样过流"
        if data[4] & (0x01 << 0):
            ERROR_FLAG = ERROR_FLAG + '\n' + "电机过温预警，默认75度"
    else:
        ERROR_FLAG = "状态正常"
    print(ERROR_FLAG)
    return ERROR_FLAG


# 电机应答反馈帧
def reply_state(id_num=127):
    """
       @brief 电机运动控制指令状态实时返回参数
       通过该函数读取电机运动控制指令实时返回的电机状态参数[angle,speed,torque,temp,error_flag,mode_status]，单位分别为degree，r/min,Nm，三个变量值均指的是电机输出轴
       其中motor_state[id_num-1][0]表示id_num号电机的角度，motor_state[id_num-1][1]表示id_num号电机的速度，motor_state[id_num-1][2]表示id_num号电机的输出扭矩
       @param id_num 需要读取的电机编号  注意该指令id_num不能为0
    """
    global READ_FLAG, ERROR_FLAG, motor_state
    try:
        if id_num <= MOTOR_NUM:
            READ_FLAG = 0
            rdata = receive_data()
            # [0x08 mode cmd_data[1] cmd_data[0] id_num data0 data1 data2 data3 data4 data5 data6 data7]
            if READ_FLAG == 1 and rdata[1] == 2:
                cmd_data = [rdata[3], rdata[2]]
                id_num = rdata[3]  # 电机返回的数据帧中电机CAN_ID位于Bit8~Bit15
                data = rdata[5:]
                motor_state[id_num - 1][0] = uint_to_float((data[0] << 8) + data[1], P_MIN, P_MAX, 16) * RAD_DEG
                motor_state[id_num - 1][1] = uint_to_float((data[2] << 8) + data[3], V_MIN, V_MAX, 16) * RAD_S_R_MIN
                motor_state[id_num - 1][2] = uint_to_float((data[4] << 8) + data[5], T_MIN, T_MAX, 16)
                motor_state[id_num - 1][3] = ((data[6] << 8) + data[7]) * 0.1
                if cmd_data[1] & 0x3F:
                    motor_state[id_num - 1][4] = 1
                    ERROR_FLAG = '状态异常：'
                    if cmd_data[1] & (0x01 << 0):
                        ERROR_FLAG = ERROR_FLAG + '\n' + "⽋压故障"
                    if cmd_data[1] & (0x01 << 1):
                        ERROR_FLAG = ERROR_FLAG + '\n' + "过流"
                    if cmd_data[1] & (0x01 << 2):
                        ERROR_FLAG = ERROR_FLAG + '\n' + "过温"
                    if cmd_data[1] & (0x01 << 3):
                        ERROR_FLAG = ERROR_FLAG + '\n' + "磁编码故障"
                    if cmd_data[1] & (0x01 << 4):
                        ERROR_FLAG = ERROR_FLAG + '\n' + "HALL编码故障"
                    if cmd_data[1] & (0x01 << 5):
                        ERROR_FLAG = ERROR_FLAG + '\n' + "未标定"
                    print(ERROR_FLAG)
                else:
                    ERROR_FLAG = '状态正常'
                    motor_state[id_num - 1][4] = 0
                mode_status = (cmd_data[1] >> 6) & 0x03
                # if mode_status == 0:
                #     print("当前模式状态为 Reset 模式")
                # elif mode_status == 1:
                #     print("当前模式状态为 Cali 模式")
                # elif mode_status == 2:
                #     print("当前模式状态为 Motor 模式")
                motor_state[id_num - 1][5] = mode_status
    except Exception as e:
        print("---error in reply_state--：", e)


# 设置运动模式
def set_mode(id_num=127, mode=0):
    """设置电机模式。

        设置电机进入不同的控制模式。

        Args:
            id_num: 需要设置的电机ID编号。
            mode: 电机模式编号
                  mode = 0: 运控模式
                  mode = 1:  位置模式
                  mode = 2:  速度模式
                  mode = 3:  电流模式

        Returns:
            无

        Raises:
            无

        """
    write_property(id_num=id_num, index=0x7005, value=mode, data_type='u8')


# 电机使能
def motor_enable(id_num=127):
    """电机使能运行函数

        电机使能运行。

        Args:
            id_num: 需要使能运行的电机ID编号

        Returns:
            无

        Raises:
            无

        """
    master_id = 0
    cmd_data = [0] * 2
    cmd_data[0] = master_id & 0xFF
    tx_data = [0] * 8
    for i in range(8):
        tx_data[i] = 0x00
    send_command(id_num=id_num, cmd_mode=3, cmd_data=cmd_data, data=tx_data, rtr=0)  # 需要用扩展帧（数据帧）进行发送
    reply_state(id_num=id_num)


"""
电机控制函数，用户使用
"""


# 位置控制
def set_angle(id_num=127, angle=0, speed=10, limit_cur=27):
    """电机角度控制函数。

    控制指定电机编号的电机按照指定的速度转动到指定的角度。

    Args:
        id_num: 需要设置的电机ID编号
        angle: 电机转动角度(度)
        speed: 最大速度限制或前馈速度（0~300r/min）
        limit_cur:电流限制（0-27A）

    """
    motor_enable(id_num=id_num)#使能
    set_mode(id_num=id_num, mode=1)#设置运动模式
    write_property(id_num=id_num, index=0x7018, value=limit_cur, data_type='f')
    write_property(id_num=id_num, index=0x7017, value=speed * R_MIN_RAD_S, data_type='f')
    write_property(id_num=id_num, index=0x7016, value=angle * DEG_RAD, data_type='f')


# 速度控制
def set_speed(id_num=127, speed=10, limit_cur=27):
    """电机速度控制函数。

    控制指定电机编号的电机按照指定的速度连续整周转动。

    Args:
        id_num: 需要设置的电机ID编号
        speed:  目标速度（-300~300r/min）
        limit_cur:电流限制（0-27A）


    """
    motor_enable(id_num=id_num)
    set_mode(id_num=id_num, mode=2)
    write_property(id_num=id_num, index=0x7018, value=limit_cur, data_type='f')
    write_property(id_num=id_num, index=0x700A, value=speed * R_MIN_RAD_S, data_type='f')


# 扭矩（电流）控制
def set_torque(id_num=127, torque=0.1):
    """电机力矩（电流）闭环控制函数。

    控制指定电机编号的电机输出指定的扭矩（Nm）

    Args:
        id_num: 需要设置的电机ID编号
        torque: 电机输出（0~12Nm）


    """
    motor_enable(id_num=id_num)
    set_mode(id_num=id_num, mode=3)
    write_property(id_num=id_num, index=0x7006, value=torque / TORQUE_CONSTANT, data_type='f')


# 阻抗控制
def impedance_control(id_num=127, pos=0, vel=0, tff=0, kp=0, kd=0):
    """运控模式电机控制指令函数。

    运控模式电机控制指令， ⽤来向电机发送控制指令

    Args:
        id_num: 需要设置的电机ID编号
        pos: 电机目标角度（度）
        vel: 电机目标速度（r/min）
        tff: 前馈扭矩（Nm)
        kp: 刚度系数(rad/Nm)
        kd: 阻尼系数(rad/s/Nm)

    Returns:
        无

    Raises:
        "---error in impedance_control---"

    """
    try:
        motor_enable(id_num=id_num)
        set_mode(id_num=id_num, mode=0)
        cmd_data = [0] * 2
        cmd_data[0] = (float_to_uint(tff, T_MIN, T_MAX, 16)) & 0xFF  # 阻抗控制里都是高位在前，低位在后，不满足小端排序
        cmd_data[1] = ((float_to_uint(tff, T_MIN, T_MAX, 16)) >> 8) & 0xFF
        tx_data = [0] * 8
        tx_data[0] = (float_to_uint(pos * DEG_RAD, P_MIN, P_MAX, 16) >> 8) & 0xFF
        tx_data[1] = (float_to_uint(pos * DEG_RAD, P_MIN, P_MAX, 16)) & 0xFF
        tx_data[2] = (float_to_uint(vel * R_MIN_RAD_S, V_MIN, V_MAX, 16) >> 8) & 0xFF
        tx_data[3] = (float_to_uint(vel * R_MIN_RAD_S, V_MIN, V_MAX, 16)) & 0xFF
        tx_data[4] = (float_to_uint(kp, KP_MIN, KP_MAX, 16) >> 8) & 0xFF
        tx_data[5] = (float_to_uint(kp, KP_MIN, KP_MAX, 16)) & 0xFF
        tx_data[6] = (float_to_uint(kd, KD_MIN, KD_MAX, 16) >> 8) & 0xFF
        tx_data[7] = (float_to_uint(kd, KD_MIN, KD_MAX, 16)) & 0xFF
        send_command(id_num=id_num, cmd_mode=1, cmd_data=cmd_data, data=tx_data, rtr=0)  # 需要用扩展帧（数据帧）进行发送
        reply_state(id_num=id_num)
    except Exception as e:
        print("---error in impedance_control--：", e)


# 电机停止运行（急停）
def motor_estop(id_num=127):
    """停止运行函数

        控制电机停止运行。

        Args:
            id_num: 需要停止运行的电机ID编号

        Returns:
            无

        Raises:
            无

        """
    master_id = 0
    cmd_data = [0] * 2
    cmd_data[0] = master_id & 0xFF
    tx_data = [0] * 8
    for i in range(8):
        tx_data[i] = 0x00
    send_command(id_num=id_num, cmd_mode=4, cmd_data=cmd_data, data=tx_data, rtr=0)  # 需要用扩展帧（数据帧）进行发送
    reply_state(id_num=id_num)


# 设置零点
def set_zero_position(id_num=127):
    """设置电机零点位置函数

    设置电机机械零点位置，会把当前电机位置设为机械零位

    Args:
        id_num: 需要设置的电机ID编号

    Returns:
        无
    Raises:
        无

    """
    mode_status = motor_state[id_num - 1][5]
    motor_estop(id_num=id_num)
    master_id = 0
    cmd_data = [0] * 2
    cmd_data[0] = master_id & 0xFF
    tx_data = [0] * 8
    tx_data[0] = 0x01
    send_command(id_num=id_num, cmd_mode=6, cmd_data=cmd_data, data=tx_data, rtr=0)  # 需要用扩展帧（数据帧）进行发送，不能用远程帧
    reply_state(id_num=id_num)
    if mode_status == 2:
        motor_enable(id_num=id_num)


# 清除错误
def clear_error(id_num=127):
    """清除错误函数

        一旦电机运行过程中出现任何错误, 如果要恢复正常控制模式，需要用clear_error清除错误

        Args:
            id_num: 需要清除错误标志的电机ID编号

        Returns:
            无

        Raises:
            无

        """
    global ERROR_FLAG
    ERROR_FLAG = '状态正常'
    master_id = 0
    cmd_data = [0] * 2
    cmd_data[0] = master_id & 0xFF
    tx_data = [0] * 8
    tx_data[0] = 0x01
    send_command(id_num=id_num, cmd_mode=4, cmd_data=cmd_data, data=tx_data, rtr=0)  # 需要用扩展帧（数据帧）进行发送
    reply_state(id_num=id_num)


# 设置电机CAN ID
def set_id(id_num=127, new_id=1):
    """设置电机ID号。

       改变电机CAN_ID号（掉电保存）,更改当前电机CAN_ID , ⽴即⽣效。

       Args:
           id_num: 需要重新设置编号的电机编号,如果不知道当前电机编号，可以用0广播，但是这时总线上只能连一个电机，否则多个电机会被设置成相同编号
           new_id: 预设ID

       Returns:
           True: 设置成功
           False:设置失败

       Raises:
           无

       """
    global MCU_ID
    motor_estop(id_num=id_num)  # 修改ID号必须在电机模式下进行  否则无法设置成功
    time.sleep(0.1)  # 等待一会，需要保存
    get_id(id_num=id_num)
    master_id = 0
    cmd_data = [0] * 2
    cmd_data[1] = new_id & 0xFF
    cmd_data[0] = master_id & 0xFF  # 预设置CAN_ID
    print(11)
    if len(MCU_ID) == 8:
        tx_data = MCU_ID
        send_command(id_num=id_num, cmd_mode=7, cmd_data=cmd_data, data=tx_data, rtr=0)  # 需要用扩展帧（数据帧）进行发送
        time.sleep(0.1)  # 等待一会，需要保存
        reply_state(id_num=id_num)
        return True
    else:
        print("set_id to " + str(new_id) + " 失败!")
        return False


# 恢复出厂设置
def init_config(id_num=127):
    """将电机参数恢复至出厂设置函数

    如果想将所有配置参数恢复至出厂设置，可以调用该函数（默认情况下恢复出厂设置后CAN_ID会恢复成127，为了方便使用，该函数自动将其改回原来的id_num）

    Args:
        id_num: 需要修改的电机ID编号

    Returns:
        无

    Raises:
        无

    """
    master_id = 0
    cmd_data = [0] * 2
    cmd_data[1] = 0x03
    cmd_data[0] = master_id & 0xFF
    tx_data = [0] * 8
    for i in range(8):
        tx_data[i] = 0x00
    send_command(id_num=id_num, cmd_mode=8, cmd_data=cmd_data, data=tx_data, rtr=0)  # 需要用扩展帧（数据帧）进行发送
    print("恢复出厂设置中。。。。。，请稍等3s之后再继续操作")
    time.sleep(3.0)
    set_id(127, id_num)
    print("恢复出厂设置成功！")


# 设置参数
def write_property(id_num=127, index=0, data_type='f', value=0):
    """修改电机属性参数

        修改电机属性参数，这里的属性参数为电机控制参数

        Args:
            id_num: 需要修改的电机ID编号
            index: 需要读取的参数地址
            data_type: 需要写入的参数数据类型：‘f’:float,'u16':uint16,'s16':int16,'u32':uint32,'s32':int32,'u8':uint8,'s8':'int8'
            value: 对应参数数据。


        Returns:
            无

        Raises:
            无

        """
    master_id = 0
    cmd_data = [0] * 2
    cmd_data[0] = master_id & 0xFF
    tx_data = [0] * 8
    tx_data[0] = index & 0xFF
    tx_data[1] = (index >> 8) & 0xFF
    value = value
    cmd_mode = 18
    if index < 0x7000:
        cmd_mode = 8
        type_list = ['u8', 's8', 'u16', 's16', 'u32', 's32', 'f']
        tx_data[2] = type_list.index(data_type)
    tx_data[4:] = format_data(data=[value], format=data_type, type="encode")
    send_command(id_num=id_num, cmd_mode=cmd_mode, cmd_data=cmd_data, data=tx_data, rtr=0)  # 需要用扩展帧（数据帧）进行发送
    reply_state(id_num=id_num)
    if cmd_mode == 8:  # 永久保存
        motor_estop(id_num=id_num)  # 永久保存时，也需要先进入待机模式
        cmd_data[1] = 0x02
        tx_data = [0] * 8
        send_command(id_num=id_num, cmd_mode=cmd_mode, cmd_data=cmd_data, data=tx_data, rtr=0)  # 需要用扩展帧（数据帧）进行发送
        time.sleep(0.1)  # 等待一会，需要保存
        reply_state(id_num=id_num)


# 读取参数
def read_property(id_num=127, index=0, data_type='f'):
    """读取电机属性参数

       读取电机属性参数，这里的属性参数包括电机状态量及电机控制参数

       Args:
            id_num: 需要修改的电机ID编号
            index: 需要读取的参数地址
            data_type: 需要读取的参数数据类型：‘f’:float,'u16':uint16,'s16':int16,'u32':uint32,'s32':int32,'u8':uint8,'s8':'int8'

       Returns:
           value: 返回对应属性参数的值

       Raises:
           无

       """
    master_id = 0
    cmd_data = [0] * 2
    cmd_data[0] = master_id & 0xFF
    tx_data = [0] * 8
    tx_data[0] = index & 0xFF
    tx_data[1] = (index >> 8) & 0xFF
    cmd_mode = 17
    if index < 0x7000:
        cmd_mode = 9
        type_list = ['u8', 's8', 'u16', 's16', 'u32', 's32', 'f']
        tx_data[2] = type_list.index(data_type)
    send_command(id_num=id_num, cmd_mode=cmd_mode, cmd_data=cmd_data, data=tx_data, rtr=0)  # 需要用扩展帧（数据帧）进行发送
    data = receive_data()
    if READ_FLAG == 1 and (data[1] == 17 or data[1] == 9):
        value = format_data(data=data[9:], format=data_type, type="decode")
        return value[0]


# 获取电机ID号（同时会读取电机MCU_ID）
def get_id(id_num=127):
    global MCU_ID
    master_id = 0xFD
    cmd_data = [0] * 2
    cmd_data[0] = master_id & 0xFF
    tx_data = [0] * 8
    send_command(id_num=id_num, cmd_mode=0, cmd_data=cmd_data, data=tx_data, rtr=0)  # 需要用扩展帧（数据帧）进行发送
    data = receive_data()
    if READ_FLAG == 1 and data[1] == 0:
        MCU_ID = data[5:]
        return id_num


# 读取电机当前角度及速度
def get_state(id_num=127):
    """读取电机的当前位置和速度

    读取电机当前位置和速度列表，单位分别为度（°）和转每分钟(r/min)

    Args:
        id_num: 需要读取的电机编号,如果不知道当前电机编号，可以用0广播，但是这时总线上只能连一个电机，否则将报错。

    Returns:
        [pos, vel]: 位置和速度列表

    Raises:
        无

    """
    global READ_FLAG, motor_state
    pos_vel = [0, 0]
    try:
        # 调用master_ID写入接口，通过电机应答反馈帧获取实时位置和速度
        write_property(id_num=id_num, index=0x7018, value=27, data_type='f')
        if READ_FLAG == 1 and id_num != 0:
            pos_vel[0] = round(motor_state[id_num - 1][0], 1)
            pos_vel[1] = round(motor_state[id_num - 1][1], 1)
            return pos_vel[:]
        else:
            return
    except Exception as e:
        print("---error in get_state---：", e)
        return False


# 读取电机当前电压及电流
def get_volcur(id_num=127):
    """读取电机的当前电压和电流

    读取电机当前电压和q轴电流列表，单位分别为伏（V）和安(A)

    Args:
        id_num: 需要读取的电机编号,如果不知道当前电机编号，可以用0广播，但是这时总线上只能连一个电机，否则将报错。

    Returns:
        [vol, cur]: 电压和电流列表

    Raises:
        无

    """
    global READ_FLAG
    vol_cur = [0, 0]
    try:
        vol_cur[0] = read_property(id_num=id_num, index=0x302b, data_type='f')
        vol_cur[1] = read_property(id_num=id_num, index=0x301e, data_type='f')
        if READ_FLAG == 1:
            vol_cur[0] = round(vol_cur[0], 1)  # 将单位从mV换算成V
            vol_cur[1] = round(vol_cur[1], 2)
            return vol_cur
        else:
            return
    except Exception as e:
        print("---error in get_volcur--：", e)
        return False







