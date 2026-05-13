import numpy as np
import math
import time
import sys
import socket

class computer(object):

    def __init__(self, model_parameter, control_step_size, pos_initial):

        # Basic parameters
        self.clientID = 0
        self.error_code = 0
        self.control_step_size = control_step_size

        # Controller parameters
        self.v_m = 1.2
        self.tan_m = 1
        self.k_lon = 1
        self.k_lat = 0.25
        
        self.eta_1 = np.multiply(np.diag([6,6,6]), 1)
        self.eta_2 = np.multiply(np.diag([0.2,0.2,0.2]), 0.5)
        self.eta_3 = np.multiply(np.diag([0.5,0.5,0.5]), 1)
        self.eta_4 = np.multiply(np.diag([1,1,1]), 20)
        self.eta_5 = np.multiply(np.diag([1,1,1]), 0.5)
        self.p = 11/13
        self.q = 13/11

        self.k_v = 0.6
        self.k_u = 0.6
        self.acc_lim = 0.4
        self.ang_vel_lim = math.pi/4
        self.yaw_vel_lim = math.pi/3
        self.xi_M = 1
        self.bar_eta_1 = 3
        self.bar_eta_2 = 0.5

        self.xi_u = 0
        self.xi_v = 0

        # For the singular issue algorithms
        self.v_d_min = 0.05
        self.e_Phi_max = math.pi * (80/180)
        self.e_Phi_min = math.pi * (30/180)
        self.gamma_v_max = 0.6
        self.gamma_v_min = 0.2
        self.F_sing = 0
        self.F_mode = 1
        self.F_stage = 2

        self.max_acc_step = 1.5 * self.acc_lim * self.control_step_size
        self.max_ang_step = 1.5 * self.ang_vel_lim * self.control_step_size

        # Basic parameters
        self.pos_hat_now = pos_initial
        self.pos_tilde_now = pos_initial
        self.W_hat_now = np.matrix([0, 0, 0])
        self.U_hat = np.matrix([0, 0, 0])
        
        self.model_parameter = model_parameter
        self.robot_pos = pos_initial
        self.robot_pos_prev = self.robot_pos
        self.robot_pos_psuedo = pos_initial
        self.robot_pos_coppelia = pos_initial
        self.local_error = np.matrix([0, 0])
        self.global_error = np.matrix([0, 0])
        self.control_nominal = np.matrix([0, 0])
        self.control_actual = np.matrix([0, 0, 0, 0])

        self.pos_ref = np.matrix([0, 0, 0])
        self.vel_ref = np.matrix([0, 0])

#################################################################### Python controller ########################################################################

    # Function 0: Wireless communication to the client computer
    def wireless_communication(self):
        # Communication related definitions
        # HOST = '192.168.115.43'  # 接收端IP地址
        # HOST = '172.20.10.9'  # 接收端IP地址
        HOST = '192.168.88.4'
        # HOST = '192.168.31.71'  # 接收端IP地址
        PORT = 3000  # 端口号
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((HOST, PORT))

        print("Wireless communication established!")

        return client_socket

########################### Angle calculation ####################################
    def angle_regulation(self, pos_info):

        # This file regulates the yaw angle within the region of [-pi, pi]
        while abs(pos_info[0,2]) > math.pi:
            pos_info[0,2] -= np.sign(pos_info[0,2]) * 2 * math.pi

        return pos_info

########################### Reference generation ####################################
    def reference_generator(self, current_time, task_flag):

        if task_flag == 1:
            self.vel_ref = np.matrix([0, 0.3 + 0.2 * np.cos(np.pi/10 * current_time)])
            self.pos_ref = np.matrix([-3, -4 + 0.3 * current_time + 2/np.pi * np.sin(np.pi/10 * current_time), 0])

        elif task_flag == 2:
            w = math.pi/6
            r = 2
            phase = 0

            g_t = np.sign( np.sin(0.5 * w * current_time + phase) )
            # g_t = 1

            # # Desired velocity
            # self.vel_ref = np.matrix([ w * r * np. sin(w * current_time + phase) * g_t,
            #                     w * r * np. cos(w * current_time + phase) ])
            
            # # Desired trajectory
            # self.pos_ref = np.matrix([ r * ( 1 - np.cos(w * current_time + phase) ) * g_t,
            #                     r * np.sin(w * current_time + phase), math.atan(self.vel_ref[0,1]/self.vel_ref[0,0]) ])
            
            # Desired velocity
            self.vel_ref = np.matrix([ w * r * np. cos(w * current_time + phase), 
                                      w * r * np. sin(w * current_time + phase) * g_t])
            
            # Desired trajectory (x coordinate -1.3)
            self.pos_ref = np.matrix([ r * np.sin(w * current_time + phase) - 3.5,  
                                       r * ( 1 - np.cos(w * current_time + phase) ) * g_t - 3.3,
                                       math.atan(self.vel_ref[0,1]/self.vel_ref[0,0]) ])
            
        elif task_flag == 3:
            self.vel_ref = np.matrix([0, 0])
            if current_time <= 10:
                # self.pos_ref = np.matrix([-2, -5, 0])
                self.pos_ref = np.matrix([-6, -6, 0])
            elif 10 < current_time <= 20:
                # self.pos_ref = np.matrix([1, -5, 0])
                self.pos_ref = np.matrix([-3.5, -6, 0])
            elif 20 < current_time <= 30:
                # self.pos_ref = np.matrix([1, 0, 0])
                self.pos_ref = np.matrix([-3, -4, 0])
            else:
                # self.pos_ref = np.matrix([-3, 0, 0])
                self.pos_ref = np.matrix([-5, -3, 0])

        else:
            v_x = -0
            v_y = -0
            w_center = 0.15
            r_center_x = 0.6
            r_center_y = 1.2
            center_x = -3 + r_center_x * np.cos(w_center * current_time) + v_x * current_time
            center_y = -3 + r_center_y * np.sin(w_center * current_time) + v_y * current_time

            # center_x = 0
            # center_y = 2

            w = math.pi/5
            r = 1.2
            phase = 0

            # Desired velocity
            self.vel_ref = np.matrix([ - r * w * np.sin(w * current_time + phase) - w_center * r_center_x * np.sin(w_center * current_time) + v_x, 
                        r * w * np.cos(w * current_time + phase) + w_center * r_center_y * np.cos(w_center * current_time) + v_y])

            # Desired trajectory
            self.pos_ref = np.matrix([ center_x + r * np.cos(w * current_time + phase), 
                        center_y + r * np.sin(w * current_time + phase), math.atan(self.vel_ref[0,1]/self.vel_ref[0,0])])

        
        return self.pos_ref, self.vel_ref
    
########################### Open loop calculation of system states ####################################
    def read_pos_psuedo(self):

        W = np.matrix([0,0,0])

        X_dot = self.control_nominal[0,0] * np.cos(self.robot_pos_psuedo[0,2]) - self.control_nominal[0,0] * self.model_parameter[0,0] / self.model_parameter[0,4] * np.sin(self.robot_pos_psuedo[0,2]) * self.control_nominal[0,1]
        Y_dot = self.control_nominal[0,0] * np.sin(self.robot_pos_psuedo[0,2]) + self.control_nominal[0,0] * self.model_parameter[0,0] / self.model_parameter[0,4] * np.cos(self.robot_pos_psuedo[0,2]) * self.control_nominal[0,1]
        Phi_dot = self.control_nominal[0,0] / self.model_parameter[0,4] * self.control_nominal[0,1]

        pos_dot = np.matrix([X_dot, Y_dot, Phi_dot]) + W

        self.robot_pos_psuedo = np.multiply(pos_dot, self.control_step_size) + self.robot_pos_psuedo

        self.robot_pos_psuedo = self.angle_regulation(self.robot_pos_psuedo)

        return self.robot_pos_psuedo

########################### Adaptive uncertainty estimation ####################################
    def adaptive_estimator(self):

        # Kinematics model
        X_dot = self.control_nominal[0,0] * np.cos(self.robot_pos_prev[0,2]) - self.control_nominal[0,0] * self.model_parameter[0,0] / self.model_parameter[0,4] * np.sin(self.robot_pos_prev[0,2]) * self.control_nominal[0,1]
        Y_dot = self.control_nominal[0,0] * np.sin(self.robot_pos_prev[0,2]) + self.control_nominal[0,0] * self.model_parameter[0,0] / self.model_parameter[0,4] * np.cos(self.robot_pos_prev[0,2]) * self.control_nominal[0,1]
        
        Phi_dot = self.control_nominal[0,0] / self.model_parameter[0,4] * self.control_nominal[0,1]
        model = np.matrix([X_dot, Y_dot, Phi_dot])
        pos_hat_dot = self.U_hat + model

        self.pos_hat_now = np.multiply(pos_hat_dot, self.control_step_size) + self.pos_hat_now

        self.pos_tilde_now = self.angle_regulation(self.robot_pos - self.pos_hat_now)

        tilde_frac_1 = [0,0,0]
        tilde_frac_2 = [0,0,0]

        for a in range(np.size(self.pos_tilde_now, 1)):
            temp = self.pos_tilde_now[0,a]
            tilde_frac_1[a] = np.sign(temp) * abs(temp) ** self.p
            tilde_frac_2[a] = np.sign(temp) * abs(temp) ** self.q

        # Adaptive update law of the estimation variable
        W_hat_dot = np.dot(self.eta_4, self.pos_tilde_now.T) - np.dot(self.eta_5, self.W_hat_now.T)

        # Update of the estimation variable
        self.W_hat_now = np.multiply(W_hat_dot.T, self.control_step_size) + self.W_hat_now 

        # Model-based update
        self.U_hat = (np.dot(self.eta_1, self.pos_tilde_now.T) + np.dot(self.eta_2, np.matrix(tilde_frac_1).T) + np.dot(self.eta_3, np.matrix(tilde_frac_2).T)).T + self.W_hat_now

    def reset(self):
        
        self.U_hat = np.matrix([0,0,0])
        self.W_hat_now = np.matrix([0,0,0])
        self.pos_hat_now = self.robot_pos
        self.pos_tilde_now = np.matrix([0,0,0])
        self.xi_u = 0
        self.xi_v = 0
        self.control_nominal = np.matrix([0,0])


########################### Calculate global and local errors ####################################    
    def error_calculation(self):

        # Global error calculation
        X_e = self.robot_pos[0,0] - self.pos_ref[0,0]
        Y_e = self.robot_pos[0,1] - self.pos_ref[0,1]

        # Local error calculation
        x_e = np.cos(self.robot_pos[0,2]) * X_e + np.sin(self.robot_pos[0,2]) * Y_e
        y_e = - np.sin(self.robot_pos[0,2]) * X_e + np.cos(self.robot_pos[0,2]) * Y_e

        self.global_error = np.matrix([X_e, Y_e])
        self.local_error = np.matrix([x_e, y_e])

        return self.global_error, self.local_error
    
########################### Different controller selections ####################################
    def kinematic_controller_vanila(self):

        # Bring back the previous control inputs for dynamic saturation thresholds
        control_prev = self.control_nominal
        v_Max = min(control_prev[0, 0] + self.acc_lim * self.control_step_size, self.v_m)
        v_Min = max(control_prev[0, 0] - self.acc_lim * self.control_step_size, - self.v_m)
        
        v_r_nom = np.cos(self.robot_pos[0,2]) * self.vel_ref[0,0] + np.sin(self.robot_pos[0,2]) * self.vel_ref[0,1] - self.k_lon * self.local_error[0,0]

        v_r = max( min(v_r_nom, v_Max), v_Min )

        # Calculate the dynamic lateral saturation threshold
        '''It is essential to consider the yaw angle limitation, the allowed steering angle should decrease as the longitudinal speed increases'''
        tan_M = self.yaw_vel_lim * self.model_parameter[0,4] / max(abs(v_r), self.v_d_min)
        tan_lim = min(tan_M, self.tan_m)

        tan_Max = min( np.tan( math.atan(control_prev[0,1]) + self.ang_vel_lim * self.control_step_size ), tan_lim )
        tan_Min = max( np.tan( math.atan(control_prev[0,1]) - self.ang_vel_lim * self.control_step_size ), - tan_lim )

        tan_nom = ( np.cos(self.robot_pos[0,2]) * self.vel_ref[0,1] - np.sin(self.robot_pos[0,2]) * self.vel_ref[0,0] \
                   - self.k_lat * self.local_error[0,1] ) * self.model_parameter[0,4]/(abs(v_r) * self.model_parameter[0,0])
        
        tan_sigma_f = max( min(tan_nom, tan_Max), tan_Min)

        self.control_nominal = np.matrix([v_r, tan_sigma_f])

        return self.control_nominal
    
    def kinematic_controller_estimator(self):
        # Bring back the previous control inputs for dynamic saturation thresholds
        control_prev = self.control_nominal
        v_Max = min(control_prev[0, 0] + self.acc_lim * self.control_step_size, self.v_m)
        v_Min = max(control_prev[0, 0] - self.acc_lim * self.control_step_size, - self.v_m)

        v_r_nom = np.cos(self.robot_pos[0,2]) * (self.vel_ref[0,0] - self.W_hat_now[0,0]) + np.sin(self.robot_pos[0,2]) * \
            (self.vel_ref[0,1] - self.W_hat_now[0,1]) - self.k_lon * self.local_error[0,0]

        v_r = max( min(v_r_nom, v_Max), v_Min )

        # Calculate the dynamic lateral saturation threshold
        '''It is essential to consider the yaw angle limitation, the allowed steering angle should decrease as the longitudinal speed increases'''
        tan_M = self.yaw_vel_lim * self.model_parameter[0,4] / max(abs(v_r), self.v_d_min)
        tan_lim = min(tan_M, self.tan_m)

        tan_Max = min( np.tan( math.atan(control_prev[0,1]) + self.ang_vel_lim * self.control_step_size ), tan_lim )
        tan_Min = max( np.tan( math.atan(control_prev[0,1]) - self.ang_vel_lim * self.control_step_size ), - tan_lim )

        tan_nom = ( np.cos(self.robot_pos[0,2]) * (self.vel_ref[0,1] - self.W_hat_now[0,1]) - np.sin(self.robot_pos[0,2]) * (self.vel_ref[0,0] - self.W_hat_now[0,0]) \
                   - self.k_lat * self.local_error[0,1] ) * self.model_parameter[0,4]/(abs(v_r) * self.model_parameter[0,0])
        
        tan_sigma_f = max( min(tan_nom, tan_Max), tan_Min)

        self.control_nominal = np.matrix([v_r, tan_sigma_f])

        return self.control_nominal
    
    def kinematic_controller_compensator(self):

        # Bring back the previous control inputs for dynamic saturation thresholds
        control_prev = self.control_nominal
        v_Max = min(control_prev[0, 0] + self.acc_lim * self.control_step_size, self.v_m)
        v_Min = max(control_prev[0, 0] - self.acc_lim * self.control_step_size, - self.v_m)

        v_r_nom = np.cos(self.robot_pos[0,2]) * self.vel_ref[0,0] + np.sin(self.robot_pos[0,2]) * self.vel_ref[0,1] \
            - self.k_lon * self.local_error[0,0] - self.k_v * self.xi_v
        v_r = max( min(v_r_nom, v_Max), v_Min )

        # Adaptive speed compensation
        Delta_v = v_r_nom - v_r

        if abs(self.xi_v) <= self.xi_M:
            self.xi_v += self.control_step_size * (self.bar_eta_1 * Delta_v - self.bar_eta_2 * self.xi_v)
        else:
            self.xi_v += self.control_step_size * (self.bar_eta_1 * Delta_v - self.bar_eta_2 * self.xi_v - (Delta_v) ** 2/self.xi_v )

        # Calculate the dynamic lateral saturation threshold
        '''It is essential to consider the yaw angle limitation, the allowed steering angle should decrease as the longitudinal speed increases'''
        tan_M = self.yaw_vel_lim * self.model_parameter[0,4] / max(abs(v_r), self.v_d_min)
        tan_lim = min(tan_M, self.tan_m)

        tan_Max = min( np.tan( math.atan(control_prev[0,1]) + self.ang_vel_lim * self.control_step_size ), tan_lim )
        tan_Min = max( np.tan( math.atan(control_prev[0,1]) - self.ang_vel_lim * self.control_step_size ), - tan_lim )

        u_f_nom = ( np.cos(self.robot_pos[0,2]) * self.vel_ref[0,1] - np.sin(self.robot_pos[0,2]) * self.vel_ref[0,0] \
                   - self.k_lat * self.local_error[0,1] ) * self.model_parameter[0,4] / self.model_parameter[0,0]

        if v_r >= self.v_d_min:
            u_f = max( min(u_f_nom, tan_Max * v_r), tan_Min * v_r)
            tan_sigma_f = u_f/v_r
        elif v_r <= - self.v_d_min:
            u_f = max( min(u_f_nom, tan_Max * v_r), self.tan_m * v_r)
            tan_sigma_f = - u_f/v_r
        else:
            tan_sigma_f = self.control_nominal[0,1]

        self.control_nominal = np.matrix([v_r, tan_sigma_f])

        return self.control_nominal

    def kinematic_controller_compensator_estimator(self):
        # Controller with both compensator and estimator

        # Bring back the previous control inputs for dynamic saturation thresholds
        control_prev = self.control_nominal
        v_Max = min(control_prev[0, 0] + self.acc_lim * self.control_step_size, self.v_m)
        v_Min = max(control_prev[0, 0] - self.acc_lim * self.control_step_size, - self.v_m)

        v_r_nom = np.cos(self.robot_pos[0,2]) * (self.vel_ref[0,0] - self.W_hat_now[0,0]) + np.sin(self.robot_pos[0,2]) * (self.vel_ref[0,1] - self.W_hat_now[0,1]) \
            - self.k_lon * self.local_error[0,0] - self.k_v * self.xi_v
        v_r = max( min(v_r_nom, v_Max), v_Min )

        # Adaptive speed compensation
        Delta_v = v_r_nom - v_r

        if abs(self.xi_v) <= self.xi_M:
            self.xi_v += self.control_step_size * (self.bar_eta_1 * Delta_v - self.bar_eta_2 * self.xi_v)
        else:
            self.xi_v += self.control_step_size * (self.bar_eta_1 * Delta_v - self.bar_eta_2 * self.xi_v - (Delta_v) ** 2/self.xi_v )

        tan_M = self.yaw_vel_lim * self.model_parameter[0,4] / max(abs(v_r), self.v_d_min)
        tan_lim = min(tan_M, self.tan_m)

        tan_Max = min( np.tan( math.atan(control_prev[0,1]) + self.ang_vel_lim * self.control_step_size ), tan_lim )
        tan_Min = max( np.tan( math.atan(control_prev[0,1]) - self.ang_vel_lim * self.control_step_size ), - tan_lim )

        u_f_nom = ( np.cos(self.robot_pos[0,2]) * (self.vel_ref[0,1] - self.W_hat_now[0,1])  - np.sin(self.robot_pos[0,2]) * (self.vel_ref[0,0] - self.W_hat_now[0,0]) \
                   - self.k_lat * self.local_error[0,1] ) * self.model_parameter[0,4] / self.model_parameter[0,0] - self.k_u * self.xi_u

        if v_r > 0:
            u_f = max( min(u_f_nom, tan_Max * v_r), tan_Min * v_r)
            tan_sigma_f = u_f/v_r
        elif v_r < 0:
            u_f = max( min(u_f_nom, tan_Min * v_r), tan_Max * v_r)
            tan_sigma_f = - u_f/v_r
        else:
            u_f = v_r * control_prev[0,1]
            tan_sigma_f = control_prev[0,1]
        
        Delta_u = u_f_nom - u_f

        if abs(self.xi_u) <= self.xi_M:
            self.xi_u += self.control_step_size * (self.bar_eta_1 * Delta_u - self.bar_eta_2 * self.xi_u)
        else:
            self.xi_u += self.control_step_size * (self.bar_eta_1 * Delta_u - self.bar_eta_2 * self.xi_u - Delta_u **2 /self.xi_u)

        self.control_nominal = np.matrix([v_r, tan_sigma_f])                                            

        return self.control_nominal
    
    def kinematic_controller_estimator_u(self):
        # Bring back the previous control inputs for dynamic saturation thresholds
        control_prev = self.control_nominal
        v_Max = min(control_prev[0, 0] + self.acc_lim * self.control_step_size, self.v_m)
        v_Min = max(control_prev[0, 0] - self.acc_lim * self.control_step_size, - self.v_m)

        v_r_nom = np.cos(self.robot_pos[0,2]) * (self.vel_ref[0,0] - self.U_hat[0,0]) + np.sin(self.robot_pos[0,2]) * \
            (self.vel_ref[0,1] - self.U_hat[0,1]) - self.k_lon * self.local_error[0,0]

        v_r = max( min(v_r_nom, v_Max), v_Min )

        # Calculate the dynamic lateral saturation threshold
        '''It is essential to consider the yaw angle limitation, the allowed steering angle should decrease as the longitudinal speed increases'''
        tan_M = self.yaw_vel_lim * self.model_parameter[0,4] / max(abs(v_r), self.v_d_min)
        tan_lim = min(tan_M, self.tan_m)

        tan_Max = min( np.tan( math.atan(control_prev[0,1]) + self.ang_vel_lim * self.control_step_size ), tan_lim )
        tan_Min = max( np.tan( math.atan(control_prev[0,1]) - self.ang_vel_lim * self.control_step_size ), - tan_lim )

        tan_nom = ( np.cos(self.robot_pos[0,2]) * (self.vel_ref[0,1] - self.U_hat[0,1]) - np.sin(self.robot_pos[0,2]) * (self.vel_ref[0,0] - self.U_hat[0,0]) \
                   - self.k_lat * self.local_error[0,1] ) * self.model_parameter[0,4]/(abs(v_r) * self.model_parameter[0,0])
        
        tan_sigma_f = max( min(tan_nom, tan_Max), tan_Min)

        self.control_nominal = np.matrix([v_r, tan_sigma_f])

        return self.control_nominal
    
    def singular_issue_justification(self):
        
        e_Phi = np.arctan2(-self.local_error[0,1], -self.local_error[0,0])
        beta = np.arctan( self.control_nominal[0,1] * self.model_parameter[0,0]/self.model_parameter[0,4] )
        v_c = np.sqrt(1 + (self.model_parameter[0,1] * self.control_nominal[0,1]) ** 2/ self.model_parameter[0,4] ** 2 ) * self.control_nominal[0,0]

        if self.F_mode == 1 and v_c >= self.v_d_min:
            e_Phi -= beta 
        else:
            v_c = 0
            beta = 0

        v_d = np.sqrt(self.vel_ref[0,0] ** 2 + self.vel_ref[0,0] ** 2)
        e_Phi = np.arctan2(np.sin(e_Phi), np.cos(e_Phi))

        # if self.F_sing == 0 and abs(e_Phi) >= self.e_Phi_max:
        #     self.F_sing = 1
        # elif self.F_sing == 1 and abs(e_Phi) <= self.e_Phi_min:
        #     self.F_sing = 0

        if (self.F_sing == 0 and abs(math.tan(e_Phi)) >= math.tan(self.e_Phi_max)) or self.control_nominal[0,0] < 0:
            self.F_sing = 1
        elif (self.F_sing == 1 and abs(math.tan(e_Phi)) <= math.tan(self.e_Phi_min)):
            self.F_sing = 0

        if (v_d >= self.v_d_min and v_c >= self.gamma_v_max * v_d) or math.sqrt(self.local_error[0,0]**2 + self.local_error[0,1] ** 2) <= 0.05 or v_c >= 0.3:
            self.F_sing = 0

        return e_Phi
    
    def hybrid_control(self, e_Phi, angle):

        if self.F_sing == 1 and self.F_mode == 1:
            self.F_mode = 2 * np.sign(e_Phi)
            self.F_stage = 1
        elif self.F_sing == 0 and abs(self.F_mode) == 2:
            self.F_mode = self.F_stage = 1

        if self.F_mode == 1:
            if self.F_stage == 1:
                self.reset()
                # self.special_control_with_saturation(np.matrix( [0,0,0,0] ))
                self.control_actual = np.matrix([0,0,0,0])
                self.F_stage = 2
            else:
                self.control_actual = self.nominal_to_actual()
        else:
            self.reset()
            if self.F_stage == 1:
                self.control_actual = np.matrix([0, 0, round(- angle/math.pi * 180, 3), round(angle/math.pi * 180, 3)])
                self.F_stage = 2
                # self.special_control_with_saturation(np.matrix( [0, 0, - angle, angle] ))
            else:
                spin_speed = max( min(0.4 * e_Phi/self.model_parameter[0,3], 0.4), -0.4)
                self.control_actual = np.matrix([-spin_speed, spin_speed, round(- angle/math.pi * 180, 3), round(angle/math.pi * 180, 3)])
                # self.special_control_with_saturation(np.matrix( [-spin_speed, spin_speed, - angle, angle] ))

        return self.control_actual
        

    def special_control_with_saturation(self, desire_input):
        
        # Create a copy of current control for modification
        # control_new = np.copy(self.control_actual.A1)  # Convert matrix to flat array
        for i in range(2):
            error = desire_input[0,i] - self.control_actual[0,i]

            if abs(error) <= self.max_acc_step:
                self.control_actual[0,i] = round(desire_input[0,i], 3)
            else:
                self.control_actual[0,i] = round(self.max_acc_step * np.sign(error) + self.control_actual[0,i], 3)
        
        for i in range(2,4):
            error = desire_input[0,i] - self.control_actual[0,i]

            if abs(error) <= self.max_acc_step:
                self.control_actual[0,i] = round(desire_input[0,i], 3)
            else:
                self.control_actual[0,i] = round(self.max_ang_step/math.pi * 180 * np.sign(error) + self.control_actual[0,i], 3)

        # Check for completion
        if np.array_equal(self.control_actual, desire_input):
            self.F_stage = 2


    
    def nominal_to_actual(self):
        v_r_l = round( (1 - self.model_parameter[0,3] * self.control_nominal[0,1]/ (2 * self.model_parameter[0,4])) * self.control_nominal[0,0] , 3)
        v_r_r = round( (1 + self.model_parameter[0,3] * self.control_nominal[0,1]/ (2 * self.model_parameter[0,4])) * self.control_nominal[0,0] , 3)
        sigma_f_l = round( math.atan(2 * self.control_nominal[0,1] * self.model_parameter[0,4] / \
                                     (2 * self.model_parameter[0,4] - self.model_parameter[0,2] * self.control_nominal[0,1]) )/math.pi * 180, 3)
        sigma_f_r = round( math.atan(2 * self.control_nominal[0,1] * self.model_parameter[0,4] / \
                                     (2 * self.model_parameter[0,4] + self.model_parameter[0,2] * self.control_nominal[0,1]) )/math.pi * 180, 3)
        self.control_actual = np.matrix([v_r_l, v_r_r, sigma_f_l, sigma_f_r])

        return self.control_actual
    
    def send_commend_to_rover(self, client_socket):

        data_to_send = np.array([self.control_actual[0,0], self.control_actual[0,1], self.control_actual[0,2], self.control_actual[0,3], 0.0, 0.0, 0.0, 0.0, 0.0],  dtype = np.float64)
        # Just for test
        # data_to_send = np.array([0.2, 0.4, 14.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype = np.float64)

        # print(data_to_send)
        data_to_send_bytes = data_to_send.tobytes()
        client_socket.sendall(data_to_send_bytes)
        

    
