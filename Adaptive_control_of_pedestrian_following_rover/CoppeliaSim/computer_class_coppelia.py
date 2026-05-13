import numpy as np
import math
import time
import sim
import sys

class computer(object):

    def __init__(self, model_parameter, control_step_size, pos_initial):

        # Basic parameters
        self.clientID = 0
        self.error_code = 0
        self.control_step_size = control_step_size

        # CoppeliaSim handles
        self.main_body_handle = 0
        self.l_f_motor_handle = 0
        self.r_f_motor_handle = 0
        self.l_r_motor_handle = 0
        self.l_r_motor_handle = 0

        # Controller parameters
        self.v_m = 2
        self.tan_m = 1
        self.k_lon = 1
        self.k_lat = 1
        
        self.eta_1 = np.multiply(np.diag([6,6,1]), 1)
        self.eta_2 = np.multiply(np.diag([0.2,0.2,0.1]), 1)
        self.eta_3 = np.multiply(np.diag([0.5,0.5,0.1]), 1)
        self.eta_4 = np.multiply(np.diag([1,1,1]), 20)
        self.eta_5 = np.multiply(np.diag([1,1,1]), 0.5)
        self.p = 11/13
        self.q = 13/11

        self.k_v = 1
        self.k_u = 1
        self.acc_lim = 1
        self.ang_vel_lim = math.pi/3
        self.xi_M = 2
        self.bar_eta_1 = 5
        self.bar_eta_2 = 0.5

        self.xi_u = 0
        self.xi_v = 0

        # For the singular issue algorithms
        self.v_d_min = 0.05
        self.e_Phi_max = math.pi * (70/180)
        self.e_Phi_min = math.pi * (20/180)
        self.gamma_v_max = 0.6
        self.gamma_v_min = 0.2
        self.F_sing = 0
        self.F_mode = 1
        self.F_stage = 2

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
        self.control_nominal = np.matrix([0, 0])
        self.control_actual = np.matrix([0, 0, 0, 0])

        self.pos_ref = np.matrix([0, 0, 0])
        self.vel_ref = np.matrix([0, 0])
#################################################################### Python Coppelia collaboration ########################################################################

    def python_to_coppelia_connection(self):
        ## Step 1: Connection
        print("Program started")
        ## Close all previous connections
        sim.simxFinish(-1)
        self.clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

        if self.clientID != -1:
            print("Connected successfully")
        else:
            sys.exit("Failed to connect.")

    
    def acquire_coppelia_handle(self):
        ## Step 2: Acquire handles
        # Rover body's handle
        self.error_code, self.main_body_handle = sim.simxGetObjectHandle(self.clientID, '/Main_Body', sim.simx_opmode_oneshot_wait)

        # Four motors' handles
        self.error_code, self.r_f_motor_handle = sim.simxGetObjectHandle(self.clientID, '/Main_Body/R_F_Motor', sim.simx_opmode_oneshot_wait)
        self.error_code, self.l_f_motor_handle = sim.simxGetObjectHandle(self.clientID, '/Main_Body/L_F_Motor', sim.simx_opmode_oneshot_wait)
        self.error_code, self.r_r_motor_handle = sim.simxGetObjectHandle(self.clientID, '/Main_Body/Right_Rear_Motor', sim.simx_opmode_oneshot_wait)
        self.error_code, self.l_r_motor_handle = sim.simxGetObjectHandle(self.clientID, '/Main_Body/Left_Rear_Motor', sim.simx_opmode_oneshot_wait)

    def read_pos_from_coppelia(self):
        self.error_code, coppelia_pos_read = sim.simxGetObjectPosition(self.clientID, self.main_body_handle, -1, sim.simx_opmode_oneshot)
        self.error_code, coppelia_ori_read = sim.simxGetObjectOrientation(self.clientID, self.main_body_handle, -1, sim.simx_opmode_oneshot)
        self.robot_pos_coppelia = np.matrix([coppelia_pos_read[0], coppelia_pos_read[1], coppelia_ori_read[2] + math.pi/2])
        
        self.robot_pos_coppelia = self.angle_regulation(self.robot_pos_coppelia)

        return self.robot_pos_coppelia
    
    def stop_simulation(self):
        sim.simxStopSimulation(self.clientID, sim.simx_opmode_oneshot)
        

#################################################################### Python controller ########################################################################

########################### Angle calculation ####################################
    def angle_regulation(self, pos_info):

        # This file regulates the yaw angle within the region of [-pi, pi]
        while abs(pos_info[0,2]) > math.pi:
            pos_info[0,2] -= np.sign(pos_info[0,2]) * 2 * math.pi

        return pos_info

########################### Reference generation ####################################
    def reference_generator(self, current_time, task_flag):

        if task_flag == 1:
            self.vel_ref = np.matrix([-0.2, 0])    
            self.pos_ref = np.matrix([1 - 0.2 * current_time, 0, 0])

        elif task_flag == 2:
            w = math.pi/5
            r = 1.2
            phase = 0

            g_t = np.sign( np.sin(0.5 * w * current_time + phase) )

            # Desired velocity
            self.vel_ref = np.matrix([ w * r * np. sin(w * current_time + phase) * g_t,
                                w * r * np. cos(w * current_time + phase) ])
            
            # Desired trajectory
            self.pos_ref = np.matrix([ r * ( 1 - np.cos(w * current_time + phase) ) * g_t,
                                r * np.sin(w * current_time + phase), math.atan(self.vel_ref[0,1]/self.vel_ref[0,0]) ])
            
        elif task_flag == 3:
            self.vel_ref = np.matrix([0, 0])
            if current_time <= 8:
                self.pos_ref = np.matrix([2,0,0])
            elif 8 < current_time <= 16:
                self.pos_ref = np.matrix([2,3,0])
            elif 16 < current_time <= 24:
                self.pos_ref = np.matrix([-2,3,0])
            else:
                # self.pos_ref = np.matrix([-3,-1,0])
                self.pos_ref = np.matrix([0,0,0])

        else:
            v_x = -0
            v_y = -0
            w_center = 0.2
            r_center = 1
            center_x = 0 + r_center * np.cos(w_center * current_time) + v_x * current_time
            center_y = 0 + r_center * np.sin(w_center * current_time) + v_y * current_time

            # center_x = 0
            # center_y = 2

            w = math.pi/5
            r = 1.2
            phase = 0

            # Desired velocity
            self.vel_ref = np.matrix([ - r * w * np.sin(w * current_time + phase) - w_center * r_center * np.sin(w_center * current_time) + v_x, 
                        r * w * np.cos(w * current_time + phase) + w_center * r_center * np.cos(w_center * current_time) + v_y])

            # Desired trajectory
            self.pos_ref = np.matrix([ center_x + r * np.cos(w * current_time + phase), 
                        center_y + r * np.sin(w * current_time + phase), math.atan(self.vel_ref[0,1]/self.vel_ref[0,0])])

        
        return self.pos_ref, self.vel_ref
    
########################### Open loop calculation of system states ####################################
    def read_pos_psuedo(self):

        W = np.matrix([0,0,0])

        X_dot = self.control_nominal[0,0] * np.cos(self.robot_pos_psuedo[0,2]) - abs(self.control_nominal[0,0]) * self.model_parameter[0,0] / self.model_parameter[0,4] * np.sin(self.robot_pos_psuedo[0,2]) * self.control_nominal[0,1]
        Y_dot = self.control_nominal[0,0] * np.sin(self.robot_pos_psuedo[0,2]) + abs(self.control_nominal[0,0]) * self.model_parameter[0,0] / self.model_parameter[0,4] * np.cos(self.robot_pos_psuedo[0,2]) * self.control_nominal[0,1]
        Phi_dot = self.control_nominal[0,0] / self.model_parameter[0,4] * self.control_nominal[0,1]

        pos_dot = np.matrix([X_dot, Y_dot, Phi_dot]) + W

        self.robot_pos_psuedo = np.multiply(pos_dot, self.control_step_size) + self.robot_pos_psuedo

        self.robot_pos_psuedo = self.angle_regulation(self.robot_pos_psuedo)

        return self.robot_pos_psuedo

########################### Adaptive uncertainty estimation ####################################
    def adaptive_estimator(self):

        # Kinematics model
        X_dot = self.control_nominal[0,0] * np.cos(self.robot_pos_prev[0,2]) - abs(self.control_nominal[0,0]) * self.model_parameter[0,0] / self.model_parameter[0,4] * np.sin(self.robot_pos_prev[0,2]) * self.control_nominal[0,1]
        Y_dot = self.control_nominal[0,0] * np.sin(self.robot_pos_prev[0,2]) + abs(self.control_nominal[0,0]) * self.model_parameter[0,0] / self.model_parameter[0,4] * np.cos(self.robot_pos_prev[0,2]) * self.control_nominal[0,1]
        
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

        return self.U_hat, self.W_hat_now, self.pos_hat_now, self.pos_tilde_now

    def reset(self):
        
        self.U_hat = np.matrix([0,0,0])
        self.W_hat_now = np.matrix([0,0,0])
        self.pos_hat_now = self.robot_pos_coppelia
        self.pos_tilde_now = np.matrix([0,0,0])
        self.xi_u = 0
        self.xi_v = 0
        self.control_nominal = np.matrix([0,0])

        return self.U_hat, self.W_hat_now, self.pos_hat_now, self.pos_tilde_now, self.xi_u, self.xi_v, self.control_nominal
            

########################### Calculate global and local errors ####################################    
    def error_calculation(self):

        # Global error calculation
        X_e = self.robot_pos[0,0] - self.pos_ref[0,0]
        Y_e = self.robot_pos[0,1] - self.pos_ref[0,1]

        # Local error calculation
        x_e = np.cos(self.robot_pos[0,2]) * X_e + np.sin(self.robot_pos[0,2]) * Y_e
        y_e = - np.sin(self.robot_pos[0,2]) * X_e + np.cos(self.robot_pos[0,2]) * Y_e

        self.local_error = np.matrix([x_e, y_e])

        return self.local_error
    
########################### Different controller selections ####################################
    def kinematic_controller_vanila(self):
        # Controller without estimator and compensator

        v_r_nom = np.cos(self.robot_pos[0,2]) * self.vel_ref[0,0] + np.sin(self.robot_pos[0,2]) * self.vel_ref[0,1] - self.k_lon * self.local_error[0,0]
        v_r = max( min(v_r_nom, self.v_m), -self.v_m )

        u_f_nom = ( np.cos(self.robot_pos[0,2]) * self.vel_ref[0,1] - np.sin(self.robot_pos[0,2]) * self.vel_ref[0,0] - self.k_lat * self.local_error[0,1] ) * self.model_parameter[0,4] / self.model_parameter[0,0]
        
        if v_r > 0:
            u_f = max( min(u_f_nom, self.tan_m * v_r), -self.tan_m * v_r)
            tan_sigma_f = u_f/abs(v_r)
        elif v_r < 0:
            u_f = max( min(u_f_nom, -self.tan_m * v_r), self.tan_m * v_r)
            tan_sigma_f = u_f/abs(v_r)
        else:
            tan_sigma_f = u_f_nom

        self.control_nominal = np.matrix([v_r, tan_sigma_f])

        return self.control_nominal
    
    def kinematic_controller_estimator(self):
        # Controller with uncertainty estimator input

        v_r_nom = np.cos(self.robot_pos[0,2]) * (self.vel_ref[0,0] - self.W_hat_now[0,0]) + np.sin(self.robot_pos[0,2]) * (self.vel_ref[0,1] - self.W_hat_now[0,1]) - self.k_lon * self.local_error[0,0]
        v_r = max( min(v_r_nom, self.v_m), -self.v_m )

        u_f_nom = ( np.cos(self.robot_pos[0,2]) * (self.vel_ref[0,1] - self.W_hat_now[0,1])  - np.sin(self.robot_pos[0,2]) * (self.vel_ref[0,0] - self.W_hat_now[0,0]) - self.k_lat * self.local_error[0,1] ) * self.model_parameter[0,4] / self.model_parameter[0,0]

        if v_r > 0:
            u_f = max( min(u_f_nom, self.tan_m * v_r), -self.tan_m * v_r)
            tan_sigma_f = u_f/abs(v_r)
        elif v_r < 0:
            u_f = max( min(u_f_nom, -self.tan_m * v_r), self.tan_m * v_r)
            tan_sigma_f = u_f/abs(v_r)
        else:
            tan_sigma_f = u_f_nom

        self.control_nominal = np.matrix([v_r, tan_sigma_f])

        return self.control_nominal
    
    def kinematic_controller_compensator(self):
        # Controller with input saturation compensator

        control_prev = self.control_nominal
        v_r_nom = np.cos(self.robot_pos[0,2]) * (self.vel_ref[0,0]) + np.sin(self.robot_pos[0,2]) * (self.vel_ref[0,1]) - self.k_lon * self.local_error[0,0] - self.k_v * self.xi_v
        v_Max = min(control_prev[0, 0] + self.acc_lim * self.control_step_size, self.v_m)
        v_Min = max(control_prev[0, 0] - self.acc_lim * self.control_step_size, - self.v_m)
        v_r = max( min(v_r_nom, v_Max), v_Min )

        # Adaptive speed compensation
        Delta_v = v_r_nom - v_r

        if abs(self.xi_v) <= self.xi_M:
            self.xi_v += self.control_step_size * (self.bar_eta_1 * Delta_v - self.bar_eta_2 * self.xi_v)
        else:
            self.xi_v += self.control_step_size * (self.bar_eta_1 * Delta_v - self.bar_eta_2 * self.xi_v - (Delta_v) ** 2/self.xi_v )

        u_f_nom = ( np.cos(self.robot_pos[0,2]) * (self.vel_ref[0,1])  - np.sin(self.robot_pos[0,2]) * (self.vel_ref[0,0]) - self.k_lat * self.local_error[0,1] ) * self.model_parameter[0,4] / self.model_parameter[0,0] - self.k_u * self.xi_u
        tan_Max = min( np.tan( math.atan(control_prev[0,1]) + self.ang_vel_lim * self.control_step_size ), self.tan_m )
        tan_Min = max( np.tan( math.atan(control_prev[0,1]) - self.ang_vel_lim * self.control_step_size ), -self.tan_m )

        if v_r >= 0:
            u_f = max( min(u_f_nom, tan_Max * v_r), tan_Min * v_r)
        else:
            u_f = max( min(u_f_nom, tan_Min * v_r), tan_Max * v_r)
        
        Delta_u = u_f_nom - u_f

        if abs(self.xi_u) <= self.xi_M:
            self.xi_u += self.control_step_size * (self.bar_eta_1 * Delta_u - self.bar_eta_2 * self.xi_u)
        else:
            self.xi_u += self.control_step_size * (self.bar_eta_1 * Delta_u - self.bar_eta_2 * self.xi_u - Delta_u **2 /self.xi_u)

        if v_r == 0:
            tan_sigma_f = tan_Max
        else:
            tan_sigma_f = u_f/abs(v_r)

        self.control_nominal = np.matrix([v_r, tan_sigma_f])

        return self.control_nominal

    def kinematic_controller_compensator_estimator(self):
        # Controller with both compensator and estimator

        control_prev = self.control_nominal
        v_r_nom = np.cos(self.robot_pos[0,2]) * (self.vel_ref[0,0] - self.W_hat_now[0,0]) + np.sin(self.robot_pos[0,2]) * (self.vel_ref[0,1] - self.W_hat_now[0,1]) - self.k_lon * self.local_error[0,0] - self.k_v * self.xi_v
        v_Max = min(control_prev[0, 0] + self.acc_lim * self.control_step_size, self.v_m)
        v_Min = max(control_prev[0, 0] - self.acc_lim * self.control_step_size, - self.v_m)
        v_r = max( min(v_r_nom, v_Max), v_Min )

        # Adaptive speed compensation
        Delta_v = v_r_nom - v_r

        if abs(self.xi_v) <= self.xi_M:
            self.xi_v += self.control_step_size * (self.bar_eta_1 * Delta_v - self.bar_eta_2 * self.xi_v)
        else:
            self.xi_v += self.control_step_size * (self.bar_eta_1 * Delta_v - self.bar_eta_2 * self.xi_v - (Delta_v) ** 2/self.xi_v )

        u_f_nom = ( np.cos(self.robot_pos[0,2]) * (self.vel_ref[0,1] - self.W_hat_now[0,1])  - np.sin(self.robot_pos[0,2]) * (self.vel_ref[0,0] - self.W_hat_now[0,0]) - self.k_lat * self.local_error[0,1] ) * self.model_parameter[0,4] / self.model_parameter[0,0] - self.k_u * self.xi_u
        tan_Max = min( np.tan( math.atan(control_prev[0,1]) + self.ang_vel_lim * self.control_step_size ), self.tan_m )
        tan_Min = max( np.tan( math.atan(control_prev[0,1]) - self.ang_vel_lim * self.control_step_size ), -self.tan_m )

        if v_r >= 0:
            u_f = max( min(u_f_nom, tan_Max * v_r), tan_Min * v_r)
        else:
            u_f = max( min(u_f_nom, tan_Min * v_r), tan_Max * v_r)
        
        Delta_u = u_f_nom - u_f

        if abs(self.xi_u) <= self.xi_M:
            self.xi_u += self.control_step_size * (self.bar_eta_1 * Delta_u - self.bar_eta_2 * self.xi_u)
        else:
            self.xi_u += self.control_step_size * (self.bar_eta_1 * Delta_u - self.bar_eta_2 * self.xi_u - Delta_u **2 /self.xi_u)

        if v_r == 0:
            tan_sigma_f = tan_Max
        else:
            tan_sigma_f = u_f/abs(v_r)

        self.control_nominal = np.matrix([v_r, tan_sigma_f])                                            

        return self.control_nominal
    
    def kinematic_controller_estimator_u(self):
        # Controller with estimator input of combined estimation (U)

        v_r_nom = np.cos(self.robot_pos[0,2]) * (self.vel_ref[0,0] - self.U_hat[0,0]) + np.sin(self.robot_pos[0,2]) * (self.vel_ref[0,1] - self.U_hat[0,1]) - self.k_lon * self.local_error[0,0]
        v_r = max( min(v_r_nom, self.v_m), -self.v_m )

        u_f_nom = ( np.cos(self.robot_pos[0,2]) * (self.vel_ref[0,1] - self.U_hat[0,1])  - np.sin(self.robot_pos[0,2]) * (self.vel_ref[0,0] - self.U_hat[0,0]) - self.k_lat * self.local_error[0,1] ) * self.model_parameter[0,4] / self.model_parameter[0,0]
        
        if v_r > 0:
            u_f = max( min(u_f_nom, self.tan_m * v_r), -self.tan_m * v_r)
            tan_sigma_f = u_f/abs(v_r)
        elif v_r < 0:
            u_f = max( min(u_f_nom, -self.tan_m * v_r), self.tan_m * v_r)
            tan_sigma_f = u_f/abs(v_r)
        else:
            tan_sigma_f = u_f_nom

        self.control_nominal = np.matrix([v_r, tan_sigma_f])

        return self.control_nominal
    
    def singular_issue_justification(self):
        
        e_Phi = np.arctan2(-self.local_error[0,1], -self.local_error[0,0])
        beta = np.arctan( self.control_nominal[0,1] * self.model_parameter[0,0]/self.model_parameter[0,4] )
        v_c = np.sqrt(1 + (self.model_parameter[0,1] * self.control_nominal[0,1]) ** 2/ self.model_parameter[0,4] ** 2 ) * self.control_nominal[0,0]

        if self.F_mode == 1 and v_c >= self.v_d_min:
            if v_c < 0:
                e_Phi -= beta + math.pi
            else:
                e_Phi -= beta 
        else:
            v_c = 0

        v_d = np.sqrt(self.vel_ref[0,0] ** 2 + self.vel_ref[0,0] ** 2)
        e_Phi = np.arctan2(np.sin(e_Phi), np.cos(e_Phi))

        if v_c >= self.v_d_min and v_d >= self.v_d_min:
            if self.F_sing == 0 and v_c <= self.gamma_v_min * v_d:
                self.F_sing = 1
            elif self.F_sing == 1 and v_c >= self.gamma_v_max * v_d:
                self.F_sing = 0
        else:
            if self.F_sing == 0 and abs(np.tan(e_Phi)) >= np.tan(self.e_Phi_max):
                self.F_sing = 1
            elif self.F_sing == 1 and abs(np.tan(e_Phi)) <= np.tan(self.e_Phi_min):
                self.F_sing = 0

        return e_Phi
    
    def hybrid_control_mode_algorithm(self, e_Phi):
        # First, select the control mode and do the control

        if self.F_sing == 1 and self.F_mode == 1:
            if (math.pi/2 <= e_Phi <= math.pi - self.e_Phi_max) or (-math.pi/2 <= e_Phi <= - self.e_Phi_max ):
                self.F_mode = -2
                self.F_stage = 1
            else:
                self.F_mode = 2
                self.F_stage = 1

        elif self.F_sing == 0 and abs(self.F_mode) == 2:
            self.F_mode = self.F_stage = 1

        # if self.F_mode == 1:
        #     _ = self.kinematic_controller_compensator_estimator()
        #     self.control_actual = self.nominal_to_actual()
        # else:
        #     self.control_actual = np.matrix([-np.sign(self.F_mode) * 0.4, np.sign(self.F_mode) * 0.4, -np.arctan2(2 * self.model_parameter[0,4],self.model_parameter[0,2]), np.arctan2(2 * self.model_parameter[0,4],self.model_parameter[0,2])])

        # if self.F_mode == 1:
        #     if self.F_stage == 1:
        #         self.special_control_with_saturation(np.matrix([0, 0, 0, 0]))
        #     else:
        #         _ = self.kinematic_controller_compensator_estimator()
        #         _ = self.nominal_to_actual()
        # else:
        #     if self.F_stage == 1:
        #         self.special_control_with_saturation(np.matrix([0, 0, -np.arctan2(2 * self.model_parameter[0,4],self.model_parameter[0,2]), np.arctan2(2 * self.model_parameter[0,4],self.model_parameter[0,2])]))
        #     else:
        #         self.special_control_with_saturation(np.matrix([0, 0, -np.arctan2(2 * self.model_parameter[0,4],self.model_parameter[0,2]), np.arctan2(2 * self.model_parameter[0,4],self.model_parameter[0,2])]))

        
    def special_control_with_saturation(self, desire_input):
        control_prev = self.control_actual
        self.acc_lim = 1
        self.ang_vel_lim = math.pi/3
        # For the rear wheels 
        for i in range(0,2):
            if control_prev[0,i] != desire_input[0,i]:
                if abs(desire_input[0,i] - control_prev[0,i]) <= self.acc_lim * self.control_step_size:
                    self.control_actual[0,i] = desire_input[0,i]
                else:
                    self.control_actual[0,i] = self.acc_lim * self.control_step_size * np.sign(desire_input[0,i] - control_prev[0,i]) + control_prev[0,i]
            else:
                self.control_actual[0,i] = desire_input[0,i]
        
        # For the front steering wheels
        for i in range(2,4):
            if control_prev[0,i] != desire_input[0,i]:
                if abs(desire_input[0,i] - control_prev[0,i]) <= self.ang_vel_lim * self.control_step_size:
                    self.control_actual[0,i] = desire_input[0,i]
                else:
                    self.control_actual[0,i] = self.ang_vel_lim * self.control_step_size * np.sign(desire_input[0,i] - control_prev[0,i]) + control_prev[0,i]
            else:
                self.control_actual[0,i] = desire_input[0,i]
        
        if np.array_equal(self.control_actual, desire_input):
            self.F_stage = 2
    
    def nominal_to_actual(self):
        v_r_l = (1 - self.model_parameter[0,3] * self.control_nominal[0,1]/ (2 * self.model_parameter[0,4])) * self.control_nominal[0,0] 
        v_r_r = (1 + self.model_parameter[0,3] * self.control_nominal[0,1]/ (2 * self.model_parameter[0,4])) * self.control_nominal[0,0] 
        sigma_f_l = math.atan(2 * self.control_nominal[0,1] * self.model_parameter[0,4] / (2 * self.model_parameter[0,4] - self.model_parameter[0,2] * self.control_nominal[0,1]) ) 
        sigma_f_r = math.atan(2 * self.control_nominal[0,1] * self.model_parameter[0,4] / (2 * self.model_parameter[0,4] + self.model_parameter[0,2] * self.control_nominal[0,1]) ) 
        self.control_actual = np.matrix([v_r_l, v_r_r, sigma_f_l, sigma_f_r])

        return self.control_actual
    
    def send_control_command_to_coppelia(self):

        v_r_l = self.control_actual[0,0]
        v_r_r = self.control_actual[0,1]
        sigma_f_l = self.control_actual[0,2]
        sigma_f_r = self.control_actual[0,3]

        ## If the control input contains fault
        # self.error_code = sim.simxSetJointTargetVelocity(self.clientID, self.l_r_motor_handle, v_r_l * 10 * 0.9, sim.simx_opmode_oneshot)
        # self.error_code = sim.simxSetJointTargetVelocity(self.clientID, self.r_r_motor_handle, v_r_r * 10 * 0.88, sim.simx_opmode_oneshot)
        # self.error_code = sim.simxSetJointTargetPosition(self.clientID, self.l_f_motor_handle, sigma_f_l * 0.95, sim.simx_opmode_oneshot)
        # self.error_code = sim.simxSetJointTargetPosition(self.clientID, self.r_f_motor_handle, sigma_f_r * 0.96, sim.simx_opmode_oneshot)

        ## If the control input is ideal
        self.error_code = sim.simxSetJointTargetVelocity(self.clientID, self.l_r_motor_handle, v_r_l * 10, sim.simx_opmode_oneshot)
        self.error_code = sim.simxSetJointTargetVelocity(self.clientID, self.r_r_motor_handle, v_r_r * 10, sim.simx_opmode_oneshot)
        self.error_code = sim.simxSetJointTargetPosition(self.clientID, self.l_f_motor_handle, sigma_f_l, sim.simx_opmode_oneshot)
        self.error_code = sim.simxSetJointTargetPosition(self.clientID, self.r_f_motor_handle, sigma_f_r, sim.simx_opmode_oneshot)

    
    # def send_control_commend_to_coppelia_hybrid(self):


    #     # self.error_code = sim.simxSetJointTargetVelocity(self.clientID, self.l_r_motor_handle, v_r_l * 10 * 0.9, sim.simx_opmode_oneshot)
    #     # self.error_code = sim.simxSetJointTargetVelocity(self.clientID, self.r_r_motor_handle, v_r_r * 10 * 0.88, sim.simx_opmode_oneshot)
    #     # self.error_code = sim.simxSetJointTargetPosition(self.clientID, self.l_f_motor_handle, sigma_f_l * 0.95, sim.simx_opmode_oneshot)
    #     # self.error_code = sim.simxSetJointTargetPosition(self.clientID, self.r_f_motor_handle, sigma_f_r * 0.96, sim.simx_opmode_oneshot)

    #     self.error_code = sim.simxSetJointTargetVelocity(self.clientID, self.l_r_motor_handle, v_r_l * 10, sim.simx_opmode_oneshot)
    #     self.error_code = sim.simxSetJointTargetVelocity(self.clientID, self.r_r_motor_handle, v_r_r * 10, sim.simx_opmode_oneshot)
    #     self.error_code = sim.simxSetJointTargetPosition(self.clientID, self.l_f_motor_handle, sigma_f_l, sim.simx_opmode_oneshot)
    #     self.error_code = sim.simxSetJointTargetPosition(self.clientID, self.r_f_motor_handle, sigma_f_r, sim.simx_opmode_oneshot)

    #     return np.matrix([v_r_l, v_r_r, sigma_f_l, sigma_f_r])
    
    # def mode_selection(self,mode_flag):
    #     # Hybrid control mode
    #     if abs(self.vel_ref[0,0]) + abs(self.vel_ref[0,1]) <= 0.05:
    #         # If the reference is a fixed point, calculate the angular difference
    #         error_vec = np.array([self.pos_ref[0,0] - self.robot_pos_coppelia[0,0], self.pos_ref[0,1] - self.robot_pos_coppelia[0,1]])
    #         orien_vec = np.array([np.cos(self.robot_pos_coppelia[0,2]), np.sin(self.robot_pos_coppelia[0,2])])

    #         # angle = np.arccos( np.dot(error_vec, orien_vec)/ (np.linalg.norm(error_vec) * np.linalg.norm(orien_vec)) )
    #         angle = np.arctan2(error_vec[0,1], error_vec[0,0]) - np.arctan2(orien_vec[0,1], orien_vec[0,0])
    #         angle = np.mod(angle + np.pi, 2 * np.pi) - np.pi
            
    #         if mode_flag == 1 and abs(np.tan(angle)) >= math.tan(math.pi/3) and math.sqrt(self.local_error[0,0] ** 2 + self.local_error[0,1] ** 2) >= 0.1:
    #             if (angle > math.pi/2 and angle < math.pi * 2 /3) or (angle > - math.pi/2 and angle < - math.pi/3):
    #                 mode_flag = -2
    #             else:
    #                 mode_flag = 2

    #         elif abs(mode_flag) == 2 and abs(np.tan(angle)) <= math.tan(math.pi/18):
    #             mode_flag = 1

    #     else:
    #         # If the reference is dynamical
    #         v_desire = math.sqrt(self.vel_ref[0,0] ** 2 + self.vel_ref[0,1] ** 2)
    #         v_nom = np.cos(self.robot_pos_coppelia[0,2]) * self.vel_ref[0,0] + np.sin(self.robot_pos_coppelia[0,2]) * self.vel_ref[0,1] - self.k_lon * self.local_error[0,0]

    #         if mode_flag == 1 and abs(v_nom)/v_desire <= 0.25:
    #             mode_flag = 2 
    #         elif abs(mode_flag) == 2 and abs(v_nom)/v_desire >= 0.7:
    #             mode_flag = 1
        
    #     return mode_flag





        








        







    




    










