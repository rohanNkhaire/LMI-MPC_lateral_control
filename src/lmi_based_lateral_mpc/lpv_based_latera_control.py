#!/usr/bin/env python3

import rospy
from math import cos, atan, sin
import numpy as np
from cvxopt import solvers
from sympy import symbols, Matrix, eye, zeros, BlockMatrix
from lmi_sdp import LMI_NSD, LMI_PSD, to_cvxopt, LMI_PD
from scipy.linalg import fractional_matrix_power
from std_msgs.msg import Float64MultiArray, Header
from geometry_msgs.msg import TwistStamped
from autoware_msgs.msg import ControlCommandStamped

class LMIMPCControl():

    def __init__(self):

        # Initializing vehicle paramters
        self.wheelbase_ = 3.22

        # Time step
        self.dt = 0.1

        self.formulate_vehicle_model = False
        self.formulate_const_LMIs = False
        self.received_inputs = False

        # Set weight matrix
        #self.Q = np.eye(2)*50
        self.Q = np.matrix([[12.5, 0],[0, 9.25]])
        self.R = np.eye(1)*3

        # input constraint
        self.u_max = 35.0

        # data subscriber
        self.state_subscriber = rospy.Subscriber("/mpc_follower/debug/debug_values", Float64MultiArray, self.traj_tracking_states, queue_size=1)
        # twist publisher
        self.twist_publisher = rospy.Publisher("twist_raw", TwistStamped, queue_size=1)
        # command publisher
        self.control_command_publisher = rospy.Publisher("ctrl_raw", ControlCommandStamped, queue_size=1)    
    
    def publishTwist(self, cmd_vel, omega_cmd):
        
        #convert steering to twist */
        twist = TwistStamped()
        twist.header = Header()
        twist.header.frame_id = "/base_link"
        twist.header.stamp = rospy.Time.now()
        twist.twist.linear.x = cmd_vel
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = omega_cmd
        self.twist_publisher.publish(twist)

    def publishControlCommand(self, cmd_vel, cntrl_ip):
        cmd = ControlCommandStamped()
        cmd.header.frame_id = "/base_link"
        cmd.header.stamp = rospy.Time.now()
        cmd.cmd.linear_velocity = cmd_vel
        cmd.cmd.linear_acceleration = 0
        cmd.cmd.steering_angle = cntrl_ip
        self.control_command_publisher.publish(cmd)     

    def traj_tracking_states(self, data):
        self.received_inputs = True
        # set init state
        self.x = Matrix([data.data[5], data.data[8]])
        self.curr_vel = max(data.data[10], 0.1)
        self.cmd_vel = data.data[9]
        self.curvature_ = data.data[14]
        if np.abs(self.curvature_) < 0.001:
            self.curvature_ = 0.5
        

    def update_controls(self): 
            
        if (self.received_inputs == True):
            # LMI initialization
            self.variables = symbols('G1 G2 Y1 Y2 gamma')
            self.G1, self.G2, self.Y1, self.Y2, self.gamma = self.variables

            self.G = Matrix([[self.G1, self.G2], [self.G2, self.G1]])
            self.Y = Matrix([[self.Y1, self.Y2]])

            # objective function
            self.min_obj = self.gamma
            if (self.formulate_vehicle_model == False):
                self.delta_r = atan(self.wheelbase_ * self.curvature_)
                # Kinematic Bicycle model
                A_k = eye(2) + Matrix([[0, self.curr_vel*(sin(self.x[1])/self.x[1])], [0, 0]])*self.dt
                B_k = Matrix([0, -1])*self.dt
                B_k_r = Matrix([0, -1])*self.dt
    
                A_k = A_k
                # set state propagation matrices
                self.A = A_k
                self.B = B_k
                self.formulate_vehicle_model = True

            if (self.formulate_const_LMIs == False):
                # initializing constant LMIs
                # LMI_2
                M2 = Matrix.vstack(
                Matrix.hstack(self.G, (self.A*self.G+self.B*self.Y).T, self.G*(fractional_matrix_power(self.Q,0.5)), self.Y.T*(fractional_matrix_power(self.R,0.5))),
                Matrix.hstack(self.A*self.G+self.B*self.Y, self.G, zeros(2,2), zeros(2,1)),
                Matrix.hstack((fractional_matrix_power(self.Q,0.5))*self.G, zeros(2,2), self.gamma*eye(2), zeros(2,1)),
                Matrix.hstack((fractional_matrix_power(self.R,0.5))*self.Y, zeros(1,2), zeros(1,2), self.gamma*eye(1)))

                self.X_G = LMI_PD(self.G)
                self.X_2 = LMI_PSD(M2)

                M3 = Matrix.vstack(
                    Matrix.hstack(eye(1)*self.u_max**2, self.Y),
                    Matrix.hstack(self.Y.T, self.G))

                self.X_3 = LMI_PSD(M3)

                self.formulate_const_LMIs = True  
                 
            # LMI_1
            M = Matrix.vstack(
            Matrix.hstack(eye(1), self.x.T),
            Matrix.hstack(self.x, self.G))

            X_1 = LMI_PSD(M)
            # Initializing problem and solving
            solvers.options['show_progress'] = False
            c, Gs, hs = to_cvxopt(self.gamma, [X_1, self.X_2, self.X_3, self.X_G], self.variables)
            sol = solvers.sdp(c, Gs=Gs, hs=hs, solver='dsdp')
            print(self.curvature_)
            print(sol['status'])
            if sol['status'] not in ["primal infeasible", "unbounded", "unknown"]:
                print(self.x)
                # Calculating optimal input - u
                G1 = sol['x'][0]
                G2 = sol['x'][1]
                Y1 = sol['x'][2]
                Y2 = sol['x'][3]

                G_val = np.matrix([[G1, G2],[G2, G1]])   
                Y_val = np.matrix([Y1, Y2])
                K = np.matmul(Y_val, np.linalg.inv(G_val))
                u = np.matmul(K,self.x)
                u_val = u[0,0]

                omega_cmd = self.curr_vel * atan(u_val) / self.wheelbase_
                # publishing twist and control cmd
                self.publishTwist(self.cmd_vel, omega_cmd)
                self.publishControlCommand(self.cmd_vel, u[0])
            
        

