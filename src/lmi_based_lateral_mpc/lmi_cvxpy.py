#!/usr/bin/env python3

import rospy
from math import cos, atan
import numpy as np
import cvxpy as cp
import time
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
        # Initializing Variables
        self.G = cp.Variable((2,2), symmetric=True)
        self.Y = cp.Variable((1,2))
        self.gamma = cp.Variable()

        # Initializing objective function
        self.obj = self.gamma

        self.formulate_vehicle_model = False
        self.formulate_const_LMIs = False
        self.received_inputs = False

        # Set weight matrix
        #self.Q = np.eye(2)*50
        self.Q = np.matrix([[250, 0],[0, 125]])
        self.R = np.eye(1)*175

        # input constraint
        self.u_max = 0.61

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
        """
        Stores the ackermann drive message for the next controller calculation

        :param ros_ackermann_drive: the current ackermann control input
        :type ros_ackermann_drive: ackermann_msgs.AckermannDrive
        :return:
        """
        self.received_inputs = True
        # set init state
        self.x = np.matrix([[data.data[5]], [data.data[8]]])
        self.curr_vel = max(data.data[10], 0.1)
        self.cmd_vel = max(data.data[9], 0.1)
        self.cmd_vel_new = max(data.data[18], 0.1)
        self.steer_lookup = data.data[19]
        self.curvature_ = data.data[14]

    def update_controls(self): 
            
        if (self.received_inputs == True):
            if (self.formulate_vehicle_model == False):
                self.delta_r = atan(self.wheelbase_ * self.curvature_)
                #self.delta_r = 0.1
                # Kinematic Bicycle model
                A_k = np.eye(2) + np.matrix([[0, self.cmd_vel], [0, 0]])*self.dt
                B_k = np.matrix([[0], [self.cmd_vel/(self.wheelbase_*cos(self.delta_r)*cos(self.delta_r))]])*self.dt
                W_k =  np.matrix([[0, 0], [0, -self.cmd_vel*self.delta_r/(self.wheelbase_*cos(self.delta_r)*cos(self.delta_r))]])*self.dt
    
                A_k = A_k+ W_k 
                # set state propagation matrices
                self.A = A_k
                self.B = B_k 
                #self.formulate_vehicle_model = True

            if (self.formulate_const_LMIs == False):
                # initializing constant LMIs
                # LMI_2
                X_21 = cp.hstack([self.G, (self.A@self.G+self.B@self.Y).T, self.G@(fractional_matrix_power(self.Q,0.5)), self.Y.T@(fractional_matrix_power(self.R,0.5))])
                X_22 = cp.hstack([self.A@self.G+self.B@self.Y, self.G, np.zeros((2,2)), np.zeros((2,1))])
                X_23 = cp.hstack([(fractional_matrix_power(self.Q,0.5))@self.G, np.zeros((2,2)), cp.multiply(self.gamma,np.eye(2)), np.zeros((2,1))])
                X_24 = cp.hstack([(fractional_matrix_power(self.R,0.5))@self.Y, np.zeros((1,2)), np.zeros((1,2)), cp.multiply(self.gamma,np.eye(1))])
                X_2 = cp.vstack([X_21,X_22,X_23,X_24])

                # LMI_3
                X_31 = cp.hstack([np.eye(1)*self.u_max**2, self.Y])
                X_32 = cp.hstack([self.Y.T, self.G])
                X_3 = cp.vstack([X_31,X_32])

                self.constraints = [X_2 >> 0]
                self.constraints += [X_3 >> 0] 

                #self.formulate_const_LMIs = True  
                 
            # LMI_1
            X_11 = cp.hstack([np.eye(1), self.x.T])
            X_12 = cp.hstack([self.x,self.G])

            X_1 = cp.vstack([X_11,X_12])
            # Constraints 
            self.constraints += [X_1 >> 0]
            # Initializing problem and solving
            #print(self.x)

            prob = cp.Problem(cp.Minimize(self.gamma),
                      self.constraints)
            start = time.time()
            prob.solve(solver=cp.CVXOPT, verbose=False, warm_start=True)
            end = time.time()
            print(end - start) 
            if prob.status not in ["infeasible", "unbounded"]:
                # Calculating optimal input - u
                G_val = self.G.value
                Y_val = self.Y.value
                K = np.matmul(Y_val, np.linalg.inv(G_val))
                u = np.matmul(K,self.x)
                omega_cmd = self.curr_vel * atan(u[0]) / self.wheelbase_
                # publishing twist and control cmd
                self.publishTwist(self.cmd_vel, omega_cmd)
                self.publishControlCommand(self.cmd_vel, u[0])
            else:
                print("MPC solver: %s" %(prob.status))
            print("solve time:", prob.solver_stats.solve_time)
        

