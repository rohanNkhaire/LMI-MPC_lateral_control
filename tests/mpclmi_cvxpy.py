#!/usr/bin/env python3

import rospy
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from math import cos, atan
import numpy as np
import cvxpy as cp
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from autoware_msgs.msg import ControlCommandStamped

class LMIMPCControl(CompatibleNode):


    def __init__(self):

        super(LMIMPCControl, self).__init__("lmi_mpc_control")

        # Initializing vehicle paramters
        self.wheelbase_ = 3.22

        # Time step
        self.dt = 0.033
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
        self.Q = np.eye(2)*5
        self.R = np.eye(1)*2

        # input constraint
        self.u_max = 35.0

        # ackermann drive commands
        self.state_subscriber = self.new_subscription(
            Float64MultiArray,
            "/mpc_follower/debug/debug_values",
            self.traj_tracking_states,
            qos_profile=10
        )

        self.twist_publisher = self.new_publisher(
            TwistStamped,
            "twist_raw",
            qos_profile=1)

        self.control_command_publisher = self.new_publisher(
            ControlCommandStamped,
            "ctrl_raw",
            qos_profile=1)    
    
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
        print(self.x)
        self.curr_vel = data.data[10]
        self.cmd_vel = data.data[9]
        self.curvature_ = data.data[14] 
        self.delta_r = atan(self.wheelbase_ * self.curvature_)

        # weight matrices
        Q = np.eye(2)*50
        R = np.eye(1)
        wheelbase_ = self.wheelbase_
        v_k = self.curr_vel
        delta_r = self.delta_r
        dt = 0.03
        x = self.x
        # LMI initialization
        G = cp.Variable((2,2), symmetric=True)
        Y = cp.Variable((1,2))
        gamma = cp.Variable()
        # Kinematic bicycle model
        A_k = np.eye(2) + np.matrix([[0, v_k], [0, 0]])*dt
        B_k = np.matrix([[0], [v_k/wheelbase_*cos(delta_r)*cos(delta_r)]])*dt
        W_k =  np.matrix([[0, 0], [-v_k/delta_r*wheelbase_*cos(delta_r)*cos(delta_r), -v_k/delta_r*wheelbase_*cos(delta_r)*cos(delta_r)]])*dt
        A_k = A_k + W_k
        # Model initialization - choose the required model - kinematic or dynamic.
        A = A_k
        B = B_k
        # LMI_1
        X_11 = cp.hstack([np.eye(1), x.T])
        X_12 = cp.hstack([x,G])
        X_1 = cp.vstack([X_11,X_12])
        # input constraint
        u_max = 35.0
        # LMI_2
        X_21 = cp.hstack([G, (A@G+B@Y).T, G@(Q**0.5), Y.T@(R**0.5)])
        X_22 = cp.hstack([A@G+B@Y, G, np.zeros((2,2)), np.zeros((2,1))])
        X_23 = cp.hstack([(Q**0.5)@G, np.zeros((2,2)), cp.multiply(gamma,np.eye(2)), np.zeros((2,1))])
        X_24 = cp.hstack([(R**0.5)@Y, np.zeros((1,2)), np.zeros((1,2)), cp.multiply(gamma,np.eye(1))])
        X_2 = cp.vstack([X_21,X_22,X_23,X_24])
        # LMI_3
        X_31 = cp.hstack([np.eye(1)*u_max**2, Y])
        X_32 = cp.hstack([Y.T, G])
        X_3 = cp.vstack([X_31,X_32])
        # setting constraints
        constraints = [X_1 >> 0]
        constraints += [X_2 >> 0]
        constraints += [X_3 >> 0] 
        # Initializing problem and solving
        prob = cp.Problem(cp.Minimize(gamma),
                  constraints)
        prob.solve(solver=cp.MOSEK, verbose=False, warm_start=False)
        if prob.status not in ["infeasible", "unbounded", "unknown"]:
            # Calculating optimal input - u
            print("DONE!")
            G_val = self.G.value
            Y_val = self.Y.value
            K = np.matmul(Y_val, np.linalg.inv(G_val))
            u = K*self.x
            omega_cmd = self.curr_vel * atan(u[0]) / self.wheelbase_
            # publishing twist and control cmd
            publishTwist(self.cmd_vel, omega_cmd)
            publishControlCommand(self.cmd_vel, u[0])
        else:
            print("MPC solver: %s" %(prob.status))
        print("solve time:", prob.solver_stats.solve_time)


    def update_controls(self): 

        def loop(timer_event=None):
            
            if (self.received_inputs == False):
                # weight matrices
                Q = np.eye(2)*50
                R = np.eye(1)
                wheelbase_ = self.wheelbase_
                v_k = self.curr_vel
                delta_r = self.delta_r
                dt = 0.03
                x = self.x
                # LMI initialization
                G = cp.Variable((2,2), symmetric=True)
                Y = cp.Variable((1,2))
                gamma = cp.Variable()

                # Kinematic bicycle model
                A_k = np.eye(2) + np.matrix([[0, v_k], [0, 0]])*dt
                B_k = np.matrix([[0], [v_k/wheelbase_*cos(delta_r)*cos(delta_r)]])*dt
                W_k =  np.matrix([[0, 0], [-v_k/delta_r*wheelbase_*cos(delta_r)*cos(delta_r), -v_k/delta_r*wheelbase_*cos(delta_r)*cos(delta_r)]])*dt

                A_k = A_k + W_k
                # Model initialization - choose the required model - kinematic or dynamic.
                A = A_k
                B = B_k

                # LMI_1
                X_11 = cp.hstack([np.eye(1), x.T])
                X_12 = cp.hstack([x,G])
                X_1 = cp.vstack([X_11,X_12])

                # input constraint
                u_max = 35.0

                # LMI_2
                X_21 = cp.hstack([G, (A@G+B@Y).T, G@(Q**0.5), Y.T@(R**0.5)])
                X_22 = cp.hstack([A@G+B@Y, G, np.zeros((2,2)), np.zeros((2,1))])
                X_23 = cp.hstack([(Q**0.5)@G, np.zeros((2,2)), cp.multiply(gamma,np.eye(2)), np.zeros((2,1))])
                X_24 = cp.hstack([(R**0.5)@Y, np.zeros((1,2)), np.zeros((1,2)), cp.multiply(gamma,np.eye(1))])
                X_2 = cp.vstack([X_21,X_22,X_23,X_24])

                # LMI_3
                X_31 = cp.hstack([np.eye(1)*u_max**2, Y])
                X_32 = cp.hstack([Y.T, G])
                X_3 = cp.vstack([X_31,X_32])
    
                # setting constraints
                constraints = [X_1 >> 0]
                constraints += [X_2 >> 0]
                constraints += [X_3 >> 0] 

                # Initializing problem and solving
                prob = cp.Problem(cp.Minimize(gamma),
                          constraints)
                prob.solve(solver=cp.MOSEK, verbose=True, warm_start=False)
                if prob.status not in ["infeasible", "unbounded", "unknown"]:
                    # Calculating optimal input - u
                    G_val = self.G.value
                    Y_val = self.Y.value
                    K = np.matmul(Y_val, np.linalg.inv(G_val))
                    u = K*self.x
                else:
                    print("MPC solver: %s" %(prob.status))
                print("solve time:", prob.solver_stats.solve_time)

                omega_cmd = self.curr_vel * atan(u[0]) / self.wheelbase_
                # publishing twist and control cmd
                publishTwist(self.cmd_vel, omega_cmd)
                publishControlCommand(self.cmd_vel, u[0])
        
        self.new_timer(0.033, loop)
        self.spin()

    def publishTwist(self, cmd_vel, omega_cmd):
        
        #convert steering to twist */
        twist = TwistStamped
        twist.header.frame_id = "/base_link"
        twist.header.stamp = rospy.time.now()
        twist.twist.linear.x = cmd_vel
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = omega_cmd
        self.twist_publisher.publish(twist)

    def publishControlCommand(self, cmd_vel, cntrl_ip):
        cmd = ControlCommandStamped
        cmd.header.frame_id = "/base_link"
        cmd.header.stamp = rospy.time.now()
        cmd.cmd.linear_velocity = cmd_vel
        cmd.cmd.linear_acceleration = 0
        cmd.cmd.steering_angle = cntrl_ip
        self.control_command_publisher.publish(cmd)     

def main(args=None):
    roscomp.init("lmi_mpc_control", args=args)

    controller = LMIMPCControl()
    controller.update_controls()

if __name__ == '__main__':
    main()        