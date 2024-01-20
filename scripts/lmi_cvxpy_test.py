#!/usr/bin/env python3

from math import cos
import numpy as np
import cvxpy as cp

def mpc_lmi():
    # Parameters
    N = 50
    dt = 0.02
    v_k = 5.0
    delta_r = 0.5
    mass_ = 683
    lf_ = 0.758
    lr_ = 1.036
    wheelbase_ = lf_ + lr_
    cf_ = 24000
    cr_ = 21000
    iz_ = 560.94
    curvature = lr_ * mass_ / (2 * cf_ * wheelbase_) - lf_ * mass_ / (2 * cr_ * wheelbase_)

    # Kinematic bicycle model
    A_k = np.eye(2) + np.matrix([[0, v_k], [0, 0]])*dt
    B_k = np.matrix([[0], [v_k/wheelbase_*cos(delta_r)*cos(delta_r)]])*dt
    W_k =  np.matrix([[0, 0], [-v_k/delta_r*wheelbase_*cos(delta_r)*cos(delta_r), -v_k/delta_r*wheelbase_*cos(delta_r)*cos(delta_r)]])*dt

    A_k = A_k + W_k
    # Model initialization - choose the required model - kinematic or dynamic.
    A = A_k
    B = B_k

    # weight matrices
    Q = np.eye(2)*50
    R = np.eye(1)

    # init state
    x = np.matrix([[2e-4], [1e-1]])

    # LMI initialization
    G = cp.Variable((2,2), symmetric=True)
    Y = cp.Variable((1,2))
    gamma = cp.Variable()

    # objective function
    min_obj = gamma

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

    constraints = [X_2 >> 0]
    constraints += [X_3 >> 0]
    print(cp.installed_solvers())
    for k in range(N-1):
        # LMI_1
        X_11 = cp.hstack([np.eye(1), x.T])
        X_12 = cp.hstack([x,G])
        X_1 = cp.vstack([X_11,X_12])
    
        constraints += [X_1 >> 0]
        
        prob = cp.Problem(cp.Minimize(gamma),
                  constraints)
        prob.solve(solver=cp.MOSEK, verbose=False, warm_start=True)
        print("solve time:", prob.solver_stats.solve_time)
        
        G_val = G.value
        Y_val = Y.value

        K = np.matmul(Y_val, np.linalg.inv(G_val))
        u = K*x

        x = A*x + B*u

if __name__ == '__main__':
    try:
        mpc_lmi()
    except rospy.ROSInterruptException:
        pass    



