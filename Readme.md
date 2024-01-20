# LMI based lateral control #
This package is used to solve lateral control of an autonomous vehicle using Linear Matrix Inequalities(LMIs) using CVXPY and CVXOPT package in ROS.

The LMIs for MPC are taken from [Kothare](https://www.sciencedirect.com/science/article/abs/pii/0005109896000635). These LMIs are solved using [pylmi](https://pypi.org/project/PyLMI-SDP/) and cvxopt, and cvxpy software.

The problem is also tested in YALMIP using SEDUMI solver.

# Solvers
CPU - 12th Gen Intel® Core™ i7-12700H

RAM - 16 GB

| Framework | Solver       | Approx. Time in sec.        |
|    :---:     | :----------      | :------------       |
|  CVXPYY        | MOSEK | 0.021 |
|            | SCS| 0.83   |
|            | CVXOPT | 0.037   |
|  CVXOPT        | DSDP | 0.0085 |
|            | CVXOPT | 0.019   |

# Dependency
- [pyLMI-SDP](https://pypi.org/project/PyLMI-SDP/) - framework for formulating LMIs.
- [CVXOPT](http://cvxopt.org/)
- [CVXPY](https://www.cvxpy.org/)

# Note
- This package requires data input as a ROStopic. The data consist of the state variables - lateral and yaw error, reference trajectory(given by planner), current velocity and steerking angle. All this data is outputed by mpc_follower package from autoware.ai.

# References
- Mayuresh V. Kothare "Robust constrained model predictive control using linear matrix inequalities".
