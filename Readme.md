# LMI based lateral control #

![](media/LMI_MPC_Kin_cvxpy_Mosek.gif)

## Objective ##
This repo solves the lateral control problem of an autonomous vehicle using Linear Matrix Inequalities(LMIs) with CVXPY and CVXOPT package in ROS.

## Dependencies ##
- [pyLMI-SDP](https://pypi.org/project/PyLMI-SDP/) - framework for formulating LMIs.
- [CVXOPT](http://cvxopt.org/)
- [CVXPY](https://www.cvxpy.org/)

## Usage ##
```bash
# git clone the repo
git clone https://github.com/rohanNkhaire/LMI-MPC_lateral_control.git

# build the package
cd LMI-MPC_lateral_control
catkin build
```

## Run an example ##
```bash
# I dont have the ROSbag for the input currently.
# The required input comes from the mpc_follower package from autoware.ai planning stack

# Launch the files
# Use CVXPY framework
ros launch lmi_based_lateral_mpc lmi_based_mpc.launch

# Use CVXOPT framework
ros launch lmi_based_lateral_mpc lmi_based_mpc_cvxopt.launch
```

### System Specifications ###
- CPU - 12th Gen Intel® Core™ i7-12700H
- RAM - 16 GB

## Solvers ##

| Framework | Solver       | Approx. Time in sec.        | License | Status |
|    :---:     | :----------      | :------------       | :-------| :--------|
|  CVXPYY        | MOSEK | 0.021 | Commercial (student trial) | Converged |
|            | SCS| 0.83   | Open Source | Slow (not real-time) |
|            | CVXOPT | 0.037   | Open Source | Converged |
|  CVXOPT        | DSDP | 0.0085 | Open Source | Not converged |
|            | CVXOPT | 0.019   | Open Source | Converged |

## Note ##
This package requires data input as a ROStopic. The data consist of the state variables - lateral and yaw error, reference trajectory(given by planner), current velocity and steerking angle. All this data is outputed by [mpc_follower](https://github.com/autowarefoundation/autoware_ai_planning/tree/master/mpc_follower) package from autoware.ai.

This repo is tested on **ROS Noetic**.

The CVXOPT framework requires *pyLMI-SDP* to initialize LMIs.

The CVXPY framework has the capability to initialize LMIs.

The LMIs for MPC are taken from [Kothare](https://www.sciencedirect.com/science/article/abs/pii/0005109896000635).

## References ##
- Mayuresh V. Kothare "Robust constrained model predictive control using linear matrix inequalities".
