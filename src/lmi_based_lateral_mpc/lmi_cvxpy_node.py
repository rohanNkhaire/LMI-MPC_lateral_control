#!/usr/bin/env python3

import rospy
import numpy as np
from lmi_based_lateral_mpc.lmi_cvxpy import LMIMPCControl

def main():
   rospy.init_node('lmi_based_lateral_mpc_node')

   mpc = LMIMPCControl()
   
   rate = 1.0 / 0.03
   ros_rate = rospy.Rate(rate)
   while not rospy.is_shutdown():
      mpc.update_controls()
      ros_rate.sleep()

if __name__ == "__main__":
   main()