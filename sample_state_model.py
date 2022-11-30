# Author:      Diego Andrade (bets636@gmail.com)
# Data:        11/30/2022
# Description:
# An example of a state space model for a simple differential drive robot with
# with the following properties:
# - The robot operates in x-y plane of global frame and has a yaw with respect
#   to global frame, in the direction of Xglobal counterclockwise to Yglobal.
# - The robot has a body frame where forward is the x-axis and z-axis aligned
#   to global frame.
# - The robot takes control inputs V and omega with repsect to robot body frame.
#   V is in the direction of Xrobot, and omega is the counterclockwise angle
#   from Xrobot to Yrobot.
# - The robot does not move if no control input given (no coasting)

# The state space model is a mathematical equation that helps you estimate the
# state of a system at time t given the state of a system at time t-1. It is
# given as follows:
#   Xt = At-1*Xt-1 + Bt-1 * Ut-1 + NOISEt-1
#
# Xt is an n x 1 matrix representing the robot state in the global frame at 
# time t. Thus Xt is called the state vector. In this example, the robots state
# is defined with 3 variables, so Xt is a 3 x 1 matrix defined as follows:
#   Xt = [ x_t, y_t, yaw_t ], where is x and y are measure in meters and yaw in
#                             rads.
#
# Xt-1 is the same as Xt but at time t-1
#
# A is an n x n matrix which expresses how the state vector changes from t-1 to
# t when no control command is executed. In this example, A would be of size
# 3 x 3. Additionally, A would be the identity matrix since the robot stays
# still when no input given.
#
# B is an n x m matrix which expresses how the state vector changes from t-1 to
# t due to control inputs, where m is the number of control inputs. In this
# example, B would be of size 3 x 2 for this example.
#
# Ut-1 is an m x 1 matrix representing the control input. In this case m = 2
# and defined as follows:
#   Ut-1 = [ vt-1, wt-1], where v is measured in meters per second and w in
#                         rads per second.
#
# NOISEt-1 is an n x 1 matrix representing the noise of the system, one noise
# term per state variable. In this example, NOISEt-1 would be of size 3 x 1.

import numpy as np

""" ******** Initial conditions ******** """
# The estimated state vector at time t-1 in the global reference frame
# [x_t_minus_1, y_t_minus_1, yaw_t_minus_1]
# [meters, meters, radians]
state_estimate_t_minus_1 = np.array([0.0,0.0,0.0])

# Yaw angle of initial state estimate
yaw_angle = 0.0 # radians

# The control input vector at time t-1 in the global reference frame
# [v, yaw_rate]
# [meters/second, radians/second]
control_vector_t_minus_1 = np.array([4.5, 0.05])

# Delta time between t-1 and t
delta_t = 1.0 # seconds

""" ***** Transformation Matrices ****** """
# The transformation matrix for Xt-1 to Xt for system alone without control
# inputs (changes due to external forces, i.e gravity).
A_t_minus_1 = np.array([[1.0, 0,   0  ],
                        [  0, 1.0, 0  ],
                        [  0, 0,   1.0]])

def getB(yaw,dt):
  """
  Calculates and returns the B matrix, a 3 x 2 matrix, for the given yaw and
  delta time.
  :param yaw: The yaw (rotation angle around the z axis) in rad 
  :param dt: The change in time from time step t-1 to t in sec
  """

  B = np.array([[np.cos(yaw)*dt, 0],
                [np.sin(yaw)*dt, 0],
                [0, dt]])
  return B

""" *************** Noise ************** """
# Noise applied to the forward kinematics (calculation of the estimated state
# at time t from the state transition model of the mobile robot).
process_noise_v_t_minus_1 = np.array([0.01,0.01,0.003])

""" *************** Main *************** """
def main():
  state_estimate_t = A_t_minus_1 @ (state_estimate_t_minus_1)
  + (getB(yaw_angle, delta_t)) @ (control_vector_t_minus_1)
  + (process_noise_v_t_minus_1)
 
  print(f'State at time t-1: {state_estimate_t_minus_1}')
  print(f'Control input at time t-1: {control_vector_t_minus_1}')
  print(f'State at time t: {state_estimate_t}') # State after delta_t seconds
 
main()
