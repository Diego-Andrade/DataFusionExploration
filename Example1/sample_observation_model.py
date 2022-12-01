# Author:      Diego Andrade (bets636@gmail.com)
# Data:        11/30/2022
# Description:
# An example of a observation model for a simple differential drive robot with
# with the following properties:
# - The robot operates in x-y plane of global frame and has a yaw with respect
#   to global frame, in the direction of Xglobal counterclockwise to Yglobal.
# - The robot has a body frame where forward is the x-axis and z-axis aligned
#   to global frame.
# - The robot takes control inputs V and omega with repsect to robot body frame.
#   V is in the direction of Xrobot, and omega is the counterclockwise angle
#   from Xrobot to Yrobot.
# - The robot does not move if no control input given (no coasting)
#
# Additionally, the robot has 3 sensor, one which measures xglobal, one which
# measures yglobal, and one which measure yawglobal.
#
# The observation model is a mathematical equation that represents a vector of
# predicted sensor measurements Y at time t as a function of the state of a 
# robotic system x at time t, plus some sensor noise at time t, denoted by 
# vector wt. It is given as follows:
#   Yt = HXt + Wt
#
# Yt is an m x 1 matrix representing the predicted sensor observations (wouldbe
# measurement) at time t for m sensors. In this example, m = 3.
#
# H is an m x n matrix which transforms a state vector a time t into a predicted
# sensor observation at time t. In this example, m = 3 and n = 3, as previously
# seen. Since the sensors all directly measure their corresponding value of
# interest, H in this case would be the identity matrix since no transformation
# is needed.
# 
# Wt is an m x 1 matrix representing the noise of each sensor as given in the
# by their corresponding datasheets. In this example, m = 3

import numpy as np

""" ******** Initial conditions ******** """
# The estimated state vector at time t in the global reference frame
# [x_t, y_t, yaw_t]
# [meters, meters, radians]
state_estimate_t = np.array([5.2,2.8,1.5708])

""" ***** Transformation Matrices ****** """
# Measurement matrix H_t used to convert the predicted state estimate at time t
# into predicted sensor measurements at time t.
H_t = np.array([    [1.0,  0,   0  ],
                    [  0,  1.0, 0  ],
                    [  0,  0,   1.0]])
                 

""" *************** Noise ************** """
# Random sensor noise
sensor_noise_w_t = np.array([0.07,0.07,0.04])


""" *************** Main *************** """
def main():
 
    estimated_sensor_observation_y_t = H_t @ (state_estimate_t) + (sensor_noise_w_t)
 
    print(f'State at time t: {state_estimate_t}')
    print(f'Estimated sensor observations at time t: {estimated_sensor_observation_y_t}')
 
main()
