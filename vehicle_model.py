import numpy as np
from math import *

def vehicle_model(wheelbase, search_length, speed, dt):
    Degree_to_Radian = pi/180    
    max_steering = 30 * Degree_to_Radian                                    # Maximum steering angle in radians (both directions)
    steering_resolution = 5 * Degree_to_Radian                              # Steering angle resolution in radian
    steering_step_num = int( 1+(2*max_steering)/steering_resolution )       # Discrete angle steps
    steering_step = np.linspace( -max_steering,max_steering,steering_step_num, endpoint = True)    # Array of discrete steering angle,include the end point
    steering_step = steering_step[steering_step != 0]        # Remove 0 steering to prevent 0 as denominator
    R_rear = wheelbase/np.tan(steering_step)                 #Calculate the turning radius of each steering of rear wheels in meter
    theta = search_length/R_rear                             #Heading change of each steering in radian
    theta_forward = (speed * dt)/R_rear                      #Heading change of each steering taking speed into account in radian
    
    dy_rear = (-(-R_rear * (np.cos(theta) - 1))).tolist()    #Displacement in x direction (left & right) in meter
    dy_rear.append(0)
    
    dx_rear = (R_rear * np.sin(theta)).tolist()              #Displacement in y direction (back & forth) in meter
    dx_rear.append(search_length)
    
    displacement_rear = np.transpose([dx_rear, dy_rear])

    dy_rear_future = (-(-R_rear * (np.cos(theta_forward) - 1))).tolist()  #Displacement in x direction (left & right) in meter taking speed into account
    dy_rear_future.append(0)
    dx_rear_future = (R_rear * np.sin(theta_forward)).tolist()            #Displacement in y direction (back & forth) in meter taking speed into account
    dx_rear_future.append(speed*dt)
    displacement_rear_forward = np.transpose([dx_rear_future, dy_rear_future])
    
    theta = theta.tolist()
    theta.append(0)
    theta_forward = theta_forward.tolist()
    theta_forward.append(0)
    steering_step = steering_step.tolist()
    steering_step.append(0)

    return theta, theta_forward, displacement_rear, displacement_rear_forward, steering_step