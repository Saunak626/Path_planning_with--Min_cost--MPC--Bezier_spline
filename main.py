
import numpy as np
import matplotlib.pyplot as plt
from math import *
from bezier import Bezier
from vehicle_model import vehicle_model
from map import mapGenerator
from planner import Planner

Degree_to_Radian = pi/180    # degree to radian

# Initial states of the vehicle
start = [0, 25]                     #Starting position in meter
start_heading = 0                   #Starting heading in degree
start_steering = 0                  #Starting steering in degree
goal = [40,40]                     #Goal posotion in meter
goal_heading = 60                   #Goal heading in degree
goal_steering = 0                   #Goal steering in degree

car_length = 5
car_width = 3
wheelbase = 2

tolerance = 3
obstacle_type = "static_obstacle"

# Convert all angles into radians
start_heading *= Degree_to_Radian
start_steering *= Degree_to_Radian
goal_heading *= Degree_to_Radian
goal_steering *= Degree_to_Radian


#############################################################################################################################################
# generate Map and obstacles
mapsize = 50   #Map size 50*50
freeGrid_num, obstacle, occupy_grid = mapGenerator(start, obstacle_type, mapsize )

# Vehicle dtnamics 
start_to_goal_distance = sqrt( (start[0] - goal[0])**2 + (start[1] - goal[1])**2 ) #distance from start point to goal point
# the vehicle state in the forward 1 second
theta, theta_forward, displacement_rear, displacement_rear_forward, steering_step = vehicle_model(wheelbase, search_length = 2.5, speed = 5, dt = 1 )


# Bezier spline calculation
bezier = Bezier(start, goal, start_heading, goal_heading, start_steering, mapsize, car_length, car_width, wheelbase, 3*mapsize,"Nofound")
bezier_spline = bezier.calculation()
#bezier.plot()

###################################################################################################################################################
# the main process
p = Planner(start, goal, start_heading, goal_heading, start_steering, mapsize, freeGrid_num, tolerance, car_length, car_width, wheelbase, obstacle_type)
path, path_tree = p.calculation()



# the trajectory points
x, y = np.vstack(path).T                # 沿着竖直方向将矩阵堆叠起来
plt.figure( figsize=(10,10) )
# plot the obstacles
plt.scatter(obstacle[:,0],obstacle[:,1],marker='*')
# plot the trajectory
plt.plot(x,y,linewidth = '3')

# plot the beizer curve
plt.plot(bezier_spline[:,0], bezier_spline[:,1], linestyle='--',color = 'y')

# plot the candidate trajectories
for i in range(len(path_tree)):
    path_tree[i] = np.vstack(path_tree[i])
    x, y = path_tree[i].T
    plt.plot(x,y,color = 'r', linewidth = 2, linestyle = ':')

#plot the vehicle and it's speed direction
plt.gca().add_patch(plt.Rectangle((start[0] - 0.5 * (car_length - wheelbase) , start[1] + 0.5* car_width ), 3, 5, fc = 'lightblue',angle = -90,edgecolor = 'black'))
plt.gca().add_patch(plt.Rectangle((goal[0] - 0.5 * (car_length - wheelbase) , goal[1] + 0.5* car_width ), 3, 5, fc = 'lightblue',angle = -90,edgecolor = 'black'))
"""
plt.gca().add_patch(plt.Rectangle((goal[0] - 0.5 * (car_width*np.sin(goal_heading) + (car_length - wheelbase)*np.cos(goal_heading)),
                                   goal[1] - 0.5 * (-car_width*np.cos(goal_heading) + (car_length - wheelbase)*np.sin(goal_heading))),
                                   3, 5, fc = 'lightblue',edgecolor = 'black', angle = -90 + goal_heading*180/pi ))
"""
plt.scatter(start[0], start[1], zorder = 10, color = 'navy')
plt.scatter(goal[0],  goal[1],  zorder = 10,  color = 'navy')
plt.arrow(start[0],start[1], wheelbase*np.cos(start_heading), wheelbase*np.sin(start_heading), head_width=0.5,zorder = 11)
plt.arrow(goal[0], goal[1],  wheelbase*np.cos(goal_heading),  wheelbase*np.sin(goal_heading),  head_width=0.5,zorder = 11)
plt.xticks(np.arange(0, mapsize+1, 5.0))
plt.yticks(np.arange(0, mapsize+1, 5.0)) 
plt.axes().set_yticks(np.arange(0.5, mapsize+0.5, 1.0), minor=True)
plt.axes().set_xticks(np.arange(-2.5, mapsize+0.5, 1.0), minor=True)

# turn the grid on
plt.grid(b=True, which='minor', color='grey', linestyle='--')
plt.show()

print("Done")