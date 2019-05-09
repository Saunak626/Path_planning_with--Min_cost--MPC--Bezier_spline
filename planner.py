import numpy as np
import matplotlib.pyplot as plt
from math import *
from vehicle_model import vehicle_model
from map import mapGenerator
from bezier import Bezier

class Planner:
    def __init__(self,start,goal,start_heading,goal_heading,start_steering,mapsize,freeGrid_num,tolerance,car_length,car_width,wheelbase,obstacle_type):
        self.start = start
        self.goal = goal
        self.start_heading = start_heading
        self.goal_heading = goal_heading
        self.start_steering = start_steering
        self.freeGrid_num = freeGrid_num
        self.mapsize = mapsize
        self.tolerance = tolerance
        self.car_length = car_length
        self.car_width = car_width
        self.wheelbase = wheelbase
        self.obstacle_type = obstacle_type

        # the bezier spline 
        # using the bezier as the referenece line
        bz = Bezier(start,goal,self.start_heading,self.goal_heading,self.start_steering,self.mapsize,self.car_length,self.car_width,self.wheelbase,self.mapsize * 3,"NoFound")
        self.bezier_spline = bz.calculation()
        #print("self.bezier_spline",self.bezier_spline)

        # the map and obstacle
        self.freeGrid_num,self.obstacle,self.danger_zone = mapGenerator(start,self.obstacle_type,self.mapsize)


    def calculation(self):
        start = self.start
        goal = self.goal

        # the vehicle state in 1.5 second
        theta,theta_forward,displacement_rear,displacement_rear_forward,steering_step = vehicle_model(self.wheelbase, search_length=2.5, speed=5, dt=1.5)
        # the raw initialization
        x = start[0]
        y = start[1]
        x_forward = x
        y_forward = y
        x_prev = x
        y_prev = y

        heading_state = self.start_heading
        rotate_angle = heading_state            # the raw initilization
        steering_state = self.start_steering
        found = 0
        cost_list = [ [0, [x, y], heading_state, steering_state] ]
        #print("cost_list",cost_list)

        next_state = cost_list      # the raw initilization

        path_discrete = list([])    # Initialize discrete path
        path = []                   # Initialize continuous path

        tree_leaf = [ [x, y] ]      # Initialize search tree leaf (search failed)
        path_tree = []              # Initialize continuous path for search trees

        search = 1                  # the A_star search iterators
        step = 1                    # the time step for MPC

        while found != 1:
            if search >= self.freeGrid_num:                         # the map has been searched all
                break

            cost_list.sort(key=lambda x: x[0])                      # sort by the x[0] element  #([total_cost, candidates[i], heading_state[i], steering_step[i]])
            # [0.5702793293265661, array([ 2.49501973, 24.86343515]), 0.10936082940740502, 0.08726646259971649]
            next_state = cost_list.pop(0)                           # pop the first element/ the min cost

            path_discrete.append( np.round(next_state[1]) )         # round [x,y] of the candidate
            path.append( next_state[1] )                            # [x,y] of the candidate

            [x, y] = next_state[1] 
            [x_forward, y_forward] = [x, y]                         # the raw initialization

            heading_state = next_state[2]
            steering_state = next_state[3]

            #############################################################################################################################
            # step is the flag used for MPC control, the online rolling optimization
            if step > 1:
                [x_prev, y_prev] = path[step - 1]       # not the very first step
            step += 1

            rotate_angle = heading_state                # the raw initilization

            # reach the goal position
            if sqrt( np.dot(np.subtract([x, y], goal), np.subtract([x, y], goal) ) ) <= self.tolerance:
                found = 1

            rotate_matrix = [ [ np.cos(rotate_angle), -np.sin(rotate_angle)],  [np.sin(rotate_angle), np.cos(rotate_angle) ] ]
            action = (np.dot(displacement_rear, rotate_matrix)).tolist()            #dot()返回的是两个数组的点积(dot product)
            #print(" action    ##############################################")
            #print(  action )

            action_forward = np.dot(displacement_rear_forward, rotate_matrix)

            candidates = np.add([x, y], action)
            #print("  candidates   ###########################################")
            #print(  candidates )

            candidates_forward = np.add([x_forward, y_forward], action_forward)

            candidates_round = np.round(candidates).astype(int)

            heading_state = np.add(heading_state, theta)
            #print(" len(candidates_round)    ###########################################")
            #print(  len(candidates_round) )


            invalid_ID = [((candidates_round[i] == path).all(1).any() | (candidates_round[i] == self.danger_zone).all(1).any()  | (candidates_round[i] == tree_leaf).all(1).any())      
                            for i in range(len(candidates_round))]

            remove_ID = np.unique(np.where((candidates < 0) | (candidates > self.mapsize))[0])

            candidates = np.delete(candidates, remove_ID, axis=0)
            candidates_forward = np.delete(candidates_forward, remove_ID, axis=0)
            heading_state = np.delete(heading_state, remove_ID, axis=0)
            candidates = np.delete(candidates, np.where(invalid_ID), axis=0)
            candidates_forward = np.delete(candidates_forward, np.where(invalid_ID), axis=0)
            heading_state = np.delete(heading_state, np.where(invalid_ID), axis=0)

            ################################################################################################################################
            # calculate the cost and add the candidates's cost to the cost_list
            #Due to the usage of Bezier spline, no expand grid or heuristic layer is pre-computed as in Min_cost, making it very efficient.
            if len(candidates) > 0:
                cost_list = []
                #print("  len(candidates)   ###########################################")
                #print(  len(candidates) )
                for i in range(len(candidates)):
                    diff = np.square(candidates[i] - self.bezier_spline)       # using the Bezier spline
                    min_dis = min(np.sqrt(np.sum(diff, axis=1)))
                    diff_forward = np.square(candidates_forward[i] - self.bezier_spline)  # using the Bezier spline
                    min_dis_forward = min(np.sqrt(np.sum(diff_forward, axis=1)))
                    total_cost = min_dis + min_dis_forward
                    cost_list.append([total_cost, candidates[i], heading_state[i], steering_step[i]])
            else:
                # the very first step 1 / the raw initilaztion
                search += 1
                if (next_state[1] == tree_leaf).all(1).any():
                    tree_leaf.append(np.round([x_prev, y_prev]))
                else:
                    tree_leaf.append((np.round([x, y])).tolist())
                x = start[0]
                y = start[1]
                x_forward = x
                y_forward = y
                x_prev = x
                y_prev = y
                heading_state = self.start_heading
                rotate_angle = heading_state
                steering_state = self.start_steering
                found = 0
                cost_list = [[0, [x, y], heading_state, steering_state]]
                next_state = cost_list
                path_discrete = list([])    # Initialize discrete path
                path_tree.append(path)
                path = []                   # Initialize continuous path
                step = 1

        # for the last point
        bz_last = Bezier (next_state[1],goal,next_state[2],self.goal_heading,self.start_steering,self.mapsize,self.car_length,self.car_width,self.wheelbase,self.tolerance * 3,"Found")
        bz_last = bz_last.calculation()
        #print("  bz_last   ###########################################")
        #print( bz_last  )
        path.append(bz_last)

        return path, path_tree
