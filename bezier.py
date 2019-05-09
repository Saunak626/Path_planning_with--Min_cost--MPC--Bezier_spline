import numpy as np
import matplotlib.pyplot as plt
from math import *

class Bezier:
    def __init__(self, start, goal, start_heading, goal_heading, start_steering, mapsize, car_length, car_width, wheelbase, dots, found):
        self.start = start
        self.goal = goal
        self.start_heading = start_heading
        self.goal_heading = goal_heading
        self.start_steering = start_steering
        self.mapsize = mapsize
        self.car_length = car_length
        self.car_width = car_width
        self.wheelbase = wheelbase
        self.dots = dots    # 3*mapsize
        self.found = found

    # Bezier spline calculation
    def calculation(self):
        init_distance = sqrt( (self.start[0] - self.goal[0]) ** 2 + (self.start[1] - self.goal[1]) ** 2 )  # start to goal distance
        t = np.linspace(0, 1, self.dots, endpoint=True)                   # Define how many points between 0 and 1 for bezier spline
        bezier_start_heading = self.start_heading - self.start_steering   # Start heading taking steering into consideration
        bezier_goal_heading = self.goal_heading                           # Goal heading without taking steering into consideration
        if self.found == "Found":
            bezier_goal_heading = self.goal_heading                       #Goal heading withou taking steering into consideration
            b0 = (1-t)**3
            b1 = 3 * (1-t)**2 * t
            b2 = 3 * (1-t) * t**2
            b3 = t**3
            B = np.array([b0, b1, b2, b3]).transpose()
            P = np.array([[self.start[0], self.start[1]],
                          [self.start[0], self.start[1]],
                          [self.goal[0] - np.cos(bezier_goal_heading), self.goal[1] - np.sin(bezier_goal_heading)],
                          [self.goal[0], self.goal[1]]])
            self.bezier_spline = np.dot(B, P)
        else:
            if abs(self.start_heading - self.goal_heading) <= pi / 4:  # If heading difference less than 45 deg, then use 3rd order bezier
                b0 = (1 - t) ** 3
                b1 = 3 * (1 - t) ** 2 * t
                b2 = 3 * (1 - t) * t ** 2
                b3 = t ** 3
                B = np.array([b0, b1, b2, b3]).transpose()
                start_vec = 10 * (init_distance / self.mapsize) + abs(bezier_start_heading - self.goal_heading) + abs(self.start_steering)
                goal_vec = 10 * (init_distance / self.mapsize) + abs(self.start_heading - self.goal_heading)
                P = np.array([[self.start[0], self.start[1]],
                              [self.start[0] + start_vec * np.cos(bezier_start_heading),
                               self.start[1] + start_vec * np.sin(bezier_start_heading)],
                              [self.goal[0] - goal_vec * np.cos(bezier_goal_heading),
                               self.goal[1] - goal_vec * np.sin(bezier_goal_heading)],
                              [self.goal[0], self.goal[1]]])
                self.bezier_spline = np.dot(B, P)
            else:  # Otherwise use 4rd order bezier
                b0 = (1 - t) ** 4
                b1 = 4 * (1 - t) ** 3 * t
                b2 = 6 * (1 - t) ** 2 * t ** 2
                b3 = 4 * (1 - t) * t ** 3
                b4 = t ** 4
                B = np.array([b0, b1, b2, b3, b4]).transpose()
                start_vec = 20 * (init_distance / self.mapsize) + abs(bezier_start_heading - self.goal_heading) + abs(self.start_steering)
                goal_vec = 20 * (init_distance / self.mapsize) + abs(self.start_heading - self.goal_heading)
                P = np.array([[self.start[0], self.start[1]],
                              [self.start[0] + start_vec * np.cos(bezier_start_heading),
                               self.start[1] + start_vec * np.sin(bezier_start_heading)],
                              [self.goal[0], self.start[1] * (1 - 1 / np.tan(abs(self.start_heading - self.goal_heading)))],
                              [self.goal[0] - goal_vec * np.cos(bezier_goal_heading),
                               self.goal[1] - goal_vec * np.sin(bezier_goal_heading)],
                              [self.goal[0], self.goal[1]]])
                self.bezier_spline = np.dot(B, P)

        return self.bezier_spline


