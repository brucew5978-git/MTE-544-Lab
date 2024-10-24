# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1

# Type of trajectory
PARABOLA=0; SIGMOID=1 

import numpy as np

class planner:
    def __init__(self, type_, trajectory_type_):

        self.type=type_
        self.trajectory_type_ = trajectory_type_

    
    def plan(self, goalPoint=[-1.0, -1.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self):

        if self.type == PARABOLA:
            
            x_values = np.linspace(0, 1.5, 50)
            # Calculate y values
            y_values = x_values**2
        
        elif self.type == SIGMOID:
            x_values = np.linspace(0, 2.5, 50)
            y_values = 2 / (1 + np.exp(-2 * x_values)) - 1

        # Combine x and y into pairs
        points = [[x, y] for x, y in zip(x_values, y_values)]

        return points

        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
        # return 

