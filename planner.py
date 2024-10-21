import math
# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1

PARABOLA = 0
SIGMOID = 1
selected_trajectory = PARABOLA
trajectory_resolution = 100

class planner:
    def __init__(self, type_):

        self.type=type_

    
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
        trajectory = []
        x_maximum = 2.5
        for x_step in range(int(trajectory_resolution*x_maximum)):
            x = x_step/trajectory_resolution

            if(selected_trajectory == PARABOLA):
                y = (x*x)
            elif(selected_trajectory == SIGMOID):
                y = 2/(1+math.exp(-2*x))

            trajectory.append([x,y])
            #print([x,y])
        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
        return trajectory

#used this to test trajectory plkanner
# if __name__=="__main__":
#     planner_class = planner(type_ = 1)
#     planner_class.trajectory_planner()