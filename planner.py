import math
from utilities import Config, PlannerType

class planner:
    def __init__(self, type_: PlannerType):
        self.type = type_
    
    def plan(self, goalPoint=[-1.0, -1.0]):
        if self.type==PlannerType.POINT:
            return self.point_planner(goalPoint)
        else:
            return self.trajectory_planner()

    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    def trajectory_planner(self, steepness=2.0):
        trajectory = []
        
        for x_step in range(int(Config.PLANNER_TRAJECTORY_RESOLUTION * Config.PLANNER_TRAJECTORY_XMAX)):
            # Calculate the x value, scaled by the gain
            x = (x_step / Config.PLANNER_TRAJECTORY_RESOLUTION) * Config.PLANNER_TRAJECTORY_GAIN

            if Config.PLANNER_PATH == PlannerType.PATH_PARABOLA:
                y = x ** 2  # Parabolic trajectory (y = x^2)
            
            elif Config.PLANNER_PATH == PlannerType.PATH_SIGMOID:
                # Sigmoid trajectory with translation to pass through (0, 0)
                y = 2 / (1 + math.exp(-steepness * x)) - 1
            else:
                raise ValueError("Unknown trajectory type selected")

            trajectory.append([x, y])
        
        return trajectory

#used this to test trajectory plkanner
# if __name__=="__main__":
#     planner_class = planner(type_ = 1)
#     planner_class.trajectory_planner()