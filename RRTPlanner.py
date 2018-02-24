import numpy
from RRTTree import RRTTree
import IPython

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        #IPython.embed()
        tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        
        plan.append(start_config)
        plan.append(goal_config)

        #print(start_config)
        #print(goal_config)

        new_vertex = self.planning_env.GenerateRandomConfiguration()
        v1,v2 = self.RRTTree.GetNearestVertex(new_vertex)

        print(v1)
        print(v2)
        return plan
