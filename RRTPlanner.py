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
        
        # plan.append(start_config)
        # plan.append(goal_config)

        extend_config = start_config

        goal_bias = 0.1
        i=1

        while True:
            # if i%20==0:
            #     random_config = goal_config
            # else:
            #     random_config = self.planning_env.GenerateRandomConfiguration()
            # print('random config is ',random_config)
            

            random_config = self.planning_env.GenerateRandomConfiguration()
            near_config_index,near_config = tree.GetNearestVertex(random_config)

            goal_near_config_index,goal_near_config = tree.GetNearestVertex(goal_config)
            # check to get closer to goal
            if numpy.linalg.norm(goal_near_config-goal_config)<=numpy.linalg.norm(near_config-random_config) or i%10==0:
                random_config = goal_config
                near_config = goal_near_config

            i+=1
            # print('near config is ',near_config)
            extend_config = self.planning_env.Extend(near_config,random_config)
            # print('extend config is ',extend_config)
            if extend_config==None:
                continue
            end_config_index  =  tree.AddVertex(extend_config)
            # print('vertices are ',tree.vertices)            
            tree.AddEdge(near_config_index, end_config_index)
            self.planning_env.PlotEdge(near_config, extend_config)
            
            if numpy.linalg.norm(extend_config-goal_config) <=epsilon:
                break

        print(tree.edges)
        # j = goal_config
        
        # while True:
        #     prev = 
        print('----------------')     
        # print(tree.edges[len(tree.edges)])
        # print('----------------')     
        # print(tree.vertices[tree.edges[len(tree.edges)]])
        # print(lastkey)
        # print(tree.edges[lastkey])
        # tree.vertices[prevkey]
        lastkey = len(tree.edges)
        print(lastkey)

        plan_index = []
        plan_index.append(lastkey)

        while True:
            prevkey = tree.edges[lastkey]
            plan_index.append(prevkey)
            print(prevkey)

            if prevkey == 0:
                break

            lastkey = prevkey
        print('----------')
        print (plan_index)
        plan_index.sort()
        print(plan_index)
        print('----------')

        for i in range(len(plan_index)):
            plan.append(tree.vertices[plan_index[i]])

        print(plan)

        return plan
