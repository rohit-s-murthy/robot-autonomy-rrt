import numpy
import matplotlib.pyplot as pl

import math

class SimpleEnvironment(object):
    
    def __init__(self, herb):
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5.], [5., 5.]]

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.0], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # goal sampling probability
        self.p = 0.0

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        
    def GenerateRandomConfiguration(self):
        config = [0] * 2;
        lower_limits, upper_limits = self.boundary_limits
        #
        # TODO: Generate and return a random configuration
        #


        while True:          
            config = numpy.random.uniform(lower_limits,upper_limits,2)
            T = self.robot.GetTransform()            # T = numpy.eye(4)
            T[0,3] = config[0]
            T[1,3] = config[1]
            self.robot.SetTransform(T)    

            if self.robot.GetEnv().CheckCollision(self.robot) ==False:
                break         

        return numpy.array(config)
        # return config

    def ComputeDistance(self, start_config, end_config):
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        #
        distance = numpy.linalg.norm(start_config-end_config)
        return distance
        # pass

    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #

        step_count = 20
        # print(start_config)
        # print(end_config)
        # print('------------')
        # extend_config = start_config
        final_config = None
        lower_limits, upper_limits = self.boundary_limits

        diff = end_config-start_config
        step = diff/step_count
           
        # print('step count is',step)
        i=1
        while i<step_count:

            extend_config = start_config + step

            if  numpy.less(extend_config ,lower_limits).any()==True or numpy.greater(extend_config ,upper_limits).any()==True:
                break
            # print(diff*epsilon)

            T = self.robot.GetTransform()
            T[0,3] = extend_config[0]
            T[1,3] = extend_config[1]             
            self.robot.SetTransform(T)    
            check = self.robot.GetEnv().CheckCollision(self.robot) 

            if check == True:
                break

            start_config = extend_config
            final_config = extend_config   

            i+=1

        # print('done with extend')
        return final_config
        # pass

    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #
        return path


    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()
        pl.pause(0.2)    

