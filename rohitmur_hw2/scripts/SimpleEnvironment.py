import numpy
import matplotlib.pyplot as pl
import time
from copy import deepcopy

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
        self.p = 0.5

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        
    def GenerateRandomConfiguration(self):
        config = [0] * 2;
        lower_limits, upper_limits = self.boundary_limits

        #
        # TODO: Generate and return a random configuration
        #

        self.env = self.robot.GetEnv()
        robot_pose = self.robot.GetTransform()
        table = self.robot.GetEnv().GetBodies()[1]

        sample_goal = False;
        bias_sample = numpy.random.uniform(0,1)
        if (bias_sample <= self.p) :
            sample_goal = True
        else:
            sample_goal = False

        if(sample_goal):
            config = self.goal_config;
        else:
            is_valid = False

            while(not is_valid):
                config[0] = numpy.random.uniform(lower_limits[0], upper_limits[0])
                config[1] = numpy.random.uniform(lower_limits[1], upper_limits[1])
                robot_pose[0][3] = config[0];
                robot_pose[1][3] = config[1];
                self.robot.SetTransform(robot_pose);
                
                is_valid = not (self.env.CheckCollision(self.robot,table))

                # if(not is_valid):
                #     print "Invalid: " + str(config[0]) + " , " + str(config[1])
                #     raw_input()


            # print "Valid: " + str(config[0]) + " , " + str(config[1])
            # raw_input()

        
        return numpy.array(config)

    def ComputeDistance(self, start_config, end_config):
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        #

        distance = numpy.linalg.norm(start_config - end_config)
        return distance


    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #

        max_step_size = 0.5     #Decrease for fine tuning

        distance = end_config - start_config    # Rohit: Use compute distance function
        dist_norm = numpy.linalg.norm(distance)
        if(dist_norm == 0):
            return numpy.array([end_config[0], end_config[1]])

        direction = distance / dist_norm
        increment = max_step_size * direction   # Rohit: Put a check to see if dist_norm < max_step_size 
        dist_norm_incr = numpy.linalg.norm(increment)

        #Creating q_new configuration and storing in 'config_temp'
        config_temp = start_config + increment

        #Checking Collision
        self.env = self.robot.GetEnv()
        robot_pose = self.robot.GetTransform()
        table = self.robot.GetEnv().GetBodies()[1]

        robot_pose[0][3] = config_temp[0];
        robot_pose[1][3] = config_temp[1];
        self.robot.SetTransform(robot_pose);

        is_valid = not (self.env.CheckCollision(self.robot,table))

        if(not is_valid):
            return None
        else:
            
            while(is_valid):
                config_return = config_temp
                
                dist_to_end_config = numpy.linalg.norm(end_config - config_return)
                if(dist_to_end_config < max_step_size):
                    return numpy.array([end_config[0], end_config[1]])

                config_temp = config_temp + increment

                self.env = self.robot.GetEnv()
                robot_pose = self.robot.GetTransform()
                table = self.robot.GetEnv().GetBodies()[1]

                robot_pose[0][3] = config_temp[0];
                robot_pose[1][3] = config_temp[1];
                self.robot.SetTransform(robot_pose);

                is_valid = not (self.env.CheckCollision(self.robot,table))

            return numpy.array([config_return[0], config_return[1]])



        # if(not is_valid):
        #     return numpy.array([0, 0, -1])         # -1 represents status TRAPPED

        # if(dist_norm_incr < dist_norm):
        #     return numpy.array([config_temp[0], config_temp[1], 0])       # 0 represents status ADVANCED
        # else:
        #     return numpy.array([end_config[0], end_config[1], 1])       # 1 represents status ADVANCED



    def interpolatePath(self, path, step_size=0.5):
        new_path = []
        step_size = 0.5
        for i in range(len(path) - 1):
            dist = numpy.linalg.norm(path[i] - path[i+1])
            num_pts = int(numpy.ceil(dist / step_size))
            for j in range(num_pts):
                new_path.append(path[i] + (float(j) / num_pts) * (path[i+1] - path[i]))
        new_path.append(path[-1])
        return new_path

    # checks if there are collisions moving between robot configs q1 and q2
    # returns True if no collision, False if there is a collision
    def checkPathCollision(self, q1, q2, increment=0.01):
        self.env = self.robot.GetEnv()
        robot_pose = self.robot.GetTransform()
        table = self.robot.GetEnv().GetBodies()[1]

        path = [q1, q2]
        new_path = self.interpolatePath(path, step_size=increment)
        # print "collision path", new_path

        for q in new_path:
            for i in range(len(q1)):
                robot_pose[i][3] = q[i]
            self.robot.SetTransform(robot_pose);
            is_valid = not (self.env.CheckCollision(self.robot,table))
            # print q, is_valid
            if not is_valid:
                return False
        return True

    def compareListsOfArrays(self, l1, l2):
        if len(l1) != len(l2):
            return False
        
        for v1, v2 in zip(l1, l2):
            if not numpy.allclose(v1, v2):
                return False

        return True


    def ShortenPath(self, path, timeout=5.0):        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #
        start = time.time()
        while (time.time() - start < timeout):
        #for i in range(3):
            # print "original path"
            # print path
            # print len(path)


            # interpolate path!
            new_path = self.interpolatePath(path)
            # print "original path plus interpolation"
            # print new_path
            # print len(new_path)

            # do actual path shortening
            new_path_2 = [new_path[0]]
            for i in range(len(new_path) - 2):
                dist_1 = numpy.linalg.norm(new_path_2[-1] - new_path[i+1]) + numpy.linalg.norm(new_path[i+1] - new_path[i+2])
                dist_2 = numpy.linalg.norm(new_path_2[-1] - new_path[i+2])
                if abs(dist_1 - dist_2) < 0.01:
                    new_path_2.append(new_path[i+1])
                elif self.checkPathCollision(new_path_2[-1], new_path[i+2]):
                    pass
                else:
                    new_path_2.append(new_path[i+1])
            new_path_2.append(new_path[-1])
            # print "new path"
            # print new_path_2
            # print len(new_path_2)
            # print "----------------------------------"


            if self.compareListsOfArrays(path, new_path_2):
                return new_path_2
            else:
                path = deepcopy(new_path_2)
        return new_path_2



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

