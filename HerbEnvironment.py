import numpy, time, random, collections
from DiscreteEnvironment import DiscreteEnvironment

compare = lambda x, y: collections.Counter(x) == collections.Counter(y)

class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.resolution = resolution
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        self.table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        
        self.robot.GetEnv().Add(self.table)

        table_pose = numpy.array([[ 0, 0, -1, 0.7], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        self.table.SetTransform(table_pose)
        
        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)

        self.p = 0.4
    
    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes

        # Implement using an eight connected world
        
        # Function to determine if the robot is in collision
        def collision_check(config):
            self.robot.SetDOFValues(config, self.robot.GetActiveDOFIndices())
            return self.robot.GetEnv().CheckCollision(self.robot, self.table) or self.robot.CheckSelfCollision()

        def extend_collision_check(start_config, end_config):
            
            #
            # TODO: Implement a function which attempts to extend from 
            #   a start configuration to a goal configuration
            #
            steps = self.ComputeDistance(start_config, end_config) * 10 # Get 10 times the number of unit values in the distance space
            path = [numpy.linspace(start_config[i], end_config[i], steps) for i in range(0, len(self.robot.GetActiveDOFIndices()))]
            path = numpy.array(path)
            for i in xrange(0, len(path[0,:])):
                config = numpy.array([[path[0,i]],
                                   [path[1,i]],
                                   [path[2,i]],
                                   [path[3,i]],
                                   [path[4,i]],
                                   [path[5,i]],
                                   [path[6,i]]])
                if(collision_check(config)):
                    return True
            return False

        # Function to determine if the perterbed config is within the grid
        def valid_id(id, start_config):
            end_config = self.discrete_env.NodeIdToConfiguration(id)
            if(extend_collision_check(start_config, end_config)):
                return False
            for i in xrange(0, len(self.lower_limits)):
                if(end_config[i] > self.upper_limits[i] or end_config[i] < self.lower_limits[i]):
                    return False # if it goes out of bounds
            return True # if it doesn't go out of bounds

        coord = self.discrete_env.NodeIdToGridCoord(node_id)
        config = self.discrete_env.NodeIdToConfiguration(node_id)

        # Return nothing if the id is in collision or out of bounds
        if collision_check(config):
            return []
        for i in xrange(0, len(self.lower_limits)):
            if(config[i] > self.upper_limits[i] or config[i] < self.lower_limits[i]):
                return []

        # Perterb the configuration
        count = 0
        for i in xrange(0, len(coord)):
            perterbed_coord1 = list(coord)
            perterbed_coord2 = list(coord)
            perterbed_coord1[i] = perterbed_coord1[i] + 1
            perterbed_coord2[i] = perterbed_coord2[i] - 1
            perterbed_id1 = self.discrete_env.GridCoordToNodeId(perterbed_coord1)
            perterbed_id2 = self.discrete_env.GridCoordToNodeId(perterbed_coord2)
            if(perterbed_id1 != node_id):
                if(valid_id(perterbed_id1, config)):
                    # Return the robot to old config
                    self.robot.SetDOFValues(config, self.robot.GetActiveDOFIndices(), True)
                    # Append the successor
                    successors.append(perterbed_id1)
            if(perterbed_id2 != node_id):
                if(valid_id(perterbed_id2, config)):
                    # Return the robot to old config
                    self.robot.SetDOFValues(config, self.robot.GetActiveDOFIndices(), True)
                    # Append the successor
                    successors.append(perterbed_id2)

        return successors

    def ComputeDistance(self, start_id, end_id):

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids

        # Use Euclidean distance
        start_config = numpy.array(self.discrete_env.NodeIdToConfiguration(start_id))
        end_config = numpy.array(self.discrete_env.NodeIdToConfiguration(end_id))
       
        return numpy.linalg.norm(end_config - start_config, 2)

    def ComputeHeuristicCost(self, start_id, goal_id):

        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids

        # Use Euclidean distance 
        start_config = numpy.array(self.discrete_env.NodeIdToConfiguration(start_id))
        goal_config = numpy.array(self.discrete_env.NodeIdToConfiguration(goal_id))

        weights = [5, 5, 5, 5, 1, 1, 1]
        for i in xrange(0, len(start_config)):
            #cost += weights[i] * numpy.linalg.norm(goal_config[i] - start_config[i], 2)
            cost += weights[i] * (goal_config[i] - start_config[i]) ** 2

        return cost ** 0.5

        def SetGoalParameters(self, goal_config, p = 0.3):
            self.goal_config = goal_config
            self.p = p
        
    def collision_check(self, config):
        self.robot.SetDOFValues(config, self.robot.GetActiveDOFIndices(), True)
        return self.robot.GetEnv().CheckCollision(self.robot, self.table) or self.robot.CheckSelfCollision()

    def GenerateRandomConfiguration(self):
        config = [0] * len(self.robot.GetActiveDOFIndices())
        #
        # TODO: Generate and return a random configuration
        #
        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()
        while(True):
            config = [random.uniform(lower_limits[i], upper_limits[i]) for i in range(0, len(lower_limits))]
            if(self.collision_check(config)):
                continue
            else:
                return numpy.array(config)

    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
        steps = self.ComputeDistance(self.discrete_env.ConfigurationToNodeId(start_config), self.discrete_env.ConfigurationToNodeId(end_config)) * 25 # Get 25 times the number of unit values in the distance space
        path = [numpy.linspace(start_config[i], end_config[i], steps) for i in range(0, len(self.robot.GetActiveDOFIndices()))]
        path = numpy.array(path)
        if (len(path[0,:]))<10:
            return None
        for i in xrange(0, len(path[0,:])):
            config = numpy.array([[path[0,i]],
                               [path[1,i]],
                               [path[2,i]],
                               [path[3,i]],
                               [path[4,i]],
                               [path[5,i]],
                               [path[6,i]]])
            if(self.collision_check(config)):
                if(i <5):
                    return None
                else:
                    if (self.ComputeDistance(self.discrete_env.ConfigurationToNodeId(path[:,int(i/2)]), self.discrete_env.ConfigurationToNodeId(config)))>0.5:
                        return numpy.array(path[:, 0:int(i/2)]).transpose()
                    else:
                        return None
                    # j=i
                    # while (j>10):
                    #     safe = self.ComputeDistance(self.discrete_env.ConfigurationToNodeId(path[:,j]), self.discrete_env.ConfigurationToNodeId(config))
                    #     if safe > 0.3:
                    #         return numpy.array(path[:, 0:j]).transpose()
                    #     j-=1
                    # return None

        return path.transpose()
        
    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #

        short_path = list(path)

        init_time = time.time()
        while time.time() - init_time < timeout:
            start = random.randint(0, len(short_path)-3)
            end = random.randint(start+1, len(short_path)-1)
            extend = self.Extend(short_path[start], short_path[end])
            if(extend == None or len(extend) == 0):
                continue
            else:
                if compare(extend[len(extend)-1,:], short_path[end]):
                    short_path[start+1:end] = []

        return numpy.array(short_path)