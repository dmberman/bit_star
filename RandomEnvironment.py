import numpy, time, random, collections, datetime
from operator import sub
from DiscreteEnvironment import DiscreteEnvironment
import pylab as pl

compare = lambda x, y: collections.Counter(x) == collections.Counter(y)

class RandomEnvironment(object):

    def __init__(self, robot, resolution,start_config,goal_config):

        self.robot = robot.robot
        self.resolution = resolution
        #self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.lower_limits = [-5,-5]
        self.upper_limits = [5., 5.]
        self.start_config = start_config
        self.goal_config = goal_config

        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        #self.robot.GetEnv().GetViewer().SetCamera(camera_pose)

        self.p = 0.4

    #maxIter = max number of iterations to try to place an obstacle without collisions
    def genNewEnvironment(self,numObjects,xAxisIndex=0,yAxisIndex=1,save=True,filename='',maxIter=100):
        self.obstacles = [None]*numObjects
        count = numObjects
        for i in range(0,numObjects):

            # Read in obstacle file
            self.obstacles[i] = (self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/obs.kinbody.xml'))
            #Add obstacle to the environment
            self.robot.GetEnv().AddKinBody(self.obstacles[i],True)

            #Check if it is in collision
            collision = True
            iter = 0
            while collision:
                iter += 1
                #Choose random location within the environment
                rand = numpy.random.rand(3,1)
                obsX = rand[0]
                obsY = rand[1]
                rot = numpy.round(rand[2]/.25)*numpy.pi/2

                obsX = (self.upper_limits[xAxisIndex]-self.lower_limits[xAxisIndex])*obsX + self.lower_limits[xAxisIndex]
                obsY = (self.upper_limits[yAxisIndex]-self.lower_limits[yAxisIndex])*obsY + self.lower_limits[yAxisIndex]

                #Generate Transform Matrix
                pose = numpy.array([[numpy.cos(rot), numpy.sin(rot), 0, obsX],
                                      [-numpy.sin(rot), numpy.cos(rot),  0, obsY],
                                      [ 0, 0, 1, 0],
                                      [ 0, 0,  0, 1]])

                #Apply Transform
                self.obstacles[i].SetTransform(pose)
                #Check for collision
                collision = self.robot.GetEnv().CheckCollision(self.obstacles[i])

                #Check that obstacle isn't covering the start or goal position
                bb = self.obstacles[i].ComputeAABB()

                pos = bb.pos()
                extents = bb.extents()

                L = pos[0] - extents[0]
                R = pos[0] + extents[0]
                B = pos[1] - extents[1]
                T = pos[1] + extents[1]

                if self.start_config[0] > L and self.start_config[0] < R:
                    if self.start_config[1] > B and self.start_config[1] < T:
                        collision = True
                        continue

                if self.goal_config[0] > L and self.goal_config[0] < R:
                    if self.goal_config[1] > B and self.goal_config[1] < T:
                        collision = True
                        continue

                if iter >= maxIter:
                    rem = self.robot.GetEnv.Remove(self.obstacles[i])
                    count -= 1

        print "Environment created with "+str(count)+" obstacles..."

        #Save environment
        if save:
            if not filename:
                now = datetime.datetime.now()
                filename = now.strftime("environments/%m%d%Y_%H%M%S")

            #Save environment
            self.robot.GetEnv().Save(filename)
            self.env_filename = filename

    def GetSuccessors(self, node_id):

        # - ----------------------4 CONNECTED VERSION ------------------------
        successors = []
        coord = [0]*self.discrete_env.dimension
        new_coord = [0]*self.discrete_env.dimension

        coord = self.discrete_env.NodeIdToGridCoord(node_id)

        for i in range(self.discrete_env.dimension):

            new_coord = list(coord)
            new_coord[i] = coord[i]-1
            new_config = self.discrete_env.GridCoordToConfiguration(new_coord)

            flag = True
            for j in range(len(new_config)):
                if (new_config[j] > self.upper_limits[j] or new_config[j] < self.lower_limits[j]):
                    flag = False

            if flag == True and not self.collision_check(self.discrete_env.GridCoordToConfiguration(new_coord)):
                successors.append(self.discrete_env.GridCoordToNodeId(new_coord))

        for i in range(self.discrete_env.dimension):
            new_coord = list(coord)
            new_coord[i] = coord[i]+1

            new_config = self.discrete_env.GridCoordToConfiguration(new_coord)

            flag = True
            for j in range(len(new_config)):
                if (new_config[j] > self.upper_limits[j] or new_config[j] < self.lower_limits[j]):
                    flag = False

            if flag == True and not self.collision_check(self.discrete_env.GridCoordToConfiguration(new_coord)):
                successors.append(self.discrete_env.GridCoordToNodeId(new_coord))

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes

        return successors

        #----------------------------8 CONNECTED VERSION -----------------------------------

        # successors = []
        # coord = [0]*2
        # new_coord = [0]*2
        #
        # coord = self.discrete_env.NodeIdToGridCoord(node_id)
        # steps = [[0,1],[0,-1],[-1,0],[1,0],[1,1],[1,-1],[-1,1],[-1,-1]]
        # #print node_id
        # #print coord
        # #print self.discrete_env.GridCoordToNodeId([-0.05,0.05])
        # #print self.discrete_env.GridCoordToConfiguration(coord)
        # for step in steps:
        #     new_coord = [coord[0] + step[0] ,coord[1] + step[1]]
        #     new_config = self.discrete_env.GridCoordToConfiguration(new_coord)
        #     #print new_config
        #     flag = True
        #     for j in range(len(new_config)):
        #         if (new_config[j] > self.upper_limits[j] or new_config[j] < self.lower_limits[j]):
        #             flag = False
        #
        #     if flag == True and not self.collision_check(self.discrete_env.GridCoordToConfiguration(new_coord)):
        #         successors.append(self.discrete_env.GridCoordToNodeId(new_coord))
        # return successors
        #print successors



    def ComputeDistance(self, start_id, end_id):

        dist = 0
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(end_id)

        dist = numpy.linalg.norm(numpy.array(start_config) - numpy.array(end_config))

        # TODO: Here you will implement a function that
        # computes the distance between the configurations given
        # by the two node ids

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):

        cost = 0
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        goal_config = self.discrete_env.NodeIdToConfiguration(goal_id)
        cost = numpy.linalg.norm(numpy.array(start_config) - numpy.array(goal_config))
        # TODO: Here you will implement a function that
        # computes the heuristic cost between the configurations
        # given by the two node ids

        return cost

    def InitializePlot(self,updatePeriod = .1):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(self.start_config[0], self.start_config[1], 'gx')
        pl.plot(self.goal_config[0], self.goal_config[1], 'rx')
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

        self.updatePeriod = updatePeriod
        self.updateTime = time.time() + self.updatePeriod
        pl.ion()
        pl.show()

    def savePlot(self,filename):
        pl.savefig(filename)

    def updatePlotAnnotation(self,text):
        self.fig.suptitle(text)

    def PlotPoint(self,config):
        pl.plot(config[0],config[1],'b*')
        if time.time() > self.updateTime:
            pl.draw()
            self.updateTime = self.updatePeriod + time.time()

    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        if time.time() > self.updateTime:
            pl.draw()
            self.updateTime = self.updatePeriod + time.time()

    def PlotRedEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'r.-', linewidth=2.5)
        if time.time() > self.updateTime:
            pl.draw()
            self.updateTime = self.updatePeriod + time.time()

    def collision_check(self, config):

        robot_pose = numpy.array([[ 1, 0,  0, config[0]],
                                    [ 0, 1,  0, config[1]],
                                    [ 0, 0,  1, 0],
                                    [ 0, 0,  0, 1]])

        self.robot.SetTransform(robot_pose)

        return self.robot.GetEnv().CheckCollision(self.robot)

