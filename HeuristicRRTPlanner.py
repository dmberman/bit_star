import numpy, random
from RRTTree import RRTTree
import random

class HeuristicRRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.quality_floor = 0.2
        self.max_cost = 0
        self.opt_cost = 0



    def Plan(self, start_config, goal_config, epsilon = 0.001):

        tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        threshold=0.3
        extendedConfig = self.planning_env.GenerateRandomConfiguration()

        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        self.g_scores = dict()
        self.f_scores = dict()

        self.g_scores[start_id] = 0
        self.f_scores[start_id] = self.planning_env.ComputeHeuristicCost(start_id, goal_id)

        m_quality=0
        prob_floor=0.4

        C_opt= self.planning_env.ComputeDistance(start_id, goal_id)

        def compDist(start_config, end_config):
            return numpy.linalg.norm(numpy.array(start_config) - numpy.array(end_config),2)

        while compDist(extendedConfig, goal_config)> threshold:

            m_quality=0.0
            r=random.random()
            while (r>m_quality):


                prob = random.random()
                if (prob < 0.2):
                    config = numpy.copy(goal_config)
                    xnnIDs,xnnVertices= tree.GetKNearestVertex((config),len(tree.vertices))
                    xnnID,xnnVertice= tree.GetNearestVertex((config))
                    m_quality=1
                else:
                    config = self.planning_env.GenerateRandomConfiguration()

                    xnnIDs,xnnVertices= tree.GetKNearestVertex((config),len(tree.vertices))
                    xnnID,xnnVertice= tree.GetNearestVertex((config))

                    C_vertex = self.planning_env.ComputeHeuristicCost(self.planning_env.discrete_env.ConfigurationToNodeId(config),goal_id)
                    C_max = max(self.f_scores.values())

                    q=(1 - (( C_vertex - C_opt)/float(C_max-C_opt)))
                    print "m_quality ", q


                    m_quality= min(prob_floor,abs(1 - (( C_vertex - C_opt)/float(C_max-C_opt))))

                r=random.random()

                print "m_quality ", m_quality

            for i in range(len(tree.vertices)):
                path = self.planning_env.Extend(xnnVertices[i],config)
                if path!=None:
                    xnnID=xnnIDs[i]
                    xnnVertice=xnnVertices[i]
                    break
            if(path != None and len(path) != 0):

                extendedConfig = path[-1,:]

                vid=tree.AddVertex(extendedConfig)
                if vid!=xnnID:
                    tree.AddEdge(xnnID,vid)
                    g_score = self.planning_env.ComputeDistance(vid, xnnID)
                    self.g_scores[vid] = g_score + self.g_scores[xnnID]
                    self.f_scores[vid] = g_score + self.planning_env.ComputeHeuristicCost(vid, goal_id)

                if self.visualize:
                    self.planning_env.PlotEdge(tree.vertices[xnnID],tree.vertices[vid])

        plan.append(goal_config)

        curr_id = goal_id
        while(curr_id != start_id):
            print "current id ",curr_id
            plan.append(tree.vertices[curr_id]) # Add the new vertex to plan
            curr_id = tree.edges[curr_id] # Get the vertex opposite the edge of the current id

        plan.append(start_config)

        plan=plan[::-1]
        print "Number of vertices in the tree:", len(tree.vertices)

        return plan,len(tree.vertices)

