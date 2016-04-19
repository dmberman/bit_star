import numpy, operator
from RRTPlanner import RRTTree
import time

class RRTConnectPlanner(object):

    '''
    Object initializer function
    '''
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.vertex_queue = dict() # self.vertex_queue[node_id] = f_score
        self.edge_queue = dict() # self.edge_queue[node_id] = f_score
        self.samples = dict() # self.edge_queue[node_id] = f_score
        
    '''
    Main Implementation for getting a plan
    '''
    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        # Initialize tree and plan
        tree = RRTTree(self.planning_env, start_config)
        plan = []

        # Initialize plot
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)

    '''
    Function to expand a vertex
    '''
    def ExpandVertex(self, vertex):
        pass

    '''
    Function to prune the tree
    '''
    def Prune(self, goal_cost):
        pass 

    '''
    Function to extend between two configurations
    '''
    def ext(self, tree, random_config):
        # Get the nearest configuration to this
        sid, nearest_config = tree.GetNearestVertex(random_config)
        # Get the interpolation between the two
        path = self.planning_env.Extend(nearest_config, random_config)
        # Return only the first two parts in the path
        if(path == None):
            return path, sid, nearest_config
        else:
            return path[0:2,:], sid, nearest_config

    '''
    Function to connnect two configurations
    '''
    def con(self, tree, random_config):
        # Get the nearest configuration to this
        sid, nearest_config = tree.GetNearestVertex(random_config)
        # Return the whole path to the end
        return self.planning_env.Extend(nearest_config, random_config), sid, nearest_config

