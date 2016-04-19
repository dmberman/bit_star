import random

class BreadthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()
        
    def Plan(self, start_config, goal_config):
        
        plan = []

        # TODO: Here you will implement the breadth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        
        if(self.visualize): # initialize plot
            self.planning_env.InitializePlot(goal_config)

        # Get the ids of the start and goal config
        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        # Open queue
        queue = []
        queue.append(start_id)
        visited = [] # visited nodes
        current_id = start_id

        found_goal = False

        while len(queue) != 0:
            # Get the head of the queue
            current_id = queue.pop(0)
            if(current_id == goal_id):
                found_goal = True
                break    
            successors = self.planning_env.GetSuccessors(current_id)
            random.shuffle(successors) # Shuffle the successors
            # Find a non-visited successor to the current_id
            for successor in successors:
                if(successor not in visited):
                    visited += [successor]
                    queue += [successor]
                    self.nodes[successor] = current_id # Store the connection 
                    if self.visualize: # Plot the edge
                        pred_config = self.planning_env.discrete_env.NodeIdToConfiguration(current_id)
                        succ_config = self.planning_env.discrete_env.NodeIdToConfiguration(successor)
                        self.planning_env.PlotEdge(pred_config, succ_config) 

        if found_goal:
            curr_id = goal_id
            while(curr_id != start_id):
                curr_confg = self.planning_env.discrete_env.NodeIdToConfiguration(curr_id)
                plan.append(curr_confg)
                curr_id = self.nodes[curr_id] # Get the vertex opposite the edge of the current id

            # Whenever the current id is the start id, append start id
            plan.append(start_config)

            return plan[::-1], len(self.nodes) # reverse the plan
        else:
            return [] # Failure

