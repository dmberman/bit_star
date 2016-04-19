import random
import numpy as np

class DepthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()

    def Plan(self, start_config, goal_config):
        
        plan = []
        
        # TODO: Here you will implement the depth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        if(self.visualize): # initialize plot
            self.planning_env.InitializePlot(goal_config)

        def dfs(current_id, goal_id, plan, visited):
            # Get the config and add it to the path
            visited += current_id
            current_config = self.planning_env.discrete_env.NodeIdToConfiguration(current_id)
            if(len(plan) == 0):
                plan = np.array(current_config)
            else:
                plan = np.vstack((plan, current_config))

            print len(plan)
            if self.visualize and len(visited) >= 2: # Plot the edge
                print plan
                self.planning_env.PlotEdge(plan[len(plan) - 2, :], plan[len(plan) - 1, :]) 

            # Check to see if you have found the goal
            if(current_id == goal_id):
                return plan

            # Get the successors
            successors = self.planning_env.GetSuccessors(current_id)
            successors_shuffled = random.shuffle(successors)
            print "Not Shuffled: ", successors
            print "Shuffled: ", successors_shuffled
            for successor in successors_shuffled:
                if successor not in visited:
                    dfs(successor, goal_id, plan, visited)

            return np.array([])

        # Get the ids of the start and goal config
        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        # Open stack
        stack = []
        stack.append(start_id)
        visited = [] # visited nodes
        current_id = start_id

        found_goal = False

        while len(stack) != 0:
            # Get the tail of the stack
            current_id = stack.pop()
            if(current_id == goal_id):
                found_goal = True
                break
            successors = self.planning_env.GetSuccessors(current_id)
            random.shuffle(successors) # Shuffle the successors
            # Find a non-visited successor to the current_id
            for successor in successors:
                if(successor not in visited):
                    visited += [successor]
                    stack += [successor]
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

