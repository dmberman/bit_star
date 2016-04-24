#!/usr/bin/env python
import argparse, numpy, openravepy, time

from HerbRobot import HerbRobot
from SimpleRobot import SimpleRobot
from RandomEnvironment import RandomEnvironment
from HerbEnvironment import HerbEnvironment
from SimpleEnvironment import SimpleEnvironment

from AStarPlanner import AStarPlanner
from DepthFirstPlanner import DepthFirstPlanner
from BreadthFirstPlanner import BreadthFirstPlanner
from HeuristicRRTPlanner import HeuristicRRTPlanner

def main(robot, planning_env, planner):
    raw_input('Press any key to begin planning')

    #float, for dividing
    n = 1
    plan_time = 0
    path_len = 0
    vertex_len = 0
    short_path_len = 0
    plan_dist = 0 
    short_plan_dist = 0

    plan_start_time = time.time()   
    #plan, nodes_expanded = planner.Plan(start_config, goal_config)
    plan = planner.Plan(start_config, goal_config)
    plan_arrray = numpy.array(plan)
    for j in range(0, len(plan)-1):
        plan_dist += numpy.linalg.norm(plan_arrray[j,:] - plan_arrray[j+1,:])
    plan_time = plan_time + (time.time() - plan_start_time)
    path_len = path_len + len(plan)

    #plan_short = planning_env.ShortenPath(plan)
    # for k in range(0, len(plan_short)-1):
    #    short_plan_dist += numpy.linalg.norm(plan_short[k,:] - plan_short[k+1,:])
    #short_path_len = short_path_len + len(plan_short)

    print ('Plan Time: ', plan_time/n)
    print ('Path Vertices: ', path_len/n)
    print "Euclidean Distance of Long Path: ", plan_dist
    print ('Short Path Vertices: ', short_path_len/n)
    print "Euclidean Distance of Shortened Path: ", short_plan_dist
   #print "Nodes expanded: ", nodes_expanded
    plannerName = 'ASTAR'
    planning_env.updatePlotAnnotation("Planner = "+plannerName+", Plan Time = "+str(plan_time)+", Path Length = "+str(path_len))

    #traj = robot.ConvertPlanToTrajectory(plan)

    #raw_input('Press any key to execute long trajectory')
    #robot.ExecuteTrajectory(traj)

    #raw_input('Press any key to execute short trajectory')
    #traj = robot.ConvertPlanToTrajectory(plan_short)
    #robot.ExecuteTrajectory(traj)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='script for testing planners')
    parser.add_argument('-r', '--robot', type=str, default='simple',
                        help='The robot to load (herb or simple)')
    parser.add_argument('-n', '--runs', type=int, default = 1,
                        help='Number of runs to do')
    parser.add_argument('-p', '--planners', type=str, default='astar bfs',
                        help='Planners to run (astar, bfs, dfs or hrrt)')
    parser.add_argument('-v', '--visualize', action='store_true',
                        help='Enable visualization of tree growth (only applicable for simple robot)')
    parser.add_argument('--resolution', type=float, default=0.1,
                        help='Set the resolution of the grid (default: 0.1)')
    parser.add_argument('-d', '--debug', action='store_true',
                        help='Enable debug logging')
    parser.add_argument('-m', '--manip', type=str, default='right',
                        help='The manipulator to plan with (right or left) - only applicable if robot is of type herb')
    args = parser.parse_args()
    
    openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
    openravepy.misc.InitOpenRAVELogging()

    if args.debug:
        openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Debug)

    env = openravepy.Environment()
    #env.SetViewer('qtcoin')
    #env.GetViewer().SetName('BIT* Viewer')

    # First setup the environment and the robot
    visualize = args.visualize


    if args.robot == 'herb':
        robot = HerbRobot(env, args.manip)
        visualize = False
    elif args.robot == 'simple':
        robot = SimpleRobot(env)
    else:
        print 'Unknown robot option: %s' % args.robot
        exit(0)


    start_config = numpy.array(robot.GetCurrentConfiguration())
    if robot.name == 'herb':
       goal_config = numpy.array([ 4.6, -1.76, 0.00, 1.96, -1.15, 0.87, -1.43] )
       # goal_config = numpy.array([ 3.68, -1.90,  0.00,  2.20,  0.00,  0.00,  0.00 ])
    else:
        goal_config = numpy.array([3.0, 0.0])

    planning_env = RandomEnvironment(robot, args.resolution,start_config,goal_config)
    planning_env.genNewEnvironment(10)

    planning_env.visualize = visualize

    # Next setup the planner
    if args.planner == 'astar':
        planner = AStarPlanner(planning_env, visualize)
    elif args.planner == 'bfs':
        planner = BreadthFirstPlanner(planning_env, visualize)
    elif args.planner == 'dfs':
        planner = DepthFirstPlanner(planning_env, visualize)
    elif args.planner == 'hrrt':
        planner = HeuristicRRTPlanner(planning_env, visualize)
    #elif args.planner == 'rrt':
    #    planner = RRTPlanner(planning_env, visualize)
    else:
        print 'Unknown planner option: %s' % args.planner
        exit(0)

    main(robot, planning_env, planner)

    import IPython
    IPython.embed()

        
    
