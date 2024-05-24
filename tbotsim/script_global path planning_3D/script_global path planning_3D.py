
from tbotlib import * 
import os
import numpy as np

start = [8,2,14,0,12]
goal = [34, 35, 33, 23, 21]

# Simulation timestep
dt = 0.0167

# Smoothing algorithm
platformsmoother = BsplineSmoother(0.003, k=3)
armsmoother = BsplineSmoother(0.005, k=3)

# Profile generators
platformprofiler = SlicedProfile6(a_t = [0.05,10], 
                                  d_t = [0.05,10], 
                                  v_t = [0.05,1], 
                                  dt = dt, 
                                  smoother = platformsmoother)

armprofiler = Profile3(a_t = 0.05, 
                       d_t = 0.05,
                       v_t = 0.1, 
                       dt = dt, 
                       smoother = armsmoother)

# Local path planner objects
platform2configuration = PlanPlatform2Configuration(graph = TbPlatformPoseGraph(goal_dist = 0.04, 
                                                                                goal_skew = 2.5, 
                                                                                directions = [0.05,0.05,0.01,2.5,2.5,2.5],
                                                                                iter_max = 10000),
                                                    profiler = platformprofiler,
                                                    workspace = TbWorkspace(padding = [-0.1,-0.1,0,-180,-180,-90],
                                                                            mode_2d = True,
                                                                            scale = [0.05,0.05,99,99,99,2], 
                                                                            mode = 'first', threshold = 1))

platform2gripper = PlanPlatform2Gripper(graph = TbPlatformAlignGraph(goal_skew = 2.5, 
                                                                     directions = [0.05,0.05,0.01,2.5,2.5,2.5], 
                                                                     iter_max = 10000),
                                        profiler = platformprofiler)

platform2hold = PlanPlatform2Hold(graph = TbPlatformAlignGraph(goal_skew = 2.5, 
                                                               directions = [0.05,0.05,0.01,2.5,2.5,2.5], 
                                                               iter_max = 10000),
                                  profiler = platformprofiler)

arm2pose = PlanArm2Pose(graph = TbArmPoseGraph(goal_dist = 0.087,
                                               directions = [0.05,0.05,0.05], 
                                               iter_max = 100000),
                        profiler = armprofiler)

localplanner = PlanPickAndPlace2(
                    platform2configuration = platform2configuration,
                    platform2gripper       = platform2gripper, 
                    platform2hold          = platform2hold, 
                    arm2pose               = arm2pose)

# Global path planner objects

platform2gripper = PlanPlatform2Gripper(graph = TbPlatformAlignGraph(goal_skew = 2.5, 
                                                                     directions = [0.0,0.0,0.0,2.5,2.5,2.5], 
                                                                     iter_max = 100),
                                        profiler = platformprofiler)

platform2hold = PlanPlatform2Hold(graph = TbPlatformAlignGraph(goal_skew = 2.5, 
                                                               directions = [0.0,0.0,0.0,2.5,2.5,2.5], 
                                                               iter_max = 100),
                                  profiler = platformprofiler)

globalplanner = GlobalPlanner(graph = TbGlobalGraph2(goal_dist = 0.01,
                                                     cost = 0.0, #0.05
                                                     platform2hold = platform2hold,
                                                     platform2gripper = platform2gripper,
                                                     workspace = TbWorkspace(padding = [-0.1,-0.1,0,-180,-180,-90], #45
                                                                             mode_2d = True,
                                                                             scale = [0.1,0.1,99,99,99,2], #0.2, 45
                                                                             mode = 'first', threshold = 1),
                                                     iter_max = 500000),
                              localplanner = localplanner)


# load assets
tbot: TbTetherbot = TbTetherbot.load(os.path.join(os.path.dirname(__file__), 'data/tetherbot.pkl'))

# place robot into start position
workspace = TbWorkspace(padding = [-0.1,-0.1,0,-180,-180,-90], #45
                        mode_2d = True,
                        scale = [0.1,0.1,99,99,99,10], #0.2, 45
                        mode = 'first')
tbot.place_all(start)
_, pose = workspace.calculate(tbot)
tbot.platform.T_world = TransformMatrix(pose)

print(tbot.platform.T_world.decompose()) 
print(tbot.stability())


commands = globalplanner.plan(tbot, start, goal, commands=CommandList())[1]

commands.save(os.path.join(os.path.dirname(__file__), 'data/commands.pkl'), overwrite=True)

