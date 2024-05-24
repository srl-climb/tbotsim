
from tbotlib import * 
import os

start = [8,2,14,0,12]
goal = [9,3,15,1,13]

#to do
# debut mode 3d

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
iter = 50000
platform2configuration = PlanPlatform2Configuration(graph = TbPlatformPoseGraph(goal_dist = 0.03, 
                                                                                goal_skew = 3, 
                                                                                directions = [0.05,0.05,0.01,2.5,0,0], #
                                                                                iter_max = iter),
                                                    profiler = platformprofiler,
                                                    workspace = TbWorkspace(padding = [-0.1,-0.1,0,-180,-180,-90],
                                                                            mode_2d = True,
                                                                            scale = [0.1,0.1,99,99,99,10], 
                                                                            mode = 'max'))

platform2gripper = PlanPlatform2Gripper(graph = TbPlatformAlignGraph(goal_skew = 3, 
                                                                     directions = [0.05,0.05,0.05,2.5,2.5,2.5], 
                                                                     iter_max = iter),
                                        profiler = platformprofiler)

platform2hold = PlanPlatform2Hold(graph = TbPlatformAlignGraph(goal_skew = 3, 
                                                               directions = [0.05,0.05,0.05,2.5,2.5,2.5], 
                                                               iter_max = iter),
                                  profiler = platformprofiler)

arm2pose = PlanArm2Pose(graph = TbArmPoseGraph(goal_dist = 0.05,
                                               directions = [0.05,0.05,0.01], 
                                               iter_max = iter),
                        profiler = armprofiler)

localplanner = PlanPickAndPlace2(
                    platform2configuration = platform2configuration,
                    platform2gripper       = platform2gripper, 
                    platform2hold          = platform2hold, 
                    arm2pose               = arm2pose)

# Global path planner objects

platform2gripper = PlanPlatform2Gripper(graph = TbPlatformAlignGraph(goal_skew = 3, 
                                                                     directions = [0.0,0.0,0.0,2.5,2.5,2.5], 
                                                                     iter_max = 50),
                                        profiler = platformprofiler)

platform2hold = PlanPlatform2Hold(graph = TbPlatformAlignGraph(goal_skew = 3, 
                                                               directions = [0.0,0.0,0.0,2.5,2.5,2.5], 
                                                               iter_max = 50),
                                  profiler = platformprofiler)

globalplanner = GlobalPlanner(graph = TbGlobalGraph2(goal_dist = 0.01,
                                                     cost = 0.05, 
                                                     platform2hold = platform2hold,
                                                     platform2gripper = platform2gripper,
                                                     workspace = TbWorkspace(padding = [-0.1,-0.1,0,-180,-180,-90], #45
                                                                             mode_2d = True,
                                                                             scale = [0.1,0.1,99,99,99,10], #0.2, 45
                                                                             mode = 'first'),
                                                     iter_max = 5000),
                             localplanner = localplanner)


# load assets
tbot = TbTetherbot.load(os.path.join(os.path.dirname(__file__), 'data/tetherbot.pkl'))

commands = globalplanner.plan(tbot, start, goal, commands=CommandList())[1]

commands.save(os.path.join(os.path.dirname(__file__), 'data/commands.pkl'), overwrite=True)
