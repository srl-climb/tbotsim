import os
from tbotlib import *

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
                                                                                directions = [0.05,0.05,0.05,5,5,5], #
                                                                                iter_max = iter),
                                                    profiler = platformprofiler,
                                                    workspace = TbWorkspace(padding = [-0.1,-0.1,0,-150,-150,-90],
                                                                            mode_2d = True,
                                                                            scale = [0.1,0.1,0.1,10,10,10], 
                                                                            mode = 'max'))

platform2gripper = PlanPlatform2Gripper(graph = TbPlatformAlignGraph(goal_skew = 3, 
                                                                     directions = [0.05,0.05,0.05,2.5,2.5,2.5], 
                                                                     iter_max = iter),
                                        profiler = platformprofiler)

platform2hold = PlanPlatform2Hold(graph = TbPlatformAlignGraph(goal_skew = 3, 
                                                               directions = [0.05,0.05,0.05,5,5,5], 
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

# load assets
tbot = TbTetherbot.load(os.path.join(os.path.dirname(__file__), 'data/tetherbot.pkl'))

# motion planning
_, commands, exitflag = localplanner.plan(tbot, grip_idx=1, hold_idx=3, start_state=0, goal_state=10, commands=CommandList())

commands.save(os.path.join(os.path.dirname(__file__), 'data/commands.pkl'), overwrite=True)