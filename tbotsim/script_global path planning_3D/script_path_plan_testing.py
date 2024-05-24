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
iter = 500000

platform2configuration = PlanPlatform2Configuration(graph = TbPlatformPoseGraph(goal_dist = 0.03, 
                                                                                goal_skew = 3, 
                                                                                directions = [0.05,0.05,0.01,2.5,0,0], #[0.05,0.05,0.01,1,1,1]
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

# load assets
tbot: TbTetherbot = TbTetherbot.load(os.path.join(os.path.dirname(__file__), 'data/tetherbot.pkl'))

grip_idx = 3
hold_idx = 14
#start = [15, 10, 14,  0, 12]
#pose = [2.93700e-01, 4.01000e-01, 2.10000e-03, 1.82850e+00, 1.62010e+00, 9.04245e+01]
#tbot.place_all(start)
#tbot.platform.T_local = TransformMatrix(pose)

""" pose = [3.45478497e-01, 1.03886540e+00, 2.37313172e-02, 1.82850000e+00, 1.62010000e+00, 8.04245000e+01]
tbot.platform.T_local = TransformMatrix(pose)
print(tbot.stability())

vi = TetherbotVisualizer(tbot)
vi.run() """
      
# motion planning
_, commands, exitflag = localplanner.plan(tbot, grip_idx=grip_idx, hold_idx=hold_idx, start_state=0, goal_state=10, commands=CommandList())

commands.save(os.path.join(os.path.dirname(__file__), 'data/commands.pkl'), overwrite=True)