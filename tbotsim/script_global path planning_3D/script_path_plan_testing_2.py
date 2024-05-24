from tbotlib import * 
import os
from time import sleep

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
platform2configuration = PlanPlatform2Configuration(graph = TbPlatformPoseGraph(goal_dist = 0.04, 
                                                                                goal_skew = 2.5, 
                                                                                directions = [0.05,0.05,0.01,2.5,2.5,2.5],
                                                                                iter_max = 50000),
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

workspace = TbWorkspace(padding = [-0.1,-0.1,0,-180,-180,-90], #45
                        mode_2d = True,
                        scale = [0.1,0.1,99,99,99,2], #0.2, 45
                        mode = 'first', threshold = 1)

################################################################

stance_start = [29 ,23 ,15 ,17 ,16]


#[15 , 9 ,14  ,8 ,12]

tbot: TbTetherbot = TbTetherbot.load(os.path.join(os.path.dirname(__file__), 'data/tetherbot.pkl'))
tbot.place_all(stance_start)

""" flag, pose = workspace.calculate(tbot)
print(pose)
print('exitflag: ', flag) """

#pose = [1.19223773  ,1.533265 ,  -0.14453648 , 6.69684826 ,12.46266395, 38.84084504] # 2.314385781632681
pose = [ 8.98400e-01 , 5.06900e-01 ,-3.59000e-02  ,1.60000e-03, -9.37800e-01,  7.33648e+01]#start pose
""" pose= [ 6.55532947e-01,  1.01791433e+00, -2.70591613e-02 , 6.50228118e+00,
  2.32784509e+00,  5.26758715e+01] """ # 3.2368960981734203
#pose = [0.79826082 , 1.53208463, -0.03725219,  2.42095592 ,10.46974314, 25.09764226]
#pose = [  1.24077501,   2.05640888 , -0.15305413 , -8.14059004,  16.00161291,-23.42507092]

print(pose)
print(tbot.B_local)
tbot.platform.T_local = TransformMatrix(pose)
tbot.platform.arm.qs = [-0.16359082 , 0.97305686 , 0.03128596]



print(tbot.stability())

""" print(workspace.calculate(tbot))
workspace.debug_plot() """

#_, commands, _ = localplanner.plan(tbot, 2, 13, CommandList())


vi = TetherbotVisualizer(tbot)
vi.run()

#commands.save(os.path.join(os.path.dirname(__file__), 'data/commands.pkl'), overwrite=True)