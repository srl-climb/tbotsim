import os
from tbotlib import *

# Feasiblity checks
arm_feasibility = FeasibilityContainer()
arm_feasibility.add(TbJointLimitFeasibility())
arm_feasibility.add(TbTetherArmCollisionFeasibility(0.02))

platform_feasibility = FeasibilityContainer()
platform_feasibility.add(TbWallPlatformCollisionFeasibility(0))
platform_feasibility.add(TbGripperPlatformCollisionFeasibility(distance = 0))
platform_feasibility.add(TbWrenchFeasibility(10, 6, threshold = 0.00))


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

workspace = TbWorkspace2(stepsize=[0.02,0.02,0.02,0,0,5], stepnum=[5,5,1,0,0,5], feasibility=platform_feasibility)

# Local path planner objects
iter = 50000
platform2configuration = PlanPlatform2Configuration(graph = TbPlatformPoseGraph(feasiblity = platform_feasibility,
                                                                                goal_dist = 0.03, 
                                                                                goal_skew = 3, 
                                                                                directions =  [0.04,0.04,0.04,4,4,4],
                                                                                iter_max = iter),
                                                    profiler = platformprofiler,
                                                    workspace = workspace)

platform2gripper = PlanPlatform2Gripper(graph = TbPlatformAlignGraph(feasiblity = platform_feasibility,
                                                                     goal_skew = 3, 
                                                                     directions = [0.02,0.02,0.02,2,2,2],
                                                                     iter_max = iter),
                                        profiler = platformprofiler)

platform2hold = PlanPlatform2Hold(graph = TbPlatformAlignGraph(feasiblity = platform_feasibility,
                                                               goal_skew = 3, 
                                                               directions = [0.02,0.02,0.02,2,2,2],
                                                               iter_max = iter),
                                  profiler = platformprofiler)

arm2pose = PlanArm2Pose(graph = TbArmPoseGraph(feasiblity = arm_feasibility,
                                               goal_dist = 0.072,
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

start = [34 ,35 ,33 ,23, 21] #20
start = [13 ,14 ,12 ,2, 0]
pose =  [ 1.4854,  1.4192, -0.1583, -0.0297, 17.0559 ,-0.1105] # (19.925083258658336, True) (-0.14662720471057597, False)
tbot.platform.arm.qs = [0.45595343, 0.646307  , 0.04845193]
grip_idx = 2
hold_idx = 33

tbot.place_all(start)

feasibility, pose = workspace.calculate(tbot)
if not feasibility:
    print(feasibility)
    exit()
else:
    print(pose)


tbot.platform.T_local = TransformMatrix(pose)
print('Stability: ' ,tbot.stability())

vi = TetherbotVisualizer(tbot)
vi.run()

#commands = localplanner.plan(tbot, grip_idx, hold_idx, commands=CommandList())[1]
#commands.save(os.path.join(os.path.dirname(__file__), 'data/commands.pkl'), overwrite=True)

""" previous stance:      [34 35 32 17 20]
current stance:       [34 35 -1 17 20]
platform pose:        [ 1.1006  1.8273 -0.0488  0.0309 17.0564  0.1146]
paltform stability:   (6.931709947416442, True)
arm state:            [-3.03060165  1.03388799  0.0402924 ]
picking gripper:  2
TbPlatformPoseGraph suceeded after 4120
TbPlatformAlignGraph suceeded after 73
TbArmPoseGraph suceeded after 8
TbArmPoseGraph suceeded after 2
TbArmPoseGraph suceeded after 2
current state
previous stance:      [34 35 -1 17 20]
current stance:       [34 35 33 17 20]
platform pose:        [ 1.4471  1.6587 -0.1664  3.352  28.4863  6.8485]
paltform stability:   (-0.21010008365452243, False)
arm state:            [-2.66213177  1.0292869  -0.05320175]
placing gripper:  2  on hold:  33
TbPlatformAlignGraph suceeded after 2
TbArmPoseGraph suceeded after 8
TbArmPoseGraph suceeded after 1
TbArmPoseGraph suceeded after 1
current state """