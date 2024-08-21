from tbotlib import * 
import os

# Feasiblity checks
arm_feasibility = FeasibilityContainer()
arm_feasibility.add(TbJointLimitFeasibility())
arm_feasibility.add(TbTetherArmCollisionFeasibility(distance = 0.02))

platform_feasibility = FeasibilityContainer()
platform_feasibility.add(TbWallPlatformCollisionFeasibility(distance = 0))
platform_feasibility.add(TbGripperPlatformCollisionFeasibility(distance = 0))
platform_feasibility.add(TbWrenchFeasibility(threshold = 1.00))

# Simulation timestep
dt = 0.0167

# Smoothing algorithm
platformsmoother = BsplineSmoother(0.003, k=3)
platformsmoother = NotSmoother()
armsmoother = BsplineSmoother(0.005, k=3)

# workspace
workspace = TbWorkspace2(stepsize=[0.02,0.02,0.02,0,0,5], stepnum=[5,5,1,0,0,5], feasibility=platform_feasibility)

# Profile generators
platformprofiler = SlicedProfile6(a_t = [0.05,10], 
                                  d_t = [0.05,10], 
                                  v_t = [0.05,1], 
                                  dt = dt, 
                                  smoother = platformsmoother)
#platformprofiler = FastProfile()
armprofiler = Profile3(a_t = 0.05, 
                       d_t = 0.05,
                       v_t = 0.1, 
                       dt = dt, 
                       smoother = armsmoother)

# Local path planner objects
iter = 50000
platform2configuration = PlanPlatform2Configuration(graph = TbPlatformPoseGraph(feasiblity = platform_feasibility,
                                                                                goal_dist = 0.03, 
                                                                                goal_skew = 3, 
                                                                                directions =  [0.02,0.02,0.02,2,2,2],
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
                                               directions = [0.05,0.05,0.02], 
                                               iter_max = iter),
                        profiler = armprofiler)

localplanner = PlanPickAndPlace2(
                    platform2configuration = platform2configuration,
                    platform2gripper       = platform2gripper, 
                    platform2hold          = platform2hold, 
                    arm2pose               = arm2pose)

# load assets
tbot: TbTetherbot = TbTetherbot.load(os.path.join(os.path.dirname(__file__), 'data/tetherbot.pkl'))

""" tbot.place_all([8,2,14,1,12])
tbot.platform.T_world = TransformMatrix([ 0.516,  0.486, 0.067, -0.9, -3.85, 94.58]) #(3.8750987365127605, True)
#Move platform to: x = 0.47, y = 0.401, z = 0.068, theta_x = 5.11, theta_y = 2.59, theta_z = 68.21
print(tbot.stability())

commands = platform2configuration.plan(tbot, grip_idx=0, commands=CommandList())[1]

print(commands.print())
print(commands[-1]._targetposes[-1].decompose())

for pose in commands[-1]._targetposes:
    tbot.platform.T_world = pose
    s = tbot.stability()[0]
    print(s) """

print('done')

f = StanceGeometricFeasibility(min_width = 0.55, max_width = 1.51)
tbot.place_all([15,16,14,1,12])
print(f.eval([15,16,14,1,12], tbot,None))
#vi = TetherbotVisualizer(tbot)
#vi.run()