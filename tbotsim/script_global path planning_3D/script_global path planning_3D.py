from tbotlib import * 
import os

start = [8,2,14,0,12]
goal = [34, 35, 33, 23, 21] 

""" start = [15,16,14,4,2]
goal = [21,16,14,4,2]  """

# Feasiblity checks
arm_feasibility = FeasibilityContainer()
arm_feasibility.add(TbJointLimitFeasibility())
arm_feasibility.add(TbTetherArmCollisionFeasibility(distance = 0.02))

platform_feasibility = FeasibilityContainer()
platform_feasibility.add(TbWallPlatformCollisionFeasibility(distance = 0))
platform_feasibility.add(TbGripperPlatformCollisionFeasibility(distance = 0))
platform_feasibility.add(TbWrenchFeasibility(threshold = 2.00))

# Simulation timestep
dt = 0.0167

# Smoothing algorithm
platformsmoother = BsplineSmoother(0.003, k=3)
armsmoother = BsplineSmoother(0.005, k=3)

# workspace
workspace = TbWorkspace2(stepsize=[0.02,0.02,0.02,0,0,5], stepnum=[5,5,1,0,0,5], feasibility=platform_feasibility)

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
                                               iter_max = 100000),
                        profiler = armprofiler)

localplanner = PlanPickAndPlace2(
                    platform2configuration = platform2configuration,
                    platform2gripper       = platform2gripper, 
                    platform2hold          = platform2hold, 
                    arm2pose               = arm2pose)

# Global path planner objects

platform2gripper = PlanPlatform2Gripper(graph = TbPlatformAlignGraph(feasiblity = platform_feasibility,
                                                                     goal_skew = 3, 
                                                                     directions = [0.02,0.02,0.02,2,2,2],
                                                                     iter_max = 150),
                                        profiler = platformprofiler)

platform2hold = PlanPlatform2Hold(graph = TbPlatformAlignGraph(feasiblity = platform_feasibility,
                                                               goal_skew = 3, 
                                                               directions = [0.02,0.02,0.02,2,2,2],
                                                               iter_max = 150),
                                  profiler = platformprofiler)

step_feasiblity = FeasibilityContainer()
step_feasiblity.add(StepDistanceFeasibility(distance=1.05))
step_feasiblity.add(StepPathFeasibility(platform2gripper = platform2gripper, platform2hold = platform2hold))
stance_feasbility = FeasibilityContainer()
stance_feasbility.add(StanceGeometricFeasibility(min_width = 0.55, max_width = 1.55))
stance_feasbility.add(StanceWrenchFeasiblity(workspace = workspace))

stepplanner = GlobalPlanner(graph = TbStepGraph(heuristic = StanceDisplacementMetric(),
                                                cost = ConstantMetric(0.025),
                                                goal_dist = 0.01,
                                                step_feasibility = step_feasiblity,
                                                stance_feasibility = stance_feasbility,
                                                iter_max = 5000),
                            localplanner = localplanner)


# load assets
tbot: TbTetherbot = TbTetherbot.load(os.path.join(os.path.dirname(__file__), 'data/tetherbot.pkl'))

tbot.place_all(start)

feasibility, pose = workspace.calculate(tbot)

if not feasibility:
    print(feasibility)
    exit()
else:
    print(pose)
    tbot.platform.T_local = TransformMatrix(pose)

profiler = Profiler()
profiler.on()

commands = stepplanner.plan(tbot, start, goal, commands=CommandList())[1]

profiler.off()
profiler.print()

#commands.save(os.path.join(os.path.dirname(__file__), 'data/commands.pkl'), overwrite=True)