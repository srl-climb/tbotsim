from tbotlib import * 
import os
from time import sleep

# Simulation timestep
dt = 0.0167

platform_feasibility = FeasibilityContainer()
platform_feasibility.add(TbWallPlatformCollisionFeasibility(0))
platform_feasibility.add(TbGripperPlatformCollisionFeasibility(distance = 0))
platform_feasibility.add(TbWrenchFeasibility(10, 6, threshold = 0.0))

workspace = TbWorkspace2(stepsize=[0.05,0.05,0.02,0,0,5], stepnum=[5,5,1,0,0,5], feasibility=platform_feasibility)

start = [34,35,33,23,15]
#start = [8,2,14,0,-1]
#start = [13,14,12,2,-1]

tbot: TbTetherbot = TbTetherbot.load(os.path.join(os.path.dirname(__file__), 'data/tetherbot.pkl'))

""" start = [34, 35, 33, 23,15]
tbot.place_all(start)
val, pose = workspace.calculate(tbot)

print(val)
print(pose)
tbot.platform.T_world = TransformMatrix(pose)
print(tbot.stability()) """

f = TbTetherArmCollisionFeasibility()
f.eval(tbot)

#_, commands, _ = localplanner.plan(tbot, 2, 13, CommandList())

vi = TetherbotVisualizer(tbot)
vi.run()

#commands.save(os.path.join(os.path.dirname(__file__), 'data/commands.pkl'), overwrite=True)