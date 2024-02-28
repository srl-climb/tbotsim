from tbotlib import yaml2planner, TbTetherbot, TetherbotVisualizer, CommandList, hyperRectangle, tic, toc, TransformMatrix
import numpy as np
from time import sleep

tbot_desc_file = 'C:/Users/ngkla/Desktop/Git/tbotsim/tbotsim/pickle/tetherbot.pkl'
planner_config_file = 'C:/Users/ngkla/Desktop/Git/tbotsim/tbotsim/planner.yaml'

simulation_dt, platform2pose, platform2configuration, arm2pose, local_planner, global_planner  = yaml2planner(planner_config_file)
tbot: TbTetherbot = TbTetherbot.load(tbot_desc_file)


tbot.platform.arm.qs = tbot.platform.arm.ivk(TransformMatrix([0.6,0.7,0.1]))
tbot.remove_all_geometries()
_, commands, exitflag = arm2pose.plan(tbot, TransformMatrix([0.6,0.2,0.1]), CommandList())

if exitflag is None:
    exit()

tbot: TbTetherbot = TbTetherbot.load(tbot_desc_file)
vi = TetherbotVisualizer(tbot)
done = True
while vi.opened:
    vi.update()
    if done:
        if commands:
            command = commands.pop(0)
        else:
            break
        command.print()
    done = command.do(tetherbot=tbot, step = 20)

vi.run()


