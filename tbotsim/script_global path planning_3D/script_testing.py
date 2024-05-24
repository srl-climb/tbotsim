from tbotlib import * 
import os
import numpy as np

workspace = TbWorkspace(padding = [-0.1,-0.1,0,-180,-180,-90], #45
                        mode_2d = True,
                        scale = [0.1,0.1,99,99,99,10], #0.2, 45
                        mode = 'first')
tbot: TbTetherbot = TbTetherbot.load(os.path.join(os.path.dirname(__file__), 'data/tetherbot.pkl'))
pose = [ 1.92652385e+00,  2.00000000e+00,  -0.52193727,  7.16197244e+00, -0.00000000e+00,  9.00000000e+01]
tbot.platform.T_world = TransformMatrix(pose)

#tbot.tension(4, False)
#start    = [34, 35, 33, 23, 21]
start    = [28, 29 ,27, 23 ,14] #0 to 23
tbot.place_all(start)

print(tbot.filter_holds(3))

_, pose = workspace.calculate(tbot)


tbot.platform.T_world = TransformMatrix(pose)
print(tbot.stability())


print('platform pose:       ', tbot.platform.T_world.decompose())               
print('paltform stability:  ', tbot.stability())
print('arm state:           ', tbot.platform.arm.qs)
print(tbot.A_world)

import networkx as nx

graph = nx.Graph()
graph.add_node(1, h=11)
graph.nodes[1]['h'] = 12
print(graph.nodes[1]['h'])



