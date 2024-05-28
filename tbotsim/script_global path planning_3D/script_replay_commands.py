import os
from tbotlib import *
import numpy as np
from time import sleep

tbot: TbTetherbot = TbTetherbot.load(os.path.join(os.path.dirname(__file__), 'data/tetherbot.pkl'))
commands = CommandList.load(os.path.join(os.path.dirname(__file__), 'data/commands.pkl'))

#tbot.place_all( [15, 10, 14 , 3, 12])

vi = TetherbotVisualizer(tbot)

#sleep(5)

done = True
counter = 0
step = 40
while commands and vi.opened:
    
    
    vi.update()

    if done:
        if commands:
            command = commands.pop(0)
            command.print()
            
        else:
            break
    print(tbot.stability())
    done = command.do(tetherbot = tbot, step = step)
