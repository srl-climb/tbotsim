import os
from tbotlib import *
import numpy as np

tbot: TbTetherbot = TbTetherbot.load(os.path.join(os.path.dirname(__file__), 'data/tetherbot.pkl'))
commands = CommandList.load(os.path.join(os.path.dirname(__file__), 'data/commands.pkl'))


vi = TetherbotVisualizer(tbot)
done = True
while commands and vi.opened:
    vi.update()

    if done:
        if commands:
            command = commands.pop(0)
            command.print()
            print(tbot.stability())
        else:
            break

    done = command.do(tetherbot=tbot)
print(tbot.stability())