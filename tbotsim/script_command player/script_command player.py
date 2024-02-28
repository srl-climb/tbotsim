from tbotlib import CommandList, TetherbotVisualizer, TbTetherbot
import os

absolute_path = os.path.dirname(__file__)

commands_path = os.path.join(absolute_path, 'commands/commands2.yaml')
tbot_path = os.path.join(absolute_path, 'pickle/tetherbot.pkl')

commands = CommandList.load(commands_path)
tbot: TbTetherbot = TbTetherbot.load(tbot_path)

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
    done = command.do(tetherbot=tbot, step = 200)
    print(tbot.l)
vi.run()

for command in commands: 
    command.print()