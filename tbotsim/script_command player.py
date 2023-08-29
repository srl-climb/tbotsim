from tbotlib import CommandList, TetherbotVisualizer, TbTetherbot

commands_path = 'C:/Users/ngkla/Desktop/Git/tbotsim/tbotsim/commands_2_2.yaml'
tbot_path = 'C:/Users/ngkla/Desktop/Git/tbotsim/tbotsim/pickle/tetherbot.pkl'

commands = CommandList.load(commands_path)
tbot: TbTetherbot = TbTetherbot.load(tbot_path)

print(tbot.W)
""" vi = TetherbotVisualizer(tbot)
done = True
while commands and vi.opened:
    vi.update()
    if done:
        command = commands.pop(0)
        command.print()
    done = command.do(tetherbot=tbot, step = 100)
    print(tbot.stability())
vi.run() """

for command in commands: 
    command.print()