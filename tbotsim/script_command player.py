from tbotlib import CommandList, TetherbotVisualizer, TbTetherbot

commands_path = '/home/climb/ros2_ws/commands/commands0.pkl'
tbot_path = '/home/climb/ros2_ws/src/tbotros_description/tbotros_description/desc/tetherbot/tetherbot.pkl'
commands_path = 'C:/Users/ngkla/Desktop/Git/tbotsim/tbotsim/commands2.yaml'
tbot_path = 'C:/Users/ngkla/Desktop/Git/tbotsim/tbotsim/pickle/tetherbot.pkl'

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