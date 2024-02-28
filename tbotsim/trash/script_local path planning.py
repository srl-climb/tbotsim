from tbotlib import *
from copy import deepcopy

serializer   = SimSerializer()
simscene     = serializer.loadSimTetherbotScene('2022_11_21\\scene.vxscene')
simtetherbot = simscene.tetherbot
tbtetherbot  = simtetherbot.toTbTetherbot(0,1000)
tbtetherbot  = TbTetherbot.example()

planner      = PlanPickAndPlace()

#tbtetherbot.place_all([5,1,7,3,4])

_, commands, exitflag = planner.plan(deepcopy(tbtetherbot), 1, 6, CommandList())
print(exitflag) 

if exitflag:
   
    """ config      = serializer.loadConfig('2022_11_21\\config.vxc')
    application = SimApplication(config)
    application.add(simscene.scene)
    application.start()

    while application.running and commands:

        application.run()

        commands.pop(0).do(simtetherbot) """


    vi = TetherbotVisualizer(tbtetherbot)

    while commands and vi.opened:

        vi.update()
        commands.pop(0).do(tbtetherbot)


    