import os
from tbotlib import *
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull

PGF = False

if PGF:
    import matplotlib as mpl
    # Use the pgf backend (must be set before pyplot imported)
    mpl.use('pgf')
    plt.rcParams.update({'font.size': 8})

def plot_workspace(workspace: TbWorkspace2) -> plt.Figure:

    cm = 1/2.54  # centimeters in inches
    fig, ax = plt.subplots(1)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_xlim(0.15,1.35)
    ax.set_ylim(0.15,1.35)

    try:
        ws = workspace._poses[workspace._vals.astype(bool),:2]
        print(ws.size)
        #hull = ConvexHull(ws)
        #ax.fill(ws[hull.vertices,0], ws[hull.vertices,1], 'k', facecolor = (0, 0, 0, 0.1), edgecolor = (0, 0, 0, 1))
        ax.scatter(*ws.T, s=0.6)
    except:
        pass
    
    fig.set_figheight(fig.get_figheight()/fig.get_figwidth()*6*cm)
    fig.set_figwidth(4*cm)

    return fig
    
# load assets
tbot: TbTetherbot = TbTetherbot.load(os.path.join(os.path.dirname(__file__), 'data/tetherbot.pkl'))
pose = TransformMatrix(np.hstack(((tbot.grippers[2].anchorpoint.T_world.r + tbot.grippers[3].anchorpoint.T_world.r)/2, (0,0,90))))

W  = TbElliptoidWrenchSet([25,25,25,2.5,2.5,2.5])
F1 = TbTetherForceSet(0, 250, parent = tbot)
F2 = TbTetherGripperForceSet(0, 250, PanorelGripperForceModel(240, 180), parent = tbot)
F3 = TbTetherForceSet(0, 90, parent = tbot)

stepsize = [0.05,0.05,0,0,0,0]
stepnum  = [15,15,0,0,0,0]
stepsize = [0.025,0.025,0,0,0,0]
stepnum  = [30,30,0,0,0,0]
#stepsize = [0.05,0.05,0,0,0,0]
#stepnum  = [3,3,0,0,0,0]

tbot.W = W
tbot.F = F1
workspace1 = TbWorkspace2(stepsize, stepnum, mode = '')
workspace1.feasiblity = TbWrenchFeasibility(solver = HyperPlaneShifting())
#workspace1.feasiblity = TbWrenchFeasibility(solver = QuickHull())
tic()
workspace1.calculate(tbot, init_pose=pose)
toc()

tbot.W = W
tbot.F = F2
workspace2 = TbWorkspace2(stepsize, stepnum, mode = '')
workspace2.feasiblity = TbWrenchFeasibility(solver = QuickHull())
tic()
workspace2.calculate(tbot, init_pose=pose)
toc()

tbot.W = W
tbot.F = F3
workspace3 = TbWorkspace2(stepsize, stepnum, mode = '')
workspace3.feasiblity = TbWrenchFeasibility(solver = HyperPlaneShifting())
#workspace3.feasiblity = TbWrenchFeasibility(solver = QuickHull())
tic()
workspace3.calculate(tbot, init_pose=pose)
toc()

figs = []
figs.append(plot_workspace(workspace1))
figs.append(plot_workspace(workspace2))
figs.append(plot_workspace(workspace3))

if PGF:
    for i, fig in zip(range(len(figs)), figs):
        fig.savefig(os.path.join(os.path.dirname(__file__), 'data/workspace_analysis_data_' + str(i) + '.pgf'), format='pgf', bbox_inches = "tight", pad_inches=0.0)
else:
    plt.show()