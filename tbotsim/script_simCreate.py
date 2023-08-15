from tbotlib import *
import numpy as np

A   = [[0.5,0,2.15], [0.5,0.5,2.15], [0.5,-0.5,2.15], [-0.5,0.5,2.15],[-0.5,-0.5,2.15], # Positions of the grippable points
           [0.8,0,2.15], [0.8,0.5,2.15], [0.8,-0.5,2.15], [-0.2,0.5,2.15],[-0.2,-0.5,2.15]]
A_i = [0,1,2,3,4]                                                                       # Initial gripper position
T_0 = [-0.0,0,2.15]                                                                     # Inital platform pose
M   = [[0,0],[0,1],[1,2],[1,3],[3,4],[3,5],[4,6],[4,7],[2,8],[2,9]]                     # Mapping (GripperIdx, PlatformIdx), length decides number of tethers 
                                                                                        # NOTE: Platform index must ascent from 0,1,2,... to n
W   = hyperRectangle(np.array([5,5,5,0.5,0.5,0.5]), np.array([-5,-5,-5,-0.5,-0.5,-0.5]))
#aorder = [0,1,2,4,3]    
aorder = [0,1,3,4,2]
simCreate(A,A_i,T_0,M,aorder,W)