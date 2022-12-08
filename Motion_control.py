# Import tdmclient Notebook environment:
import tdmclient.notebook
import math
import numpy as np


def get_turn(x,y,orientation):
    if x==1 and y==0:
        dir = 0
    elif x==0 and y==-1:
        dir = 1
    elif x==-1 and y==0:
        dir = 2
    elif x==0 and y==1:
        dir = 3
    else:
        dir = orientation
    new_orientation = dir - orientation
    if abs(new_orientation)>2:
        return (new_orientation)-4*np.sign(new_orientation)
    else:
        return new_orientation

def kalman_adjust(dx,dy,kalman_pos_x,kalman_pos_y,orientation):
   x_mm = dx*125-kalman_pos_x
   y_mm = dy*125-kalman_pos_y
   if (orientation == 0 or orientation == 2):
      turn = get_turn(0,np.sign(y_mm),orientation)
      adjust = turn*(y_mm)
   else:
      turn = get_turn(np.sign(x_mm),0,orientation)
      adjust = turn*(x_mm)
   print("adjust = ",adjust)
   return adjust



motors(0, 0, verbose=True)   