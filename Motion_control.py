# Import tdmclient Notebook environment:
import numpy as np
from tdmclient import aw



def correct_orientation(orientation):
    if((orientation > 60) & (orientation < 120)):
        return 1
    elif((orientation > 150) & (orientation < 210)):
        return 2
    elif((orientation > 240) & (orientation < 300)): #mettre define
        return 3
    else:
        return 0


def motors(node, left, right):
    v =  {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }
    aw(node.set_variables(v))

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
      adjust_turn = turn*(y_mm)
      adjust_speed = -x_mm*np.sign(orientation-1)
   else:
      turn = get_turn(np.sign(x_mm),0,orientation)
      adjust_turn = turn*(x_mm)
      adjust_speed = -y_mm*np.sign(orientation-2)
   print("adjust turn = ",adjust_turn, "adjust speed = ", adjust_speed)
   return adjust_turn,adjust_speed



#motors(0, 0, verbose=True)   