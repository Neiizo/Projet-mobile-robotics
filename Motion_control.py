# Import tdmclient Notebook environment:
import numpy as np
from tdmclient import aw
import math

#faire une classe pour stocker toutes les variables, et éviter les appels avec milles arguments

def correct_orientation(orientation):
    if((orientation > 60) & (orientation < 120)): #a mettre avec des defines et une margin
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
    if((x==1) & (y==0)):
        dir = 0
    elif((x==0) & (y==-1)):
        dir = 1
    elif((x==-1) & (y==0)):
        dir = 2
    elif((x==0) & (y==1)):
        dir = 3
    else:
        dir = orientation
    new_orientation = dir - orientation
    if abs(new_orientation)>2:
        return (new_orientation)-4*np.sign(new_orientation)
    else:
        return new_orientation

def kalman_adjust(kp, next_target_x, next_target_y, dx,dy,kalman_pos,dir,orientation, half_cell_width):
   delta_x = next_target_x - kalman_pos[0]
   delta_y = next_target_y - kalman_pos[1]
   desired_angle = (math.degrees(math.atan2(delta_y,  delta_x))) % 360
   adjust_turn = 0
   delta_angle = desired_angle - orientation
   
   if(delta_angle > 270):
      delta_angle = delta_angle - 360
   if(delta_angle < -270):
      delta_angle = delta_angle + 360
   
   if(np.abs(delta_angle) > 7): 
      adjust_turn = (int)(np.round(kp * delta_angle))
   return delta_x, delta_y, adjust_turn



def robot_turn(signturn, SPEED , speed_conversion, node, client):
    turn_duration2 = 55 / (SPEED * speed_conversion)
    turn_duration1 = 20 / (SPEED * speed_conversion)
    if signturn > 0:
        motors(node, 20, 100)
        aw(client.sleep(turn_duration2)) #changer ca pour le tour
        motors(node, -120, 50)
        aw(client.sleep(turn_duration2)) #changer ca pour le tour
        motors(node, -60, -60)
        aw(client.sleep(turn_duration1)) #changer ca pour le tour
        motors(node, 0, 0)
    elif signturn < 0:
        motors(node, 100, 20)
        aw(client.sleep(turn_duration2)) #changer ca pour le tour
        motors(node,50, -120)
        aw(client.sleep(turn_duration2)) #changer ca pour le tour
        motors(node, -60, -60)
        aw(client.sleep(turn_duration1)) #changer ca pour le tour
        motors(node, 0, 0)
    else:
        motors(node, 0, 0)


#motors(0, 0, verbose=True)   
