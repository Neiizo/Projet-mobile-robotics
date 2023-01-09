# Import tdmclient Notebook environment:
import numpy as np
from tdmclient import aw
import math

DEBUG = False
class MotionControl(object):
    def __init__(self, node, client, dt, SPEED_L, SPEED_R):
        self.node = node                    
        self.client = client
        self.dt = dt                        # Sampling time in [s]
        self.orientation = 0
        self.speed_conversion = 0           # Speed conversion from the thymio's speed to [mm/s]. This value is set by our calibration process
        self.kp = 2.5                     # Proportionnal gain, used for our orientation correction. This has been set experimentally
        self.speed = np.array([SPEED_L, SPEED_R])   # speed for our thymio's command, set in thymio's unit
        self.SPEED_AVG = (SPEED_L + SPEED_R)/2      # average of the speed, used for computation of the locomotion time
        self.MARGIN = 45                    # Margin for the orientation's angle's treatment, in [°]
        self.step_duration = 0
        self.turn_duration = 0
        self.angle_threshold = 7            # Threshold set experimentally, above which we do not correct the angle


        
    def motors(self, left, right):
        #####################################################
        # left  : desired speed for the left motor
        # right :  desired speed for the right motor
        #####
        # Sets the desired speed for the thymio
        #####################################################
        v =  {
            "motor.left.target": [left],
            "motor.right.target": [right],
        }
        aw(self.node.set_variables(v))



    def correct_orientation(self, vis_dir):
        #####################################################
        # vis_dir : angle of the thymio from the camera's perspective
        #####
        # checks the direction (angle) from the camera, compute the orientation of the thymio
        #####################################################
        if((vis_dir > 90 - self.MARGIN) & (vis_dir < 90 + self.MARGIN)):
            self.orientation = 1
            return self.orientation # a enlever une fois que le reste sera dans le même fichier
        elif((vis_dir > 180 - self.MARGIN) & (vis_dir < 180 + self.MARGIN)):
            self.orientation = 2
            return self.orientation # a enlever une fois que le reste sera dans le même fichier
        elif((vis_dir > 270 - self.MARGIN) & (vis_dir < 270 + self.MARGIN)):
            self.orientation= 3
            return self.orientation # a enlever une fois que le reste sera dans le même fichier
        else:
            self.orientation = 0
            return self.orientation # a enlever une fois que le reste sera dans le même fichier



    def get_turn(self, x,y, orientation):
        #####################################################
        # x             : futur coordinate on the grid 
        # y             : futur coordinate on the grid 
        # orientation   : actual orientation of the robot
        #####
        # computes the turn neccessary to reach the new orientation from the actual orientation
        #####################################################
        if((x==1) & (y==0)):
            orientation_des = 0
        elif((x==0) & (y==-1)):
            orientation_des = 1
        elif((x==-1) & (y==0)):
            orientation_des = 2
        elif((x==0) & (y==1)):
            orientation_des = 3
        else:
            orientation_des = orientation
        turn_orientation = orientation_des - orientation
        if abs(turn_orientation) > 2:
            return (turn_orientation) - 4*np.sign(turn_orientation)
        else:
            return turn_orientation



    def kalman_adjust(self, next_target_x, next_target_y, kalman_pos, angle, isCamOn):  
        #####################################################
        # next_target_x : coordinate of the next cell the thymio needs to reach, on the x axis, in terms of cells.
        # next_target_y : coordinate of the next cell the thymio needs to reach, on the y axis, in terms of cells.
        # kalman_pos    : estimated position of the thymio from the Kalman's filter, in [mm]
        # angle         : angle of the thymio from the camera's perspective
        #####
        # computes required angle for the thymio to reach the next cells, and compares its actual angle. A proportionnal controller 
        # will then be used to correct this and redirect the thymio
        #####################################################
        if(not isCamOn):
            delta_x = next_target_x - kalman_pos[0]
            delta_y = next_target_y - kalman_pos[1]
            return delta_x, delta_y
        else:
            delta_x = next_target_x - kalman_pos[0]
            delta_y = next_target_y - kalman_pos[1]
            desired_angle = (math.degrees(math.atan2(delta_y,  delta_x))) % 360
            adjust_turn = 0
            delta_angle = desired_angle - angle
            
            if(np.abs(delta_angle) > 270):
                delta_angle = delta_angle - np.sign(delta_angle)*360
            
            if(np.abs(delta_angle) > self.angle_threshold): 
                adjust_turn = (int)(np.round(self.kp * delta_angle))
            if(adjust_turn != 0):
                correcting_speed_x = self.speed[0] - adjust_turn
                correcting_speed_y = self.speed[1] + adjust_turn
                self.motors(correcting_speed_x, correcting_speed_y)
                aw(self.client.sleep(2*self.dt))
                self.motors(self.speed[0], self.speed[1])
            return delta_x, delta_y

    def adjust_angle(self, vision):
        #####################################################
        # vision : used to get vision.thymio_orientation which gives the angle of the thymio
        #####
        #  computes the error angle between the orientation desired from motion control and the actual angle of the robot 
        #  according to the camera
        #####################################################
        vision.update_coordinates()  
        turn_speed_l = 0
        turn_speed_r = 0
        if(vision.thymio == True): # faire que si on voit le thymio
            angle = vision.thymio_orientation # angle of the robot
            if((self.orientation == 0) & (angle > 270)):  
                adjust_angle = angle - 360
            else:
                adjust_angle = angle - self.orientation*90 
            # if(DEBUG == True):
            #     print("adjust angle before moving :", adjust_angle)   
            turn_speed_l = int(self.speed[0]*np.sign(adjust_angle))
            turn_speed_r = int(-self.speed[1]*np.sign(adjust_angle))
            self.motors(turn_speed_l, turn_speed_r)
            aw(self.client.sleep(5*abs(adjust_angle)//180)) # a changer pour un check sur time ?
            self.motors(0, 0)


    def robot_turn(self, signturn):  
        #####################################################
        # signturn : sign that tells if the thymio should turn left or right. (-) for left, (+) for right
        #####
        #  depending on the sign, makes the robot turn left of right with respect to the center of the cell 
        # (not the center of rotation of the thymio)
        #####################################################
        turn_duration2 = 55 / (self.SPEED_AVG * self.speed_conversion)
        turn_duration1 = 20 / (self.SPEED_AVG * self.speed_conversion)
        if signturn > 0:
            self.motors(20, 100)
            aw(self.client.sleep(turn_duration2)) #changer ca pour le tour
            self.motors(-120, 50)
            aw(self.client.sleep(turn_duration2)) #changer ca pour le tour
            self.motors(-60, -60)
            aw(self.client.sleep(turn_duration1)) #changer ca pour le tour
            self.motors(0, 0)
        elif signturn < 0:
            self.motors(100, 20)
            aw(self.client.sleep(turn_duration2)) #changer ca pour le tour
            self.motors(50, -120)
            aw(self.client.sleep(turn_duration2)) #changer ca pour le tour
            self.motors(-60, -60)
            aw(self.client.sleep(turn_duration1)) #changer ca pour le tour
            self.motors(0, 0)
        else:
            self.motors(0, 0)

