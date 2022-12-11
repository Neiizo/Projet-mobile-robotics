# Import tdmclient Notebook environment:
import numpy as np
from tdmclient import aw
import math
import time
# import local_nav as ln
from djikstra import djikstra_algo,create_plot
from kalmanfilter import KalmanFilter


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
        self


    # def start_locomotion(self, half_cell_width, vision):
    #     self.step_duration = half_cell_width*2 / (self.SPEED_AVG * self.speed_conversion)  # a mettre dans la classe
    #     self.turn_duration = 98 / (self.SPEED_AVG * self.speed_conversion)
    #     # A TOUT METTRE DANS MOTION CONTROL
    #     restart = True
    #     jump = False
    #     jump_x, jump_y = 0,0
    #     ajdust = 0
    #     index = 0
    #     MARGIN = 30
    #     while (restart == True):
    #         restart = False
    #         vision.update_coordinates()
    #         if(DEBUG == True):
    #             print("thymio real pos : ", vision.thymio_real_pos)
    #             print("thymio pos in grid : ", vision.thymio_position)
    #             print("goal pos : ", vision.goal_position)
    #             print("thymio angle = :", vision.thymio_orientation)
    #         vision.grid[vision.thymio_position[1]][vision.thymio_position[0]] = 0
    #         vision.grid[vision.goal_position[1]][vision.goal_position[0]] = 0
    #         shortest_path = djikstra_algo(vision.grid.T, vision.thymio_position, vision.goal_position)
    #         if(DEBUG == True):
    #             print(shortest_path)
    #         KF = KalmanFilter(self.dt, vision.thymio_real_pos, mc..speed_conversion, Q_cam, Q_gnd, R_cam, R_gnd)  # we initialize the filter
    #         self.orientation = self.correct_orientation(vision.thymio_orientation)
    #         x = vision.thymio_position[0]
    #         y = vision.thymio_position[1]
    #         turn_speed = np.array([0, 0])
    #         for dx,dy in np.transpose(shortest_path):
    #             if jump:
    #                 x = dx      #actualize the coordinates of the robot
    #                 y = dy      #actualize the coordinates of the robot
    #                 index += 1
    #                 if jump_x == dx and jump_y == dy:
    #                     jump = False
    #                 continue  
    #             vision.update_coordinates()
    #             if (not vision.same_goal):
    #                 restart = True
    #                 break
    #             turn = self.get_turn(dx-x,dy-y,self.orientation)
    #             self.orientation = (self.orientation + turn)%4
    #             for i in range(abs(turn)):
    #                 self.robot_turn(np.sign(turn))
    #             self.adjust_angle(vision)
    #             if (((dx-x)!=0) | ((dy-y)!=0)):
    #                 local = ln.obstacle_detect(self.node)
    #                 if local:
    #                     if(DEBUG == True):
    #                         print("obstacle",len(shortest_path[1]))
    #                     if (index+2) > len(shortest_path[1]):
    #                         jump,jump_x,jump_y = ln.obstacle_avoid(vision, self, x, y, shortest_path, len(shortest_path[1]), self.node, self.client)
    #                     else:
    #                         jump,jump_x,jump_y = ln.obstacle_avoid(vision, self, x, y, shortest_path, index, self.node, self.client)
    #                 else:   
    #                     if (((dx-x)!=0) | ((dy-y)!=0)):
    #                         step_done = False
    #                         start_move = time.time()
    #                         self.motors(self.speed[0], self.speed[1])
    #                         temp = 0
    #                         next_target_x = dx *half_cell_width*2 + half_cell_width
    #                         next_target_y = (vision.rows - 1 - dy) *half_cell_width*2 + half_cell_width
                            
    #                         while (step_done != True):  
    #                             vision.update_coordinates()
    #                             kalman_pos= KF.filter(vision.thymio, vision.thymio_real_pos, self.orientation, self.speed, vision.thymio_orientation, 0, 0, GND_THRESHOLD)  # A CHANGER AVEC LES VRAIES VALEURS
    #                             # if(DEBUG == True):
    #                             #     print("estimated position ", kalman_pos)
    #                             #     print("position from camera ", vision.thymio_real_pos)
    #                             delta_x, delta_y= self.kalman_adjust(next_target_x, next_target_y, kalman_pos, vision.thymio_orientation)
    #                             # if(DEBUG == True):
    #                             #     print((correcting_speed_x, correcting_speed_y))
    #                             current = time.time()
    #                             temp = current - start_move
    #                             if((np.abs(delta_x) < MARGIN) & (np.abs(delta_y) < MARGIN)):
    #                                 step_done = True
    #                                 self.motors(0, 0) 
    #                             elif(temp > self.step_duration):
    #                                 step_done = True  
    #                                 self.motors(0, 0)
                        
    #                         self.adjust_angle(vision)

    #             x = dx      #actualize the coordinates of the robot
    #             y = dy      #actualize the coordinates of the robot
    #         index += 1


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
        # x             :
        # y             :
        # orientation   :
        #####
        # computes the turn 
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



    def kalman_adjust(self, next_target_x, next_target_y, kalman_pos, angle):  
        #####################################################
        # next_target_x : coordinate of the next cell the thymio needs to reach, on the x axis, in terms of cells.
        # next_target_y : coordinate of the next cell the thymio needs to reach, on the y axis, in terms of cells.
        # kalman_pos    : estimated position of the thymio from the Kalman's filter, in [mm]
        # angle         : angle of the thymio from the camera's perspective
        #####
        # computes required angle for the thymio to reach the next cells, and compares its actual angle. A proportionnal controller 
        # will then be used to correct this and redirect the thymio
        #####################################################
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
        # signturn : sign that tells if the thymio should turn left or right. (-) for left, (+) for right A VERIFIER
        #####
        #  
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

