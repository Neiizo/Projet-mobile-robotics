import numpy as np

# Initialisation made with [name of the desired class] = KalmanFilter(dt, point, speed_to_mms, q_cam, q_gnd, r_cam, r_gnd)
class KalmanFilter(object):
    def __init__(self, dt, point, speed_to_mms, q_cam, q_gnd, r_cam, r_gnd):
        self.dt = dt
        self.E = point #position in x and y from the camera

        self.A = np.diag([1, 1])
        self.B = dt * self.A
        self.H = self.A
        self.Q = self.A
        self.Q_cam = self.A
        self.Q_gnd = self.A
        self.R = self.A
        self.R_cam = self.A
        self.R_gnd = self.A
        self.P = self.A
        self.u = np.array([[0], [0]])
        self.i = self.u
        self.y_gnd = self.u
        self.speed_to_mms = speed_to_mms

        self.inner_strip_width  = 47
        self.outer_strip_width = 4
        self.compute_Q(q_cam, q_gnd)
        self.compute_R(r_cam, r_gnd)


    def compute_Q(self, q_cam, q_gnd): 
        ##################################################### REECRIRE LES DEFINITION ET REVOIR CHAQUE Q ET R
        # q_cam : value of variance of position state for the camera
        # q_gnd : value of variance of position state for the gnd_sensor
        #####
        # Defines the matrix R from the values q1 and q2 computed for calibration
        #####################################################
        
        self.Q_cam = np.diag(q_cam)
        self.Q_gnd = np.diag(q_gnd)

    def compute_R(self, r_cam, r_gnd):
        #####################################################
        # r_cam : value of variance of position measurement for the camera
        # r_gnd : value of variance of position measurement for the gnd_sensor
        #####
        # Defines the matrix R from the values r1 and r2 computed for calibration
        #####################################################

         self.R_cam = np.diag(r_cam)
         self.R_gnd = np.diag(r_gnd)


    def filter(self, isCamOn, pos_from_cam, direction, speed, gnd, gnd_prev, 
               transition_threshold):
        #####################################################
        # isCamOn               : bool representing wether the webcam is available or not
        # pos_from_cam          : 2x1 vector giving the position of the robot, from the webcam in [mm]
        # direction             : Character chain defining what direction the thymio is supposed to follow (0, 1, 2 or 3)
        # speed                 : 2x1 vector giving the speed of the thymio, from the thymio's sensor.
        # gnd                   : value from the ground sensor of the thymio
        # gnd_prev              : value from the ground sensor of the thymio from last's itteration
        # transition_threshold  : value of the threshold at which we consider we crossed a line or not
        #####
        # Will choose the behaviour of the filter, wether we have the camera available or not and will update the filter, 
        # the state estimation vector and the covariance estimation vector
        #####################################################

        self.predict(speed)
        if(isCamOn == 1):
            #camera is on: we will use it as it provides a better overview
            self.updateWithCam(pos_from_cam)
        else:
            #camera is off: we now rely on ground sensor
            self.updateWithGnd(pos_from_cam, direction, gnd, gnd_prev, transition_threshold)
        
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  
        self.E = self.E + np.dot(K, self.i)
        self.P = self.P - np.dot(K, np.dot(self.H, self.P))
        return self.E



    def predict(self, speed): 
        #####################################################
        # Updates the state estimation vector aswell as the state covariance vector
        #####################################################

        u = np.array([[speed[0]*self.speed_to_mms],[speed[1]*self.speed_to_mms]])
        # State estimation
        self.E = np.dot(self.A, self.E) + np.dot(self.B, u) 
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q


    def updateWithCam(self, pos_from_cam):
        #####################################################
        # sets the appropriate observation of the true state y, and compute the corresponding innovation
        #####################################################
        
        self.Q = self.Q_cam
        self.R = self.R_cam
        self.H = np.eye(2).astype(int)
        y = pos_from_cam
        self.i = y - self.E

    def updateWithGnd(self, pos_from_cam, direction, gnd, gnd_prev, transition_threshold):
        #####################################################
        # sets the appropriate observation of the true state y, and compute the corresponding innovation
        #####################################################
        
        if(np.any(self.y_gnd)):
            case_width = self.inner_strip_width + self.outer_strip_width # a mettre comme paramètre de la classe 
            # à verifier si ca marche correctement
            self.y_gnd[0] = pos_from_cam[0] - pos_from_cam[0]%(case_width)
            self.y_gnd[1] = pos_from_cam[0] - pos_from_cam[1]%(case_width)
        if((gnd < transition_threshold)^(gnd_prev < transition_threshold)):
            if(gnd < gnd_prev):
                stripe_width = self.inner_strip_width 
            else:
                stripe_width = self.outer_strip_width
        
            if(direction == 0): # positive horizontal mouvement
                self.y_gnd[0] = self.y_gnd[0] + stripe_width
                self.H = np.diag([1, 0])
            elif(direction == 1): # positive vertical mouvement
                self.y_gnd[1] = self.y_gnd[1] + stripe_width
                self.H = np.diag([0, 1])
            elif(direction == 2): # negative horizontal mouvement
                self.y_gnd[1] = self.y_gnd[0] - stripe_width
                self.H = np.diag([1, 0])
            elif(direction == 3): # negative vertical mouvement
                self.y_gnd[1] = self.y_gnd[1] - stripe_width
                self.H = np.diag([0, 1])
            
        else:
            self.H = np.diag([0, 0, 1, 1])

        self.R = np.dot(self.R_gnd, self.H)
        self.i = self.y_gnd - self.E