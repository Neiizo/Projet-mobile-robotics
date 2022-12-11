import numpy as np

class KalmanFilter(object):
    def __init__(self, dt, point, speed_to_mms, q_cam, r_cam):
        self.dt = dt
        self.E = point      # position in x and y from the camera

        self.A = np.diag([1, 1])
        self.B = dt * self.A
        self.H = self.A
        self.Q = np.diag(q_cam)
        self.R = np.diag(r_cam)
        self.P = self.A
        self.u = np.array([[0], [0]])
        self.i = self.u
        self.speed_to_mms = speed_to_mms


    def filter(self, isCamOn, pos_from_cam, speed, orientation):
        #####################################################
        # isCamOn               : bool representing wether the webcam is available or not
        # pos_from_cam          : 2x1 vector giving the position of the robot, from the webcam in [mm]
        # speed                 : 2x1 vector giving the speed of the thymio, from the thymio's sensor.
        #####
        # Will choose the behaviour of the filter, wether we have the camera available or not and will update the filter, 
        # the state estimation vector and the covariance estimation vector
        #####################################################

        self.predict(speed,  orientation)
        if(isCamOn == 1):
            #camera is on: we will use it as it provides a better overview
            self.updateWithCam(pos_from_cam)
            S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
            K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  
        else:
            K = 0

        
        self.E = self.E + np.dot(K, self.i)
        self.P = self.P - np.dot(K, np.dot(self.H, self.P))
        return self.E



    def predict(self, speed, direction): 
        #####################################################
        # direction : Angle from the camera's perspective, to determine the direction the thymio is going
        #####
        # Updates the state estimation vector aswell as the state covariance vector
        #####################################################
        u = np.zeros((2,1))
        speed_avg = (speed[0] + speed[1]) /2
        u[0] = speed_avg * self.speed_to_mms * np.cos(direction)
        u[1] = speed_avg * self.speed_to_mms * np.sin(direction)
        self.E = np.dot(self.A, self.E) + np.dot(self.B, u) 
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q


    def updateWithCam(self, pos_from_cam):
        #####################################################
        # pos_from_cam : position of the thymio's from the camera's perspective
        #####
        # sets the appropriate observation of the true state y, and compute the corresponding innovation
        #####################################################
        self.H = np.eye(2).astype(int)
        y = pos_from_cam
        self.i = y - self.E