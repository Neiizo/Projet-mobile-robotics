
import numpy as np
import utils

class Kalman_filter():
    """Initialization"""
    def __init__(self, vision):
        self.pos_x = vision.robot.center[0][0]
        self.pos_y = vision.robot.center[0][1]
        self.theta = vision.robot.orientation
        self.phi = np.arctan2(self.pos_y, self.pos_x)
        self.P_est = 1000 * np.ones(3)                                                #in seconds
        self.rho_est = np.array([self.pos_x, self.pos_y, self.theta])
        self.R = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.01]])
        self.q1 = 1
        self.q2 = 1
        self.q3 = 0.1
        self.Q = np.array([[self.q1, 0, 0], [0, self.q2, 0], [0, 0, self.q3]])
        self.A = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        self.C = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        self.update_values(vision, [0, 0], 0.01)

    def update_values(self, vision, robot_speed, time):
        self.time = time
        self.v = vision.mm2px * utils.THYMIO_SPEED_2_MM * 0.8 * (robot_speed[0] + robot_speed[1]) / 2
        self.w = utils.THYMIO_SPEED_2_MM * (robot_speed[0] - robot_speed[1]) / utils.THYMIO_WIDTH_MM
        self.u = np.array([self.v, self.w])
        self.alpha = self.time * self.w
        self.B = np.array([[self.time * np.cos(self.alpha + self.rho_est[2]), 0], [self.time * np.sin(self.alpha + self.rho_est[2]), 0], [0, self.time]])

    def update_kalman(self, vision, robot_speed, time):
        """ Main body of the Kalman Filter """
        self.update_values(vision, robot_speed, time)
        rho_est_a_priori = np.dot(self.A, self.rho_est) + np.dot(self.B, self.u)
        p_est_a_priori = np.dot(self.A, np.dot(self.P_est, self.A.T)) + self.Q
        if vision.robot.detected():
            """ Kalman with camera on """
            self.pos_x = vision.robot.center[0][0]
            self.pos_y = vision.robot.center[0][1]
            self.theta = vision.robot.orientation

            S = np.dot(self.C, np.dot(p_est_a_priori, self.C.T)) + self.R
            K = np.dot(p_est_a_priori, np.dot(self.C.T, np.linalg.inv(S)))

        else:
            """Kalman without the camera, using only the speed of the robot"""
            K = 0
            self.pos_x = int(self.rho_est[0])
            self.pos_y = int(self.rho_est[1])
            self.theta = int(self.rho_est[2])

        y = np.array([self.pos_x, self.pos_y, self.theta])
        i = y - np.dot(self.C, rho_est_a_priori)
        self.rho_est = rho_est_a_priori + np.dot(K, i)
        self.rho_est[2] = (self.rho_est[2] + np.pi) % (2 * np.pi) - np.pi
        self.P_est = np.dot((np.identity(3) - np.dot(K, self.C)), p_est_a_priori)
