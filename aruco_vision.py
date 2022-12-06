import cv2
import numpy as np
import math
import matplotlib.pyplot as plt

from geometry import max_area, cyclic_intersection_pts


class Vision:

    def __init__(self):
        self.img = None # original image taken by camera
        self.p_img = None # processed image
        self.mask = None
        self.img_mask = None
        self.img_final = None #image resized and cropped with or without thymio

        self.transform = None
        self.width = None
        self.height = None

        self.offset = 0

        self.rows = 6
        self.columns = 9
        self.cellx = None
        self.celly = None
        self.grid = None
        self.thymio_position = None
        self.thymio_orientation = None
        self.thymio_deviation = None
        self.goal_position = None

        self.occupancy_grid()

    def take_picture(self):
        """
            input:      None
            output:     picture taken by the camera
        """
        cap = cv2.VideoCapture(0) # A CHANGER SUIVANT LORDI

        # Check if camera opened successfully
        if not cap.isOpened():
            print("Error opening video stream")

        # Getting a first frame for the width and height of the plot
        ret, frame = cap.read()
        if(frame is None):
            print("oopsy")
        
        # frame = cv2.imread('input/frame.png', cv2.IMREAD_COLOR)

        cv2.imwrite('input/frame.png', frame)
        cap.release()
        self.img = frame.copy()

    def image_processing(self):
        """
            input:      the image taken by the camera
            output:     processed image after normalizing, blurring, converting to B&W and applying a threshold
        """

        # Normalize the image manage lightning
        img_norm = np.zeros_like(self.img)
        img_norm = cv2.normalize(self.img, None, alpha = 0, beta = 255, norm_type=cv2.NORM_MINMAX)

        # Blur the image to denoise: GaussianBlur
        kernel_size = 5
        blur = cv2.GaussianBlur(img_norm, (kernel_size, kernel_size), 0)

        # Convert to B&W
        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)

        # Apply thresholding
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 29, 5)
        cv2.imwrite('intermediate/processedImage.png', thresh)

        self.p_img = thresh.copy()

    def create_mask(self):
        """
            input:      the image to resize according to the biggest contour
            output:     the mask corresponding to the image

            This function finds the grid in the image and creates a mask to cut off everything out of the grid
        """

        img = self.p_img

        # Find the contours
        contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Find contour with biggest area (corresponds to grid)
        best_cnt = max_area(contours)

        # Create the mask corresponding to that contour
        mask = np.zeros((img.shape),np.uint8)
        cv2.drawContours(mask,[best_cnt],0,255,-1)
        cv2.drawContours(mask,[best_cnt],0,0,2)
        cv2.imwrite('intermediate/mask.png', mask)

        self.mask = mask.copy()

    def apply_mask(self):
        """
            input:      the image to which apply the mask and the mask
            output:     the image resized according to the mask
        """

        out = np.zeros_like(self.img)
        out[self.mask == 255] = self.img[self.mask == 255]
        cv2.imwrite('intermediate/maskOnImage.png', out)
        self.img_mask = out


    def create_transform(self):
        """
            Creates a transform that maps the vertices of the grid to the corresponding vertices of the
            smallest fitting rectangle
            Set the self.transform variable
        """
        img1 = self.mask.copy()
        ret,thresh = cv2.threshold(img1,127,255,0)

        
        contours,_ = cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cnt = max_area(contours)
        img_cnt = self.img.copy()
        cv2.drawContours(img_cnt, cnt, -1, (0, 255, 0), 20)
        cv2.imwrite('intermediate/contour.png', img_cnt)


        # compute rotated rectangle (minimum area)
        rect = cv2.minAreaRect(cnt)
        (x, y), (w, h), angle = rect
        box = cv2.boxPoints(rect)
        box = np.int0(box)
      

        min_rect = self.img.copy()
        cv2.drawContours(min_rect,[box],0,(0,255,0),20)
        cv2.imwrite('intermediate/minRect.png', min_rect)

        # Sorts the 4 points clockwise
        rect_pts = cyclic_intersection_pts(box)

        # Convex hull
        hull_list = []
        for i in range(len(contours)):
            hull = cv2.convexHull(contours[i])
            hull_list.append(hull)

        # For each of the 4 points in rect_pts, we check which one is closest in hull_list
        pts_mask = np.zeros(shape=(4, 2))
        for r in range(len(rect_pts)):
            min_dist = 2000
            for k in range(len(hull_list)):
                for j in range(len(hull_list[k])):
                    d = math.dist(rect_pts[r], hull_list[k][j][0])
                    if d < min_dist:
                        min_dist = d
                        pts_mask[r] = hull_list[k][j][0]

        pts_mask[0] = pts_mask[0] + np.array([-self.offset,-self.offset])
        pts_mask[1] = pts_mask[1] + np.array([self.offset,-self.offset])
        pts_mask[2] = pts_mask[2] + np.array([self.offset,self.offset])
        pts_mask[3] = pts_mask[3] + np.array([-self.offset,self.offset])

        # List the output points in the same order as input
        # Top-left, top-right, bottom-right, bottom-left
        if w > h:
            width = w
            height = h
        else:
            width = h
            height = w
        dstPts = [[0, 0], [width, 0], [width, height], [0, height]]

        # Get the transform
        m = cv2.getPerspectiveTransform(np.float32(pts_mask), np.float32(dstPts))
        self.transform = m
        self.width = width
        self.height = height

    def apply_transform(self):
        """
            Applies the previously computed transform to the image
            Sets the self.img_final variable
        """
        # transform the image
        out = self.img_mask.copy()
        out = cv2.warpPerspective(self.img, self.transform, (int(self.width), int(self.height)))
        cv2.imwrite('output/transformedImage.png', out)
        self.img_final = out

    def create_grid(self, error = 0):
        """
            Creates the occupancy grid corresponding to the self.img_final
            white (free) = 0
            black (obstacle) = 1
        """

        # Apply a threshold to the image
        t = 100
        ret, thresh1 = cv2.threshold(self.img_final,t,255,cv2.THRESH_BINARY)
        cv2.imwrite('intermediate/thresh.png', thresh1)

        # Computes size of a cell in the image [pixels]
        h,w,c = self.img_final.shape
        self.cellx = round(w/self.columns)
        self.celly = round(h/self.rows)

        # Empty grid of size rows x columns
        data = np.ones((self.rows, self.columns))


        # For each cell, compute the mean value of pixels, if bigger than threshold --> white
        for k in range(0, self.rows):
            for j in range(0, self.columns):
                cell = thresh1[(k*self.celly + error + self.offset):((k+1)*self.celly + error + self.offset), (j*self.cellx + error + self.offset):((j+1)*self.cellx + error + self.offset)]
                m = np.average(cell)
                if m > t:
                    data[k, j] = 0
        self.grid = data


    def occupancy_grid(self):
        """
            Calls all the functions necessary to compute the occupancy grid
            Call it only once per simulation
        """
        self.take_picture()
        self.image_processing()     
        self.create_mask()
        self.apply_mask()
        self.create_transform()
        self.apply_transform()
        self.create_grid()

    def orientation(self, point1, point2):
        """
            input: two points on the image
            output: angle between the x-axis and the normal to the points
        """
        dX = point2[0] - point1[0]
        dY = point2[1] - point1[1]
        alpha = math.atan2(dX,dY)
        return math.degrees(alpha%360)


    def aruco(self):
        gray = cv2.cvtColor(self.img_final, cv2.COLOR_BGR2GRAY)

        # Apply thresholding
        ret, thresh = cv2.threshold(gray,100,255,cv2.THRESH_BINARY)


        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(thresh, arucoDict,parameters=arucoParams)
        return corners,ids

    def compute_coordinates(self, middle):
            cX = (int)((middle[0] - self.offset) / self.cellx)
            cY = (int)((middle[1] - self.offset)/self.celly)    
            return (cX,cY)

    def coordinates(self):
        corners, ids = self.aruco()
        for c in range(0, len(corners)):
            centre = 0
            for i in range(0, 4):
                centre += corners[c][0][i]
            centre = centre / 4
            if ids[c][0] == 2: #thymio
                self.thymio_position = self.compute_coordinates(centre)
                self.thymio_orientation = self.orientation(corners[c][0][1], corners[c][0][2])
                m_cell0 = ((self.thymio_position[0] - 0.5)*self.cellx, (self.thymio_position[1] - 0.5)*self.celly)
                self.thymio_deviation = (centre[0] - m_cell0[0], centre[1] - m_cell0[1])

            if ids[c][0] == 1: #goal
                self.goal_position = self.compute_coordinates(centre)


    def update_coordinates(self):
        """
            Updates the occupancy_grid based based on the new image
        """
        self.take_picture()
        self.apply_mask()
        self.apply_transform()
        self.coordinates()
