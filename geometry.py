import cv2
import numpy as np

def max_area(contours, area_min = 100):
    # Find biggest area
    max_area = 0
    c = 0
    best_cnt = None
    for i in contours:
            area = cv2.contourArea(i)
            if area > area_min:
                    if area > max_area:
                        max_area = area
                        best_cnt = i
            c+=1
    return best_cnt

def cyclic_intersection_pts(pts):
    """
    Sorts 4 points in clockwise direction with the first point been closest to 0,0
    Assumption:
        There are exactly 4 points in the input and
        from a rectangle which is not very distorted
    """
    if pts.shape[0] != 4:
        return None

    # Calculate the center
    center = np.mean(pts, axis=0)

    # Sort the points in clockwise
    cyclic_pts = [
        # Top-left
        pts[np.where(np.logical_and(pts[:, 0] < center[0], pts[:, 1] < center[1]))[0][0], :],
        # Top-right
        pts[np.where(np.logical_and(pts[:, 0] > center[0], pts[:, 1] < center[1]))[0][0], :],
        # Bottom-Right
        pts[np.where(np.logical_and(pts[:, 0] > center[0], pts[:, 1] > center[1]))[0][0], :],
        # Bottom-Left
        pts[np.where(np.logical_and(pts[:, 0] < center[0], pts[:, 1] > center[1]))[0][0], :]
    ]

    return np.array(cyclic_pts)

"""
def contours(img, low, high):
    # Apply a threshold and return the contours of the resulting mask
    mask = cv2.inRange(img, low, high)
    blur = cv2.GaussianBlur(mask, (9, 9), 0)
    contours, hierarchies = cv2.findContours(blur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def find_coordinates(contour):
    # Returns the coordinates of the center of that contour
    cX = 0
    cY = 0
    M = cv2.moments(contour)
    cX = int(M["m10"] / (M["m00"] + 1e-5))
    cY = int(M["m01"] / (M["m00"] + 1e-5))
    return np.array([cX, cY])
"""
