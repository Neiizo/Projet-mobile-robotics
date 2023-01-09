# Projet-mobile-robotics
In this project, we were tasked in controlling a thymio. It needed to go from point A to B, by any means necessary. It also needed to include 2 sensors, obstacle avoidance, trajectory replanning and finally, a filter.

## Trajectory planning
Because we decided to work on a grid, we opted for a djisktra 4N algorithm. This way, our robot can only go up, left, down or right. We take a picture with a camera that is standing above the terrain, and compute the grid's position via multiple algorithm from the opencv library. Then, we give the grid to the djisktra algorithm, that will compute the shortest path

## Local avoidance
The local avoidance needed to be working entirely with the on board sensors. We weren't allowed to detect new obstacles via the camera and compute a new path accordingly. The thymio moves forward in it's path planning, and if it were to detect an obstacle, it would try to avoid it, first from the right, and if it can't from the left. It continues moving until it doesn't detect an obstacles on its side anymore, and will try going around, until it reaches the second next cell in the path planning

## The Kalman filter
In order to increase our position's estimation, we used the kalman's filter. It takes the detected position from the camera, and computes and estimated position, using known values of variance on speed and position, both for the camera's position and the thymio's speed sensor. This estimated position is the one used for any computation that requires a position, except for the local avoidance. The local avoidance works without the camera. The main functions that relies on the kalman's filter, is our function called ```kalman_adjust```. It computes the delta between the thymio's current position, and the center of the next cell, and adjusts the angle of the thymio to ensure it reaches it.

For further information, you are encouraged to compile and read our [Jupyter notebook](report.ipynb)
