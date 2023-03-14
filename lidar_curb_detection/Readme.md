# Functionality LiDAR - Algorithmus
The applied LiDAR is centered on the robot at an angle $\alpha$ horizontal and provides a point cloud. It points in the direction of travel.

## Transforming the Pointcloud
All points facing the ground are used, all other points are discarded. Then all points are transformed from polar coordinates to Cartesian coordinates. The horizontal center of the robot forms $x = 0$, the positive direction points to the right. $y = 0$ is formed by the height of the LiDAR and increases towards the ground. The values are ordered by x in ascending order. Subsequently, the detection of the curb takes place.

## Detect Curbstones
Starting from the inside of the robot wheels, three neighboring points (1), (2), (3) are selected in a stepwise manner. Two vectors of (1)&rarr;(2) and (2)&rarr;(3) are formed. Their angular difference is determined. If the angle difference exceeds a threshold value (parameter: detection_thr/angle_thr), there is a chance of finding a curb. The height of (1) is used as reference height and compared with (3) and successor points, which are within a certain distance (parameter: /detection_thr/advanced_ray_check_thr). If a difference threshold remains exceeded, the false detection of a pothole as a curb is excluded. In case of a pothole, the search continues as long as the maximum number of checks (parameter: detection_thr/max_check_length) has not yet been performed or further values exist. If the vector (2)&rarr;(3) points more towards the sky than the vector (1)&rarr;(2) and both the angle threshold and the height difference threshold are met, an obstacle is detected and immediately set as a boundary. The detected boundaries are returned to the node as a positive distance to the LiDAR and stored within a vector. In case a boundary was not found, the value "0" is returned. Since the robot must have a width, "0" cannot be reached realistically.

## Launch
Launch Node via `ros2 launch lidar_curb_detection lidar_curb_detection.launch.py`
### Arguments
- visualize: <br>
    &xrarr; Visualize 2D-PointCloud (relief of Street), location of wheels and Limits (left and right). Valid choices are: ['True', 'False'] <br>
    &xrarr; (default: 'False') <br>

- log_level: <br>
    &xrarr; Set log level to node logger. Valid choices are: ['debug', 'info', 'warn', 'error', 'fatal'] <br>
    &xrarr; (default: 'info') <br>
> type `ros2 launch lidar_curb_detection lidar_curb_detection.launch.py -s` for help at inserting

# Default Parameters:
### Topics:
Parameter   | Default Value     | Description
:-----------|:-----------------:|:-----------
laserscan   |/sick/scan         | For Receiving the Pointcloud of the LiDAR
pub         |/lidar_path_width  | Topic for Publishing the solutions

### Robot_specific:
Parameter       | Default Value     | Description
:---------------|:-----------------:|:-----------
wheel_inside    | .2854             | Width of the Wheels   
wheel_width     | .1143             | Distance of the vertical Plane in the center of the Robot to the vertical inside Plane of the Wheels   
mounting_angle  | $\frac{\pi}{6}$   | The Mounting-Angle of the LiDAR

### Detection_thr:
Parameter               | Default Value     | Description
:-----------------------|:-----------------:|:-----------
angle_thr               | 4                 | Minimum angle between two Vectors that is needed to detect a Curbstone 
height_diff             | .05               | Height_Difference that must exist, to detect a Curbstone
advanced_ray_check_thr  | .2                | Distance a possible Curbstone should be proofed, that it isn't a Pothole           
max_check_length        | 50                | Quantity of Rays that should be checked for a Curbstone

## Filters:
### Bubble
Parameter       | Default Value     | Description
:---------------|:-----------------:|:-----------
distance_thr    | .2                | Threshold that tells the filter, how far another Point is allowed to be away to be valid
quantity_check  | 14                | Amount of Limits that is checked in each direction (before and behind)
quantity_thr    | 15                | Minimum amount of valid Points that needs to lay inside the distance_thr

### Avg_dist:
Parameter       | Default Value     | Description
:---------------|:-----------------:|:-----------
quantity_check  | 15                | Amount of Limits that is checked in each direction (before and behind)
counter_thr     | 12                | Minimum amount of taken distance-differences that is needed for the average-calculation, otherwise the limit is not valid
avg_dist_thr    | .1                | distance-averages below this Threshold are validating the Limit

### Island:
Parameter       | Default Value     | Description
:---------------|:-----------------:|:-----------
quantity_check  | 45                | Amount of Limits that is checked in each direction (before and behind)
counter_thr     | 40                | Needed Quantity of Valid Limits to validate the looked up Limit