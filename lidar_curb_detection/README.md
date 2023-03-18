# lidar_curb_detection (lcd)

## Description
The lcd package implements a node that is meant to calculate the distance from the center of the robot to the left, and to the right. If both Values exist and are declared as valid, it also calculates the width of the path.

## Concept
### Transforming the Pointcloud
All points facing the ground are used, all other points are discarded. Then all points are transformed from polar coordinates to Cartesian coordinates. The horizontal center of the robot forms $x = 0$, the positive direction points to the right. $y = 0$ is formed by the height of the LiDAR and increases towards the ground. The values are ordered by x in ascending order. Subsequently, the detection of the curb takes place.

### Detect Curbstones and other Limits (walls, signs, etc.)
Starting from the inside of the robot wheels, three neighboring points (1), (2), (3) are selected in a stepwise manner. Two vectors of (1)&rarr;(2) and (2)&rarr;(3) are formed. Their angular difference is determined. If the angle difference exceeds a threshold value (parameter: detection_thr/angle_thr), there is a chance of finding a curb. The height of (1) is used as reference height and compared with (3) and successor points, which are within a certain distance (parameter: /detection_thr/advanced_ray_check_thr). If a difference threshold remains exceeded, the false detection of a pothole as a curb is excluded. In case of a pothole, the search continues as long as the maximum number of checks (parameter: detection_thr/max_check_length) has not yet been performed or further values exist. If the vector (2)&rarr;(3) points more towards the sky than the vector (1)&rarr;(2) and both the angle threshold and the height difference threshold are met, an obstacle is detected and immediately set as a boundary. The detected boundaries are returned to the node as a positive distance to the LiDAR and stored within a vector. In case a boundary was not found, the value "0" is returned. Since the robot must have a width, "0" cannot be reached realistically.

### Filters
For a detailed description of the used filters, take a look at its [README](../filters/README.md).

## Parameters
- __topics__:
    - __dmc__:
      - __laserscan__: For receiving the Pointcloud of the LiDAR. <br>
            _type_: string <br>
            _default_: "/sick/scan" <br>
    - __pub__:  Topic for Publishing the solutions. <br>
        _type_: string <br>
        _default_: "/lidar_path_width" <br>
- __robot_specific__: <br>
    - __wheel_inside__: Width of the Wheels. <br>
        _type_: double <br>
        _default_: .2854 <br>
        _unit_: meters <br>
    - __wheel_width__: Distance of the vertical Plane in the center of the Robot to the vertical inside Plane of the Wheels. <br>
        _type_: double <br>
        _default_: .1143 <br>
        _unit_: meters <br>
    - __mounting_angle__: The Mounting-Angle of the LiDAR. <br>
        _type_: double <br>
        _default_: $\frac{\pi}{6}$ <br>
        _unit_: radiant <br>
- __detection_thr__: <br>
    - __angle_thr__:  Minimum angle between two Vectors that is needed to detect a Curbstone. <br>
        _type_: integer <br>
        _default_: 4 <br>
        _unit_: degree <br>
    - __height_diff__:  Height_Difference that must exist, to detect a Curbstone. <br>
        _type_: double <br>
        _default_: .05 <br>
        _unit_: meters <br>
    - __advanced_ray_check_thr__: Distance a possible Curbstone should be proofed, that it isn't a Pothole. <br>
        _type_: double <br>
        _default_: .2 <br>
        _unit_: meters <br>
- __bubble__: <br>
    - __distance_thr__: Threshold that tells the filter, how far another Point is allowed to be away to be valid. <br>
        _type_: double <br>
        _default_: .4 <br>
        _unit_: meters <br>
    - __quantity_check__: Amount of Limits that is checked in each direction (before and behind). <br>
        _type_: int <br>
        _default_: 14 <br>
        _unit_: quantity <br>
    - __quantity_thr__: Minimum amount of valid Points that needs to lay inside the distance_thr. <br>
        _type_: int <br>
        _default_: 15 <br>
        _unit_: quantity <br>
- __avg_dist__: <br>
    - __avg_dist_thr__: distance-averages below this Threshold are validating the Limit. <br>
        _type_: double <br>
        _default_: .2 <br>
        _unit_: meters <br>
    - __quantity_check__: Amount of Limits that is checked in each direction (before and behind). <br>
        _type_: integer <br>
        _default_: 15 <br>
        _unit_: quantity <br>
    - __counter_thr__:  Minimum amount of taken distance-differences that is needed for the average-calculation, otherwise the limit is not valid. <br>
        _type_: integer <br>
        _default_: 12 <br>
        _unit_: quantity <br>
- __island__: <br>
    - __quantity_check__: Amount of Limits that is checked in each direction (before and behind). <br>
        _type_: integer <br>
        _default_: 45 <br>
        _unit_: quantity <br>
    - __counter_thr__:  Needed Quantity of Valid Limits to validate the looked up Limit. <br>
        _type_: integer <br>
        _default_: 40 <br>
        _unit_: quantity <br>

## Launch

#### `lidar_curb_detection.launch.py`

Start the lcd itself with its parameter file.

Options:
- __visualize__: Visualize 2D-PointCloud (relief of Street), location of wheels and Limits (left and right). <br>
   _default_: False <br>
   _options_: True | False <br>

 - __log_level__: Setup the general output log level for this node.  
   _default_: info  
   _options_: debug | info | warn | error | fatal

#### `sublaunch.lidar_curb_detection.launch.py`

This launch file is thought for the Deciders [start_all.launch.py](../decider/README.md#start_alllaunchpy)
