# lidar_curb_detection (lcd)

## Description
The lcd package implements a node that is meant to calculate the distance from the center of the robot to the left and right road boundary. If both values exist and are declared as valid, it also calculates the width of the path.

## Concept
### Transforming the LaserScan
All points facing the ground are used, all other points are disgarded. Then all points are transformed from polar coordinates to cartesian coordinates. The horizontal center of the robot forms $x = 0$, the positive direction points to the right. $y = 0$ is formed by the height of the LiDAR and increases towards the ground. The values are ordered by x in ascending order. Subsequently, the detection of the curb takes place.

### Detect Curbstones and other Limits (walls, signs, etc.)
Starting from the inside of the robot wheels, three neighboring points (1), (2), (3) are selected in a stepwise manner. Two vectors (1)&rarr;(2) and (2)&rarr;(3) are formed. Their angular difference is determined. If the angle difference exceeds a threshold value (parameter: detection_thr/angle_thr), there is a chance of finding a curb. The height of (1) is used as a reference height and gets compared to (3) and its successor points, which are within a certain distance (parameter: /detection_thr/advanced_ray_check_thr). If the difference remains above the threshold a pothole is out of the question. Otherwise the search continues as long as the maximum number of checks (parameter: detection_thr/max_check_length) has not yet been performed and further values exist. If the vector (2)&rarr;(3) points more towards the sky than the vector (1)&rarr;(2) and both the angle threshold and the height difference threshold are met, an obstacle is detected and immediately set as a boundary. The detected boundaries are returned to the node as a positive distance from the LiDAR and stored within a buffer. In case a boundary was not found, the value "$0$" is returned. Since the robot must have a width, "$0$" cannot be reached realistically.

### Postfilters
For a detailed description of the used filters, take a look at its [README](../filters/README.md).

* This package also publishes invalid values: <br>
    &rarr; "$0$" if the algorithm was not able to detect a limit <br>
    &rarr; "$<0$" if the detected value does not pass one of the filters

## Parameters
- __topics__:
    - __dmc__:
      - __laserscan__: For receiving the LaserScan of the LiDAR. <br>
            _type_: string <br>
            _default_: "/sick/scan" <br>
    - __pub__:  Topic for Publishing the results. <br>
        _type_: string <br>
        _default_: "/lidar_path_width" <br>
- __robot_specific__: <br>
    - __wheel_inside__: Width of the wheels. <br>
        _type_: double <br>
        _default_: $.2854$ <br>
        _unit_: meters <br>
    - __wheel_width__: Distance of the vertical plane in the center of the robot to the vertical inside plane of the wheels. <br>
        _type_: double <br>
        _default_: $.1143$ <br>
        _unit_: meters <br>
    - __mounting_angle__: The mounting angle of the LiDAR. <br>
        _type_: double <br>
        _default_: $\frac{\pi}{6}$ <br>
        _unit_: radiant <br>
- __detection_thr__: <br>
    - __angle_thr__:  Minimum angle between two vectors to detect a curbstone. <br>
        _type_: integer <br>
        _default_: $4$ <br>
        _unit_: degree <br>
    - __height_diff__:  The height difference that must be present to detect a curbstone. <br>
        _type_: double <br>
        _default_: $.05$ <br>
        _unit_: meters <br>
    - __advanced_ray_check_thr__: The distance over which a possible curbstone should be checked to validate that it is not a pothole. <br>
        _type_: double <br>
        _default_: $.2$ <br>
        _unit_: meters <br>
- __bubble__: <br>
    - __distance_thr__: Threshold that tells the filter how far away another point can be to still be considered valid. <br>
        _type_: double <br>
        _default_: $.4$ <br>
        _unit_: meters <br>
    - __quantity_check__: Amount of limits that are checked in each direction (front and rear). <br>
        _type_: int <br>
        _default_: $14$ <br>
        _unit_: quantity <br>
    - __quantity_thr__: Minimum required amount of valid points inside the __distance_thr__. <br>
        _type_: int <br>
        _default_: $15$ <br>
        _unit_: quantity <br>
- __avg_dist__: <br>
    - __avg_dist_thr__: The maximum a distance difference can be while still being considered valid. <br>
        _type_: double <br>
        _default_: $.2$ <br>
        _unit_: meters <br>
    - __quantity_check__: Amount of limits that are checked in each direction (front and rear). <br>
        _type_: integer <br>
        _default_: $15$ <br>
        _unit_: quantity <br>
    - __counter_thr__:  Minimum amount of distance differences that are needed to calculate the average. Lower ammounts result in invalid limits. <br>
        _type_: integer <br>
        _default_: $12$ <br>
        _unit_: quantity <br>
- __island__: <br>
    - __quantity_check__: Amount of limits that are checked in each direction (front and rear). <br>
        _type_: integer <br>
        _default_: $45$ <br>
        _unit_: quantity <br>
    - __counter_thr__:  The quantity of valid limits to validate the current limit. <br>
        _type_: integer <br>
        _default_: $40$ <br>
        _unit_: quantity <br>

## Launch

#### `lidar_curb_detection.launch.py`

Start the lcd itself with its parameter file.

Options:
- __visualize__: Visualize 2D pointcloud (relief of street), location of wheels and limits (left and right). <br>
   _default_: False <br>
   _options_: True | False <br>

 - __log_level__: Setup the general output log level for this node.  
   _default_: info  
   _options_: debug | info | warn | error | fatal

#### `sublaunch.lidar_curb_detection.launch.py`

This launch file is meant to be used by [start_all.launch.py](../decider/launch/start_all.launch.py) only.