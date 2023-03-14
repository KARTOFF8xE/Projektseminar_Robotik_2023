# Functionality LiDAR - Algorithmus
The applied LiDAR is centered on the robot at an angle $\alpha$ horizontal and provides a point cloud. It points in the direction of travel.

## Transforming the Pointcloud
All points facing the ground are used, all other points are discarded. Then all points are transformed from polar coordinates to Cartesian coordinates. The horizontal center of the robot forms $x = 0$, the positive direction points to the right. $y = 0$ is formed by the height of the LiDAR and increases towards the ground. The values are ordered by x in ascending order. Subsequently, the detection of the curb takes place.

## Detect Curbstones
Starting from the inside of the robot wheels, three neighboring points (1), (2), (3) are selected in a stepwise manner. Two vectors of (1)&rarr;(2) and (2)&rarr;(3) are formed. Their angular difference is determined. If the angle difference exceeds a threshold value (parameter: //TODO), there is a chance of finding a curb. The height of (1) is used as reference height and compared with (3) and successor points, which are within a certain distance (parameter: //TODO). If a difference threshold remains exceeded, the false detection of a pothole as a curb is excluded. In case of a pothole, the search continues as long as the maximum number of checks (parameter: //TODO) has not yet been performed or further values exist. If the vector (2)&rarr;(3) points more towards the sky than the vector (1)&rarr;(2) and both the angle threshold and the height difference threshold are met, an obstacle is detected and immediately set as a boundary. The detected boundaries are returned to the node as a positive distance to the LiDAR and stored within a vector. In case a boundary was not found, the value "0" is returned. Since the robot must have a width, "0" cannot be reached realistically.

# Default Parameters:
