# visualize_path

## Description
The visualize_path package implements a node that is meant to visualize the latest 1000 published limits by [lidar_curb_detection](../lidar_curb_detection/README.md), [wayfinding](../wayfinding/README.md) or [decider](../decider/README.md).

## Concept
Storing the received values for each left and right limit and visualizing the latest 1000. Valid values are inked in green (left limits) and red (right limits). If a package also publishes invalid values (like [lidar_curb_detection](../lidar_curb_detection/README.md) and [wayfinding](../wayfinding/README.md)) it colors them in petrol.

## Parameters
- __robot_specific__: <br>
    - __wheel_inside__: Width of the Wheels. <br>
        _type_: double <br>
        _default_: $.2854$ <br>
        _unit_: meters <br>
    - __wheel_width__: Distance of the vertical plane in the center of the robot to the vertical inside plane of the wheels. <br>
        _type_: double <br>
        _default_: $.1143$ <br>
        _unit_: meters <br>

## Launch

#### `visualize_path.launch.py`

Start the visualize_path itself with its given argument file.

Options:
- __topic__: Path width topic to listen to and visualize. <br>
    _default_: none <br>
    _options_ (recommended): "/lidar_path_width" | "/camera_path_width" | "/path_width" <br>        


#### `sublaunch.visualize_path.launch.py`

This launch file is meant to be used by [start_all.launch.py](../decider/launch/start_all.launch.py) only.
