# Projektseminar Robotik 2022/23

## Packages

### decider
[->README](decider/README.md)

### filters
[->README](filters/README.md)

### lidar_curb_detection
[->README](lidar_curb_detection/README.md)

### visualize_path
[->README](visualize_path/README.md)

### wayfinding
[->README](wayfinding/README.md)

### zed_interfaces
[->README](https://github.com/stereolabs/zed-ros2-interfaces/blob/main/README.md) <!-- das war der default branch: f1517b1153ae85ef5820c3103731e0d1b94e7210 ?!? -->

## Setup & Info

### Repository usage

This Repository is meant as an assortment of ros2 (foxy) packages. Therefore it is to be used/cloned as the src folder in your ros workspace.

```
ros2_ws/
├─ build/
│  └─ …
├─ install/
│  └─ …
├─ log/
│  └─ …
└─ src/ <- This Repository
   └─ …
```

### rosbag foxy-future setup

```bash
$ sudo apt update
$ sudo apt install ros-foxy-test-msgs python-pybind11

$ source /opt/ros/foxy/setup.bash
$ mkdir -p ~/rosbag_ws/src
$ cd ~/rosbag_ws
$ git clone -b foxy-future https://github.com/ros2/rosbag2.git src
$ colcon build

$ source install/setup.bash
```

## Teilnehmername + MatrikelNr
- Johannes Kohl (~~@jkohl~~ @Nummer42O): 64751
- Georg Muck (~~@gmuck~~ @KARTOFF8xE): 63652
- Nico Zumpe (~~@nzumpe~~ @Tiltplank): 62351
