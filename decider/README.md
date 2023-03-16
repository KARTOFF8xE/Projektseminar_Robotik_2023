# decider

## Description
The decider package implements a node that merges the camera and lidar outputs into one.

## Concept
For each datastream there is a buffer which will take in ALL published messages as a `filters::limit` regardless of their validity.
On a set rate a method will be called that goes over all values from each buffer (starting with the oldest) and compares them. The first values which are close enough, meaning their time difference is lower then a given threshold.
If no pair is found then the method exists without publishing a message.
Otherwise the pair gets returned and merged.
Merging means in this case that, if both the cameras and lidar limits have e.g. a value to the left, the resulting left value would be the minimum of both values.
The minimum is chosen over the average since in this case the values coming from the camera can be quite eradic and would have a high impact on the average. As to why those values are like that, consult the wayfinding [README](../wayfinding/README.md).
If none of both limits have a valid left value it would result in 0.

## Parameters
 - topics:
  - sub:
   - camera: The topic on which the wayfinding node publishes its results. (type: string, default: /camera_path_width)
   - lidar: The topic on which the lidar_curb_detection node publishes its results. (type: string, default: /lidar_path_width)
  - pub: The topic on which the decider node should publish its results. (type: string, default: /path_width)
 - params:
  - timer_delay: The time between merging function calls. (type: int, default: 200, unit: ms)
  - time_diff_thr: The threshold for what time difference is considered "close enough". (type: double, default: 1.0, unit: s)

## Launch

### `decider.launch.py`

Start the decider itself with its parameter file.

Options:
 - log_evel: Setup the general output log level for this node. (default: info, options: debug | info | warn | error | fatal)

### `start_all.launch.py`

This launch file is made to start the whole assembly alltogether in one go.
Meaning: wayfinding, lidar_curb_detection, decider and up to two visualize_path instances.
The topics for the flux and dmc bags automatically get loaded from the packages `launch/default_topics.yaml` and their parameter file.

Options:
 - bag_dir: The path to the directory containing the bag files. (default: &lt;cwd&gt; )
 - bag_rate: The rate at which to replay the bag file. (default: 1.0)
 - flux: The path to the flux bag (relative to bag_dir) from which to play back from. (default: '')
 - dmc: The path to the dmc bag (relative to the bag_dir) from which to play back from. (default: '')
 - debug_visualize: Wether or not to activate visualizations in lidar_curb_detection and wayfinding that allow further inside in their inner workings. (default: False, options: True | False)
 - output_visualize: Wether or not to visualize the output topics with visualize_path package. (default: none, options: both | lidar | camera | none)
 - log_level: Setup the general output log level for this node. (default: info, options: debug | info | warn | error | fatal)