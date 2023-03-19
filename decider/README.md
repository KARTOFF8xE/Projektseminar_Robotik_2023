# decider

## Description
The decider package implements a node that merges the camera and lidar outputs into one.  
Furthermore it provides a launch file to start the decider itself and one to launch _everything_.

## Concept
For each datastream there is a buffer which will take in ALL published messages as a `filters::limit` regardless of their validity.  
On a set rate a method will be called that goes over all values from each buffer (starting with the oldest) and compares them. The first values which are close enough, meaning their time difference is lower then a given threshold.  
If no pair is found then the method exists without publishing a message, otherwise the pair gets returned and merged.  
Merging, in this case, means that if both, the cameras and the lidars, limits have e.g. a value to the left, the resulting left value would be the minimum¹ of both values.  
When only one of both limits has a valid left value, it is chosen. If none of both limits have a valid left value it would result in $0$.

* This package only publishes valid values. <br>

¹ The minimum is chosen over the average since in this case the values coming from the camera can be quite eradic and would have a high impact on the average. As to why those values are like that, consult the wayfinding [README](../wayfinding/README.md).

## Parameters
 - __topics__:
   - __sub__:
     - __camera__: The topic on which the wayfinding node publishes its results.  
       _type_: string  
       _default_: "/camera_path_width"
     - __lidar__: The topic on which the lidar_curb_detection node publishes its results.  
       _type_: string  
       _default_: "/lidar_path_width"
    - __pub__: The topic on which the decider node should publish its results.  
      _type_: string  
      _default_: "/path_width"
 - __params__:
    - __timer_delay__: The time between merging function calls.  
      _type_: int  
      _default_: $200$  
      _unit_: milliseconds
    - __time_diff_thr__: The threshold for what time difference is considered "close enough".  
      _type_: double  
      _default_: $1.0$  
      _unit_: seconds

## Launch

#### `decider.launch.py`

Start the decider itself with its parameter file.

Options:
 - __log_evel__: Setup the general output log level for this node.  
   _default_: info  
   _options_: debug | info | warn | error | fatal

#### `start_all.launch.py`

This launch file is made to start the whole assembly of wayfinding, lidar_curb_detection, decider and up to two visualize_path instances together.  
The topics for the flux and dmc bags automatically get loaded from the `launch/default_topics.yaml` and parameter file of the package.

Options:
 - __bag_dir__: The path to the directory containing the bag files.  
   _default_: &lt;cwd&gt;
 - __bag_rate__: The rate at which to replay the bag file.  
   _default_: $1.0$
 - __flux__: The path to the flux bag (relative to bag_dir) from which to play back from.  
   _default_: ''
 - __dmc__: The path to the dmc bag (relative to the bag_dir) from which to play back from.  
   _default_: ''
 - __debug_visualize__: Wether or not to activate visualizations in lidar_curb_detection and wayfinding that allow further insight in their inner workings.  
   _default_: False  
   _options_: True | False
 - __output_visualize__: Wether or not to visualize the output topics with visualize_path package.  
   _default_: none  
   _options_: both | lidar | camera | none
 - __log_level__: Setup the general output log level for this node.  
   _default_: info  
   _options_: debug | info | warn | error | fatal