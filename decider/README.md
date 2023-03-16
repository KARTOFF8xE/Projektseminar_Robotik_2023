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
   - camera: The topic on which the wayfinding node publishes its results. (default: /camera_path_width)
   - lidar: The topic on which the lidar_curb_detection node publishes its results. (default: /lidar_path_width)
  - pub: The topic on which the decider node should publish its results. (default: /path_width)
 - params:
  - timer_delay