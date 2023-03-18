# filters

## Description
The filters package does not implement a node. It rather providesa library of some simple filters for the [lidar_curb_detection-](../lidar_curb_detection/README.md) and the [wayfinding](../wayfinding/README.md) packages, aswell as an internal datatype for handling limits.

## post_filters
For evaluating the `Bubblefilter` and the `Noisefilter` only valid values are used. All three filters only check for valid values.

### Bubblefilter
This filter is used to detect runaways. The to evaluate value is compared pairwise with a specified amount of its predecessors and successors. For the comparison the difference of each pair is calculated. If the difference is below a given threshold, the specific predecessor (or successor) is called a "neighbor". In case there are to less neighbors. The checked value gets declared as invalid (and soone made negative).

### Noisefilter
This filter is used to detect the noise surround the to evaluate Limit. Similar to the previous filter we evaluate any value pairwise by help of its predecessors and successors. But this time we take the difference of any value with the next value. So for $n$ points we receive $n-1$ differences. If the average of those differences is above a given threshold, the checked value gets declared as invalid (and soone made negative).

### Islandfilter
This filter is used to detect Islands. During tests we saw that sometimes a tiny amount of valid values exists like an Island inside of a huge amount of invalid values. Those values are not really usefull. To illuminate those we check for any value itself how many predecessors and successors are valid. If the amount of valid values is too low, the checked value gets declared as invalid (and soone made negative).

## encoding_filter
This function provides a mapping from ros2 sensor_msgs/msg/Image encodings to OpenCV image encodings and color conversions.