# wayfinding

## Description
The wayfinding implements a node that intents to calculate the distances to the left and right path borders by interpreting an input image.  
It requires the camera to be mounted straight forward with no tilt or roll. It also requires a 3D pointcloud corresponding to the image pixels.

## Concept
The algorithm can be split into the six main parts: top down transformation, metric aquisition, metric evaluation, line evaluation, transformation into 3D space and filtering.  
Those segments will be explained below.

### Top Down Transformation
The top down transformation relies on OpenCVs `warpPerspective` and `findHomography` functions.  
The latter takes in four points on the source and four points on the target image to produce an according transformation matrix. This transformation matrix then gets used by `warpPerspective` to transform/warp the source image accordingly.  
The destination points for our instance are simply the corner points of the new image which is the same size as the original. This is relatively arbitrary though since a output image size is needed but nothing really restricts this in this case. So it is chosen like that simply for consistency reasons.  
The source points on the other hand are a more delicate matter. Those points lay on the lines going to the vanishing point of the image and can be calculated when that point is known. This must be the case so that the lines in the resulting image can all savely be assumed to be parallel as seen below:  
![Source: Wikipedia, 18.03.2023](https://upload.wikimedia.org/wikipedia/commons/3/3c/Vanishing_point.svg)  
Since the vanishing point is the one spot where parallel image lines eventually converge/cross a RANSAC approach could be used to calculate it. But this very quickly turned out to be unreliable and was deemed not feasable.  
Instead the camera position was assumed, or rather required, to be fixed and [this](https://www.coursera.org/lecture/robotics-perception/vanishing-points-how-to-compute-camera-orientation-flqF4) approach from the University of Pennsylvania was used to determine the vanishing point from the intrinsic camera matrix K and the current orientation (static + temporary offsets meassured by the camera-internal IMU) which provided a way more reliable result.  
With the vanishing point known the source points for the vanishing lines where chosen far outside the original image to capture as much as possible of the road ahead. Be aware that this approach results in black triangles or dead spots in the resulting image due to an obvious lack of data.
Lastly an upper horizontal cut-off line was chosen at 20% between the vanishing points y value and the bottom of the image to maximize the viewport but minimize tearing of the image in the warping process.

### Metric Aquisition
Firstly we should describe what the metric is:  
The metric describes the quality of the image in terms of noisiness of the edge image and ammount of rouge lines that do not account to our efforts.
The higher the metric the worse the image is for our algorithm and the harsher the filter algorithms have to be.

It uses hough lines detected in the image to display this quality numerically.
The hough lines are, in our case, a set of lines detected in the image by a probabilistic Hough transform. This function `cv::HoughLinesP` takes in a black and white edge image produced by `cv::Canny` using the Canny algorithm and produces a set of lines which adhere to the following rules:
 - they must get at least a specified ammound of votes
 - they must have at least a specified length
 - the points they are made of can only be a specified length apart

The final metric value itself is just an accumulation of the single values of _every_ deteced line.
Their respective metrics get calculated like:
$ line\_metric = angle\_factor \cdot length + horizontal\_factor \cdot horizontal + vertical\_factor \cdot vertical $
with the values as follows (all factors in the range from 0.0 to 1.0):
 - `angle_factor`: lower the closer the line angle matches the optimal angle
 - `line_length`: length of the line
 - `horizontal_factor`: higher the further away from the horizontal threshold the line center is
 - `horizontal`: horizontal distance between line and image center
 - `vertical_factor`: higher the further away from the vertical threshold the line center is
 - `vertical`: distance from the top of the image

### Metric Evaluation
The actual evaluation turned out to be way less complex then initially anticipated.
It turned out that the actual parameters for the canny as well as probabilistic hough algorithms didn't need adaptive tweaking so only two thresholds were introduced:
 - `metric < 5'000`: For metric values that are too low it turned out to yield better results to disable prefilters alltogether to get more lines to evaluate.
 - `metric > 100'000`: For metric values that are too high a gaussian filter with 25% higher Y then X filtering produces the required results.
For all other cases the default median filter did well enough. Any filters stronger then the employed gaussian filter blurred the image to a point where it became almost completely useless.

### Line Evaluation
With those "new" filters the same process to extract lines from the image was employed but this time to evaluate them directly.
For this purpose the lines got filtered to be outside of "the tube", a left and right threshold around the center of the image defined relative to the images width.
The purpose of this is to ignore any lines that would lay in the middle of the road ahead and originate from patches, sinkholes, manhole covers, etc. The width of this tube is 20% by default in order not to accidently remove the wrong lines on narrower roads.
Also the lines get filtered by angle. Since the image is transformed into a top down projection all path borders should be about vertical, so every angles below a certain angle (default: 75°) get removed aswell since they are most certainly irrelevant.

To determine points on the left and right a scan line algorithm was employed which scans horizontally at a certain height, relative (from the bottom) to the image height, for the first intersection with one of the detected and filtered lines.  
This relative height is 5% by default. A lower value was tried too in an effort to bring the scan line closer to where the lidar takes its values but this idea was discarded as the scan line was so low that it missed most of the detected lines and searched primarily in the dead zones of the projection.

### 3D Translation
The translation into three dimensional values for real world application/evaluation depends on a 3D pointcloud which is ordered in the shape of the source image.
As it can be represented as an two dimensional, three channel Matrix, just like the source image itself, it was experimented with transforming the pointcloud itself via `cv::WarpPerspective` but this resulted in undefined behavior showing as segmentation faults on reading values from the resulting cloud.
So instead it was opted for reverse transformation of the detected points in the top down view to aquire the corresponding points in the orignal image. Those were then used with the original point cloud to receive the 3D points and calculate the real live distance to the left and right. If no point is found for a side, this sides value defaults to zero.

### Postfilters
For a detailed description of the used filters, take a look at the post filters section [here](../filters/README.md).

## Parameters:
 - __topics__:
   - __flux__:
     - __camera_info__ : The topic on which to find the `sensor_msgs/msg/CameraInfo` topic for the used camera.  
       _type_: string  
       _default_: "/zed2i_c1/zed_node_c1/rgb/camera_info"
     - __imu__ : The topic on which to find the `sensor_msgs/msg/Imu` topic for the used IMU.  
       _type_: string  
       _default_: "/imu/data"
     - __pointcloud__ : The topic on which to find the `sensor_msgs/msg/PointCloud2` topic correlating to the camer.  
       _type_: string  
       _default_: "/zed2i_c1/zed_node_c1/point_cloud/cloud_registered"
     - __rgb_image__ : The topic on which to fine the `sensor_msgs/msg/Image` topic from the camera feed.  
       _type_: string  
       _default_: "/zed2i_c1/zed_node_c1/rgb/image_rect_color"
   - __pub__: The topic on which wayfinding should publish its results.  
     _type_: string  
     _default_: "/camera_path_width"
 - __params__:
   - __angle_filter__: The minimum angle a line can have to still be evaluated.  
     _type_: double  
     _default_: 75.0
   - __optimal_line_angle__: The angle that is considered the most realistic for a path border in a top down view perspective.  
     _type_: double  
     _default_: 90.0
   - __rel_scan_line_height__: The relative height (from the bottom) at which to scan for left and right borders.
     _type_: double  
     _default_: 0.05
   - __filter__:
     - __distance_thr__: Threshold that tells the filter how far away another point can be to still be considered valid.  
       _type_: double  
       _default_: 0.2
     - __quantity_check__: Amount of limits that are checked in each direction (front and behind).  
       _type_:  int  
       _default_: 14
     - __quantity_thr__: Minimum required amount of valid points inside the __distance_thr__.  
       _type_: int  
       _default_: 15
   - __avg_dist__:
     - __quantity_check__: Amount of limits that are checked in each direction (front and behind).  
       _type_: int  
       _default_: 15
     - __counter_thr__: Minimum amount of distance differences that are needed to calculate the average. Lower ammounts result in invalid limits.  
       _type_: int  
       _default_: 12
     - __avg_dist_thr__: The maximum a distance difference can be while still being considered valid.  
       _type_: double  
       _default_: 0.1
   - __island__:
     - __quantity_check__: Amount of limits that are checked in each direction (front and behind).  
       _type_: int  
       _default_: 45
     - __counter_thr__: The quantity of valid limits to validate the current up limit.  
       _type_: int  
       _default_: 40
 - __debug__:
   - __metric_output_file__: File name or path (without extension) to determine where the output csv-file should get saved.  
     _type_: string  
     _default_: "metric_table"

## Launch

#### `wayfinding.sublaunch.launch.py`

This launch file is meant to be used by [start_all.launch.py](../decider/launch/start_all.launch.py) only.

Options:
 - __visualize__: Wether or not to activate visualizations that allow further insight in the workings.  
   _default_: False  
   _options_: True | False
 - __log_level__: Setup the general output log level for this node.  
   _default_: info  
   _options_: debug | info | warn | error | fatal

#### `wayfinding.launch.py`

Start the wayfinding node itself with its param file.  
The topics for the flux and dmc bags automatically get loaded from the [launch/default_topics.yaml](launch/default_topics.yaml) and parameter file of the package.

Options:
 - __bag_dir__: The path to the directory containing the bag files.  
   _default_: &lt;cwd&gt;
 - __bag_rate__: The rate at which to replay the bag file.  
   _default_: 1.0
 - __flux__: The path to the flux bag (relative to bag_dir) from which to play back from.  
   _default_: ''
 - __dmc__: The path to the dmc bag (relative to the bag_dir) from which to play back from.  
   _default_: ''
 - __visualize__: Wether or not to activate visualizations that allow further insight in the workings.  
   _default_: False  
   _options_: True | False
 - __log_level__: Setup the general output log level for this node.  
   _default_: info  
   _options_: debug | info | warn | error | fatal
 - __use_gdb__: Wether or not to start the node in gdb in a seperate gnome-terminal window.  
   _note_: Add `--cmake-args -DCMAKE_BUILD_TYPE=Debug` or `--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo` to your `colcon build` command to give gdb access to symbols.  
   _default_: False  
   _options: True | False
 - __echo_output__: Wether or not to automatically echo the output on the command line.  
   _default_: False  
   _options_: True | False