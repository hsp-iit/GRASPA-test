## aruco-pose-estimation

This is a Yarp module to estimate the pose of either a ArUco marker board and single Aruco Markers.
The code has been designed starting from the OpenCv sample codes for estimating the pose of 
[boards](https://github.com/opencv/opencv_contrib/blob/master/modules/aruco/samples/detect_board.cpp) and [single marker pose](https://github.com/opencv/opencv_contrib/blob/master/modules/aruco/samples/detect_markers.cpp).

## How to install the code
Information on installation is available on the main [README.md](https://github.com/robotology-playground/RAL-benchmark-test#how-to-compile-the-code).


## Before running the code
- Put the robot in front of the benchmark layout. The pose needs to be fixed in order to have consistent data during the acquisition.

 TODO: ADD IMAGE
 
- Put the markers on the robot hand. We obtained good performance by using:
   - Original ArUco  dictionary
   - Marker lenghts of 0.04 m
   - Markers id 65 and 67. You can generate and print your markers [at this page](http://chev.me/arucogen/).
   
    TODO ADD IMAGE
    
  **Note**: Be sure to  attach the marker on the dorso of the hand as shown in the image. The markers should also be printed on a rigid
  support in order to have stable pose estimates. We're planning to provide some printable CAD models to be mounted on the robot hand as
  support for the markers. The positioning of the other marker is less strict, since the relative pose between the two is estimated during
  an initial calibration phase.
   
    
## How to run the code
- Turn on your iCub and run all the [basic modules](https://github.com/robotology/icub-main/blob/master/app/iCubStartup/scripts/iCubStartup.xml.template).
- Turn on the robot cameras.
- Launch the `aruco-pose-estimation` modules as expressed in [this xml](https://github.com/robotology-playground/RAL-benchmark-test/blob/master/app/data_collection.xml.template#L4) together 
  with the [yarpviews](https://github.com/robotology-playground/RAL-benchmark-test/blob/master/app/data_collection.xml.template#L40). 
  According to the configuration file used, the module can be used to estimate the pose of the [marker board](https://github.com/robotology-playground/RAL-benchmark-test/blob/master/src/aruco-pose-estimation/conf/config_base.ini) 
  or of [the markers on the robot hand](https://github.com/robotology-playground/RAL-benchmark-test/blob/master/src/aruco-pose-estimation/conf/config_hand.ini#L11).
  
  
## `aruco-pose-estimation` for markers board
This is an example of estimated pose of the marker board.

TODO ADD IMAGE

On the left you can see the pose of the Aruco board. On the right, we draw the estimated pose of the reference frame of the benchmark layouts.
  
The estimated pose is available in streaming at the port with name: `/<port-prefix>/marker-estimate/estimate:o`.
Different port prefixes are used to run multiple modules of `aruco-pose-estimation`, [one](https://github.com/robotology-playground/RAL-benchmark-test/blob/master/src/aruco-pose-estimation/conf/config_base.ini#L7) for estimating the board pose and
[one](https://github.com/robotology-playground/RAL-benchmark-test/blob/master/src/aruco-pose-estimation/conf/config_hand.ini#L9) for the markers on the robot hand.
  
## `aruco-pose-estimation` for markers on the robot hand
Before using the module to get the pose of the robot hand
  
