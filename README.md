# RAL-benchmark-test
This repo contains the code for testing the RAL grasping benchmark on iCub.


## Overview
This repo provides the modules to acquire on the iCub robot all the data needed by the RAL grasping benchmark.
In particular, the repo contains the following modules:
- [`aruko-pose-estimation`](https://github.com/robotology-playground/RAL-benchmark-test/tree/master/src/aruko-pose-estimation), to estimate the 6D pose of the marker on the base of the layout and of the robot hand.
- [`camera-calibration-test`](https://github.com/robotology-playground/RAL-benchmark-test/tree/master/src/camera-calibration-test), to collect data to test the **camera-kinematics calibration** of the robot,  as specified by the benchmark.
- [`reaching-test`](https://github.com/robotology-playground/RAL-benchmark-test/tree/master/src/reaching-test), to collect data to test the **reachibility** of the robot,  as specified by the benchmark.
- [`grasp-and-stability`](https://github.com/robotology-playground/RAL-benchmark-test/tree/master/src/grasp-and-stability), to execute the grasp, the trajectory to test the grasp-stability and save the data as specified by the benchmark.

The `aruko-pose-estimation` module relies on [`gaze-ctril-lib`](https://github.com/robotology-playground/RAL-benchmark-test/tree/master/src/gaze-ctrl-lib), a high-level library to communicate with the iCub `iKinGazeCtrl`.

## Dependencies
- YARP
- icub-main
- opencv

## How to compile the code
```
git clone https://github.com/robotology-playground/RAL-benchmark-test.git
mkdir build
cd build
ccmake ..
make install
```

By default, all the modules provided within this repo are compiled.
You can compile separately the modules by switching `OFF` the relative CMAKE variables.


## How to run the code
- Run all the modules in [this file](https://github.com/robotology-playground/RAL-benchmark-test/blob/master/app/data_collection.xml.template)
  **Note**: There are two instances of `aruko-pose-estimation` running with different port prefixes to estimate both the layout pose and the hand pose (when collecting data for the camera-kinematics calibration test).
- Connect all the ports
- Use `rpc` communication to communicate with each module and collect data

### Test robot reachability

1. Connect to the module via `rpc` service:
```
yarp rpc /reaching-test/cmd:rpc
```

2. Ask the current pose to be tested
```ask_new_pose```
This pose is obtained from the benchmark layout (`file-layout` in the [`config.ini`](https://github.com/robotology-playground/RAL-benchmark-test/blob/master/src/reaching-test/conf/config.ini#L1)), where poses are expressed in the marker reference frame,  and expressed in the robot reference frame thanks to the estimate of the marker pose provided by `aruko-pose-estimation`.
3. Reach the pose just showed
```execute_new_pose```
4. Repeat from 2. until the last pose has been reached.
5. When finish, save all the reached poses with the command:
```save_reached_poses```
They will be saved in an [`.xml` file](https://github.com/robotology-playground/RAL-benchmark-test/blob/master/src/reaching-test/conf/config.ini#L4) with the code structure to be processed by the score computation script.

Extra commands:
- `reset` makes the counter of the poses 0 to start acquisition from scratch again;
- `increase_pose` increases the counter if you want to skip one specific pose.

### Test camera-kinematics calibration
The `rpc` commands are exactly the same as for the reachability test.
The difference between `reaching-test` and `camera-calibration-test` is that:
- in `reaching-test` the reached poses are collected using the kinematics;
- in `camera-calibration-test` the reached_poses are collected using a marker on mounted on the robot hand.


### Grasp object and test stability

1. Connect to the module via `rpc` service:
```
yarp rpc /grasp-and-stability/cmd:rpc
```
2. Ask to compute a grasping poses, communicating with [`cardinal-point-grasps`](https://github.com/robotology/cardinal-points-grasp)
```
get_grasp <arm_name> <object_name>
```
      - valid <arm_name> : `left` or `right`
      - valid <obejct_name> : the names of [these folders]()

 3. Ask to grasp the object:
 ```
 grasp
 ```
 This command will ask [ARE](http://www.icub.org/software_documentation/group__actionsRenderingEngine.html) to grasp the object and lift it of a given height (defined within the benchmark).

 4. Generate the other points of the trajectory to test grasp stability:
 ```
generate_trajectory
 ```
 5. Execute the trajectory
 ```
 execute_trajectory
 ```
 6. Save the grasping poses in a `.xml file` according to the benchmark:
 ```
 save_grasp_data <graspability> <grasped> <grasp_stability>
 ```
      where
      - <graspability> is `1` is the object is graspable by the robot and `0` otherwise
      - <grasped> is `11 is the robot grasped the object
      - <grasp_stability> is `n_reached_waypoints`/4 (since 4 is the number of total waypoints of the trajectory)


Other commands:
- `home`: make the robot go in the home position using ARE.
