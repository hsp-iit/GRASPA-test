## camera-calibration-test
This is a Yarp module to **automatically collect the information** required for the **camera calibration test** defined by **GRASPA 1.0**.
The goal of this test is to compute scores that evaluate the calibration of the vision system of the robot within the benchmark layout.

<p align="center">
<img src="https://github.com/robotology-playground/GRASPA-test/blob/master/misc/camera-calib2.jpg" width=300>
</p>

The code makes the robot **reach the desired pose, acquires the pose actually reached by using vision and saves the files
according to the benchmark standard**.
[Here](https://github.com/robotology-playground/GRASPA-test/tree/master/experiment_data/right_arm/camera_calibration) are
the files filled with the data collected on the iCub.


## How to install the code
Information on installation is available on the main [README.md](https://github.com/robotology-playground/GRASPA-test#how-to-compile-the-code).

## Before running the code
To collect coherent data, this code requires both the [`aruco-pose-estimation`](https://github.com/robotology-playground/GRASPA-test/tree/master/src/aruco-pose-estimation) modules to be running for estimating the **pose of the
benchmark layout** and **of the robot hand**.
The pose of the **layout reference frame** is required to **properly save the poses** actually reached by the robot.
The pose of the **hand** is acquired using markers in order t**o test the calibration of the robot vision system**.

## How to run the code
 - Launch both the [`aruco-pose-estimation`](https://github.com/robotology-playground/GRASPA-test/blob/master/app/data_collection.xml.template#L4) modules to estimate the board and the hand pose, together with [their yarpviews](https://github.com/robotology-playground/GRASPA-test/blob/master/app/data_collection.xml.template#L40);
 - Launch [`camera-calibration-test`](https://github.com/robotology-playground/GRASPA-test/blob/master/app/data_collection.xml.template#L19) module;
 - Connect.

The data collection is to be done sending the commands to the robot via `rpc`:

1. Connect to the `rpc` port:
   ```
   yarp rpc /camera-calibration-test/cmd:rpc
   ```

2. **Ask the current pose** to be tested:
   ```
   >> ask_new_pose
   ```
   The retrieved pose is **one of the poses defined within the benchmark reachability files
   expressed in the robot reference frame**, thanks to the estimate of the pose of the reference frame layout provided by the `aruco-pose-estimation` module.  

3. **Reach the pose** just showed:
   ```
   >> execute_new_pose <arm>
   ```
   where `<arm>` can be either `right` or `left`.


   <p align="center">
   <img src="https://github.com/robotology-playground/GRASPA-test/blob/master/misc/camera-calib-viewer.jpg" width=300>
   </p>

   When the robot has reached the pose, the **robot looks at its hand** and **acquired the reached pose** by using the estimate given by
   the set of **two markers**.

4. **Repeat** from 2. until the last pose is reached.
5. When finish, **save all the reached poses** with the command:
   ```
   >> save_reached_poses
   ```
   The saved poses are expressed in the **layout reference frame**, in order to be comparable with the benchmark desired poses.
   These data are stored is an [xml file](https://github.com/robotology-playground/GRASPA-test/blob/master/src/camera-calibration-test/conf/config.ini#L4) with the proper structure required by the benchmark.

**Note**: The procedure needs to be executed for the [**set of poses**](https://github.com/robotology/GRASPA-benchmark/tree/master/data/scenes/camera_calibration) defined by **GRASPA 1.0**.

Extra commands:
- `help (<module_name>)` provides all the methods available or the help of a specific command;
- `reset` makes the counter of the poses 0 to start acquisition from scratch again;
- `increase_pose` increases the counter if you want to skip one specific pose (**Note** this can be done just for debugging. All the poses needs to be acquired to obtain properly scores.).
