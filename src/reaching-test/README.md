## reaching-test
This is a Yarp module to automatically collect the information required for the reachability test defined by **GRASPA 1.0**.
The goal of this test is to compute scores that evaluate the reachability of the robot within the benchmark layout.

<p align="center">
<img src="https://github.com/robotology-playground/RAL-benchmark-test/blob/master/misc/icub-reach.jpg" width=300>
</p>

The code makes the robot reach the desired pose, acquires the pose actually reached querying the forward kinematics and saves the files
according to the benchmark convention.
[Here](https://github.com/robotology-playground/RAL-benchmark-test/tree/master/experiment_data/right_arm/reaching_test) are 
the files filled with the data collected on the iCub.


## How to install the code
Information on installation is available on the main [README.md](https://github.com/robotology-playground/RAL-benchmark-test#how-to-compile-the-code).

## Before running the code
To collect coherent data, this code requires the [`aruco-pose-estimation`](https://github.com/robotology-playground/RAL-benchmark-test/tree/master/src/aruco-pose-estimation) module to be running for estimating the ArUco board pose, and consequently,
the pose of the reference frame of the layout.
This is required to properly save the poses actually reached by the robot in the layout reference frame.

## How to run the code
 - Launch the [`aruco-pose-estimation`](https://github.com/robotology-playground/RAL-benchmark-test/blob/master/app/data_collection.xml.template#L4) module to estimate the board pose, together with [its yarpview](https://github.com/robotology-playground/RAL-benchmark-test/blob/master/app/data_collection.xml.template#L40);
 - Select the [file](https://github.com/robotology-playground/RAL-benchmark-test/blob/master/src/reaching-test/conf/config.ini#L1)
    including the benchmark poses you want to test your robot on.
 - Launch [`reaching-test`](https://github.com/robotology-playground/RAL-benchmark-test/blob/master/app/data_collection.xml.template#L14) module;
 - Connect.

The data collection is to be done sending the commands to the robot via `rpc`:

1. Connect to the `rpc` port:
   ```
   yarp rpc /reaching-test/cmd:rpc
   ```

2. Ask the current pose to be tested:
   ```
   >> ask_new_pose
   ```
   The retrieved pose is one of the poses defined within the benchmark reachability files 
   expressed in the robot reference frame, thanks to the estimate of the pose of the reference frame layout provided by the `aruco-pose-estimation` module.
  

3. Reach the pose just showed:
   ```
   >> execute_new_pose <arm>
   ```
   where `<arm>` can `right` or `left`.
4. Repeat from 2. until the last pose is reached.
5. When finish, save all the reached poses with the command:
   ```
   >> save_reached_poses
   ```
   The saved poses are expressed in the layout reference frame, in order to be comparable with the benchmark desired poses.
   These data are stored is an [xml file](https://github.com/robotology-playground/RAL-benchmark-test/blob/master/src/reaching-test/conf/config.ini#L4) with the proper structure required by the benchmark.

**Note**: The procedure needs to be executed for the three set of poses defined by GRASPA 1.0.

Extra commands:
- `help (<module_name>)` provides all the method available or the help of a specific command;
- `reset` makes the counter of the poses 0 to start acquisition from scratch again;
- `increase_pose` increases the counter if you want to skip one specific pose.
 

## Example of collected ata 

<img src="https://user-images.githubusercontent.com/9597070/62217821-a6291300-b3ab-11e9-97e4-01e1b27e07e1.png" width=300> <img src="https://user-images.githubusercontent.com/9597070/62217786-93aed980-b3ab-11e9-91de-ee7f393722d6.png" width=300>

On the left, a subset of the desired poses, defined by GRASPA 1.0. 

On the right, the poses actually reached by the iCub using the right arm.
