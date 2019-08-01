## grasp-and-stability

This is a Yarp module to make the robot grasp objects, execute the trajectory for assessing the grasp stability and save all the data
according to the **GRASPA 1.0** procedure.

<p align="center">
<img src="https://github.com/robotology-playground/RAL-benchmark-test/blob/master/misc/grasp.jpg" width=200> <img src="https://github.com/robotology-playground/RAL-benchmark-test/blob/master/misc/traj1.jpg" width=300> <img src="https://github.com/robotology-playground/RAL-benchmark-test/blob/master/misc/grasp.jpg" width=200> <img src="https://github.com/robotology-playground/RAL-benchmark-test/blob/master/misc/traj3.jpg" width=200>
</p>

[Here](https://github.com/robotology-playground/RAL-benchmark-test/tree/master/experiment_data/right_arm/grasps_data) are the files 
collecting all the data acquired with the iCub robot in the three layouts defined within the benchmark. The grasp planner we evaluated
with GRASPA 1.0 is the **Cardinal Point Grasp** [2].

## How to install the code
Information on installation is available on the main [README.md](https://github.com/robotology-playground/RAL-benchmark-test#how-to-compile-the-code).


## Before running the code
To collect coherent data, this code requires the [`aruco-pose-estimation`](https://github.com/robotology-playground/RAL-benchmark-test/tree/master/src/aruco-pose-estimation) module to be running for estimating the ArUco board pose, and consequently,
the pose of the reference frame of the layout.
This is required to properly save the poses actually reached by the robot in the layout reference frame.

Before running the code, you need to have the grasp planner algorithm running. Instructions on how to run the code are available [at this link](https://github.com/robotology/cardinal-points-grasp).

## How to run the code
- Launch the [`aruco-pose-estimation`](https://github.com/robotology-playground/RAL-benchmark-test/blob/master/app/data_collection.xml.template#L4) module to estimate the board pose, together with [its yarpview](https://github.com/robotology-playground/RAL-benchmark-test/blob/master/app/data_collection.xml.template#L40);
- Launch the [`grasp-and-stability`](https://github.com/robotology-playground/RAL-benchmark-test/blob/master/app/data_collection.xml.template#L24)
  module;
- Connect.

The data collection is to be done sending the commands to the robot via `rpc`:

1. Connect to the module via `rpc` service:
   ```
   yarp rpc /grasp-and-stability/cmd:rpc
   ```
2. Ask to compute a grasping poses, communicating with [`cardinal-point-grasps`](https://github.com/robotology/cardinal-points-grasp)
    ```
    >> get_grasp <arm> <object_name>
    ```
   
     - valid `<arm>` : `left` or `right`
     - valid `<obejct_name>` : the names of [these folders](https://github.com/fbottarel/RAL-benchmark-code/tree/master/data/objects/YCB)
      
3. Ask to grasp the object:
   ```
   >> grasp
   ```

4. Generate the points of the trajectory to test grasp stability:
   ```
   >> generate_trajectory
   ```
5. Execute the trajectory
   ```
   >> execute_trajectory
   ```
6. Add the data regarding the execution of the task:
   ```
   add_object_data <grasped> <grasp_stability> <hit_objects>
   ```
    - `<grasped>` is `1` if the robot grasped the object;
    - `<grasp_stability>` is `n_reached_waypoints`/4 (since 4 is the number of total waypoints of the trajectory);
    - <`hit_objects>` is the number of other objects hit while approaching the object (put `0.0` if evaluating in isolation).
    
7. Repeat from 2 to execute multiple trials and store the data.

8. Save the grasping poses in a `.xml file` according to the benchmark:
   ```
   >> save_grasp_data <graspability> 
   ```
    where
    - `<graspability>` is `1` if the object is graspable by the robot and `0` otherwise.


Other commands:
- `home`: make the robot go in the home position.
- `set_layout_name`: to properly associate the layout name to the object data. Proper values are:
`Benchmark_Layout_0`,  `Benchmark_Layout_1`,`Benchmark_Layout_2`.
- `reset_poses`: to clear all the data collected so far.
- `remove_last_grasp`: to remove the last grasp pose stored.
- `remove_last_object_data`: to remove the last data collected, such as `<grasped>`, `<grasp_stability>` and `<hit_obstacles>`.

[2] PDH. Nguyen, F. Bottarel, U. Pattacini, M. Hoffmann, L. Natale, G. Metta,
    _Merging physical and social interaction for effective human-robot collaboration_, 
    Humanoids, 2018.
