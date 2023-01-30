This directory contains data relative to GRASPA evaluation on the Franka Panda.
Find 
- the gripper description and meshes in the `panda_gripper` subdirectory
- the experiment raw logs in `raw_grasps`
- the result of the experiment themselves for each algorithm, saved as `panda_<algorithm_name>`

Each folder, `panda_<algorithm_name>`, has the same structure that includes three subfolders:
- `camera_calibration` collects results of calibration procedure for each layout (empty, 0, 1 and 2). The data, which are stored in different .xml files,
   provides the reached poses, which are acquired through the robot vision system. In particular each pose is saved with tags `<ManipulationObject name="Reachable_frameXY">` with X={0,1,2,3} and Y={0,1,2,3}.
- `grasps_data` stores the grasping results for each algorithm and for each layout. The data are saved in .xml files that have `<ManipulationObject name="target_object">` tag in order to identify the target object for the experiment. The model of the object (for simulation and visualization) is provided by the tags `<Visualization><File type="inventor">path_to_file.stl</File></Visualization>`  and     `<CollisionModel><File type="inventor">path_to_file.stl</File></CollisionModel>`. The robotic manipulation system and the scenario are described in the tag `<GraspSet EndEffector="robot_end_effector" RobotType="robot_arm_type" name="Benchmark_Layout_0">`. For each trial, the preshape poses are collected in the tag `<Grasp Creation="auto" Preshape="Grasp Preshape" name="Grasp 0" quality="0">` 


  
- `reaching_test`
