This directory contains data relative to GRASPA evaluation on the Franka Panda.
Find 
- the gripper description and meshes in the `panda_gripper` subdirectory
- the experiment raw logs in `raw_grasps`
- the result of the experiment themselves for each algorithm, saved as `panda_<algorithm_name>`

Each folder, `panda_<algorithm_name>`, has the same structure that includes three subfolders:
- `camera_calibration` collects results of calibration procedure for each layout (empty, 0, 1 and 2). The data, which are stored in different .xml files,
   provides the reached poses, which are acquired through the robot vision system. In particular each pose is saved with tags `<ManipulationObject name="Reachable_frameXY">` with X={0,1,2,3} and Y={0,1,2,3}.
- `grasps_data` 
- `reaching_test`
