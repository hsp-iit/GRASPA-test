
The raw data of the grasping poses, which are executed in the experiments, are collected in this folder. 
For each grasp trial, the data are saved in a distinct file `grasp_<object>_00<N>.json`,  where `<object>` is the name of the target object and `<N>` (from 0 to 4) is the trial number.
For example, the file in the path `\dexnet\layout_0\grasp_foam_brick.json` stores the data of the first trial to grab the foam brick, the grasping pose is identified by Dexnet in the Layout 0.

In each file, the data are arranged in a structure as shown here:  

```json
{
 "GRASPA_board_pose":
  {
  "position": 
    {
    "y": -0.26138328138338995,
    "x": 0.40437682723353324, 
    "z": 0.14416693447880322
    }, 
  "orientation": 
    {
    "y": -0.0061241152713158295,
    "x": 0.012574988059201883,
    "z": -0.6931334527954932,
    "w": 0.7206735609853135
    }
   },
 "grasped_score": "1",
 "object": "foam_brick",
 "grasp_pose": 
  {
  "position": 
    {
    "y": 0.14970505169335224,
    "x": 0.5470586712787067, 
    "z": 0.1733135580528603
    },
  "orientation": 
    {
    "y": 0.9903418321570593,
    "x": 0.13716849011509727,
    "z": 0.01717233808780748,
    "w": 0.010628810090584459
    }
  }, 
  "stability_score": 1.0
 }
```
  
For sake of clarity, the main parameters of the file are described in the table:

| Parameter | Description | 
| --- | --- | 
**object** | Name of the target object|
**GRASPA_board_pose** | Position of the GRASPA board w.r.t. the robot reference frame |
**grasp_pose**  | Position of the gripper w.r.t. the robot reference frame |
**grasped_score** | Grasped quality score (defined in GRASPA)|
**stability_score** | Grasp stability score (defined in GRASPA)|

For each grasp trial, we reported also the .xml file with the GRASPA results. These .xml files are copied from the `grasps_data` folder, which is described [here](https://github.com/hsp-iit/GRASPA-test/tree/master/experiment_data/panda).
