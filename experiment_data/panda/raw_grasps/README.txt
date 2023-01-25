
This folder collects all the data in order to replicate the grasping poses that are executed in the trials for each layout and for each objects. 
For each trial the data are saved in a distinct file grasp_<object>_00<N>.json  where <object> is the name of the target object and <N> (from 0 to 4) is the trial number.
For example, the file in the path \dexnet\layout_0\grasp_foam_brick.json store the data for the first trial to grab the foam brick, the grasping pose is identified by dexnet in the Layout 0.

In each file the data should be arranged in a structure as shown here:  

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

"object" the name of the target object
The "GRASPA_board_pose" is the position of the GRASPA board w.r.t. the robot reference frame
The "grasp_pose" is the position of the gripper w.r.t. the robot reference frame
"grasped_score" defined in GRASPA
"stability_score": defined in GRASPA
