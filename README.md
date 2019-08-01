# RAL-benchmark-test

This repo provides the modules to acquire on the iCub robot all the data needed by the RAL grasping benchmark.
In particular, the repo contains the following modules:
- [`aruco-pose-estimation`](https://github.com/robotology-playground/RAL-benchmark-test/tree/master/src/aruco-pose-estimation) to estimate the 6D pose of the marker on the base of the layout and on the robot hand.
- [`camera-calibration-test`](https://github.com/robotology-playground/RAL-benchmark-test/tree/master/src/camera-calibration-test) to collect data to test the **camera-kinematics calibration** of the robot,  as specified by the benchmark.
- [`reaching-test`](https://github.com/robotology-playground/RAL-benchmark-test/tree/master/src/reaching-test) to collect data to test the **reachibility** of the robot,  as specified by the benchmark.
- [`grasp-and-stability`](https://github.com/robotology-playground/RAL-benchmark-test/tree/master/src/grasp-and-stability) to execute the grasp, the trajectory to test the grasp-stability and save the data as specified by the benchmark.

The `aruco-pose-estimation` module relies on [`gaze-ctril-lib`](https://github.com/robotology-playground/RAL-benchmark-test/tree/master/src/gaze-ctrl-lib), a high-level library to communicate with the iCub `iKinGazeCtrl`.

The `experiment_data` directory contains results and logs of benchmark evaluation on iCub, as they will be published in the paper. 

## Dependencies
- [`YARP`](http://www.yarp.it/)
- [icub-main](https://github.com/robotology/icub-main)
- [opencv](https://opencv.org/) (for the `aruco-pose-estimation` module)

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

The detailed information on how to run each module is available iat the following links
- [`aruco-pose-estimation`](https://github.com/robotology-playground/RAL-benchmark-test/tree/master/src/aruco-pose-estimation)
- [`camera-calibration-test`](https://github.com/robotology-playground/RAL-benchmark-test/tree/master/src/camera-calibration-test)
- [`reaching-test`](https://github.com/robotology-playground/RAL-benchmark-test/tree/master/src/reaching-test) 
- [`grasp-and-stability`](https://github.com/robotology-playground/RAL-benchmark-test/tree/master/src/grasp-and-stability) 


