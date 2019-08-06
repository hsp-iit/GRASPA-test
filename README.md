# GRASPA-test
<p align="center">
<img src="https://github.com/robotology-playground/GRASPA-test/blob/master/misc/benchmark-setup2.jpg" width=400>
</p>

This repo provides the modules to acquire on the iCub robot all the data needed by **GRASP 1.0 (GRASPA is a Robotic Arm grAsping Performance benchmArk)**, a new benchmarking protocol recently proposed in the literature to evalute the performance of grasp planners.
More information on **GRASPA 1.0** is available [at this link](https://github.com/robotology/GRASPA-benchmark).

In particular, the repo contains the following modules:
- [`aruco-pose-estimation`](https://github.com/robotology-playground/GRASPA-test/tree/master/src/aruco-pose-estimation) to **estimate the 6D pose of the markers** on the **base** of the layout and on the **robot hand**.
- [`camera-calibration-test`](https://github.com/robotology-playground/GRASPA-test/tree/master/src/camera-calibration-test) to collect data to test the **camera-kinematics calibration** of the robot,  as specified by the benchmark.
- [`reaching-test`](https://github.com/robotology-playground/GRASPA-test/tree/master/src/reaching-test) to collect data to test the **reachibility** of the robot,  as specified by the benchmark.
- [`grasp-and-stability`](https://github.com/robotology-playground/GRASPA-test/tree/master/src/grasp-and-stability) to execute the **grasp, the trajectory to assess the grasp-stability** and **save the data** as specified by the benchmark.

The `aruco-pose-estimation` module relies on [`gaze-ctril-lib`](https://github.com/robotology-playground/GRASPA-test/tree/master/src/gaze-ctrl-lib), a high-level library to communicate with the iCub `iKinGazeCtrl`.

The [`experiment_data`](https://github.com/robotology-playground/GRASPA-test/tree/master/experiment_data) directory contains the **results**  of  thebenchmark evaluation on the iCub, as they will be published in the paper.

## Dependencies
- [`YARP`](http://www.yarp.it/)
- [`icub-main`](https://github.com/robotology/icub-main)
- [`opencv`](https://opencv.org/) (for the `aruco-pose-estimation` module)

## How to compile the code
```
git clone https://github.com/robotology-playground/GRASPA-test.git
mkdir build
cd build
ccmake ..
make install
```

By default, all the modules provided within this repo are compiled.
You can compile separately the modules by switching `OFF` the relative CMAKE variables.


## How to run the code

The detailed information on how to run each module is available at the following links
- [`aruco-pose-estimation`](https://github.com/robotology-playground/GRASPA-test/tree/master/src/aruco-pose-estimation)
- [`camera-calibration-test`](https://github.com/robotology-playground/GRASPA-test/tree/master/src/camera-calibration-test)
- [`reaching-test`](https://github.com/robotology-playground/GRASPA-test/tree/master/src/reaching-test)
- [`grasp-and-stability`](https://github.com/robotology-playground/GRASPA-test/tree/master/src/grasp-and-stability)
