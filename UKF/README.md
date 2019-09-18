# LIDAR and RADAR Data Fusion with Unscented Kalman Filter
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)
[![Build Status](https://travis-ci.org/zhujun98/sensor-fusion.svg?branch=master)](https://travis-ci.org/zhujun98/sensor-fusion)

Jun Zhu

## Introduction

In this project, a stream of simulated mixed LIDAR and RADAR data will be used 
to estimate the trajectory of an object moving in a curved trajectory by using 
the unscented Kalman filter for both data sets. The theory and formulas used in 
this project are summarized [here](../KalmanFilter.pdf). 

The measurement noises for the LIDAR data are Sx = Sy = 0.15 m. 

The measurement noises for the RADAR data are Sr = 0.30 m, Sphi = 0.03 rad and Sr' = 0.3 m/s. 

The motion model used in this project is the __CTRV (constant turn rate and velocity)__ model, as shown below:

![CTRV](../misc/CTRV_model.png)

## Dependencies

#### [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) >= 3.3.3

```shell script
$ git clone --branch 3.3.7 https://github.com/eigenteam/eigen-git-mirror.git
$ cd eigen-git-mirror
$ mkdir build && cd build
$ cmake .. && make install
```

## Build and run

#### Build

```shell script
$ mkdir build && cd build
$ cmake .. && make
```

#### Run

```shell script
$ ./ukf input output
```

## Visualize the result

You can use this [Jupyter notebook](../kf-visualization.ipynb) to build the code, 
process the [data](./data) and visualize the result.

Check https://nbviewer.jupyter.org/ if github fails to render the notebook.