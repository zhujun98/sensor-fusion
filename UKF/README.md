# LIDAR and RADAR Data Fusion with Unscented Kalman Filter
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Jun Zhu

## Introduction

In this project, a stream of simulated mixed LIDAR and RADAR data will be used to estimate the trajectory of an object moving in a curved trajectory by using the unscented Kalman filter for both data sets. The theory and formulas used in this project can be found [here](../KalmanFilter.pdf). The measurement noises for the LIDAR data are $\sigma_x$ = $\sigma_y$ = 0.15 m. The measurement noises for the RADAR data are $\sigma_r$ = 0.30 m, $\sigma_\phi$ = 0.03 rad and $\sigma_\dot{r}$ = 0.3 m/s. 

## Simulation results

The results for two data sets with UKF implementation are shown below.

* [Test data 1](./data/sample-laser-radar-measurement-data-1.txt)

RMSE  | LIDAR and RADAR 
-------    | ------------------------
x         |  0.072
y         | 0.080
vx       | 0.59
vy       | 0.58

![basic-data-1](./output/basic-output-1.png)
![basic-data-1](./output/basic-output-1-vx.png)
![basic-data-1](./output/basic-output-1-vy.png)
![basic-data-1](./output/basic-output-1-lidar_NIS.png)
![basic-data-1](./output/basic-output-1-radar_NIS.png)

*[Test data 2](./data/sample-laser-radar-measurement-data-1.txt)

RMSE  | LIDAR and RADAR 
-------    | ------------------------
x         |  0.19
y         | 0.19
vx       | 0.39
vy       | 0.52

![basic-data-2](./output/basic-output-2.png)
![basic-data-2](./output/basic-output-2-vx.png)
![basic-data-2](./output/basic-output-2-vy.png)
![basic-data-2](./output/basic-output-2-lidar_NIS.png)
![basic-data-2](./output/basic-output-2-radar_NIS.png)


## Summary

Due to the discrepancy between these two data sets, it is difficult to tune the acceleration noise and yaw acceleration noise to meet the NIS requires of both. As a compromise, an acceleration noise of 0.7 m/s^2^ and a yaw acceleration noise of 0.6 rad/s^2 are used here.