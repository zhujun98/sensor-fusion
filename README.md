# Sensor Fusion with Kalman Filter
Jun Zhu
[![Build Status](https://travis-ci.org/zhujun98/sensor-fusion.svg?branch=master)](https://travis-ci.org/zhujun98/sensor-fusion)


![alt text](./misc/theme.png)

Sensor fusion with different implementions of Kalman filter.

A stream of simulated mixed Lidar and Radar data will be used to estimate the 
trajectory of a car moving in curved trajectory. The theory used in this project 
is summarized [here](./KalmanFilter.pdf)

![alt text](./misc/flow_chart.png)

### [1] [Extended Kalman Filter](./EKF) (LIDAR and RADAR)

![](./misc/EKF_show.png)

### [2] [Uncented Kalman Filter](./UKF) (LIDAR and RADAR)

![](./misc/UKF_show.png)

### [3] [LIDAR Obstacle Detection](./SFND_Lidar_Obstacle_Detection)
