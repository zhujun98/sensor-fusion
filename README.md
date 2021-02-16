# Sensor Fusion with Kalman Filter

[![Build Status](https://dev.azure.com/zhujun981661/zhujun981661/_apis/build/status/zhujun98.sensor-fusion?branchName=master)](https://dev.azure.com/zhujun981661/zhujun981661/_build/latest?definitionId=3&branchName=master)

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

![Raw data 1 ](./misc/raw_lidar_1.gif)
![Processed data 1](./misc/processed_lidar_1.gif)
