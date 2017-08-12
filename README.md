# CarND-Path-Planning-Project

---

[//]: # (Image References)

[image1]: Path_Planning_FlowChart.png "Model Flowchart"
[image2]: trajectories_derivation.png "Trajectories Derivation"

## Table of Contents

1. Project Goal
2. Model Overview
3. Code Structure

## 1. Project Goal

The objective of this project is to implement a behavior planning module and a trajectory generation module to successfully drive a full lap without incidents.

## 2. Model Overview

A flowchart of the model is demonstrated here.

![Model Flowchart][image1]

The model consists of mainly three parts:
1. Preprocessing
2. Behavior planning
3. Trajectories generation

### 2.1 Preprocessing

The preprocessing step is responsible for three tasks:
1. refine the waypoints in the map using spline fitting
2. read and initialize previous state variables, such as s, d, speed, etc.
3. process data from sensor fusion and store them in the list for each lane respectively.

#### 2.1.1 Map spline fitting

In the original map, the s distance between each way point is about 30 meters, which is too coarse for trajectories generation. Therefore, they are refined by spline fitting as developed by [Tino Kluge](http://kluge.in-chemnitz.de/opensource/spline/). [x, y] coordinates are expressed as a function of s.

#### 2.1.2 Previous states initializing

The latest data in the history are imported/generated in order to generate smooth paths, including previous s, d, speed, acceleration.
One thing noteworthy is that, instead of storing previous path in [x, y] coordinates, they are stored in Frenet coordinates [s, d] in the code, in order to avoid the error introduced by translating between [x, y] and [s, d], since the trajectories are created in Frenet coordinates.

#### 2.1.1 Sensor data processing

The sensor fusion data are separated into three lists, each for one lane respectively. The states of the cars in each lane are further sorted so they are in descending order in terms of s, so it is easier to post process the data iin the behavior planning module.

### 2.2 Behavior planning

The behavior module determines the end status of the car, which will be passed to trajectories generation module later. this module consists of two functions,

```
void lane_keeping(vector<vector<double>>  sensor_car_list_current, double car_s, double prev_s, double s_buffer, double &v_init, double &v_end, double car_speed, bool &flag)
```
and
```
void lane_changing(double car_s, double prev_s, double &v_init, double &v_end, double car_speed, double car_d, double &d_init, double &d_end, int direction, bool &flag)
```

The former function calculates the status in the scenario of lane keeping, while the latter one deals with the scenario of lane changing.

### 2.3 Trajectories generation

The trajectories generation module takes in the end status from behavior planning and the start status from path history and generate a smooth path to minimize jerk.
Because the Jerk Minimizing Trajectory (JMT) requires the position, speed and acceleration for both the end and start status to be known, it is assumed that the speed of the car follows a sigmoid equation. For example:

![Trajectories Derivation][image2]

Then we are able to evaluate the status of the car after certain time increment and utilize the JMT function to generate a smooth path. 

## 3. Code Structure
