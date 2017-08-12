# CarND-Path-Planning-Project

---

[//]: # (Image References)

[image1]: Path_Planning_FlowChart.png "Model Flowchart"

## Table of Contents

1. Project Goal
2. Model Overview
3. Code Structure

## Project Goal

The objective of this project is to implement a behavior planning module and a trajectory generation module to successfully drive a full lap without incidents.

## Model Overview

A flowchart of the model is demonstrated here.

![Model Flowchart][image1]

The model consists of mainly three parts:
1. preprocessing
2. behavior planning
3. trajectories generation

### Preprocessing

The preprocessing step is responsible for two tasks:
1. read and initialize previous state variables, such as s, d, speed, etc.
1.1 despite the fact that the 
2. process data from sensor fusion and store them in the list for each lane respectively.

## Code Structure
