# Motion Planner for Foliation Planning

## Overview
This repository contains the implementation of our constrained motion planning algorithms, tools, and utilities designed for solving foliation planning. That is, we modified Moveit! from source to make it support constrained motion planning and extanded constraint format(include grasp pose).

### Warning
THis Repo requires Ubuntu 18.04, and we did not test it in another version. Besides that, due to Ubuntu  18.04 met its EOL, any package update may cause the system failure. Therefore, we are planning to migrate all planners to Ubuntu 22.04 later. 

## Directory
Here are the important directories with their descriptions.

### moveit
The modified Moveit! source code to support constrained motion planning and foliation planning.

### moveit_msgs
Contains the modified moveit msg to support constrained motion planning.

### constrained_ompl_planners
The constrained motion planners develop on ompl, and it requires our Moveit! to load them for working properly.

