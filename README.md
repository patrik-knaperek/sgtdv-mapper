# **Mapper package**

___

&copy; **SGT Driverless**

**Authors:** Martin Lučan, Patrik Knaperek, Filip Botka

**Objective:** Creating a map of the track based on cone detections and vehicle pose.

___

## Introduction

A temporary and elementary substitute of SLAM node. `mapper` node takes the landmarks from cone detection fusion node and writes them to a map of the track based on the current vehicle pose.

### ROS Interface

**Subscribed topics**
* `/odometry/pose` [[`sgtdv_msgs/CarPose`](../sgtdv_msgs/msg/CarPose.msg)] : odometry pose fusion
* `/fusion/cones` [[`sgtdv_msgs/ConeWithCovStampedArr`](../sgtdv_msgs/msg/ConeWithCovStampedArr.msg)] : cone detections fusion

**Published topics**
* `/slam/pose`[[`sgtdv_msgs/CarPose`](../sgtdv_msgs/msg/CarPose.msg)] : car pose
* `/slam/map`[[`sgtdv_msgs/ConeArr`](../sgtdv_msgs/msg/ConeArr.msg)] : track map

*If `SGT_DEBUG_STATE` macro enabled*
* `/slam/debug_state`[[`sgtdv_msgs/DebugState`](../sgtdv_msgs/msg/DebugState.msg)] : node lifecycle information (active/inactive, number of landmarks)

**Parameters**
* `/euclid_treshold` : data association criteria (wether to create new landmark from cone detection or not)

### Related packages
* [`fusion`](../fusion/README.md) : `/fusion/cones` publisher
* [`odometry_interface`](../odometry_interface/README.md) : `/odometry/pose` publisher
* [`path_planning`](../path_planning/README.md) : `/slam/pose` and `/slam/map` subscriber

## Compilation

* standalone
```sh
$ cd ${SGT_ROOT}
$ catkin build mapper -DCMAKE_BUILD_TYPE=Release
```
* FSSIM setup
```sh
$ cd ${SGT_ROOT}
$ source ./scripts/build_sim.sh
```
* RC car setup
```sh
$ cd ${SGT_ROOT}
$ source ./scripts/build_rc.sh
```

### Compilation configuration
* [`SGT_Macros.h`](../SGT_Macros.h) :
  - `SGT_DEBUG_STATE` : publish node lifecycle information

## Launch
* standalone
```sh
$ source ${SGT_ROOT}/devel/setup.bash
$ roslaunch mapper mapper.launch
```
* FSSIM setup
```sh
$ source ${SGT_ROOT}/devel/setup.bash
$ roslaunch master trackdrive.launch
```
* RC car setup
```sh
$ source ${SGT_ROOT}/devel/setup.bash
$ roslaunch master rc.launch
```

### Launch configuration
* [`mapper.yaml`](./config/mapper.yaml)