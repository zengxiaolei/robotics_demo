Share my robotic demos with all! Add star🌟 to this repo if you like it

## Control
### anti-cogging
![anti-cogging](./drive/anti_cogging/anti-cogging.gif)

Anti-Cogging demo in ros2.

reference:

- [odrive_ros2_control](https://github.com/Factor-Robotics/odrive_ros2_control)
- [odriverobotics](https://docs.odriverobotics.com/v/0.5.4/index.html)
- [odrive_diff_drive](https://github.com/zengxiaolei/odrive_diff_drive)



### pure pursuit path tracking with adaptive forwards or backwards heading

<img src="./path_tracking/pure_pursuit/demo/purepurpsuit.gif" width="600">

reference:

- [Document](https://arxiv.org/abs/1604.07446)



### velocity smoother (1€-Filter)

#### simulation test
<img src="./one_euro_filter/velocity_smoother/velocty_smoother.png" width="500">

```yaml
move_base_vel: original navigation velocity command without obstacle avoidance
cmd_vel: raw navigation velocity command with obstacle avoidance
cmd_vel_filtered: smoothed velocity command with obstacle avoidance
```



#### usecase with an industrial AGV

<img src="./one_euro_filter/velocity_smoother/use_case.png" width="500">

```yaml
move_base_vel: original navigation velocity command without obstacle avoidance
cmd_vel: smoothed velocity command with obstacle avoidance
```

reference:

- [1€-Filter](https://hal.inria.fr/hal-00670496/document)



### A* planner

<img src="./a_star/demo/astar.gif" width="400">

reference:

- [Video Reference in bilibili](https://www.bilibili.com/video/av32847834/?redirectFrom=h5)



## SLAM
### localization recovery by apriltag
<img src="./localization/localization_recovery.gif" width="800">



### SLAM with erased map

<img src="./localization/localization.gif" width="800">



### VO

#### realsense D453i mono camera + IMU
<img src="./localization/VO/vins_mono.jpg" width="800">

reference:

- [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)



#### realsense D453i stereo cameras+IMU

<img src="./localization/VO/vins_fusion.jpg" width="800">

reference:

- [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)



#### ORBSLAM2 with realsense D455 

<img src="./localization/VO/orbslam2-d455.gif" width="800">

reference:

[ORBSLAM2](https://github.com/raulmur/ORB_SLAM2)



## Detection
### pallet detection by realsense D435
#### detection demo
<img src="./pcl/pallet_detect/pallet-detect-d435.gif" width="800">



#### docking demo

<img src="./pcl/pallet_detect/pallet-docking-d435.gif" width="800">



### obstacle detour
<img src="./path_tracking/navigate/obstacle_bypass.gif" width="400">

The vehicle will wait for some time and then actively detour, if the obstacle does not leave within some time.

reference:

- [Document](http://wiki.ros.org/teb_local_planner)


### Parking Feasibilty Checking and Autonomous Adjustment
<img src="./parking_feasibility/demo/parking_feasibility.gif" width="300">



### Rotate Parking for Pose Adjustment
<img src="./parking_feasibility/demo/rotate_park.gif" width="300">



### K-Means clustering and 2D circle fitting

<img src="./clustering_and_fitting/demo/km_cf.gif" width="400">

    pink o:                    sample points
    grey- purple - yellow o:   clustered points
    red △:                     clustered center
    green --:                  fitted circle from clustered points
    green o:                   circle center



reference:

- [PDF Document of Aalborg Universitet](http://kom.aau.dk/group/04gr742/pdf/kmeans_worksheet.pdf)         



### geometry fitting of obstacles 

#### simulation in stage
<img src="./clustering_and_fitting/demo/obstacle_geometry.gif" width="500">

```yaml
red: laser data
green: fitted circle
yellow: fitted straight line
blue: robot footprint
visulization: obstacle_detector.msg in RViz
```



#### real case with PEPPERL+FUCHS lidar

<img src="./clustering_and_fitting/demo/r2000_detection_case.png" width="500">

```yaml
black: laser data
green: fitted circle & line
samples_per_scan: 4200
scan_frequency: 15
```



reference:

-  [lidar info](https://www.pepperl-fuchs.com/global/en/classid_53.htm?view=productdetails&prodid=86557)




## Behavior 
### behavior engine 
#### behavior creation 
<img src="./flexbe_usecase/demo/flexbe_edit.gif" width="800">



#### behavior execution

<img src="./flexbe_usecase/demo/flexbe_exe.gif" width="800">



#### simulation

<img src="./flexbe_usecase/demo/flexbe_agv.gif" width="600">

reference:

- [FlexBE](http://philserver.bplaced.net/fbe/)  



### state machine(SMACH) usecase

<img src="./smach_usecase/demo/ros_smach_usecase.gif" width="840">



reference: 

- [Document in ROS Wiki](http://wiki.ros.org/smach/UseCase)






## Scheduling
### traffic cross behavior
<img src="./multi_robots/demo/junction_traffic.gif" width="400">

```yaml
#No need to share information between robots
#Robots move independently of each other
controller_frequency: 2.0Hz （Due to computational load on my own computer）
base_local_planner: teb_local_planner/TebLocalPlannerROS                    
max_vel_x: 1.0m/s
play speed: ×4
```

reference:

- [teb_local_planner](https://github.com/rst-tu-dortmund/teb_local_planner)



### area lock manager

<img src="./multi_robots/demo/area_lock.gif" width="850">



### scheduling and routing for multi agents

<img src="./multi_robots/demo/multi_robots.gif" width="850">

```yaml
Scheduling: Manhattan distance
Routing: global path planning based on information sharing & junction behavior of independence      
Clicked point: Navigation tasks are automatically assigned to the nearest and idle robot.   
controller_frequency: 2.0Hz （Due to computational load on my own computer）
base_local_planner: teb_local_planner/TebLocalPlannerROS
max_vel_x: 1.0m/s
play speed: ×4
```



#### run 

git clone this repository  and its dependency package ( [homing_local_planner](https://github.com/zengxiaolei/homing_local_planner) ) to your ROS (noetic) workspace

```shell
cd ~/your_ros_ws/src
git clone https://github.com/zengxiaolei/robotics_demo.git
cd ..
catkin_make

roslaunch multi_robots_schedule test.launch
```

Note: The launched simulation is slightly different from the local planner and configuration in the gif above. You can play with this algorithm through the rviz user interface.



## Todo

Open source more algorithms and demoscases.



*☕ Buy Me a Coffee by AliPay*
<img src="./BuyMeaCoffee.jpg" width="200">

