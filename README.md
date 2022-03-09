share my robotic demos with all! Add star🌟 to this repo if you like it
----

## Pallet Detection by Ralsense D435
### Detection Demo
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/pcl/pallet_detect/pallet-detect-d435.gif" width="800">

### Docking Demo
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/pcl/pallet_detect/pallet-docking-d435.gif" width="800">


----

## Localization Recovery by Apriltag

<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/localization/localization_recovery.gif" width="800">

-------



## SLAM with Erased Map

<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/localization/localization.gif" width="800">


-------


## Pure Pursuit Path Tracking with Adaptive Forwards or Backwards Heading
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/path_tracking/pure_pursuit/demo/purepurpsuit.gif" width="600">

[Document](https://arxiv.org/abs/1604.07446)


-------



## VO

### Realsense D453i Mono Camera + IMU
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/localization/VO/vins_mono.jpg" width="800">
[VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)

-------

### Realsense D453i Stereo Cameras+IMU
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/localization/VO/vins_fusion.jpg" width="800">
[VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)

------------

### ORBSLAM2 with Realsense D455 
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/localization/VO/orbslam2-d455.gif" width="800">
[ORBSLAM2](https://github.com/raulmur/ORB_SLAM2)

-----------

## Velocity Smoother (1€-Filter)
### Simulation in Stage
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/one_euro_filter/velocity_smoother/velocty_smoother.png" width="500">

    move_base_vel:            original navigation velocity command without obstacle avoidance
    cmd_vel:                  raw navigation velocity command with obstacle avoidance
    cmd_vel_filtered:         smoothed velocity command with obstacle avoidance
    
------
  
### Usecase with an Industrial AGV
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/one_euro_filter/velocity_smoother/use_case.png" width="500">

    move_base_vel:            original navigation velocity command without obstacle avoidance
    cmd_vel:                  smoothed velocity command with obstacle avoidance
   
[1€-Filter](https://hal.inria.fr/hal-00670496/document)
  
-------------

## Agv Behavior Engine 
### Behavior Creation 
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/flexbe_usecase/demo/flexbe_edit.gif" width="800">

### Behavior Execution
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/flexbe_usecase/demo/flexbe_exe.gif" width="800">

### Agv Simulation
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/flexbe_usecase/demo/flexbe_agv.gif" width="600">

[FlexBE](http://philserver.bplaced.net/fbe/)  

------

## State Machine(SMACH) Usecase
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/smach_usecase/demo/ros_smach_usecase.gif" width="840">
  
 [Document in ROS Wiki](http://wiki.ros.org/smach/UseCase)


------

## Parking Feasibilty Checking and Autonomous Adjustment
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/parking_feasibility/demo/parking_feasibility.gif" width="300">

------

## Rotate Parking for Pose Adjustment
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/parking_feasibility/demo/rotate_park.gif" width="300">

------

## K-Means Clustering and 2D Circle Fitting
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/clustering_and_fitting/demo/km_cf.gif" width="400">

    pink o:                    sample points
    grey- purple - yellow o:   clustered points
    red △:                     clustered center
    green --:                  fitted circle from clustered points
    green o:                   circle center
          
[PDF Document of Aalborg Universitet](http://kom.aau.dk/group/04gr742/pdf/kmeans_worksheet.pdf)         

------

## Geometry Fitting of Obstacles 
### Simulation in Stage
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/clustering_and_fitting/demo/obstacle_geometry.gif" width="500">

    red:                      laser data
    green:                    fitted circle
    yellow:                   fitted straight line
    blue:                     robot footprint
    visulization:             obstacle_detector.msg in RViz


### Real Case with PEPPERL+FUCHS Lidar Sensor
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/clustering_and_fitting/demo/r2000_detection_case.png" width="500">

    black:                    laser data
    green:                    fitted circle & line
    samples_per_scan:         4200
    scan_frequency:           15
 
 [lidar info](https://www.pepperl-fuchs.com/global/en/classid_53.htm?view=productdetails&prodid=86557)


------
## Multi Robots
### Robots Junction Behavior
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/multi_robots/demo/junction_traffic.gif" width="400">

    No need to share information between robots
    Robots move independently of each other

    controller_frequency：     2.0Hz （Due to computational load on my own computer）
    base_local_planner:        teb_local_planner/TebLocalPlannerROS                    
    max_vel_x:                 1.0m/s

    play speed:                ×4
          
[teb_local_planner](https://github.com/rst-tu-dortmund/teb_local_planner)
    
------    

### Scheduling and Routing for Multi Robots
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/multi_robots/demo/multi_robots.gif" width="850">

    Scheduling:                Manhattan distance
    Routing:                   global path planning based on information sharing
                                     & junction behavior of independence
                   
    Clicked point:            Navigation tasks are automatically assigned to the nearest and idle robot.
          
    controller_frequency：    2.0Hz （Due to computational load on my own computer）
    base_local_planner:       teb_local_planner/TebLocalPlannerROS
    max_vel_x:                1.0m/s
          
    play speed:               ×4


------


### Area Lock Manager

<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/multi_robots/demo/area_lock.gif" width="850">


--------

## A*
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/a_star/demo/astar.gif" width="400">

[Video Reference in bilibili](https://www.bilibili.com/video/av32847834/?redirectFrom=h5)


------

<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/BuyMeaCoffee.jpg" width="200">

☕ Buy Me a Coffee by AliPay
