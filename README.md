## State Machine(SMACH) Usecase
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/smach_usecase/demo/ros_smach_usecase.gif" width="840">
  
 [Document in ROS Wiki](http://wiki.ros.org/smach/UseCase)

-------

## A*
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/a_star/demo/astar.gif" width="600">

[Video Reference in bilibili](https://www.bilibili.com/video/av32847834/?redirectFrom=h5)

------

## Parking Feasibilty Checking and Autonomous Adjustment
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/parking_feasibility/demo/parking_feasibility.gif" width="500">

------

## K-Means Clustering and 2D Circle Fitting
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/clustering_and_fitting/demo/km_cf.gif" width="600">

    pink o:                    sample points
    grey- purple - yellow o:   clustered points
    red △:                     clustered center
    green --:                  fitted circle from clustered points
    green o:                   circle center
          
[PDF Document of Aalborg Universitet](http://kom.aau.dk/group/04gr742/pdf/kmeans_worksheet.pdf)         

------

## Geometry Fitting of Obstacles 
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/clustering_and_fitting/demo/obstacle_geometry.gif" width="600">

    red:                      laser data
    green:                    fitted circle
    yellow:                   fitted straight line
    blue:                     robot footprint
    visulization:             obstacle_detector.msg in RViz

------

## Robots Junction Behavior
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/multi_robots/demo/junction_traffic.gif" width="600">

    No need to share information between robots
    Robots move independently of each other

    controller_frequency：     2.0Hz （Due to computational load on my own computer）
    base_local_planner:        teb_local_planner/TebLocalPlannerROS                    
    max_vel_x:                 1.0m/s

    play speed:                ×4
          
[teb_local_planner](https://github.com/rst-tu-dortmund/teb_local_planner)
    
------    

## Scheduling and Routing for Multi Robots
<img src="https://github.com/zengxiaolei/robotics_demo/blob/master/multi_robots/demo/multi_robots.gif" width="850">

    Scheduling:                Manhattan distance
    Routing:                   global path planning based on information sharing
                                     & junction behavior of independence
                   
    Clicked point:            Navigation tasks are automatically assigned to the nearest and idle robot.
          
    controller_frequency：    2.0Hz （Due to computational load on my own computer）
    base_local_planner:       teb_local_planner/TebLocalPlannerROS
    max_vel_x:                1.0m/s
          
    play speed:               ×4
