#!/usr/bin/env python3
"""
Author: lei.zeng@tu-dortmund.de
"""

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Polygon, Point32, PolygonStamped, PoseArray, Pose
from obstacle_detector.msg import Obstacles, CircleObstacle
import numpy as np
import operator
from dynamic_reconfigure.server import Server
import tf
import tf2_ros
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_msgs.msg import Float32
from one_euro_filter import OneEuroFilter
import time
import yaml
import getpass


def relative_to_absolute_path(relative_path):
    if relative_path[0] == '~':
        absolute_path = '/home/' + getpass.getuser() + relative_path[1:]
    else:
        absolute_path = relative_path
    return absolute_path


class VelocityControl:
    def __init__(self):
        self._namespace = rospy.get_namespace().strip('/')

        self._buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._buf)

        self.follow_rules = rospy.get_param("~follow_traffic_rules", False)
        self.show_cautious_zone = rospy.get_param("~show_cautious_zone", False)
        self.pub_cautious_zone = rospy.Publisher('cautious_zone',
                                                 PolygonStamped,
                                                 queue_size=1)

        if self.follow_rules:
            junction_yaml_path = relative_to_absolute_path(
                rospy.get_param("~junction_yaml_path", "~/special_areas.yaml"))
            with open(junction_yaml_path, 'r') as stream:
                try:
                    areas = yaml.safe_load(stream).items()
                    self.junctionList = [
                        a for a in areas if a[0][:5] == 'cross'
                    ]
                except:
                    rospy.logwarn('[control] Failed to load cross areas')
        else:
            self.junctionList = None

        self._distance_to_junction, self._cjam_num = 0, 0
        self.junction_slowdown_distance = rospy.get_param(
            "~junction_slowdown_distance", 2.5)
        self.junction_slowdown_ratio = rospy.get_param(
            "~junction_slowdown_ratio", 0.5)
        self.jam_check_band = rospy.get_param("~jam_check_band", 0.5)
        self.junction_aim_rad = rospy.get_param("~junction_aim_rad", 0.1)
        self._get_frame = False

        self.linear_filter = OneEuroFilter(t0=time.time(),
                                           x0=0.0,
                                           dx0=0.0,
                                           min_cutoff=1.0,
                                           beta=0.07,
                                           d_cutoff=1.0)
        self.omega_filter = OneEuroFilter(t0=time.time(),
                                          x0=0.0,
                                          dx0=0.0,
                                          min_cutoff=1.0,
                                          beta=0.07,
                                          d_cutoff=1.0)

        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pub_block = rospy.Publisher('block_obstacles',
                                         Obstacles,
                                         queue_size=1)
        self.use_one_euro_filter = rospy.get_param("~use_one_euro_filter",
                                                   True)

        rospy.Subscriber("move_base_vel",
                         Twist,
                         self.moveBaseVelCallback,
                         queue_size=10)

        rospy.Subscriber("robot_pose",
                         Pose,
                         self.poseCallback,
                         queue_size=1)

    def trafficLight(self, direction):
        second = time.localtime().tm_sec
        if 0 <= second <= 25:
            light = ('RED', 'GREEN')
        elif 30 <= second <= 55:
            light = ('GREEN', 'RED')
        else:
            light = ('YELLOW', 'YELLOW')
        if direction == 'vertical':
            return light[0]
        elif direction == 'horizontal':
            return light[1]
        else:
            rospy.logerr("[control] Error by using traffic light")
            return 'nosense'

    def junctionCheck(self, x, y, yaw):
        if self.junctionList:
            for junction in self.junctionList:
                if self.junctionDirection(x, y, yaw, junction) != 'safe':
                    return self.junctionDirection(x, y, yaw, junction)
            return 'safe'
        else:
            return 'safe'

    def junctionDirection(self, x, y, yaw, junction):
        if self.pointDistanceRobot(
            (x - junction[1]['x']),
                (y - junction[1]['y'])) > self.junction_slowdown_distance:
            return 'safe'
        else:
            yaw_to_junction = math.atan2(junction[1]['y'] - y,
                                         junction[1]['x'] - x)
            if abs(x - junction[1]['x']) > abs(
                (y - junction[1]['y'])) and self.radContinousDiff(
                    yaw, yaw_to_junction) < self.junction_aim_rad:
                self._distance_to_junction = abs(x - junction[1]['x'])
                return 'horizontal'
            elif abs(x - junction[1]['x']) < abs(
                (y - junction[1]['y']
                 )) and abs(yaw - yaw_to_junction) < self.junction_aim_rad:
                self._distance_to_junction = abs((y - junction[1]['y']))
                return 'vertical'
            else:
                return 'safe'

    def poseCallback(self, poseMsg):
        _, _, pose_yaw = tf.transformations.euler_from_quaternion([
            poseMsg.orientation.x, poseMsg.orientation.y,
            poseMsg.orientation.z, poseMsg.orientation.w
        ])
        self.__robot_pose = [
            poseMsg.position.x, poseMsg.position.y, pose_yaw
        ]

    def radContinousDiff(self, r1, r2):
        dr = (r1 - r2) % (math.pi * 2)
        if dr > math.pi:
            dr = 2 * math.pi - dr
        return dr

    def pointDistanceRobot(self, x, y):
        return np.sqrt(x**2 + y**2)

    def pointRadRobot(self, y, x):
        # -(-pi,pi]
        return math.atan2(y, x)

    def publishCautiousZone(self):
        if self._get_frame:
            cautious_zone_msg = PolygonStamped()
            cautious_zone_msg.header.frame_id = self._frame_id
            cautious_zone_msg.header.stamp = rospy.Time.now()
            cautious_zone_msg.polygon = self.polygonMsgGeneration(
                (self._distance_to_junction - self.jam_check_band,
                 self._distance_to_junction + self.jam_check_band),
                (-self.junction_slowdown_distance,
                 self.junction_slowdown_distance))

            self.pub_cautious_zone.publish(cautious_zone_msg)

    def polygonMsgGeneration(self, px_tuples, py_tuples):
        polygon_msg = Polygon()

        point_msg = Point32()
        point_msg.x, point_msg.y = min(px_tuples), max(py_tuples)
        polygon_msg.points.append(point_msg)

        point_msg = Point32()
        point_msg.x, point_msg.y = min(px_tuples), min(py_tuples)
        polygon_msg.points.append(point_msg)

        point_msg = Point32()
        point_msg.x, point_msg.y = max(px_tuples), min(py_tuples)
        polygon_msg.points.append(point_msg)

        point_msg = Point32()
        point_msg.x, point_msg.y = max(px_tuples), max(py_tuples)
        polygon_msg.points.append(point_msg)

        return polygon_msg

    def moveBaseVelCallback(self, navVelMsg):
        if self.follow_rules:
            try:
                jc = self.junctionCheck(self.__robot_pose[0],
                                        self.__robot_pose[1],
                                        self.__robot_pose[2])
                if jc == 'safe':
                    self._junction_slowdown_ratio = 1.0
                elif self.trafficLight(jc) == 'GREEN':
                    self._junction_slowdown_ratio = self.junction_slowdown_ratio + \
                        (1.0-self.junction_slowdown_ratio) / \
                        self.junction_slowdown_distance * self._distance_to_junction
                    rospy.loginfo_throttle(
                        0.5, '[control] %s: %s-cross with GREEN light' %
                        (self._namespace, jc))
                elif self._cjam_num == 0:
                    self._junction_slowdown_ratio = self.junction_slowdown_ratio + \
                        (1.0-self.junction_slowdown_ratio) / \
                        self.junction_slowdown_distance * self._distance_to_junction
                    rospy.loginfo_throttle(
                        0.5, '[control] %s: %s-cross wihout jam, ignoring light' %
                        (self._namespace, jc))
                else:
                    rospy.logwarn_throttle(
                        0.5, '[control] %s: waiting for the green light at cross' %
                        (self._namespace))
                    self._junction_slowdown_ratio = 0
            except Exception as e:
                print(e)
            try:
                if self.show_cautious_zone and jc != 'safe':
                    self.publishCautiousZone()
            except Exception as e:
                print(e)

        twist_avoidace = Twist()

        twist_avoidace = navVelMsg

        if self.use_one_euro_filter:
            if not ((navVelMsg.linear.x == 0) and (navVelMsg.angular.z == 0)):
                twist_avoidace.linear.x = round(
                    self.linear_filter(time.time(), twist_avoidace.linear.x),
                    5)
                twist_avoidace.angular.z = round(
                    self.omega_filter(time.time(), twist_avoidace.angular.z),
                    5)
            else:
                twist_avoidace.linear.x = 0.0
                twist_avoidace.angular.z = 0.0

        if self.follow_rules:
            twist_avoidace.linear.x = twist_avoidace.linear.x * self._junction_slowdown_ratio

        self.pub_vel.publish(twist_avoidace)


def main():
    rospy.init_node('velocity_control')
    VelocityControl()
    try:
        rospy.spin()
    except:
        print("Shutting down")


if __name__ == '__main__':
    main()
