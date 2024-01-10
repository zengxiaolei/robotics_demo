#!/usr/bin/env python3

"""
Author: lei.zeng@tu-dortmund.de
"""

import rospy
import numpy as np
from people_msgs.msg import Person, People
from nav_msgs.msg import Path
from geometry_msgs.msg import Quaternion, PointStamped, Pose2D, Pose
from std_msgs.msg import Int8
from multi_robot_test.srv import TaskAssignment, TaskAssignmentResponse
from move_base_msgs.msg import MoveBaseActionGoal
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import LaserScan


class PlanSocialInteration():
    def __init__(self, robots_num=3):
        self._pub_people_list = []
        self._pub_goal_list = []
        self._pub_scan_list = []

        self._sub_pose_list = []
        self._sub_ctrl_list = []

        self._robots_pose_list = []
        self._robots_path_list = []

        self._robot_num = robots_num

        for robot_id in range(robots_num):

            self._robots_pose_list.append(Pose())
            self._robots_path_list.append(Path())

            pub_scan = rospy.Publisher(
                '/robot_{0}/scan_proc'.format(robot_id), LaserScan, queue_size=1)
            self._pub_scan_list.append(pub_scan)

            pub_people = rospy.Publisher(
                '/robot_{0}/people'.format(robot_id), People, queue_size=1)
            self._pub_people_list.append(pub_people)

            pub_goal = rospy.Publisher(
                '/robot_{0}/move_base/goal'.format(robot_id), MoveBaseActionGoal, queue_size=1)
            self._pub_goal_list.append(pub_goal)

            rospy.Subscriber('/robot_{0}/move_base/GlobalPlanner/plan'.format(
                robot_id), Path, self.plan_cb, (robot_id), queue_size=10)

            rospy.Subscriber('/robot_{0}/scan'.format(
                robot_id), LaserScan, self.scan_cb, (robot_id), queue_size=1)

            sub_pose = rospy.Subscriber(
                '/robot_{0}/robot_pose'.format(
                    robot_id), Pose, self.pose_cb, (robot_id), queue_size=1)
            self._sub_pose_list.append(sub_pose)

            sub_ctrl = rospy.Subscriber(
                '/control_clock', Int8, self.ctrl_cb, (robot_id), queue_size=10)

        self.server_task_assignment = rospy.Service(
            '/task_assignment', TaskAssignment, self.handlerTaskAssign)

        self.subClickedPoint = rospy.Subscriber(
            '/clicked_point', PointStamped, self.click_cb, queue_size=10)

    def scan_cb(self, msg, args):
        if msg.header.frame_id[0] == '/':
            msg.header.frame_id = msg.header.frame_id[1:]
        self._pub_scan_list[args].publish(msg)

    def click_cb(self, click_msg):
        rospy.wait_for_service('task_assignment')
        task_assign = rospy.ServiceProxy(
            'task_assignment', TaskAssignment)

        p_2d = Pose2D()
        p_2d.x = click_msg.point.x
        p_2d.y = click_msg.point.y
        task_assign(p_2d)

    def handlerTaskAssign(self, req_msg):
        res_msg = TaskAssignmentResponse()
        print(res_msg.robot_no)
        manhattan_distance_list = [abs(req_msg.task_goal.x - self._robots_pose_list[i].position.x)
                                   + abs(req_msg.task_goal.y -
                                         self._robots_pose_list[i].position.y)
                                   + (rospy.Time.now() - self._robots_path_list[i].header.stamp < rospy.Duration(secs=2))*1000
                                   for i in range(self._robot_num)]

        if min(manhattan_distance_list) >= 1000:
            rospy.logwarn('All Buzzy!!!')
            return res_msg

        res_msg.robot_no = manhattan_distance_list.index(
            min(manhattan_distance_list))
        goal_msg = MoveBaseActionGoal()
        goal_msg.goal.target_pose.header.stamp = rospy.Time.now()
        goal_msg.goal.target_pose.header.frame_id = 'map'
        goal_msg.goal.target_pose.pose.position.x = req_msg.task_goal.x
        goal_msg.goal.target_pose.pose.position.y = req_msg.task_goal.y
        quat = quaternion_from_euler(0.0, 0.0, req_msg.task_goal.theta)
        goal_msg.goal.target_pose.pose.orientation = Quaternion(*quat.tolist())

        self._pub_goal_list[res_msg.robot_no].publish(goal_msg)

        return res_msg

    def plan_cb(self, plan_msg, args):
        self._robots_path_list[args] = plan_msg

    def pose_cb(self, pose_msg, args):
        self._robots_pose_list[args] = pose_msg

    def ctrl_cb(self, ctrl_msg, args):
        try:
            self.pub_people_integration(self._pub_people_list[args],  args)
        except:
            pass

    def pub_people_integration(self, pub, robot_id):
        people_msg = People()
        people_msg.header.frame_id = 'map'

        for i in range(self._robot_num):
            try:
                if i != robot_id:
                    people_msg.people = people_msg.people + \
                        self.path2person_list(
                            self._robots_path_list[i], look_ahead=LOOKAHEAD_PATH_DISTANCE)
            except:
                pass
            try:
                if i != robot_id:
                    people_msg.people = people_msg.people + \
                        self.pose2persoon(self._robots_pose_list[i])
            except:
                pass

        pub.publish(people_msg)

    def pose2persoon(self, pose_msg):
        person_list = []
        one_person = Person()
        one_person.position.x = pose_msg.position.x
        one_person.position.y = pose_msg.position.y
        one_person.reliability = 1.0
        person_inflation = self.single_grid_inflate(one_person, 0.6, 0.05)
        person_list = person_list + person_inflation
        return person_list

    def path2person_list(self, path, look_ahead=25.0):
        person_list = []
        try:

            diff1 = np.sqrt((path.poses[2].pose.position.x - path.poses[1].pose.position.x)**2 + (
                path.poses[2].pose.position.y - path.poses[1].pose.position.y)**2)

            diff2 = np.sqrt((path.poses[3].pose.position.x - path.poses[2].pose.position.x)**2 + (
                path.poses[3].pose.position.y - path.poses[2].pose.position.y)**2)

            diff = np.mean([diff1, diff2])
        except:
            diff = 0.025

        look_ahead_num = int(look_ahead / diff)
        end_index = min(len(path.poses),  look_ahead_num)

        for p in range(end_index):
            one_person = Person()
            one_person.position.x = path.poses[p].pose.position.x
            one_person.position.y = path.poses[p].pose.position.y
            one_person.reliability = 1.0
            inflation_dist = PATH_INFLATION_END
            computation_interval_end = int(PATH_INFLATION_END*10 / 0.1)
            computation_interval_front = computation_interval_end * \
                int(1.0*PATH_INFLATION_FRONT/PATH_INFLATION_END-1)
            computation_interval_middle = computation_interval_end * \
                int(1.0*PATH_INFLATION_MIDDLE/PATH_INFLATION_END-1)

            if p*diff <= 1.0 or p % computation_interval_end != 0:
                continue
            elif 1 < p*diff < PATH_DISTANCE_FONT and p % computation_interval_front == 0:
                inflation_dist = PATH_INFLATION_FRONT
            elif PATH_DISTANCE_FONT <= p*diff < PATH_DISTANCE_MIDDLE and p % computation_interval_middle == 0:
                inflation_dist = PATH_INFLATION_MIDDLE

            person_inflation = self.single_grid_inflate(
                one_person, inflation_dist, 0.05)
            # person_list.append(one_person)
            person_list = person_list + person_inflation

        return person_list

    def single_grid_inflate(self, person, inflation_range, interval):
        person_list = []
        center_x = person.position.x
        center_y = person.position.y
        for in_x in list(np.arange(center_x-inflation_range, center_x+inflation_range+interval, interval)):
            for in_y in list(np.arange(center_y - inflation_range, center_y+inflation_range+interval, interval)):
                one_person = Person()
                one_person.position.x = in_x
                one_person.position.y = in_y
                one_person.reliability = 1.0
                person_list.append(one_person)
        return person_list


def main():
    PlanSocialInteration(ROBOTS_NUMBER)

    pub_rate = rospy.Publisher('/control_clock', Int8, queue_size=10)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub_rate.publish(0)
        rate.sleep()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    rospy.init_node('multi_robots_social_network')
    ROBOTS_NUMBER = rospy.get_param(
        "~robots_num", 7)
    LOOKAHEAD_PATH_DISTANCE = rospy.get_param(
        "~lookahead_path_distance", 10)
    PATH_INFLATION_FRONT = rospy.get_param(
        "~path_inflation_front", 0.8)
    PATH_DISTANCE_FONT = rospy.get_param(
        "~path_distance_front", 4.0)
    PATH_INFLATION_MIDDLE = rospy.get_param(
        "~path_inflation_middle", 0.4)
    PATH_DISTANCE_MIDDLE = rospy.get_param(
        "~path_distance_middle", 7.0)
    PATH_INFLATION_END = rospy.get_param(
        "~path_inflation_end", 0.1)

    main()
