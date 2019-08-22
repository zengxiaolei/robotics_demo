#!/usr/bin/env python

from turtlesim.srv import Spawn, TeleportAbsolute, TeleportAbsoluteRequest
from turtle_actionlib.msg import ShapeAction, ShapeActionGoal
from std_srvs.srv import Empty
import std_msgs.msg
import smach
import smach_ros
import rospy
import roslib
roslib.load_manifest('smach_usecase')


def monitor_cb(ud, msg):
    return False


def main():
    rospy.init_node('smach_usecase_executive')

    sm_root = smach.StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'])

    with sm_root:
        smach.StateMachine.add('RESET', smach_ros.ServiceState(
            'reset', Empty),
            transitions={'succeeded': 'SPAWN'})
        smach.StateMachine.add('SPAWN', smach_ros.ServiceState(
            'spawn', Spawn),
            transitions={'succeeded': 'TELEPORT1'})

        smach.StateMachine.add('TELEPORT1', smach_ros.ServiceState(
            'turtle1/teleport_absolute', TeleportAbsolute, request=TeleportAbsoluteRequest(5.0, 1.0, 0.0)),
            transitions={'succeeded': 'TELEPORT2'})
        smach.StateMachine.add('TELEPORT2', smach_ros.ServiceState(
            'turtle2/teleport_absolute', TeleportAbsolute, request=TeleportAbsoluteRequest(9.0, 5.0, 0.0)),
            transitions={'succeeded': 'DRAW_SHAPES'})

        sm_concurrence = smach.Concurrence(outcomes=['succeeded', 'aborted', 'preempted'],
                                           default_outcome='succeeded',
                                           outcome_map={'succeeded': {'BIG': 'succeeded', 'SMALL': 'succeeded'}})
        with sm_concurrence:
            shape_goal1 = ShapeActionGoal()
            shape_goal1.goal.edges = 11
            shape_goal1.goal.radius = 4.0
            smach.Concurrence.add('BIG', smach_ros.SimpleActionState(
                'turtle_shape1', ShapeAction, goal=shape_goal1.goal))

            sm_con_monitor = smach.Concurrence(outcomes=['succeeded', 'aborted', 'preempted'],
                                               default_outcome='succeeded',
                                               child_termination_cb=lambda state_outcomes: True,
                                               outcome_map={'succeeded': {'DRAW': 'succeeded'},
                                                            'aborted': {'MONITOR': 'invalid'},
                                                            'preempted': {'MONITOR': 'preempted', 'DRAW': 'preempted'}})
            with sm_con_monitor:
                shape_goal2 = ShapeActionGoal()
                shape_goal2.goal.edges = 6
                shape_goal2.goal.radius = 0.5
                smach.Concurrence.add('DRAW', smach_ros.SimpleActionState(
                    'turtle_shape2', ShapeAction, goal=shape_goal2.goal))

                smach.Concurrence.add('MONITOR', smach_ros.MonitorState(
                    "/turtle2_stop", std_msgs.msg.Empty, monitor_cb))

            smach.Concurrence.add('SMALL', sm_con_monitor)

        smach.StateMachine.add('DRAW_SHAPES', sm_concurrence)

        # shape_goal1 = ShapeActionGoal()
        # shape_goal1.goal.edges = 11
        # shape_goal1.goal.radius = 4.0
        # smach.StateMachine.add('BIG', smach_ros.SimpleActionState(
        #     'turtle_shape1', ShapeAction, goal=shape_goal1.goal),
        #     transitions={'succeeded': 'SMALL'})

        # shape_goal2 = ShapeActionGoal()
        # shape_goal2.goal.edges = 6
        # shape_goal2.goal.radius = 1.0
        # smach.StateMachine.add('SMALL', smach_ros.SimpleActionState(
        #     'turtle_shape2', ShapeAction, goal=shape_goal2.goal))

    sis = smach_ros.IntrospectionServer('smach_server', sm_root, '/SM_ROOT')
    sis.start()
    outcome = sm_root.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
