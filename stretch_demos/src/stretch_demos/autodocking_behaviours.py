#!/usr/bin/env python3

import rospy
import actionlib
import py_trees
from actionlib_msgs.msg import GoalStatus
from math import sqrt, pow
from geometry_msgs.msg import Pose, TransformStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from stretch_demos.msg import VisualServoAction, VisualServoGoal, VisualServoResult, VisualServoFeedback
import tf2_ros

class MoveBaseActionClient(py_trees.behaviour.Behaviour):
    def __init__(self, tf2_buffer, name="ActionClient", override_feedback_message_on_running="moving"):
        super(MoveBaseActionClient, self).__init__(name)
        self.action_client = None
        self.sent_goal = False
        self.action_spec = MoveBaseAction
        self.action_goal = MoveBaseGoal()
        self.action_namespace = "move_base"
        self.override_feedback_message_on_running = override_feedback_message_on_running
        self.tf2_buffer = tf2_buffer

    def setup(self, timeout):
        self.logger.debug("%s.setup()" % self.__class__.__name__)
        self.action_client = actionlib.SimpleActionClient(
            self.action_namespace,
            self.action_spec
        )
        if not self.action_client.wait_for_server(rospy.Duration(timeout)):
            self.logger.error("{0}.setup() could not connect to the action server at '{1}'".format(self.__class__.__name__, self.action_namespace))
            self.action_client = None
            return False
        return True

    def initialise(self):
        # Update the goal data here
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        transform = self.tf2_buffer.lookup_transform('map', 'predock_pose', rospy.Time(), rospy.Duration(2.0))
        goal_pose = Pose()
        goal_pose.position.x = transform.transform.translation.x
        goal_pose.position.y = transform.transform.translation.y
        goal_pose.position.z = transform.transform.translation.z
        goal_pose.orientation.x = transform.transform.rotation.x
        goal_pose.orientation.y = transform.transform.rotation.y
        goal_pose.orientation.z = transform.transform.rotation.z
        goal_pose.orientation.w = transform.transform.rotation.w
        self.action_goal.target_pose.header.frame_id = "map"
        self.action_goal.target_pose.header.stamp = rospy.Time.now()
        self.action_goal.target_pose.pose = goal_pose
        self.sent_goal = False

    def update(self):
        self.logger.debug("{0}.update()".format(self.__class__.__name__))
        if not self.action_client:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.Status.INVALID
        
        if not self.sent_goal:
            self.action_client.send_goal(self.action_goal)
            self.sent_goal = True
            self.feedback_message = "sent goal to the action server"
            return py_trees.Status.RUNNING

        self.feedback_message = self.action_client.get_goal_status_text()
        if self.action_client.get_state() in [GoalStatus.ABORTED,
                                              GoalStatus.PREEMPTED]:
            return py_trees.Status.FAILURE
        result = self.action_client.get_result()
        if result:
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = self.override_feedback_message_on_running
            return py_trees.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("%s.terminate()" % self.__class__.__name__)


class CheckTF(py_trees.behaviour.Behaviour):
    def __init__(self, tf2_buffer, name="CheckTF", source_frame="base_link", target_frame=None):
        super(CheckTF, self).__init__(name)
        self.source_frame = source_frame
        self.target_frame = target_frame
        self.tf2_buffer = tf2_buffer

    def setup(self, timeout):
        self.logger.debug("%s.setup()" % self.__class__.__name__)
        return True

    def initialise(self):
        # Update the goal data here
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        transform = TransformStamped()
        try:
            transform = self.tf2_buffer.lookup_transform(self.source_frame, self.target_frame, rospy.Time(), rospy.Duration(1.0))
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            self.error = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2))       
        except tf2_ros.ExtrapolationException:
            self.error = 10.0

    def update(self):
        """
        Check only to see whether the underlying action server has
        succeeded, is running, or has cancelled/aborted for some reason and
        map these to the usual behaviour return states.
        """
        self.logger.debug("{0}.update()".format(self.__class__.__name__))
        if self.error < 0.05:
            return py_trees.Status.SUCCESS
        else:
            return py_trees.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("%s.terminate()" % self.__class__.__name__)


class VisualServoing(py_trees.behaviour.Behaviour):
    def __init__(self, name="VisualServoing", source_frame="base_link", target_frame=None, override_feedback_message_on_running="servoing"):
        super(VisualServoing, self).__init__(name)
        self.action_client = None
        self.sent_goal = False
        self.action_spec = VisualServoAction
        self.action_goal = VisualServoGoal()
        self.action_namespace = "autodock_visual_servo"
        self.source_frame = source_frame
        self.target_frame = target_frame
        self.override_feedback_message_on_running = override_feedback_message_on_running

    def setup(self, timeout):
        self.logger.debug("%s.setup()" % self.__class__.__name__)
        self.action_client = actionlib.SimpleActionClient(
            self.action_namespace,
            self.action_spec
        )
        if not self.action_client.wait_for_server(rospy.Duration(timeout)):
            self.logger.error("{0}.setup() could not connect to the action server at '{1}'".format(self.__class__.__name__, self.action_namespace))
            self.action_client = None
            return False
        return True

    def initialise(self):
        # Update the goal data here
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.action_goal = VisualServoGoal()
        self.action_goal.source_frame = self.source_frame
        self.action_goal.target_frame = self.target_frame
        
    def update(self):
        """
        Check only to see whether the underlying action server has
        succeeded, is running, or has cancelled/aborted for some reason and
        map these to the usual behaviour return states.
        """
        self.logger.debug("{0}.update()".format(self.__class__.__name__))
        if not self.action_client:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.Status.INVALID
        
        if not self.sent_goal:
            self.action_client.send_goal(self.action_goal)
            self.sent_goal = True
            self.feedback_message = "sent goal to the action server"
            return py_trees.Status.RUNNING

        self.feedback_message = self.action_client.get_goal_status_text()
        if self.action_client.get_state() in [GoalStatus.ABORTED,
                                              GoalStatus.PREEMPTED]:
            return py_trees.Status.FAILURE
        result = self.action_client.get_result()
        if result:
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = self.override_feedback_message_on_running
            return py_trees.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("%s.terminate()" % self.__class__.__name__)
