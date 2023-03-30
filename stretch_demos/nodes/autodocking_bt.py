#!/usr/bin/env python3

import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import sys
import functools
from stretch_demos.autodocking_behaviours import MoveBaseActionClient
from stretch_demos.autodocking_behaviours import CheckTF
from stretch_demos.autodocking_behaviours import VisualServoing
from stretch_demos.msg import ArucoHeadScanAction, ArucoHeadScanGoal
from geometry_msgs.msg import Pose
from sensor_msgs.msg import BatteryState
import hello_helpers.hello_misc as hm


class AutodockingBT(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

    def create_root(self):
        # behaviours
        
        autodocking_seq_root = py_trees.composites.Sequence("autodocking")
        dock_found_fb = py_trees.composites.Selector("dock_found_fb")
        at_predock_fb = py_trees.composites.Selector("at_predock_fb")
        charging_fb = py_trees.composites.Selector("charging_fb")

        predock_found_sub = py_trees_ros.subscribers.CheckData(
            name="predock_found_sub?",
            topic_name='/predock_pose',
            expected_value=None,
            topic_type=Pose,
            fail_if_no_data=True,fail_if_bad_comparison=False)
        
        is_charging_sub = py_trees_ros.subscribers.CheckData(
            name="battery_charging?",
            topic_name='/battery',
            variable_name='present',
            expected_value=True,
            topic_type=BatteryState,
            fail_if_no_data=True,fail_if_bad_comparison=True)

        # at_predock_tf = CheckTF(self.tf2_buffer, name="at_predock_tf", target_frame='predock_pose')

        aruco_goal = ArucoHeadScanGoal()
        aruco_goal.aruco_id = 245
        aruco_goal.tilt_angle = -0.68
        aruco_goal.publish_to_map = True
        aruco_goal.fill_in_blindspot_with_second_scan = False
        aruco_goal.fast_scan = False
        head_scan_action = py_trees_ros.actions.ActionClient( # Publishes predock pose to /predock_pose topic and tf frame called /predock_pose
            name="ArucoHeadScan",
            action_namespace="ArucoHeadScan",
            action_spec=ArucoHeadScanAction,
            action_goal=aruco_goal,
            override_feedback_message_on_running="rotating"
        )

        predock_action = MoveBaseActionClient(
            self.tf2_buffer,
            name="predock_action",
            override_feedback_message_on_running="moving"
        )
        invert_predock = py_trees.decorators.SuccessIsFailure(name='invert_predock', child=predock_action)

        dock_action = VisualServoing(
            name='dock_action',
            source_frame='docking_station',
            target_frame='charging_port',
            override_feedback_message_on_running="docking"
        )

        # tree
        autodocking_seq_root.add_children([dock_found_fb, at_predock_fb, dock_action, charging_fb])
        dock_found_fb.add_children([predock_found_sub, head_scan_action])
        at_predock_fb.add_children([predock_action])
        charging_fb.add_children([is_charging_sub, invert_predock])
        return autodocking_seq_root

    def shutdown(self, behaviour_tree):
        behaviour_tree.interrupt()

    def main(self):
        """
        Entry point for the demo script.
        """
        hm.HelloNode.main(self, 'funmap', 'funmap')

        root = self.create_root()
        self.behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
        rospy.on_shutdown(functools.partial(self.shutdown, self.behaviour_tree))
        if not self.behaviour_tree.setup(timeout=15):
            console.logerror("failed to setup the tree, aborting.")
            sys.exit(1)
        
        def print_tree(tree):
            print(py_trees.display.unicode_tree(root=tree.root, show_status=True))

        try:
            self.behaviour_tree.tick_tock(
                500
                # period_ms=500,
                # number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK,
                # pre_tick_handler=None,
                # post_tick_handler=print_tree
            )
        except KeyboardInterrupt:
            self.behaviour_tree.interrupt()


def main():
    node = AutodockingBT()
    node.main()


if __name__ == '__main__':
    main()
