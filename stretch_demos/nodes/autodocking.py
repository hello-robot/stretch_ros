#!/usr/bin/env python3

import argparse
import sys
import time
import typing
import py_trees
import py_trees.console as console
from autodocking_behaviors import *


def create_root() -> py_trees.behaviour.Behaviour:

    """
    Create the root behaviour and it's subtree.
    Returns:
        the root behaviour
    """

    root = py_trees.composites.Sequence(name="Root", memory=False)
    retry = py_trees.decorators.Retry(child=root, num_failures=3, name="Retry")

    cam_scanner = py_trees.composites.Selector(name="Fallback_Scan", memory=False)
    move_predock = py_trees.composites.Selector(name="Fallback_Predock", memory=False)
    move_dock = py_trees.composites.Sequence(name="Sequence_Dock", memory=False)
    charging = py_trees.composites.Selector(name="Fallback_Charging", memory=False)

    condition_aruco_detected_scan = ArucoDetect(name="arucoDetectedScan")
    condition_aruco_detected_dock = ArucoDetect(name="arucoDetectedDock")    
    condtion_predock_pose = PredockPose(name="predockPose")
    condition_charging = Charging(name="charging")
    action_move_predock = MoveToPredock(name="moveToPredock")
    action_move_predock_again = MoveToPredock(name="moveToPredockAgain")
    success_is_failure = py_trees.decorators.SuccessIsFailure(name="SuccessIsFail", child=action_move_predock_again)

    action_move_dock = MoveToDock(name="moveToDock")
    action_camera_scan = CameraScan(name="cameraScan")

    root.add_children([cam_scanner, move_predock, move_dock, charging])
    cam_scanner.add_children([condition_aruco_detected_scan, action_camera_scan])
    move_predock.add_children([condtion_predock_pose, action_move_predock])
    move_dock.add_children([condition_aruco_detected_dock, action_move_dock])
    charging.add_children([condition_charging, success_is_failure])

    return retry


if __name__ == '__main__':

    """Entry point for the demo script."""

    py_trees.logging.level = py_trees.logging.Level.DEBUG

    retry = create_root()

    ####################
    # Rendering
    ####################

    # if args.render:
    #     py_trees.display.render_dot_tree(root)
    #     sys.exit()

    ####################
    # Execute
    ####################

    retry.setup_with_descendants()

    for i in range(1, 6):
        try:
            print("\n--------- Tick {0} ---------\n".format(i))
            retry.tick_once()
            print("\n")
            print(py_trees.display.unicode_tree(root=retry, show_status=True))
            time.sleep(1.0)
        except KeyboardInterrupt:

            break

    print("\n")
