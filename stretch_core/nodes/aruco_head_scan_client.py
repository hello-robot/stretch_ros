from stretch_core.msg import ArucoHeadScanGoal, ArucoHeadScanAction
import actionlib
import rospy
import sys
from detect_aruco_markers import ArucoHeadScan

def main():
    aruco_head_scan_client = actionlib.SimpleActionClient('ArucoHeadScan', ArucoHeadScanAction)
    server_reached = aruco_head_scan_client.wait_for_server(timeout=rospy.Duration(10.0))
    if not server_reached:
            rospy.signal_shutdown('Unable to connect to aruco head scan action server. Timeout exceeded.')
            sys.exit()

    goal = ArucoHeadScanGoal()
    goal.aruco_id = 245
    goal.tilt_angle = -0.68
    goal.fill_in_blindspot_with_second_scan = False
    goal.fast_scan = False

    aruco_head_scan_client.send_goal(goal)
    aruco_head_scan_client.wait_for_result()


if __name__ == '__main__':
    head_scan = ArucoHeadScan()
    rospy.loginfo('Ensure stretch_driver is launched before executing')
    main()
