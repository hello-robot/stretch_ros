#!/usr/bin/env python3

import rospy
import hello_helpers.hello_misc as hm
from sensor_msgs.msg import BatteryState


class RetractArmNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

    def main(self):
        hm.HelloNode.main(self, 'retract_arm', 'retract_arm', wait_for_first_pointcloud=False)
        self.move_to_pose({'gripper_aperture': 0.125})
        self.move_to_pose({'joint_lift': 0.8})
        self.move_to_pose({'joint_arm': 0.0})
        self.move_to_pose({'joint_wrist_yaw': 0.0, 'joint_wrist_roll': 0.0})
        self.move_to_pose({'joint_wrist_pitch': -1.0})


if __name__ == "__main__":
    node = RetractArmNode()
    node.main()

