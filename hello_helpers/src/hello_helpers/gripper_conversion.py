#!/usr/bin/env python3

class GripperConversion:
    def __init__(self):
        # robotis position values (gripper.py)
        #      0 is very close to closed (fingers almost or barely touching)
        #     50 is maximally open
        #   -100 is maximally closed (maximum force applied to the object - might be risky for a large object)
        #
        # aperture is 12.5 cm wide when open (0.125 m, 125 mm)
        #
        # finger angle
        #   0.0 just barely closed
        #   fully opened is

        # from stretch_gripper.xacro
        # scale_finger_length = 0.9
        # scale_finger_length * 0.19011
        # = 0.171099
        self.finger_length_m = 0.171

        self.open_aperture_m = 0.09 #0.125
        self.closed_aperture_m = 0.0

        self.open_robotis = 70.0
        self.closed_robotis = 0.0

        self.robotis_to_aperture_slope = ((self.open_aperture_m - self.closed_aperture_m) / (self.open_robotis - self.closed_robotis))

    def robotis_to_aperture(self, robotis_in):
        # linear model
        aperture_m = (self.robotis_to_aperture_slope * (robotis_in - self.closed_robotis)) + self.closed_aperture_m
        return aperture_m

    def aperture_to_robotis(self, aperture_m):
        # linear model
        robotis_out = ((aperture_m - self.closed_aperture_m) / self.robotis_to_aperture_slope) + self.closed_robotis
        return robotis_out

    def aperture_to_finger_rad(self, aperture_m):
        # arc length / radius = ang_rad
        finger_rad = (aperture_m/2.0)/self.finger_length_m
        return finger_rad

    def finger_rad_to_aperture(self, finger_rad):
        aperture_m = 2.0 * (finger_rad * self.finger_length_m)
        return aperture_m

    def finger_to_robotis(self, finger_ang_rad):
        aperture_m = self.finger_rad_to_aperture(finger_ang_rad)
        robotis_out = self.aperture_to_robotis(aperture_m)
        return robotis_out

    def robotis_to_finger(self, robotis_pct):
        aperture_m = self.robotis_to_aperture(robotis_pct)
        finger_rad = self.aperture_to_finger_rad(aperture_m)
        return finger_rad

    def status_to_all(self, gripper_status):
        aperture_m = self.robotis_to_aperture(gripper_status['pos_pct'])
        finger_rad = self.aperture_to_finger_rad(aperture_m)
        finger_effort = gripper_status['effort']
        finger_vel = (self.robotis_to_aperture_slope * gripper_status['vel'])/2.0
        return aperture_m, finger_rad, finger_effort, finger_vel
