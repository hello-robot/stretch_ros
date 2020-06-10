#!/bin/bash

echo "Attempt to update the URDF using the most recent xacro and calibration files."
echo
echo "This should be run after any change to the xacro files that you want to be incorporated into the calibrated UDRF. It creates a new URDF for the robot using the current xacro files and the most recent head and tool calibration files."
echo 

rosrun stretch_calibration update_uncalibrated_urdf.sh
roslaunch stretch_calibration use_prior_head_calibration_to_update_urdf.launch
rosrun stretch_calibration update_with_most_recent_calibration.sh

echo
echo "Finished with attempt to update the URDF using the most recent xacro and calibration files."
