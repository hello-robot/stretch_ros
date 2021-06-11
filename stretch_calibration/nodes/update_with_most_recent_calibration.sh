#!/bin/bash
source $HOME/catkin_ws/devel/setup.bash

echo "Attempt to update the calibration files using the most recently performed calibration."

echo "-----------------------------------------------------------"
echo "Find the most recent controller calibration file."

FILENAME=$HELLO_FLEET_PATH/$HELLO_FLEET_ID"/calibration_ros/controller_calibration_head_"

for file in `ls $FILENAME*.yaml | sort -r`; do
    MOSTRECENT=$file
    break 1
done

echo "Found $MOSTRECENT."
echo "Making it the new controller calibration file."
echo "cp $MOSTRECENT `rospack find stretch_core`/config/controller_calibration_head.yaml"

cp $MOSTRECENT `rospack find stretch_core`/config/controller_calibration_head.yaml

echo "-----------------------------------------------------------"
echo "Find the most recent calibrated URDF file."

FILENAME=$HELLO_FLEET_PATH/$HELLO_FLEET_ID"/calibration_ros/head_calibrated_"

for file in `ls $FILENAME*.urdf | sort -r`; do
    MOSTRECENT=$file
    break 1
done

echo "Found $MOSTRECENT."
echo "Making it the new URDF file."
echo "cp $MOSTRECENT `rospack find stretch_description`/urdf/stretch.urdf"

cp $MOSTRECENT `rospack find stretch_description`/urdf/stretch.urdf

echo "-----------------------------------------------------------"
echo "Finished with attempt to update the calibration files."
