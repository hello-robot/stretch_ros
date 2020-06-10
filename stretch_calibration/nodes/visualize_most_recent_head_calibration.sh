#!/bin/bash

echo "-----------------------------------------------------------"
echo "Find the most recent optimization results file."

FILENAME=$HELLO_FLEET_PATH/$HELLO_FLEET_ID"/calibration_ros/head_calibration_result_"

for file in `ls $FILENAME*.yaml | sort -r`; do
    MOSTRECENT_OPTIMIZATION_FILE=$file
    break 1
done

echo "Found $MOSTRECENT_OPTIMIZATION_FILE"

echo "-----------------------------------------------------------"
echo "Find the most recent controller calibration file."

FILENAME=$HELLO_FLEET_PATH/$HELLO_FLEET_ID"/calibration_ros/controller_calibration_head_"

for file in `ls $FILENAME*.yaml | sort -r`; do
    MOSTRECENT_CONTROLLER_FILE=$file
    break 1
done

echo "Found $MOSTRECENT_CONTROLLER_FILE"

echo "-----------------------------------------------------------"
echo "Find the most recent calibrated URDF file."

FILENAME=$HELLO_FLEET_PATH/$HELLO_FLEET_ID"/calibration_ros/head_calibrated_"

for file in `ls $FILENAME*.urdf | sort -r`; do
    MOSTRECENT_URDF=$file
    break 1
done

echo "Found $MOSTRECENT_URDF"

echo "-----------------------------------------------------------"

echo "Launch the visualization using these three files."
echo "roslaunch stretch_calibration visualize_head_calibration.launch optimization_result_yaml_file:=$MOSTRECENT_OPTIMIZATION_FILE calibrated_controller_yaml_file:=$MOSTRECENT_CONTROLLER_FILE calibrated_urdf_file:=$MOSTRECENT_URDF" 

roslaunch stretch_calibration visualize_head_calibration.launch optimization_result_yaml_file:=$MOSTRECENT_OPTIMIZATION_FILE calibrated_controller_yaml_file:=$MOSTRECENT_CONTROLLER_FILE calibrated_urdf_file:=$MOSTRECENT_URDF
