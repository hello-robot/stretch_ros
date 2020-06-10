#!/bin/bash

echo "Attempt to revert to the previous calibration."

echo "First, attempt to move most recent calibration files to the reversion directory"

REVERSION_DIR=$HELLO_FLEET_PATH/$HELLO_FLEET_ID"/calibration_ros/reverted/"

echo "Creating $REVERSION_DIR if it does not already exist."
mkdir $REVERSION_DIR

echo "-----------------------------------------------------------"
echo "Find the most recent optimization results file."

FILENAME=$HELLO_FLEET_PATH/$HELLO_FLEET_ID"/calibration_ros/head_calibration_result_"

for file in `ls $FILENAME*.yaml | sort -r`; do
    MOSTRECENT_OPTIMIZATION_FILE=$file
    break 1
done

echo "Found $MOSTRECENT_OPTIMIZATION_FILE"

echo "Moving $MOSTRECENT_OPTIMIZATION_FILE to the reversion directory."

echo "mv $MOSTRECENT_OPTIMIZATION_FILE $REVERSION_DIR"

mv $MOSTRECENT_OPTIMIZATION_FILE $REVERSION_DIR

echo "-----------------------------------------------------------"
echo "Find the most recent controller calibration file."

FILENAME=$HELLO_FLEET_PATH/$HELLO_FLEET_ID"/calibration_ros/controller_calibration_head_"

for file in `ls $FILENAME*.yaml | sort -r`; do
    MOSTRECENT_CONTROLLER_FILE=$file
    break 1
done

echo "Found $MOSTRECENT_CONTROLLER_FILE"

echo "Moving $MOSTRECENT_CONTROLLER_FILE to the reversion directory."

echo "mv $MOSTRECENT_CONTROLLER_FILE $REVERSION_DIR"

mv $MOSTRECENT_CONTROLLER_FILE $REVERSION_DIR

echo "-----------------------------------------------------------"
echo "Find the most recent calibrated URDF file."

FILENAME=$HELLO_FLEET_PATH/$HELLO_FLEET_ID"/calibration_ros/head_calibrated_"

for file in `ls $FILENAME*.urdf | sort -r`; do
    MOSTRECENT_URDF=$file
    break 1
done

echo "Found $MOSTRECENT_URDF"

echo "Moving $MOSTRECENT_URDF to the reversion directory."

echo "mv $MOSTRECENT_URDF $REVERSION_DIR"

mv $MOSTRECENT_URDF $REVERSION_DIR

echo "-----------------------------------------------------------"

echo "Now, update calibration using the most recent files remaining in the calibration directory."

echo "rosrun stretch_calibration update_with_most_recent_calibration.sh"

rosrun stretch_calibration update_with_most_recent_calibration.sh

echo "-----------------------------------------------------------"
echo "Done."
