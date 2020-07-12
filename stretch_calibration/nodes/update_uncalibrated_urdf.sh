#!/bin/bash

echo ""
echo "Convert the current xacro files to a fresh uncalibrated URDF file."
echo ""

echo "rosrun xacro xacro `rospack find stretch_description`/urdf/stretch_description.xacro > `rospack find stretch_description`/urdf/stretch_uncalibrated.urdf"

rosrun xacro xacro `rospack find stretch_description`/urdf/stretch_description.xacro > `rospack find stretch_description`/urdf/stretch_uncalibrated.urdf
