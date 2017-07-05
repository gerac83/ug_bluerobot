#!/bin/bash
#Create clopema.urdf file from calibrated data.
#\param First argument is path to the devel folder.

source `echo $1`/setup.bash
roscore 2> /dev/null &
# Get its PID
PID=$!
file=`pwd`
roscd clopema_description/robots/
partner=$(env  | grep CLOPEMA_PARTNER | grep -oe '[^=]*$');
rosrun xacro xacro.py clopema_`echo $partner`.urdf.xacro > clopema.urdf
rosrun clopema_description mechanical_calibration
kill -INT $PID
cd $file

