# ug_bluerobot
Motoman dual-arm robot base components

# Installation for Melodic

**NOTE:** If you have Kinetic, checkout kinetic-devel!

pip install docopt==0.6.1

echo "export CLOPEMA_PARTNER=LOCAL">>~/.bashrc

source ~/.bashrc

sudo apt-get install ros-melodic-openni-launch ros-melodic-perception-pcl ros-melodic-python-orocos-kdl libgsl-dev libeigen3-dev ros-melodic-eigen-conversions ros-melodic-industrial-msgs ros-melodic-industrial-robot-client ros-melodic-diagnostic-updater ros-melodic-moveit* libsdl2-dev libsdl1.2-dev libblas-dev liblapack-dev libeigen3-dev ros-melodic-smach ros-melodic-smach-ros

To test, source devel/setup.bash:

roslaunch clopema_launch virtual_robot.launch

**Remember to set the planning library in MoveIT pane**

# Notes from Gerardo

Not WORKING
clopema_stereo
