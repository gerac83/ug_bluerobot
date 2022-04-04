# ug_bluerobot
Motoman dual-arm robot base components

# Installation for Kinetic

pip install docopt==0.6.1

echo "export CLOPEMA_PARTNER=LOCAL">>~/.bashrc

source ~/.bashrc

sudo apt-get install ros-kinetic-openni-launch ros-kinetic-perception-pcl ros-kinetic-python-orocos-kdl libgsl-dev libeigen2-dev libeigen3-dev ros-kinetic-eigen-conversions ros-kinetic-industrial-msgs ros-kinetic-industrial-robot-client ros-kinetic-diagnostic-updater ros-kinetic-moveit* libsdl2-dev libsdl1.2-dev libblas-dev liblapack-dev libeigen3-dev

To test, source devel/setup.bash:

roslaunch clopema_launch virtual_robot.launch

**Remember to set the planning library in MoveIT pane**

# Notes from Gerardo

Not WORKING
clopema_stereo
