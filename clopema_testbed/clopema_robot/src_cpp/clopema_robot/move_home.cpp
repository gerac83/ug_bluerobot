#include "clopema_robot/robot_commander.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_home", ros::init_options::AnonymousName);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    clopema_robot::ClopemaRobotCommander robot("arms");
    robot.move_to_named_target(clopema_robot::NAMED_TARGET_ARMS_HOME);

    clopema_robot::ClopemaRobotCommander robot_ext("ext");
    robot_ext.move_to_named_target(clopema_robot::NAMED_TARGET_EXT_HOME);

    robot.setServoPowerOff();
    spinner.stop();
}
