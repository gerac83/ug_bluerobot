#include <clopema_controller/ClopemaJointRelayHandler.h>

namespace industrial_robot_client {
    namespace joint_relay_handler {
        ClopemaJointRelayHandler::ClopemaJointRelayHandler() : publish_states_separately(false) {
        }

        ClopemaJointRelayHandler::~ClopemaJointRelayHandler() {
        }

        bool ClopemaJointRelayHandler::init(SmplMsgConnection* connection, std::vector<std::vector<std::string> >& joint_names) {
            pub_joint_control_state_ = node_.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 1);
            pub_joint_sensor_state_ = node_.advertise<sensor_msgs::JointState>("joint_states", 1);
            grouped_joint_names = joint_names;
            group_received.resize(joint_names.size(), false);

            int n = 0;
            for(unsigned int i = 0; i < joint_names.size(); ++i) {
                n += joint_names[i].size();
            }
            joints_position.resize(n, 0.0);
            joints_vel.resize(n, 0.0);

            return init((int)StandardMsgTypes::JOINT_FEEDBACK, connection);
        }

        bool ClopemaJointRelayHandler::internalCB(JointFeedbackMessage& in) {
            control_msgs::FollowJointTrajectoryFeedback control_state;
            sensor_msgs::JointState sensor_state;
            bool rtn = true;

            if(create_messages(in, control_state, sensor_state)) {
                this->pub_joint_control_state_.publish(control_state);
                this->pub_joint_sensor_state_.publish(sensor_state);
            } else
                rtn = false;

            // Reply back to the controller if the sender requested it.
            if(CommTypes::SERVICE_REQUEST == in.getMessageType()) {
                SimpleMessage reply;
                in.toReply(reply, rtn ? ReplyTypes::SUCCESS : ReplyTypes::FAILURE);
                this->getConnection()->sendMsg(reply);
            }

            return rtn;
        }

        bool ClopemaJointRelayHandler::internalCB(SimpleMessage& in) {
            JointFeedbackMessage jmsg;
            if(!jmsg.init(in)) {
                ROS_WARN_STREAM("Cannot convert message to JointFeedbackMessage");
            }
            internalCB(jmsg);
            return true;
        }

        bool ClopemaJointRelayHandler::JointDataToVector(const JointData& joints, std::vector<double>& vec, int len) {
            if((len < 0) || (len > joints.getMaxNumJoints())) {
                ROS_ERROR("Failed to copy JointData.  Len (%d) out of range (0 to %d)", len, joints.getMaxNumJoints());
                return false;
            }

            vec.resize(len);
            for(int i = 0; i < len; ++i)
                vec[i] = joints.getJoint(i);

            return true;
        }

        bool ClopemaJointRelayHandler::create_messages(JointFeedbackMessage& msg_in, control_msgs::FollowJointTrajectoryFeedback& control_state, sensor_msgs::JointState& sensor_state) {
            if(msg_in.getRobotID() < 0 && msg_in.getRobotID() >= this->grouped_joint_names.size()) {
                ROS_WARN_STREAM("Unknown robotID: " << msg_in.getRobotID());
                return false;
            }
            int jnum = grouped_joint_names[msg_in.getRobotID()].size();

            JointData values;
            std::vector<double> joint_pos, joint_vel;
            
            if(!msg_in.getPositions(values)) {
                ROS_WARN_STREAM("Cannot get positions from feedback message");
                return false;
            }
            if(!JointDataToVector(values, joint_pos, jnum)) {
                ROS_WARN_STREAM("Cannot convert joint data to vector");
                return false;
            }
            
            if(!msg_in.getVelocities(values)) {
//                 ROS_WARN_STREAM("Cannot get velocities from feedback message");
//                 return false; //TODO: in controller set valid fields!
            }
            if(!JointDataToVector(values, joint_vel, jnum)) {
                ROS_WARN_STREAM("Cannot convert joint data to vector");
                return false;
            }

            std::vector<std::string> pub_joint_names;
            std::vector<double> pub_joint_pos;
            std::vector<double> pub_joint_vel;
            if(publish_states_separately) {
                pub_joint_pos = joint_pos;
                pub_joint_vel = joint_vel;
                pub_joint_names = grouped_joint_names[msg_in.getRobotID()];
            } else {

                int offset = 0;
                for(unsigned int i = 0; i < msg_in.getRobotID(); ++i) {
                    offset += grouped_joint_names[i].size();
                }
                for(unsigned int i = 0; i < jnum; ++i) {
                    joints_position[i + offset] = joint_pos[i];
                    joints_vel[i + offset] = joint_vel[i];
                }
                group_received[msg_in.getRobotID()] = true;

                //check whether all groups were received othervise return false
                for(unsigned int i = 0; i < group_received.size(); ++i) {
                    if(!group_received[i]) {
                        return false; //Not ready to publish yet
                    }
                }

                for(unsigned int i = 0; i < grouped_joint_names.size(); ++i) {
                    for(unsigned int j = 0; j < grouped_joint_names[i].size(); ++j) {
                        pub_joint_names.push_back(grouped_joint_names[i].at(j));
                    }
                }
                pub_joint_pos = joints_position;
                pub_joint_vel = joints_vel;

                for(unsigned int i = 0; i < group_received.size(); ++i) {
                    group_received[i] = false;
                }
            }

            // assign values to messages
            control_msgs::FollowJointTrajectoryFeedback tmp_control_state;  // always start with a "clean" message
            tmp_control_state.header.stamp = ros::Time::now();
            tmp_control_state.joint_names = pub_joint_names;
            tmp_control_state.actual.positions = pub_joint_pos;
            tmp_control_state.actual.velocities = pub_joint_vel;
            control_state = tmp_control_state;

            sensor_msgs::JointState tmp_sensor_state;
            tmp_sensor_state.header.stamp = ros::Time::now();
            tmp_sensor_state.name = pub_joint_names;
            tmp_sensor_state.position = pub_joint_pos;
            tmp_sensor_state.velocity = pub_joint_vel;
            sensor_state = tmp_sensor_state;

            return true;
        }


    }
}



