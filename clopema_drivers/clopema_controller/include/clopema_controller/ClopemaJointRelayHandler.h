#ifndef CLOPEMAJOINTRELAYHANDLER_H
#define CLOPEMAJOINTRELAYHANDLER_H

#include <industrial_robot_client/joint_relay_handler.h>
#include <simple_message/messages/joint_feedback_message.h>
#include <simple_message/joint_data.h>

namespace industrial_robot_client {
    namespace joint_relay_handler {

        using industrial::shared_types::shared_real;
        using industrial::smpl_msg_connection::SmplMsgConnection;
        using namespace industrial::simple_message;
        using industrial::joint_feedback_message::JointFeedbackMessage;
        using industrial::joint_data::JointData;


        class ClopemaJointRelayHandler : public industrial::message_handler::MessageHandler {

                // since this class defines a different init(), this helps find the base-class init()
                using industrial::message_handler::MessageHandler::init;
            public:
                ClopemaJointRelayHandler();
                ~ClopemaJointRelayHandler();


                /**
                 * \brief Class initializer
                 *
                 * \param connection simple message connection that will be used to send replies.
                 * \param joint_names list of joint-names for msg-publishing.
                 *   - Count and order should match data from robot connection.
                 *   - Use blank-name to exclude a joint from publishing.
                 *
                 * \return true on success, false otherwise (an invalid message type)
                 */
                bool init(SmplMsgConnection* connection, std::vector<std::vector<std::string> >& joint_names);

                static bool JointDataToVector(const industrial::joint_data::JointData &joints, std::vector<double> &vec, int len);
            protected:
                /** \brief Process Joint Feedback Message
                  * \details Based on param 'publish_states_separately' publish or store state for feature publishing */
                bool internalCB(JointFeedbackMessage& in);

                /**
                * \brief Convert joint message into publish message-types
                *
                * \param[in] msg_in Joint message from robot connection
                * \param[out] control_state FollowJointTrajectoryFeedback message for ROS publishing
                * \param[out] sensor_state JointState message for ROS publishing
                *
                * \return true on success, false otherwise
                */
                bool create_messages(JointFeedbackMessage& msg_in, control_msgs::FollowJointTrajectoryFeedback& control_state, sensor_msgs::JointState& sensor_state);

            private:
                bool internalCB(SimpleMessage& in);
                

            protected:
                /** \brief Whether publish the message for each group separataly or at once when information from all groups are received */
                bool publish_states_separately;

                ros::Publisher pub_joint_control_state_;
                ros::Publisher pub_joint_sensor_state_;
                ros::NodeHandle node_;
                std::vector<std::vector<std::string> > grouped_joint_names;
                std::vector<double> joints_position;
                std::vector<double> joints_vel;
                std::vector<bool> group_received;

        };
    }
}
#endif // CLOPEMAJOINTRELAYHANDLER_H
