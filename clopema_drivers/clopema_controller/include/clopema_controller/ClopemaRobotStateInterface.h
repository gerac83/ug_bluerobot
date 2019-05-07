#ifndef CLOPEMAROBOTSTATEINTERFACE_H
#define CLOPEMAROBOTSTATEINTERFACE_H

#include <vector>
#include <string>
#include <boost/format.hpp>
#include <simple_message/smpl_msg_connection.h>
#include <simple_message/message_manager.h>
#include <simple_message/message_handler.h>
#include <simple_message/socket/tcp_client.h>
#include <simple_message/smpl_msg_connection.h>


#include <clopema_controller/ClopemaJointRelayHandler.h>
#include <industrial_robot_client/robot_status_relay_handler.h>

namespace industrial_robot_client {
    namespace robot_state_interface {

        using industrial::smpl_msg_connection::SmplMsgConnection;
        using industrial::message_manager::MessageManager;
        using industrial::message_handler::MessageHandler;
        using industrial::tcp_client::TcpClient;
        using industrial_robot_client::joint_relay_handler::ClopemaJointRelayHandler;
        namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;

        class ClopemaRobotStateInterface {
            public:
                ClopemaRobotStateInterface();
                ~ClopemaRobotStateInterface();


                /**
                * \brief Initialize robot connection using default method.
                *
                * \param default_ip default IP address to use for robot connection [OPTIONAL]
                *                    - this value will be used if ROS param "robot_ip_address" cannot be read
                * \param default_port default port to use for robot connection [OPTIONAL]
                *                    - this value will be used if ROS param "~port" cannot be read
                *
                * \return true on success, false otherwise
                */
                bool init(std::string default_ip = "", int default_port = StandardSocketPorts::STATE);

                /**
                * \brief Initialize robot connection using specified method.
                *
                * \param connection new robot-connection instance (ALREADY INITIALIZED).
                *
                * \return true on success, false otherwise
                */
                bool init(SmplMsgConnection* connection);

                /**
                * \brief Begin processing messages and publishing topics.
                */
                void run();

                /**
                 * \brief get current robot-connection instance.
                 *
                 * \return current robot connection object
                 */
                SmplMsgConnection* get_connection() {
                    return this->connection_;
                }

                /**
                 * \brief get active message-manager object
                 *
                 * \return current message-manager object
                 */
                MessageManager* get_manager() {
                    return &this->manager_;
                }

                /**
                * \brief Add a new handler.
                *
                * \param new message-handler for a specific msg-type (ALREADY INITIALIZED).
                * \param replace existing handler (of same msg-type), if exists
                */
                void add_handler(MessageHandler* handler, bool allow_replace = true) {
                    this->manager_.add(handler, allow_replace);
                }

            protected:
                TcpClient default_tcp_connection_;
                SmplMsgConnection* connection_;
                MessageManager manager_;

                ClopemaJointRelayHandler joint_handler_clopema_;
                industrial_robot_client::robot_status_relay_handler::RobotStatusRelayHandler default_robot_status_handler_;
        };
    }
}
#endif // CLOPEMAROBOTSTATEINTERFACE_H
