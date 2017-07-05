#include <clopema_controller/ClopemaRobotStateInterface.h>

namespace industrial_robot_client {
    namespace robot_state_interface {
        ClopemaRobotStateInterface::ClopemaRobotStateInterface() {
            this->connection_ = NULL;
        }

        ClopemaRobotStateInterface::~ClopemaRobotStateInterface() {
        }

        bool ClopemaRobotStateInterface::init(std::string default_ip, int default_port) {
            std::string ip;
            int port;

            // override IP/port with ROS params, if available
            ros::param::param<std::string>("~robot_ip_address", ip, default_ip);
            ros::param::param<int>("~port", port, default_port);

            // check for valid parameter values
            if(ip.empty()) {
                ROS_ERROR("No valid robot IP address found.  Please set ROS 'robot_ip_address' param");
                return false;
            }
            if(port <= 0) {
                ROS_ERROR("No valid robot IP port found.  Please set ROS '~port' param");
                return false;
            }

            char* ip_addr = strdup(ip.c_str());  // connection.init() requires "char*", not "const char*"
            ROS_INFO("Robot state connecting to IP address: '%s:%d'", ip_addr, port);
            default_tcp_connection_.init(ip_addr, port);
            free(ip_addr);

            return init(&default_tcp_connection_);
        }

        bool ClopemaRobotStateInterface::init(SmplMsgConnection* connection) {
            this->connection_ = connection;
            if(!connection_->makeConnect())
                return false;

            // initialize message-manager
            if(!manager_.init(connection_))
                return false;

            // initialize default handlers
            std::vector<std::vector<std::string> > grouped_joints_names;
            for(unsigned int i = 1; i < 3; ++i) {
                std::vector<std::string> jn;
                jn.push_back((boost::format("r%d_joint_%s") % i % "s").str());
                jn.push_back((boost::format("r%d_joint_%s") % i % "l").str());
                jn.push_back((boost::format("r%d_joint_%s") % i % "u").str());
                jn.push_back((boost::format("r%d_joint_%s") % i % "r").str());
                jn.push_back((boost::format("r%d_joint_%s") % i % "b").str());
                jn.push_back((boost::format("r%d_joint_%s") % i % "t").str());
                grouped_joints_names.push_back(jn);
            }
            {
                std::vector<std::string> jn;
                jn.push_back("ext_axis");
                grouped_joints_names.push_back(jn);
            }

            if(!joint_handler_clopema_.init(connection_, grouped_joints_names))
                return false;
            this->add_handler(&joint_handler_clopema_);

            if(!default_robot_status_handler_.init(connection_)) {
                ROS_WARN_STREAM("Cannot initialize robot status handler");
                return false;
            }
            this->add_handler(&default_robot_status_handler_);

            return true;
        }


        void ClopemaRobotStateInterface::run() {
            while(ros::ok()) {
                manager_.spinOnce();
            }
        }

    }
}

