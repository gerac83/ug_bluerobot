/**
* Copyright (c) CTU in Prague  - All Rights Reserved
* Created on: Oct 15, 2013
*     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
*  Institute: Czech Technical University in Prague
*/

#include <clopema_kinematics/clopema_arm_kinematics_plugin.h>

/*Register ClopemaArmKinematicsPlugin as the KinemtacsBase implementation*/
PLUGINLIB_EXPORT_CLASS(ClopemaArmKinematicsPlugin, kinematics::KinematicsBase);


bool ClopemaArmKinematicsPlugin::getAllIKT_cb(clopema_kinematics::ClopemaGetAllIktRequest& req, clopema_kinematics::ClopemaGetAllIktResponse& res) {
    std::vector<double> ik_seed_state(num_joints_, 0.0);
    std::vector<std::vector<double> > solutions;
    moveit_msgs::MoveItErrorCodes error_code;
    this->getPositionsIK(req.pose,ik_seed_state,solutions,error_code);

    res.solutions.layout.data_offset = 0;
    res.solutions.layout.dim.resize(2);
    res.solutions.layout.dim[0].label = "solutions";
    res.solutions.layout.dim[0].size = solutions.size();
    res.solutions.layout.dim[0].stride = num_joints_ * solutions.size();
    res.solutions.layout.dim[1].label = "joints";
    res.solutions.layout.dim[1].size = num_joints_;
    res.solutions.layout.dim[1].stride = num_joints_;

    res.solutions.data.resize(res.solutions.layout.dim[0].stride, 0.0);
    for (int i = 0; i < solutions.size(); ++i) {
        for (int j = 0; j < solutions[i].size(); ++j) {
            res.solutions.data[i * res.solutions.layout.dim[1].stride  + j] =
                    solutions[i].at(j);
        }
    }

    return true;
}

/*
 * Get all configurations which obeys limits.
 */
bool ClopemaArmKinematicsPlugin::getPositionsIK(
        const geometry_msgs::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        std::vector<std::vector<double> > &solutions,
        moveit_msgs::MoveItErrorCodes& error_code) const {
    std::vector<double> vfree(0);
    KDL::Frame frame;
    tf::poseMsgToKDL(ik_pose, frame);

    int numsol = ik_solver_->solve(frame, vfree);
    ik_solver_->reduce_joints(num_joints_);
    ROS_DEBUG_STREAM("Number of IKT solution" << numsol);

    std::vector<std::vector<double> > tmp_solutions;
    if (numsol) {
        for (int s = 0; s < numsol; ++s) {
            std::vector<double> sol;
            ik_solver_->getSolution(s, sol);
            bool obeys_limits = true;
            for (unsigned int i = 0; i < sol.size(); i++) {
                if (joint_has_limits_vector_[i]
                        && (sol[i] < joint_min_vector_[i]
                                || sol[i] > joint_max_vector_[i])) {
                    obeys_limits = false;
                    break;
                }
            }
            if (obeys_limits) {
                tmp_solutions.push_back(sol);
            }
        }
    }

    std::vector<double> squared_error;
    for (unsigned int i = 0; i < tmp_solutions.size(); ++i) {
        if (ik_seed_state.size() != tmp_solutions[i].size()) {
            ROS_ERROR_STREAM(
                    "Seed state have no equal size to the solution vector.");
            squared_error.push_back(-1);
            continue;
        }

        double err = 0;
        for (unsigned int j = 0; j < tmp_solutions[i].size(); ++j) {
            err += (tmp_solutions[i][j] - ik_seed_state[i])
                    * (tmp_solutions[i][j] - ik_seed_state[i]);
        }
        squared_error.push_back(err);
    }
    while (true) {
        if (squared_error.size() == 0) {
            break;
        }
        int pos = std::distance(squared_error.begin(),
                std::min_element(squared_error.begin(), squared_error.end()));
        if (squared_error[pos] != -1) {
            solutions.push_back(tmp_solutions[pos]);
        }
        tmp_solutions.erase(tmp_solutions.begin() + pos);
        squared_error.erase(squared_error.begin() + pos);
    }

    if (solutions.empty()) {
        error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
        return false;
    } else {
        error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        return true;
    }
}

bool ClopemaArmKinematicsPlugin::getPositionIK(
        const geometry_msgs::Pose& ik_pose,
        const std::vector<double>& ik_seed_state, std::vector<double>& solution,
        moveit_msgs::MoveItErrorCodes& error_code,
        const kinematics::KinematicsQueryOptions &options) const {
    std::vector<double> vfree(0);
    KDL::Frame frame;
    tf::poseMsgToKDL(ik_pose, frame);

    int numsol = ik_solver_->solve(frame, vfree);
    ik_solver_->reduce_joints(num_joints_);
    ROS_DEBUG_STREAM("Number of IKT solution" << numsol);
    if (numsol) {
        double min_squared_err = -1;
        int min_err_i = -1;
        for (int s = 0; s < numsol; ++s) {
            std::vector<double> sol;
            ik_solver_->getSolution(s, sol);
            bool obeys_limits = true;
            for (unsigned int i = 0; i < sol.size(); i++) {
                if (joint_has_limits_vector_[i]
                        && (sol[i] < joint_min_vector_[i]
                                || sol[i] > joint_max_vector_[i])) {
                    obeys_limits = false;
                    break;
                }
            }
            if (obeys_limits) {
                if (ik_seed_state.size() != sol.size()) {
                    ROS_ERROR_STREAM(
                            "Seed state have no equal size to the solution vector.");
                    error_code.val =
                            moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
                    return false;
                }
                //compute squared err
                double err = 0;
                for (unsigned int i = 0; i < sol.size(); i++) {
                    err += (sol[i] - ik_seed_state[i])
                            * (sol[i] - ik_seed_state[i]);
                }
                if (min_squared_err == -1) {
                    min_squared_err = err;
                    min_err_i = s;
                } else if (min_squared_err > err) {
                    min_squared_err = err;
                    min_err_i = s;
                }
            }
        }
        if (min_err_i != -1) {
            ik_solver_->getSolution(min_err_i, solution);
            error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
            return true;
        }
    }

    error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
}

bool ClopemaArmKinematicsPlugin::searchPositionIK(
        const geometry_msgs::Pose& ik_pose,
        const std::vector<double>& ik_seed_state, double timeout,
        std::vector<double>& solution,
        moveit_msgs::MoveItErrorCodes& error_code,
        const kinematics::KinematicsQueryOptions &options) const {
    return getPositionIK(ik_pose, ik_seed_state, solution, error_code);
}

bool ClopemaArmKinematicsPlugin::searchPositionIK(
        const geometry_msgs::Pose& ik_pose,
        const std::vector<double>& ik_seed_state, double timeout,
        const std::vector<double>& consistency_limits,
        std::vector<double>& solution,
        moveit_msgs::MoveItErrorCodes& error_code,
        const kinematics::KinematicsQueryOptions &options) const {
    return getPositionIK(ik_pose, ik_seed_state, solution, error_code);
}

bool ClopemaArmKinematicsPlugin::searchPositionIK(
        const geometry_msgs::Pose& ik_pose,
        const std::vector<double>& ik_seed_state, double timeout,
        std::vector<double>& solution, const IKCallbackFn& solution_callback,
        moveit_msgs::MoveItErrorCodes& error_code,
        const kinematics::KinematicsQueryOptions &options) const {

    std::vector<std::vector<double> > solutions;
    if (!getPositionsIK(ik_pose, ik_seed_state, solutions, error_code)) {
        ROS_DEBUG_STREAM("No solution whatsoever");
        error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
        return false;
    }
    for (int i = 0; i < (int) solutions.size(); ++i) {
        solution = solutions.at(i);
        solution_callback(ik_pose, solution, error_code);
        if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
            ROS_DEBUG_STREAM("Solution passes");
            return true;
        }
    }
    ROS_DEBUG_STREAM("Solution has error code " << error_code);
    return false;
}

bool ClopemaArmKinematicsPlugin::searchPositionIK(
        const geometry_msgs::Pose& ik_pose,
        const std::vector<double>& ik_seed_state, double timeout,
        const std::vector<double>& consistency_limits,
        std::vector<double>& solution, const IKCallbackFn& solution_callback,
        moveit_msgs::MoveItErrorCodes& error_code,
        const kinematics::KinematicsQueryOptions &options) const {
    return searchPositionIK(ik_pose, ik_seed_state, timeout, solution,
            solution_callback, error_code);
}

int ClopemaArmKinematicsPlugin::getKDLSegmentIndex(const KDL::Chain &chain,
        const std::string &name) const {
    int i = 0; // segment number
    while (i < (int) chain.getNrOfSegments()) {
        if (chain.getSegment(i).getName() == name) {
            return i + 1;
        }
        i++;
    }
    return -1;
}

bool ClopemaArmKinematicsPlugin::getPositionFK(
        const std::vector<std::string>& link_names,
        const std::vector<double>& joint_angles,
        std::vector<geometry_msgs::Pose>& poses) const {
    KDL::Frame p_out;
    KDL::JntArray jnt_pos_in;

    if (joint_angles.size() != num_joints_) {
        ROS_ERROR_STREAM("[FK] Number of joints is not equal to our solver");
        return false;
    }
    jnt_pos_in.resize(num_joints_);
    for (int i = 0; i < num_joints_; i++) {
        jnt_pos_in(i) = joint_angles[i];
    }

    poses.resize(link_names.size());

    bool valid = true;
    for (unsigned int i = 0; i < poses.size(); i++) {
        if (jnt_to_pose_solver_->JntToCart(jnt_pos_in, p_out,
                getKDLSegmentIndex(kdl_chain_, link_names[i])) >= 0) {
            tf::poseKDLToMsg(p_out, poses[i]);
        } else {
            ROS_ERROR("Could not compute FK for %s", link_names[i].c_str());
            valid = false;
        }
    }
    return valid;

    return false;
}

bool ClopemaArmKinematicsPlugin::initialize(
        const std::string& robot_description, const std::string& group_name,
        const std::string& base_frame, const std::string& tip_frame,
        double search_discretization) {
    setValues(robot_description, group_name, base_frame, tip_frame,
            search_discretization);
    tip_frames_.push_back(tip_frame_);

    ros::NodeHandle node_handle("~/" + group_name);
    urdf::Model robot_model;
    std::string xml_string;
    std::string urdf_xml, full_urdf_xml;
    node_handle.param("urdf_xml", urdf_xml, std::string("robot_description"));
    node_handle.searchParam(urdf_xml, full_urdf_xml);

    ROS_DEBUG("Reading xml file from parameter server\n");
    if (!node_handle.getParam(full_urdf_xml, xml_string)) {
        ROS_FATAL("Could not load the xml from parameter server: %s\n",
                urdf_xml.c_str());
        return false;
    }
    node_handle.param(full_urdf_xml, xml_string, std::string());
    robot_model.initString(xml_string);

    if (group_name == "r1_arm" || group_name == "r2_arm" || group_name == "r1_arm_and_manipulator") {
        num_joints_ = 6;
        ik_solver_ = new clopema_kinematics::IKSolver;

        if (tip_frame.substr(0, 2) == "r1" || tip_frame.substr(0, 3) == "/r1") {
            ik_solver_->setDHFromParam(node_handle, "r1");
        } else {
            ik_solver_->setDHFromParam(node_handle, "r2");
        }
    } else if (group_name == "r1_xtion" || group_name == "r2_xtion") {
        num_joints_ = 5;
        ik_solver_ = new clopema_kinematics::IKSolver;
        ik_solver_->setDefaultDHParamXtion();
        ik_solver_->setDHParamXtion(group_name.substr(0, 2) + "_link_6",
                tip_frame, xml_string);
    } else {
        ROS_ERROR_STREAM(
                "Not known group for CLOPEMA IKT solver: " << group_name);
        return false;
    }

    std::string ik_srv_name = "/" + group_name + "/get_ik_all";
    if(!ros::service::exists(ik_srv_name, false))
        srv_get_ik_all = node_handle.advertiseService(ik_srv_name, &ClopemaArmKinematicsPlugin::getAllIKT_cb, this);

    //Forward kinematics setting
    KDL::Tree my_tree;
    if (!kdl_parser::treeFromString(xml_string, my_tree)) {
        ROS_ERROR("[FK]: Failed to construct KDL tree.");
        return false;
    }
    my_tree.getChain(base_frame, tip_frame, kdl_chain_);
    jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

    //Joint limits initialization
    boost::shared_ptr<urdf::Link> link = boost::const_pointer_cast<urdf::Link>(
            robot_model.getLink(tip_frame_));
    while (link->name != base_frame_ && joint_names_.size() <= num_joints_) {
        link_names_.push_back(link->name);
        boost::shared_ptr<urdf::Joint> joint = link->parent_joint;
        if (joint) {
            if (joint->type != urdf::Joint::UNKNOWN
                    && joint->type != urdf::Joint::FIXED) {
                joint_names_.push_back(joint->name);
                float lower, upper;
                int hasLimits;
                if (joint->type != urdf::Joint::CONTINUOUS) {
                    if (joint->safety) {
                        lower = joint->safety->soft_lower_limit;
                        upper = joint->safety->soft_upper_limit;
                    } else {
                        lower = joint->limits->lower;
                        upper = joint->limits->upper;
                    }
                    hasLimits = 1;
                } else {
                    lower = -M_PI;
                    upper = M_PI;
                    hasLimits = 0;
                }
                if (hasLimits) {
                    joint_has_limits_vector_.push_back(true);
                    joint_min_vector_.push_back(lower);
                    joint_max_vector_.push_back(upper);
                } else {
                    joint_has_limits_vector_.push_back(false);
                    joint_min_vector_.push_back(-M_PI);
                    joint_max_vector_.push_back(M_PI);
                }
            }
        } else {
            ROS_WARN("no joint corresponding to %s", link->name.c_str());
        }
        link = link->getParent();
    }

    if (joint_names_.size() != num_joints_) {
        ROS_FATAL("Joints number mismatch.");
        return false;
    }

    std::reverse(link_names_.begin(), link_names_.end());
    std::reverse(joint_names_.begin(), joint_names_.end());
    std::reverse(joint_min_vector_.begin(), joint_min_vector_.end());
    std::reverse(joint_max_vector_.begin(), joint_max_vector_.end());
    std::reverse(joint_has_limits_vector_.begin(),
            joint_has_limits_vector_.end());

    for (size_t i = 0; i < num_joints_; ++i)
        ROS_DEBUG_STREAM(
                joint_names_[i] << " " << joint_min_vector_[i] << " " << joint_max_vector_[i] << " " << joint_has_limits_vector_[i]);

    return true;
}

const std::vector<std::string>& ClopemaArmKinematicsPlugin::getJointNames() const {
    return joint_names_;
}

void ClopemaArmKinematicsPlugin::setValues(const std::string& robot_description,
        const std::string& group_name, const std::string& base_frame,
        const std::string& tip_frame, double search_discretization) {
    this->robot_description_ = robot_description;
    this->group_name_ = group_name;
    this->base_frame_ = base_frame;
    this->tip_frame_ = tip_frame;
    this->search_discretization_ = search_discretization;
}

const std::vector<std::string>& ClopemaArmKinematicsPlugin::getLinkNames() const {
    return link_names_;
}

