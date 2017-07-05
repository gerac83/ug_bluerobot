/**
 * Mechanical calibration node will read the robot_description
 * and save the calibrated version to the output file.
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <kdl/frames.hpp>

#include "urdf_parser.cpp"

#define bfs boost::filesystem

std::vector<std::vector<double> > DH_1; /*First robot DH notation - may changed based on calibration file*/
std::vector<std::vector<double> > DH_2; /*Second robot DH notation - may changed based on calibration file*/
#define ALPHA 0
#define A 1
#define THETA 2
#define D 3
/** \brief apply calibration data from file
 *  \param filename - path to the calibration file
 *  \param model - urdf model which will be modified
 *  \return true if successful
 */
bool apply_calibration_data(const bfs::path & filename, urdf::Model& model);

/** \brief Get default DH parameters*/
std::vector<std::vector<double> > get_default_dh();

int main(int argc, char **argv) {
    ros::init(argc, argv, "mechanical_calibration");
    ros::NodeHandle node("~");

    /*Set default DH parameters*/
    DH_1 = get_default_dh();
    DH_2 = get_default_dh();

    //Load clopema_partner environment variable
    std::string partner("");
    ros::get_environment_variable(partner, "CLOPEMA_PARTNER");
    if (partner.empty()) {
        ROS_ERROR("CLOPEMA_PARTNER was not set.");
        return -1;
    }

    //Load clopema_description package path
    std::string pkg_path = ros::package::getPath("clopema_description");
    if (pkg_path.empty()) {
        ROS_ERROR("Package clopema_description not found");
        return -1;
    }

    //Load clopema_description urdf model
    bfs::path model_path(pkg_path);
    model_path /= "robots/clopema.urdf";
    urdf::Model model;
    if (!model.initFile(model_path.c_str())) {
        ROS_ERROR_STREAM("Can not load model: " << model_path);
        return -1;
    }

    //Load and apply calibration data
    bfs::path calib_folder(pkg_path);
    calib_folder = calib_folder / ("calibration_" + partner);
    ROS_DEBUG_STREAM("Calibration folder: " << calib_folder);
    if (!(bfs::exists(calib_folder) && bfs::is_directory(calib_folder))) {
        ROS_INFO("Calibration folder does not exists, using default calibration");
    } else {
        typedef std::vector<bfs::path> vec;
        vec v;
        std::copy(bfs::directory_iterator(calib_folder), bfs::directory_iterator(), back_inserter(v));
        for (vec::const_iterator it(v.begin()); it != v.end(); ++it) {
            bfs::path calib_file(*it);
            ROS_INFO_STREAM(calib_file);
            if(calib_file.extension().string() == ".yaml") {
                continue;
            }
            if(calib_file.extension().string() == ".bin") {
                continue;
            }
            if(!apply_calibration_data(calib_file,model)) {
                ROS_ERROR_STREAM("Can not apply calibration data: " << calib_file);
            }
        }
    }

    //Save calibrated model
    bfs::path calibrated_path(pkg_path);
    calibrated_path /= "robots/clopema.urdf";
    std::ofstream calibrated_file(calibrated_path.c_str());
    if (!calibrated_file) {
        ROS_ERROR("Can not open file for writing.");
        return -1;
    }
    calibrated_file << getXML(model);
    calibrated_file.close();

    //Save DH parameters in YAML format
    bfs::path dh_path(pkg_path);
    dh_path /= "robots/dh.yaml";
    std::ofstream dh_file(dh_path.c_str());
    if (!dh_file) {
        ROS_ERROR("Can not open file for writing.");
        return -1;
    }
    dh_file << "DH:" << std::endl;
    dh_file << "  r1:" << std::endl;
    for (int i = 0; i < 7; ++i) {
        dh_file << "    '" << i << "': {a: " << DH_1[i][A] << ", alpha: "
                << DH_1[i][ALPHA] << ", d: " << DH_1[i][D] << ", theta: "
                << DH_1[i][THETA] << "}" << std::endl;
    }
    dh_file << "  r2:" << std::endl;
    for (int i = 0; i < 7; ++i) {
        dh_file << "    '" << i << "': {a: " << DH_2[i][A] << ", alpha: "
                << DH_2[i][ALPHA] << ", d: " << DH_2[i][D] << ", theta: "
                << DH_2[i][THETA] << "}" << std::endl;
    }
    dh_file.close();

    return 0;
}

std::vector<std::vector<double> > get_default_dh() {
    std::vector<std::vector<double> > DH;
    std::vector<double> tmp_dh(4, 0);
    tmp_dh[ALPHA] = 0;
    tmp_dh[A] = 0;
    tmp_dh[THETA] = 0;
    tmp_dh[D] = 0;
    DH.push_back(tmp_dh);
    tmp_dh[ALPHA] = -M_PI_2;
    tmp_dh[A] = 0.150;
    tmp_dh[THETA] = 0;
    tmp_dh[D] = 0.450;
    DH.push_back(tmp_dh);
    tmp_dh[ALPHA] = M_PI;
    tmp_dh[A] = 0.614;
    tmp_dh[THETA] = 3 * M_PI_2;
    tmp_dh[D] = 0;
    DH.push_back(tmp_dh);
    tmp_dh[ALPHA] = -M_PI_2;
    tmp_dh[A] = 0.200;
    tmp_dh[THETA] = 0;
    tmp_dh[D] = 0;
    DH.push_back(tmp_dh);
    tmp_dh[ALPHA] = M_PI_2;
    tmp_dh[A] = 0.0;
    tmp_dh[THETA] = 0;
    tmp_dh[D] = -0.640;
    DH.push_back(tmp_dh);
    tmp_dh[ALPHA] = M_PI_2;
    tmp_dh[A] = 0.030;
    tmp_dh[THETA] = M_PI_2 * 3;
    tmp_dh[D] = 0;
    DH.push_back(tmp_dh);
    tmp_dh[ALPHA] = 0;
    tmp_dh[A] = 0;
    tmp_dh[THETA] = 0;
    tmp_dh[D] = 0.200;
    DH.push_back(tmp_dh);
    return DH;
}

bool apply_calibration_data(const bfs::path & filename, urdf::Model& model) {
    std::ifstream in(filename.c_str());
    if (!in) {
        ROS_ERROR_STREAM("Can not open file " << filename <<", using default calibration.");
        return false;
    }

    int calib_type(0);
    std::string calib_type_str("");
    while(calib_type_str.empty() && !in.eof()) {
        std::getline(in,calib_type_str);
    }
    std::istringstream(calib_type_str) >> calib_type;

    bool ret_value = true;
    ROS_INFO_STREAM("Calibration file type: " << calib_type);
    switch(calib_type) {
        case 1:
        {
            while (!in.eof()) {
                std::string joint_name;
                std::getline(in,joint_name);
                if(joint_name.empty()) {
                    continue;
                }
                boost::shared_ptr<const urdf::Joint> joint_ptr = model.getJoint(joint_name);
                if(!joint_ptr) {
                    ret_value = false;
                    ROS_ERROR_STREAM("Joint does not exist, name: " << joint_name);
                } else {
                    std::string calibration_values("");
                    std::getline(in,calibration_values);
                    urdf::Joint joint = *joint_ptr;
                    double r,p,y;
                    std::istringstream(calibration_values)
                    >> joint.parent_to_joint_origin_transform.position.x
                    >> joint.parent_to_joint_origin_transform.position.y
                    >> joint.parent_to_joint_origin_transform.position.z
                    >> r
                    >> p
                    >> y;
                    joint.parent_to_joint_origin_transform.rotation.setFromRPY(r,p,y);
                    model.joints_.find(joint_name)->second.reset(new urdf::Joint(joint));
                    ROS_DEBUG_STREAM("New joint pose: ");
                    ROS_DEBUG_STREAM(joint.parent_to_joint_origin_transform.position.x);
                    ROS_DEBUG_STREAM(joint.parent_to_joint_origin_transform.position.y);
                    ROS_DEBUG_STREAM(joint.parent_to_joint_origin_transform.position.z);
                    ROS_DEBUG_STREAM(joint.parent_to_joint_origin_transform.rotation.w);
                    ROS_DEBUG_STREAM(joint.parent_to_joint_origin_transform.rotation.x);
                    ROS_DEBUG_STREAM(joint.parent_to_joint_origin_transform.rotation.y);
                    ROS_DEBUG_STREAM(joint.parent_to_joint_origin_transform.rotation.z);
                }
            }
            break;
        }
        case 2:
        {
            std::string link_name;
            std::getline(in,link_name);
            if (link_name.empty()) {
                ret_value = false;
                break;
            }

            boost::shared_ptr<const urdf::Link> link = model.getLink(link_name);
            if (!link) {
                ROS_ERROR_STREAM("The link with name " << link_name << "does not exists.");
                ret_value = false;
                break;
            }

            KDL::Frame f = KDL::Frame::Identity();
            double data[4][4];
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    if (in.eof()) {
                        ROS_ERROR_STREAM("Calibration data invalid.");
                        ret_value = false;
                        break;
                    }
                    in >> data[i][j];
                }
            }
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    f.M.data[i * 3 + j] = data[i][j];
                }
                f.p.data[i] = data[i][3];
            }
            f = f.Inverse();

            boost::shared_ptr<const urdf::Joint> joint_ptr = link->parent_joint;
            if(!joint_ptr) {
                ret_value = false;
                ROS_ERROR_STREAM("Parent joint does not exist, are you calibrating root link?.");
            } else {
                urdf::Joint joint = *joint_ptr;
                joint.parent_to_joint_origin_transform.position.x = f.p.x();
                joint.parent_to_joint_origin_transform.position.y = f.p.y();
                joint.parent_to_joint_origin_transform.position.z = f.p.z();
                double r,p,y;
                f.M.GetRPY(r, p, y);
                joint.parent_to_joint_origin_transform.rotation.setFromRPY(r,p,y);

                model.joints_.find(link->parent_joint->name)->second.reset(new urdf::Joint(joint));
            }
            break;
        }
        case 3:
        {
            std::string arm_prefix;
            std::getline(in,arm_prefix);
            if (arm_prefix.empty()) {
                ret_value = false;
                break;
            }
            if(!(arm_prefix == "r1" || arm_prefix == "r2")) {
                ROS_ERROR_STREAM("Unsupported arm prefix: " << arm_prefix);
                ret_value = false;
                break;
            }

            std::vector<std::string> joint_names;
            joint_names.push_back(arm_prefix + "_joint_s");
            joint_names.push_back(arm_prefix + "_joint_l");
            joint_names.push_back(arm_prefix + "_joint_u");
            joint_names.push_back(arm_prefix + "_joint_r");
            joint_names.push_back(arm_prefix + "_joint_b");
            joint_names.push_back(arm_prefix + "_joint_t");
            joint_names.push_back(arm_prefix + "_joint_t_f");
            /*theta,d,alpha,a*/
            double data[joint_names.size()][4];
            for (int i = 0; i < joint_names.size(); ++i) {
                for (int j = 0; j < 4; ++j) {
                    if (in.eof()) {
                        ROS_ERROR_STREAM("Calibration data invalid.");
                        ret_value = false;
                        break;
                    }
                    in >> data[i][j];
                }
            }

            for (int i = 0; i < joint_names.size(); ++i) {
                if(arm_prefix == "r1") {
                    DH_1[i][THETA] = data[i][0];
                    DH_1[i][D] = data[i][1];
                    DH_1[i][ALPHA] = data[i][2];
                    DH_1[i][A] = data[i][3];
                } else if(arm_prefix == "r2") {
                    DH_2[i][THETA] = data[i][0];
                    DH_2[i][D] = data[i][1];
                    DH_2[i][ALPHA] = data[i][2];
                    DH_2[i][A] = data[i][3];
                }

                KDL::Frame f = KDL::Frame::DH(data[i][3], data[i][2], data[i][1], data[i][0]);
                boost::shared_ptr<const urdf::Joint> joint_ptr = model.getJoint(joint_names[i]);
                if(!joint_ptr) {
                    ret_value = false;
                    ROS_ERROR_STREAM("Joint does not exist: " << joint_names[i]);
                } else {
                    urdf::Joint joint = *joint_ptr;
                    joint.parent_to_joint_origin_transform.position.x = f.p.x();
                    joint.parent_to_joint_origin_transform.position.y = f.p.y();
                    joint.parent_to_joint_origin_transform.position.z = f.p.z();
                    double r,p,y;
                    f.M.GetRPY(r, p, y);
                    joint.parent_to_joint_origin_transform.rotation.setFromRPY(r,p,y);
                    model.joints_.find(joint_names[i])->second.reset(new urdf::Joint(joint));
                }
            }

            break;
        }
        case 4:
        { //MDH
            std::string arm_prefix;
            std::getline(in,arm_prefix);
            if (arm_prefix.empty()) {
                ret_value = false;
                break;
            }
            if(!(arm_prefix == "r1" || arm_prefix == "r2")) {
                ROS_ERROR_STREAM("Unsupported arm prefix: " << arm_prefix);
                ret_value = false;
                break;
            }

            std::vector<std::string> joint_names;
            joint_names.push_back(arm_prefix + "_joint_s");
            joint_names.push_back(arm_prefix + "_joint_l");
            joint_names.push_back(arm_prefix + "_joint_u");
            joint_names.push_back(arm_prefix + "_joint_r");
            joint_names.push_back(arm_prefix + "_joint_b");
            joint_names.push_back(arm_prefix + "_joint_t");
            joint_names.push_back(arm_prefix + "_joint_t_f");
            /*theta,d,alpha,a, mdh?*/
            double data[joint_names.size()][5];
            for (int i = 0; i < joint_names.size(); ++i) {
                for (int j = 0; j < 5; ++j) {
                    if (in.eof()) {
                        ROS_ERROR_STREAM("Calibration data invalid.");
                        ret_value = false;
                        break;
                    }
                    in >> data[i][j];
                }
            }

            for (int i = 0; i < joint_names.size(); ++i) {
                /*if(arm_prefix == "r1") { //TODO transform MDH to DH
                    DH_1[i][THETA] = data[i][2];
                    DH_1[i][D] = data[i][3];
                    DH_1[i][ALPHA] = data[i][0];
                    DH_1[i][A] = data[i][1];
                } else if(arm_prefix == "r2") {
                    DH_2[i][THETA] = data[i][2];
                    DH_2[i][D] = data[i][3];
                    DH_2[i][ALPHA] = data[i][0];
                    DH_2[i][A] = data[i][1];
                }*/

                KDL::Frame f = KDL::Frame::DH(data[i][1], data[i][0], data[i][3], data[i][2]);
                if(fabs(data[i][4] -1) < 0.00001) {
                    f = KDL::Frame::Identity();
                    KDL::Frame rz = KDL::Frame::Identity();
                    KDL::Frame rx = KDL::Frame::Identity();
                    KDL::Frame ry = KDL::Frame::Identity();
                    KDL::Frame tx = KDL::Frame::Identity();
                    rz.M = KDL::Rotation::RotZ(data[i][2]);
                    rx.M = KDL::Rotation::RotX(data[i][0]);
                    ry.M = KDL::Rotation::RotY(data[i][1]);
                    tx.p = KDL::Vector(data[i][3],0,0);
                    f = rz*tx*rx*ry;
                }
                boost::shared_ptr<const urdf::Joint> joint_ptr = model.getJoint(joint_names[i]);
                if(!joint_ptr) {
                    ret_value = false;
                    ROS_ERROR_STREAM("Joint does not exist: " << joint_names[i]);
                } else {
                    urdf::Joint joint = *joint_ptr;
                    joint.parent_to_joint_origin_transform.position.x = f.p.x();
                    joint.parent_to_joint_origin_transform.position.y = f.p.y();
                    joint.parent_to_joint_origin_transform.position.z = f.p.z();
                    double r,p,y;
                    f.M.GetRPY(r, p, y);
                    joint.parent_to_joint_origin_transform.rotation.setFromRPY(r,p,y);
                    model.joints_.find(joint_names[i])->second.reset(new urdf::Joint(joint));
                }
            }

            break;
        }
        case 5:
        {
            std::string link_name;
            std::getline(in,link_name);
            if (link_name.empty()) {
                ret_value = false;
                break;
            }

            std::vector<double> data(7,0);
            for (int j = 0; j < data.size(); ++j) {
                if (in.eof()) {
                    ROS_ERROR_STREAM("Calibration data invalid.");
                    ret_value = false;
                    break;
                }
                in >> data[j];
            }

            boost::shared_ptr<const urdf::Link> link = model.getLink(link_name);
            if (!link) {
                ROS_ERROR_STREAM("The link with name " << link_name << "does not exists.");
                ret_value = false;
                break;
            }

            if(link->inertial) {
                link->inertial->origin.rotation.setFromRPY(data[3],data[4],data[5]);
                link->inertial->origin.position.x = data[0];
                link->inertial->origin.position.y = data[1];
                link->inertial->origin.position.z = data[2];
                link->inertial->mass = data[6];
            } else {
                ROS_ERROR_STREAM("Link has no inertial part.");
                ret_value = false;
                break;
            }

            break;
        }
        default:
        {
            ROS_ERROR_STREAM("Unsupported calibration data type: " << calib_type);
            ret_value =false;
            break;
        }
    }

    in.close();
    return ret_value;
}
