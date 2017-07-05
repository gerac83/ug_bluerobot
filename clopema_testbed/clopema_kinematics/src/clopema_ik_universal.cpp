/**
* Copyright (c) CTU in Prague  - All Rights Reserved
* Created on: Oct 15, 2013
*     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
*  Institute: Czech Technical University in Prague
*/

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cfloat>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <clopema_kinematics/universal_ikt.h>

namespace clopema_kinematics {

#define ROBOT_NAME        "MA1400"
#define ROBOT_DH_NOTATION \
  { -M_PI_2,     M_PI, -M_PI_2, M_PI_2,   M_PI_2,   0,   /* alpha */ \
        150,      614,     200,      0,       30,   0,   /* a */ \
          0, M_PI_2*3,       0,      0, M_PI_2*3,   0,   /* theta offset */ \
        450,        0,       0,   -640,        0, 200, } /* d */

#define ROBOT_DH_NOTATION_XTION \
  { -M_PI_2,     M_PI, -M_PI_2, M_PI_2,   M_PI_2,   0,   /* alpha */ \
        150,      614,     200,      0,       130,   0,   /* a */ \
          0, M_3_PI_2,       0,      0, M_3_PI_2,   0,   /* theta offset */ \
        450,        0,       0,   -640,        0, 150, } /* d */

#define ROBOT_JOINT_LIMITS \
  { DEG(-170), DEG(-90), DEG(-83), DEG(-150), DEG(-45), DEG(-200),  /* min */ \
    DEG(170),  DEG(155), DEG(92),  DEG(150),  DEG(180), DEG(200), } /* max */

#define ROBOT_IKT_BASE_MATRICES         base_ikt_matrices
#define ROBOT_IKT_J6_MATRICES           j6_ikt_matrices

class IKSolver {
public:

    IKSolver() :
            node_() {
        robot = (gen6r_robot_t*) malloc(sizeof(gen6r_robot_t));
        robot->name = ROBOT_NAME;

        double jl[] = ROBOT_JOINT_LIMITS;
        for (int i = 0; i < 6 * 2; i++)
            robot->joint_limits[i] = jl[i];

        robot->form_base_ikt_matrices = &ROBOT_IKT_BASE_MATRICES;
        robot->form_j6_ikt_matrices = &ROBOT_IKT_J6_MATRICES;
        robot->create_singular_position = NULL;
        memcpy(&robot_limits, &default_limits, sizeof(gen6rikt_limits_t));
        robot_limits.test_joint_limits = 0;
        setDefaultDHParam();
    }

    void setDHFromParam(ros::NodeHandle & node,
            const std::string & arm_prefix) {
        for (int i = 0; i < 6; ++i) {
            double val;
            std::stringstream name;
            name << "/DH/" << arm_prefix << "/" << (i + 1);
            if (node_.getParam(name.str() + "/alpha", val)) {
                robot->denavit_hartenberg[i] = val;
            } else {
                ROS_WARN_STREAM(
                        "Can not find DH parameter ALPHA on server, using default parameters");
                break;
            }
            if (node_.getParam(name.str() + "/a", val)) {
                robot->denavit_hartenberg[i + 6] = val * 1000;
            } else {
                ROS_WARN_STREAM(
                        "Can not find DH parameter ALPHA on server, using default parameters");
                break;
            }

            if (node_.getParam(name.str() + "/theta", val)) {
                robot->denavit_hartenberg[i + 2 * 6] = val;
            } else {
                ROS_WARN_STREAM(
                        "Can not find DH parameter ALPHA on server, using default parameters");
                break;
            }

            if (node_.getParam(name.str() + "/d", val)) {
                robot->denavit_hartenberg[i + 3 * 6] = val * 1000;
            } else {
                ROS_WARN_STREAM(
                        "Can not find DH parameter ALPHA on server, using default parameters");
                break;
            }
        }
    }

    void setDefaultDHParam() {
        double dh[] = ROBOT_DH_NOTATION;
        for (int i = 0; i < 6 * 4; i++)
            robot->denavit_hartenberg[i] = dh[i];
    }

    /*Xtion specifics functions*/

    bool equal_frames(KDL::Frame hec, KDL::Frame f) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                if (fabs(hec(i, j) - f(i, j)) > eps)
                    return false;
                if (std::isnan(hec(i, j)))
                    return false;
                if (std::isnan(f(i, j)))
                    return false;
            }
        }
        return true;
    }

    bool equal_dh_rot(KDL::Frame hec, double val_al5, double val_th5,
            double val_th6) {
        KDL::Frame f;
        f.Identity();
        f = KDL::Frame::DH(0, val_al5, 0, val_th5)
                * KDL::Frame::DH(0, 0, 0, val_th6);

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                if (fabs(hec(i, j) - f(i, j)) > eps)
                    return false;
            }
        }

        return true;
    }

    void setDefaultDHParamXtion() {
        double dh[] = ROBOT_DH_NOTATION_XTION;
        for (int i = 0; i < 6 * 4; i++)
            robot->denavit_hartenberg[i] = dh[i];
    }

    /** \brief Set DH parameters for Xtion
     *  \details alpha, a, theta, d
     */
    bool setDHParamXtion(std::string base_frame, std::string xtion_frame,
            std::string xml_string) {
        if (base_frame.empty() || xtion_frame.empty()) {
            return false;
        }

        KDL::Tree my_tree;
        if (!kdl_parser::treeFromString(xml_string, my_tree)) {
            ROS_ERROR("[Xtion IKT]: Failed to construct KDL tree.");
            return false;
        }
        KDL::Chain transform_chain;
        my_tree.getChain(base_frame, xtion_frame, transform_chain);
        if (transform_chain.getNrOfJoints() != 0) {
            ROS_ERROR(
                    "[XTION IKT]: Nr of joints is more then 0 in transformation to the tip link");
            return false;
        }
        KDL::Frame hec;
        KDL::ChainFkSolverPos_recursive transform_fk =
                KDL::ChainFkSolverPos_recursive(transform_chain);
        transform_fk.JntToCart(KDL::JntArray(), hec);

        /*Rotation to match DH and ROS frames*/
        KDL::Frame rot = KDL::Frame::Identity();
        KDL::Frame rot1 = KDL::Frame::Identity();
        //rot.M = KDL::Rotation::RotX(-M_PI_2) * KDL::Rotation::RotY(M_PI_2);
        rot1.M = KDL::Rotation::RotZ(-M_PI_2);
        hec = rot * hec * rot1;

        double dh_al5, dh_th5, dh_th6, dh_a5, dh_d5, dh_d6;
        try {
            std::vector<std::vector<double> > possible_values;
            std::vector<double> values;
            values.resize(3, -10);
            double val_al5, val_th5, val_th6;
            for (int sign_al5 = -1; sign_al5 <= 1; sign_al5 += 2) {
                val_al5 = acos(hec(2, 2)) * sign_al5;
                for (int sign_th5 = -1; sign_th5 <= 1; sign_th5 += 2) {
                    if (fabs(sin(val_al5)) < eps) {
                        val_th5 = 0;
                    } else {
                        val_th5 = asin(
                                round(hec(0, 2) / sin(val_al5) * 100000)
                                        / 100000) * sign_th5;
                    }
                    for (int sign_th6 = -1; sign_th6 <= 1; sign_th6 += 2) {
                        if (fabs(sin(val_al5)) < eps) {
                            val_th6 = acos(hec(1, 1));
                        } else {
                            val_th6 = asin(
                                    round(hec(2, 0) / sin(val_al5) * 100000)
                                            / 100000) * sign_th6;
                        }
                        if (equal_dh_rot(hec, val_al5, val_th5, val_th6)) {
                            values[0] = val_al5;
                            values[1] = val_th5;
                            values[2] = val_th6;
                            possible_values.push_back(values);

                        }
                    }
                }
            }
            if (possible_values.size() == 0) {
                ROS_ERROR_STREAM("[XTION IKT]: Not valid angles found for DH.");
                return false;
            }
            for (size_t values_i = 0; values_i < possible_values.size();
                    ++values_i) {

                val_al5 = possible_values[values_i].at(0);
                val_th5 = possible_values[values_i].at(1);
                val_th6 = possible_values[values_i].at(2);

                double a5, d5, d6;
                if (fabs(sin(val_al5)) < eps) {
                    if (fabs(cos(val_th5)) < eps) {
                        a5 = hec(1, 3) / sin(val_th5);
                    } else {
                        a5 = hec(0, 3) / cos(val_th5);
                    }
                    d5 = hec(2, 3);
                    d6 = 0;
                } else {
                    if (fabs(cos(val_th5)) < eps) {
                        a5 = hec(1, 3) / sin(val_th5);
                        d6 = hec(0, 3) / (sin(val_al5) * sin(val_th5));
                    } else if (fabs(sin(val_th5)) < eps) {
                        a5 = hec(0, 3) / cos(val_th5);
                        d6 = -hec(1, 3) / (sin(val_al5) * cos(val_th5));
                    } else {
                        d6 = (-hec(1, 3)
                                + hec(0, 3) * sin(val_th5) / cos(val_th5))
                                / (sin(val_th5) * sin(val_th5) * sin(val_al5)
                                        / cos(val_th5)
                                        + sin(val_al5) * cos(val_th5));
                        a5 = (hec(0, 3) - d6 * sin(val_al5) * sin(val_th5))
                                / cos(val_th5);
                    }
                    d5 = hec(2, 3) - d6 * cos(val_al5);
                }

                dh_a5 = a5;
                dh_al5 = val_al5;
                dh_th5 = val_th5;
                dh_th6 = val_th6;
                dh_d5 = d5;
                dh_d6 = d6;
                KDL::Frame f;
                f = KDL::Frame::DH(dh_a5, dh_al5, dh_d5, dh_th5)
                        * KDL::Frame::DH(0, 0, dh_d6, dh_th6);
                if (equal_frames(hec, f)) {
                    break;
                }
            }
        } catch (...) {
            ROS_ERROR_STREAM(
                    "[XTION IK]: Exception occurred in DH computation. Using default DH parameter");
            setDefaultDHParamXtion();
            return false;
        }

        //alpha a theta d
        setDefaultDHParamXtion();
        robot->denavit_hartenberg[4 + 0 * 6] = dh_al5;
        robot->denavit_hartenberg[5 + 0 * 6] = 0;
        robot->denavit_hartenberg[4 + 1 * 6] = dh_a5 * 1000;
        robot->denavit_hartenberg[5 + 1 * 6] = 0;
        robot->denavit_hartenberg[4 + 2 * 6] = dh_th5;
        robot->denavit_hartenberg[5 + 2 * 6] = dh_th6;
        robot->denavit_hartenberg[4 + 3 * 6] = dh_d5 * 1000;
        robot->denavit_hartenberg[5 + 3 * 6] = dh_d6 * 1000;

        KDL::Frame f;
        f = KDL::Frame::DH(dh_a5, dh_al5, dh_d5, dh_th5)
                * KDL::Frame::DH(0, 0, dh_d6, dh_th6);

        if (!equal_frames(hec, f)) {
            ROS_ERROR_STREAM(
                    "[XTION IKT] Not equal frames. Using default parameters.");
            setDefaultDHParam();

            ROS_WARN_STREAM("ALPHA5: " << dh_al5);
            ROS_WARN_STREAM("A5: " << dh_a5);
            ROS_WARN_STREAM("THETA5: " << dh_th5);
            ROS_WARN_STREAM("D5: " << dh_d5);
            ROS_WARN_STREAM("THETA6: " << dh_th6);
            ROS_WARN_STREAM("D6: " << dh_d6);

            ROS_WARN_STREAM("HEC");
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    std::cout << hec(i, j) << "\t";
                }
                std::cout << std::endl;
            }
            ROS_WARN_STREAM("DH");
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    std::cout << f(i, j) << "\t";
                }
                std::cout << std::endl;
            }
        }

        return true;
    }

    virtual ~IKSolver() {
    }

    virtual int solve(KDL::Frame &pose_frame,
            const std::vector<double> &ik_seed_state) {
        solutions_.clear();
        double *MhV;
        double J[6 * 16];
        MhV = (double*) malloc(16 * sizeof(double));
        for (int i = 0; i < 16; ++i) {
            MhV[i] = 0;
        }
        int no_sol = 0;

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                MhV[_2D_IDX(i + 1, j + 1, 4, 4)] = pose_frame.M(i, j);
            }
            MhV[_2D_IDX(i + 1, 4, 4, 4)] = pose_frame.p[i] * 1000;
        }
        MhV[_2D_IDX(4, 4, 4, 4)] = 1;

        try {
            general_6r_ikt(J, &no_sol, robot, MhV, &robot_limits, NULL);
        } catch (...) {
            ROS_WARN_STREAM("General6RIKT failed");
            no_sol = 0;
            ROS_INFO_STREAM("MhV:");
            for (int i = 1; i <= 4; ++i) {
                for (int j = 1; j <= 4; ++j) {
                    printf("%f ", MhV[_2D_IDX(i, j, 4, 4)]);
                }
                printf("\n");
            }
        }
        for (int i = 1; i <= no_sol; ++i) {
            std::vector<double> solution;
            for (int j = 1; j <= 6; j++) {
                solution.push_back(J[_2D_IDX(j, i, 6, 16)]);
            }
            solutions_.push_back(solution);
        }
        free(MhV);
        return solutions_.size();
    }

    virtual void getSolution(int i, std::vector<double> &solution) {
        solution = solutions_.at(i);
    }

    virtual void getClosestSolution(const std::vector<double> &ik_seed_state,
            std::vector<double> &solution) {
        double mindist = DBL_MAX;
        int minindex = -1;
        std::vector<double> sol;
        for (size_t i = 0; i < solutions_.size(); ++i) {
            getSolution(i, sol);
            double dist = harmonize(ik_seed_state, sol);
            if (minindex == -1 || dist < mindist) {
                minindex = i;
                mindist = dist;
            }
        }
        if (minindex >= 0) {
            getSolution(minindex, solution);
            harmonize(ik_seed_state, solution);
        }
    }

    void printSolutions() {
        std::stringstream ss;
        for (int i = 0; i < (int) solutions_.size(); ++i) {
            for (int j = 0; j < (int) solutions_.at(i).size(); ++j) {
                ss << solutions_.at(i).at(j) << " ";
            }
            ss << std::endl;
        }
        ROS_INFO_STREAM(ss.str());
    }

    double harmonize(const std::vector<double> &ik_seed_state,
            std::vector<double> &solution) {
        double dist_sqr = 0;
        std::vector<double> ss = ik_seed_state;
        for (size_t i = 0; i < ik_seed_state.size(); ++i) {
            while (ss[i] > 2 * M_PI) {
                ss[i] -= 2 * M_PI;
            }
            while (ss[i] < 2 * M_PI) {
                ss[i] += 2 * M_PI;
            }
            while (solution[i] > 2 * M_PI) {
                solution[i] -= 2 * M_PI;
            }
            while (solution[i] < 2 * M_PI) {
                solution[i] += 2 * M_PI;
            }
            dist_sqr += fabs(ik_seed_state[i] - solution[i]);
        }
        return dist_sqr;
    }

    /** Reduce number of joints from the solutions */
    void reduce_joints(int number_of_joints) {
        for (unsigned int i = 0; i < solutions_.size(); ++i) {
            solutions_[i].resize(number_of_joints, 0);
        }
    }
public:
    ros::NodeHandle node_;
    std::vector<std::vector<double> > solutions_;
    gen6r_robot_t *robot;
    gen6rikt_limits_t robot_limits;
    //static const double eps = 0.0001;
    const double eps = 0.0001;
};

}
