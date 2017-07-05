#include <clopema_libs/ForceGrabber.h>
#include <eigen_conversions/eigen_msg.h>
#include <fstream>

namespace clopema_libs {
    ForceGrabber::ForceGrabber(bool wait_for_init) : node("~") {
        sub_filtered_1 = node.subscribe("/r1_force_data_filtered", 1, &ForceGrabber::cb_fitered_1, this);
        sub_filtered_2 = node.subscribe("/r2_force_data_filtered", 1, &ForceGrabber::cb_fitered_2, this);
        sub_downsampled_1 = node.subscribe("/r1_force_data_downsampled", 1, &ForceGrabber::cb_downsampled_1, this);
        sub_downsampled_2 = node.subscribe("/r2_force_data_downsampled", 1, &ForceGrabber::cb_downsampled_2, this);
        if(wait_for_init)
            while(!is_ready() && !is_ready_downsampled() && ros::ok()) {
                ros::spinOnce();
                ros::Duration(0.05).sleep();
            }
    }

    ForceGrabber::~ForceGrabber() {
        sub_downsampled_1.shutdown();
        sub_downsampled_2.shutdown();
        sub_filtered_1.shutdown();
        sub_filtered_2.shutdown();
        {
            boost::unique_lock<boost::mutex> l(mutex_wrench);   
        }
        {
            boost::unique_lock<boost::mutex> l(mutex_downsampled);
        }
        //now mutexes are relased and can be destroyed
    }

    void ForceGrabber::cb_fitered_1(const geometry_msgs::WrenchStampedConstPtr& msg) {
        Vector3d f, t, foff, toff;
        tf::vectorMsgToEigen(msg->wrench.force, f);
        tf::vectorMsgToEigen(msg->wrench.torque, t);
        boost::unique_lock<boost::mutex> l(mutex_wrench);
        if(!filtered_f1)
            filtered_f1.reset(new Vector3d);
        if(!filtered_t1)
            filtered_t1.reset(new Vector3d);

        if(!filtered_offset_f1) {
            filtered_offset_f1.reset(new Eigen::Vector3d(f));
        }
        foff = *filtered_offset_f1;

        if(!filtered_offset_t1) {
            filtered_offset_t1.reset(new Eigen::Vector3d(t));
        }
        toff = *filtered_offset_t1;

        *filtered_f1 = f - foff;
        *filtered_t1 = t - toff;
    }

    void ForceGrabber::cb_fitered_2(const geometry_msgs::WrenchStampedConstPtr& msg) {
        Vector3d f, t, foff, toff;
        tf::vectorMsgToEigen(msg->wrench.force, f);
        tf::vectorMsgToEigen(msg->wrench.torque, t);
        boost::unique_lock<boost::mutex> l(mutex_wrench);
        if(!filtered_f2)
            filtered_f2.reset(new Vector3d);
        if(!filtered_t2)
            filtered_t2.reset(new Vector3d);

        if(!filtered_offset_f2) {
            filtered_offset_f2.reset(new Eigen::Vector3d(f));
        }
        foff = *filtered_offset_f2;

        if(!filtered_offset_t2) {
            filtered_offset_t2.reset(new Eigen::Vector3d(t));
        }
        toff = *filtered_offset_t2;

        *filtered_f2 = f - foff;
        *filtered_t2 = t - toff;
    }

    Vector3d ForceGrabber::get_filtered_f1() {
        if(!is_ready())
            return Vector3d::Zero();
        boost::unique_lock<boost::mutex> l(mutex_wrench);
        return *filtered_f1;
    }

    Vector3d ForceGrabber::get_filtered_f2() {
        if(!is_ready())
            return Vector3d::Zero();
        boost::unique_lock<boost::mutex> l(mutex_wrench);
        return *filtered_f2;
    }

    Vector3d ForceGrabber::get_filtered_t1() {
        if(!is_ready())
            return Vector3d::Zero();
        boost::unique_lock<boost::mutex> l(mutex_wrench);
        return *filtered_t1;
    }

    Vector3d ForceGrabber::get_filtered_t2() {
        if(!is_ready())
            return Vector3d::Zero();
        boost::unique_lock<boost::mutex> l(mutex_wrench);
        return *filtered_t2;
    }

    bool ForceGrabber::is_ready() {
        boost::unique_lock<boost::mutex> l(mutex_wrench);
        if(!filtered_f1)
            return false;
        if(!filtered_f2)
            return false;
        if(!filtered_t1)
            return false;
        if(!filtered_t2)
            return false;

        return true;
    }

    void ForceGrabber::reset_filtered_offsets() {
        boost::unique_lock<boost::mutex> l(mutex_wrench);
        filtered_offset_f1.reset();
        filtered_offset_f2.reset();
        filtered_offset_t1.reset();
        filtered_offset_t2.reset();
    }

    bool ForceGrabber::save_filtered(const std::string& filename, bool warn) {
        if(warn) {
            if(!is_ready())
                ROS_WARN_STREAM("Forces were not received - saving zeros instead.");
        }

        Vector3d f1 = get_filtered_f1();
        Vector3d t1 = get_filtered_t1();
        Vector3d f2 = get_filtered_f2();
        Vector3d t2 = get_filtered_t2();
        using namespace std;
        ofstream of;
        of.open(filename.c_str());
        of << "f1[xyz],t1[xyz],f2[xyz],t2[xyz]" << endl;
        of << f1[0] << "," << f1[1] << "," << f1[2] << ",";
        of << t1[0] << "," << t1[1] << "," << t1[2] << ",";
        of << f2[0] << "," << f2[1] << "," << f2[2] << ",";
        of << t2[0] << "," << t2[1] << "," << t2[2];
        of.close();
    }

    void ForceGrabber::cb_downsampled_1(const geometry_msgs::WrenchStampedConstPtr& msg) {
        Vector3d f, t;
        tf::vectorMsgToEigen(msg->wrench.force, f);
        tf::vectorMsgToEigen(msg->wrench.torque, t);

        boost::unique_lock<boost::mutex> l(mutex_downsampled);
        if(!downsampled_f1)
            downsampled_f1.reset(new Vector3d);
        if(!downsampled_t1)
            downsampled_t1.reset(new Vector3d);
        *downsampled_f1 = f;
        *downsampled_t1 = t;
    }

    void ForceGrabber::cb_downsampled_2(const geometry_msgs::WrenchStampedConstPtr& msg) {
        Vector3d f, t;
        tf::vectorMsgToEigen(msg->wrench.force, f);
        tf::vectorMsgToEigen(msg->wrench.torque, t);

        boost::unique_lock<boost::mutex> l(mutex_downsampled);
        if(!downsampled_f2)
            downsampled_f2.reset(new Vector3d);
        if(!downsampled_t2)
            downsampled_t2.reset(new Vector3d);
        *downsampled_f2 = f;
        *downsampled_t2 = t;
    }

    bool ForceGrabber::save_downsampled(const std::string& filename, bool append, bool warn) {
        if(warn) {
            if(!is_ready_downsampled())
                ROS_WARN_STREAM("Forces were not received - saving zeros instead.");
        }

        Vector3d f1 = get_downsampled_f1();
        Vector3d t1 = get_downsampled_t1();
        Vector3d f2 = get_downsampled_f2();
        Vector3d t2 = get_downsampled_t2();
        using namespace std;
        ofstream of;
        if(append)
            of.open(filename.c_str(), std::ios_base::app);
        else
            of.open(filename.c_str());
        if(append)
            of << endl;
        else
            of << "f1[xyz],t1[xyz],f2[xyz],t2[xyz]" << endl;
        of << f1[0] << "," << f1[1] << "," << f1[2] << ",";
        of << t1[0] << "," << t1[1] << "," << t1[2] << ",";
        of << f2[0] << "," << f2[1] << "," << f2[2] << ",";
        of << t2[0] << "," << t2[1] << "," << t2[2];
        of.close();
    }

    Vector3d ForceGrabber::get_downsampled_f1() {
        if(!is_ready_downsampled())
            return Vector3d::Zero();
        boost::unique_lock<boost::mutex> l(mutex_downsampled);
        return *downsampled_f1;
    }

    Vector3d ForceGrabber::get_downsampled_f2() {
        if(!is_ready_downsampled())
            return Vector3d::Zero();
        boost::unique_lock<boost::mutex> l(mutex_downsampled);
        return *downsampled_f2;
    }

    Vector3d ForceGrabber::get_downsampled_t1() {
        if(!is_ready_downsampled())
            return Vector3d::Zero();
        boost::unique_lock<boost::mutex> l(mutex_downsampled);
        return *downsampled_t1;
    }

    Vector3d ForceGrabber::get_downsampled_t2() {
        if(!is_ready_downsampled())
            return Vector3d::Zero();
        boost::unique_lock<boost::mutex> l(mutex_downsampled);
        return *downsampled_t2;
    }

    bool ForceGrabber::is_ready_downsampled() {
        boost::unique_lock<boost::mutex> l(mutex_downsampled);
        if(!downsampled_f1)
            return false;
        if(!downsampled_f2)
            return false;
        if(!downsampled_t1)
            return false;
        if(!downsampled_t2)
            return false;
        return true;
    }

}
