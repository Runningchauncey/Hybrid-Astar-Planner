#ifndef MPROS_PUBLISHER_H
#define MPROS_PUBLISHER_H

#include <ros/ros.h>
#include "gazebo_msgs/ModelState.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class MPPublisher
{
public:
    MPPublisher() = default;

    ~MPPublisher() = default;

    void config(ros::NodeHandle &nh)
    {
        nh_ = &nh;
        gazebo_model_pub = nh_->advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", qs_);
    }

    void init(std::vector<double> &start_ps, std::vector<double> &target_ps)
    {
        start_ps_ = &start_ps;
        target_ps_ = &target_ps;
    }

    void publishInitPose()
    {
        if(!start_ps_)
        {
            return;
        }
        gazebo_msgs::ModelState msg;
        msg.reference_frame = "map";
        msg.model_name = "vehiclemodel";
        msg.pose.position.x = start_ps_->at(0);
        msg.pose.position.y = start_ps_->at(1);
        tf2::Quaternion q;
        q.setRPY(0, 0, start_ps_->at(2));
        msg.pose.orientation = tf2::toMsg(q);

        gazebo_model_pub.publish(msg);
    }

private:
    ros::NodeHandle *nh_;

    int qs_{10};

    ros::Publisher traj_pub;
    ros::Publisher gazebo_model_pub;

public:
    std::vector<double> *start_ps_;
    std::vector<double> *target_ps_;
    std::vector<double> curr_ps_;
    double curr_vel_;
};

#endif // MPROS_PUBLISHER_H