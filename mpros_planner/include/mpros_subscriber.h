#ifndef MPROS_SUBSCRIBER_H
#define MPROS_SUBSCRIBER_H

#include <vector>
#include <mutex>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <gazebo_msgs/ModelState.h>
#include <tf2/utils.h>

class MPSubscriber
{
public:
    MPSubscriber() = default;

    ~MPSubscriber() {}

    void config(ros::NodeHandle &nh)
    {
        nh_ = &nh;
        pose_sub_ = nh_->subscribe("vehicle/center_pose", qs_, &MPSubscriber::vehiclePoseCB, this);
        vel_sub_ = nh_->subscribe("vehicle/velocity", 20, &MPSubscriber::vehicleVelCB, this);
        start_ps_sub_ = nh_->subscribe<geometry_msgs::PoseWithCovarianceStamped>(
            "initialpose", qs_, &MPSubscriber::startPoseCB, this);
        target_ps_sub_ = nh_->subscribe<geometry_msgs::PoseStamped>(
            "move_base_simple/goal", qs_, &MPSubscriber::targetPoseCB, this);
        gazebo_model_pub_ = nh_->advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", qs_);
        map_sub_ = nh_->subscribe("map", qs_, &MPSubscriber::occupancyGridCB, this);
    }

    void init(std::vector<double> &start_ps,
              std::vector<double> &target_ps,
              std::vector<double> &curr_ps,
              double &curr_vel,
              nav_msgs::OccupancyGrid &og)
    {
        start_ps_ = &start_ps;
        target_ps_ = &target_ps;
        curr_ps_ = &curr_ps;
        curr_vel_ = &curr_vel;
        og_ = &og;
        inited_ = true;
    }

    void startPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
    {
        if (!inited_)
        {
            return;
        }
        *start_ps_ = std::vector<double>{msg->pose.pose.position.x,
                                         msg->pose.pose.position.y,
                                         tf2::getYaw(msg->pose.pose.orientation)};
        ROS_DEBUG_STREAM("start pose set to: "
                         << (*start_ps_)[0] << " "
                         << (*start_ps_)[1] << " "
                         << (*start_ps_)[2]);
        publishInitPose();
    }
    /**
     * @brief callback function for target pose
     *
     * @param msg
     */
    void targetPoseCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (!inited_)
        {
            return;
        }
        *target_ps_ = std::vector<double>{msg->pose.position.x,
                                          msg->pose.position.y,
                                          tf2::getYaw(msg->pose.orientation)};
        ROS_DEBUG_STREAM("target pose set to: "
                         << (*target_ps_)[0] << " "
                         << (*target_ps_)[1] << " "
                         << (*target_ps_)[2]);
    }

    void vehiclePoseCB(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
    {
        if (!inited_)
        {
            return;
        }
        double curr_x, curr_y, curr_yaw;
        curr_x = pose_msg->pose.position.x;
        curr_y = pose_msg->pose.position.y;
        curr_yaw = tf2::getYaw(pose_msg->pose.orientation);

        *curr_ps_ = std::vector<double>{curr_x, curr_y, curr_yaw};
        ROS_DEBUG_STREAM("current pose set to: "
                         << (*curr_ps_)[0] << " "
                         << (*curr_ps_)[1] << " "
                         << (*curr_ps_)[2]);
    }

    void vehicleVelCB(const geometry_msgs::Twist::ConstPtr &vel_msg)
    {
        if (!inited_)
        {
            return;
        }
        *curr_vel_ = std::sqrt(vel_msg->linear.x * vel_msg->linear.x + vel_msg->linear.y * vel_msg->linear.y);
        ROS_DEBUG_STREAM("current velocity set to: "
                         << (*curr_vel_));
    }

    void occupancyGridCB(const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
    {
        if (!inited_)
        {
            return;
        }
        *og_ = *map_msg;
    }

    void publishInitPose()
    {
        if (!start_ps_)
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

        gazebo_model_pub_.publish(msg);
    }

private:
    bool inited_ = false;

    ros::NodeHandle *nh_;

    int qs_{10};

    ros::Subscriber pose_sub_;
    ros::Subscriber vel_sub_;
    ros::Subscriber start_ps_sub_;
    ros::Subscriber target_ps_sub_;
    ros::Subscriber map_sub_;

    ros::Publisher gazebo_model_pub_;

    std::vector<double> *start_ps_ = nullptr;
    std::vector<double> *target_ps_ = nullptr;
    std::vector<double> *curr_ps_ = nullptr;
    double *curr_vel_ = nullptr;
    nav_msgs::OccupancyGrid *og_ = nullptr;
};

#endif // MPROS_SUBSCRIBER_H