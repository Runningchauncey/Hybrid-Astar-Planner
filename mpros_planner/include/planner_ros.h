#ifndef PLANNER_ROS_H
#define PLANNER_ROS_H

#include <iostream>
#include <vector>
#include <string>
#include <mutex>
#include <memory>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include "mpros_algorithm/hybrid_a_star.h"
#include "mpros_speed_profile/speed_profiler.h"
#include "mpros_utils/visualizer.h"
#include "mpros_navigation_map/nav_map.h"
#include "mpros_subscriber.h"
#include "mpros_publisher.h"

class PlannerROS
{
public:
    PlannerROS(ros::NodeHandle &nh)
    {
        nh_ = nh;
        // configuration of members
        subscriber_.config(nh_);
        publisher_.config(nh_);
        // configuring navigation map
        map_.config(nh_);
        // init visualizer
        vis_ = Visualizer(nh_);
        // init planner and configure
        planner_ = std::make_shared<HybridAstar>();
        planner_->config(nh_);
    }

    /**
     * @brief initialize all components
     *
     */
    void initialize();

    void setLoggerLevel(int level_idx);

    void plan();

private:
    /**
     * @brief estimate pose for next planning step
     *
     */
    void estimatePose();

private:
    ros::console::levels::Level logger_level_ = ros::console::levels::Level(1);
    ros::NodeHandle nh_;
    ros::Subscriber start_ps_sub_;
    ros::Subscriber target_ps_sub_;
    ros::Subscriber curr_ps_sub_;
    ros::Subscriber curr_vel_sub_;
    MPSubscriber subscriber_;
    MPPublisher publisher_;

    ros::Publisher traj_pub_;

    // member monitored by subscriber
    std::vector<double> start_ps_;
    std::vector<double> target_ps_;
    std::vector<double> curr_ps_;
    double curr_vel_{0};
    nav_msgs::OccupancyGrid og_;

    // helper member of monitored member to check update
    uint64_t og_time_sec_{0};

    NavMap map_;
    Visualizer vis_;
    SpeedProfiler profiler_;
    std::shared_ptr<HybridAstar> planner_;

    int curr_seg_no = 0;
    bool subgoal_reached = false, stop = false;
    double pos_tolerance = 1;
    double yaw_tolerance = 0.3;
    double vel_tolerance = 0.05;

    // result of planner
    std::vector<PathPoint> plan_;
};

#endif // PLANNER_ROS_H