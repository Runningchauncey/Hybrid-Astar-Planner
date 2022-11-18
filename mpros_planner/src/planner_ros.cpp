#include "planner_ros.h"

void PlannerROS::setLoggerLevel(int idx)
{
    logger_level_ = ros::console::levels::Level(idx);
    if (ros::console::set_logger_level(
            ROSCONSOLE_DEFAULT_NAME, logger_level_))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
}

void PlannerROS::initialize()
{
    // mark initial empty occupancy grid
    og_.header.stamp.sec = 0;
    og_.header.stamp.nsec = 0;
    // pass the monitored member to subscriber to be upsated
    subscriber_.init(start_ps_, target_ps_, curr_ps_, curr_vel_, og_);
}

void PlannerROS::plan()
{
    if (og_time_sec_ != og_.header.stamp.toNSec()) // if the og is updated
    {
        og_time_sec_ = og_.header.stamp.toNSec();
        map_.setOccupancyMap(og_);
    }
    // check start pose
    if (start_ps_.empty())
    {
        ROS_INFO_ONCE("[Planner] Start pose not set!");
        return;
    }
    // check target pose
    if (target_ps_.empty())
    {
        ROS_INFO_ONCE("[Planner] Target pose not set");
        return;
    }
    // build the goal map
    map_.updateGoal(target_ps_[0], target_ps_[1]);
    // init planner
    planner_->initialize(start_ps_, target_ps_, map_, vis_);

    // make plan
    if (planner_->Plan())
    {
        std::vector<PathPoint> new_plan = planner_->getNewPath();
        vis_.publish(planner_->getPathMsg(), "new_path");
        planner_->reset();
        // TODO: evaluate path and compare by comparing cost of goal point
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh;
    PlannerROS planner(nh);
    planner.initialize();
    planner.setLoggerLevel(1);

    ros::Rate r(10);
    while (nh.ok())
    {
        ros::spinOnce();
        planner.plan();
    }

    return 0;
}