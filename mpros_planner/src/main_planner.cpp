#include <vector>
#include <iostream>
#include <Eigen/Core>

#include <boost/bind.hpp>
#include "matplotlibcpp.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"
#include "gazebo_msgs/ModelState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "mpros_msgs/Bahn.h"
#include "mpros_msgs/Waypoint.h"
#include "mpros_navigation_map/nav_map.h"
#include "mpros_algorithm/hybrid_a_star.h"
#include "mpros_utils/visualizer.h"
#include "mpros_speed_profile/speed_profiler.h"
#include "mpros_utils/planner_utils.h"
#include "mpros_smoother/smoother_wrapper.h"


namespace plt = matplotlibcpp;

/**
 * @brief Plot curve with elaborative information
 *
 * @param path Input: path as a vector of PathPoint
 */
void PlotCurv(const std::vector<PathPoint> &path)
{
    std::cout << "path points num: " << path.size() << std::endl;
    std::vector<double> curv, vel, accel, dist, time, yaw;

    for (auto pathpoint_iter = path.begin(); pathpoint_iter != path.end(); ++pathpoint_iter)
    {
        time.push_back(pathpoint_iter->time_);
        curv.push_back(pathpoint_iter->curv_);
        vel.push_back(pathpoint_iter->vel_);
        accel.push_back(pathpoint_iter->accel_);
        dist.push_back(pathpoint_iter->dist_);
        yaw.push_back(pathpoint_iter->yaw_);
    }
    plt::figure();
    plt::subplot(3, 2, 1);
    plt::plot(dist, curv);
    plt::title("curvature - Dist");
    plt::subplot(3, 2, 2);
    plt::plot(time, dist);
    plt::title("Dist - Time");
    plt::subplot(3, 2, 3);
    plt::plot(dist, yaw);
    plt::title("Yaw - Dist");
    plt::subplot(3, 2, 5);
    plt::plot(dist, accel);
    plt::title("Acceleration - Dist");
    plt::subplot(3, 2, 4);
    plt::plot(time, vel);
    plt::title("Velocity - Time");
    plt::subplot(3, 2, 6);
    plt::plot(time, accel);
    plt::title("Accel - Time");
    plt::show();
}

/**
 * @brief Publish trajectory to on given publisher
 * 
 * @param path Input: path in vector
 * @param pub Input: publisher
 */
void PubTrajectory(std::vector<PathPoint> &path, ros::Publisher &pub)
{
    mpros_msgs::Bahn traj_msg;
    mpros_msgs::Waypoint waypoint;

    // set up Bahn msg
    for (auto pathpoint_iter = path.begin(); pathpoint_iter != path.end(); ++pathpoint_iter)
    {
        waypoint.pose.pose.position.x = pathpoint_iter->getX();
        waypoint.pose.pose.position.y = pathpoint_iter->getY();
        waypoint.curvature = pathpoint_iter->curv_;
        waypoint.twist.twist.linear.x = pathpoint_iter->vel_;

        double yaw = pathpoint_iter->getYaw();
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        q.normalize();
        waypoint.pose.pose.orientation = tf2::toMsg(q);

        traj_msg.waypoints.push_back(waypoint);
    }
    traj_msg.header.frame_id = "/map";
    pub.publish(traj_msg);
}

/**
 * @brief Relocate vehicle model to given pose by publish desired pose on "/gazebo/set_model_state"
 * 
 * @param pub 
 * @param pose 
 */
void relocateModel(const ros::Publisher &pub, const std::vector<double> &pose)
{
    gazebo_msgs::ModelState msg;
    msg.reference_frame = "map";
    msg.model_name = "vehiclemodel";
    msg.pose.position.x = pose[0];
    msg.pose.position.y = pose[1];
    tf2::Quaternion q;
    q.setRPY(0, 0, pose[2]);
    msg.pose.orientation = tf2::toMsg(q);

    pub.publish(msg);
}

/**
 * @brief Callback for "/initialpose"
 * 
 * @param msg Input: pose msg
 * @param pub Input: publisher for relocate the vehicle model
 * @param start_ps Output: initial pose in vector
 * @param flag Output: if the initial pose is set up
 */
void startPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,
                 const ros::Publisher &pub,
                 std::vector<double> *start_ps,
                 bool *flag)
{
    *start_ps = std::vector<double>{msg->pose.pose.position.x,
                                    msg->pose.pose.position.y,
                                    tf2::getYaw(msg->pose.pose.orientation)};
    *flag = true;
    ROS_INFO_STREAM("start pose set to: "
                    << start_ps->operator[](0) << " "
                    << start_ps->operator[](1) << " "
                    << start_ps->operator[](2));
    relocateModel(pub, *start_ps);
}

/**
 * @brief Callback for "move_base_simple/goal", get the target pose and set the goal cost map
 * 
 * @param msg Input: pose msg
 * @param target_ps Output: target pose in vector
 * @param flag Output: if the target pose is set up
 * @param map Input: map object in which goal cost map to be set
 */
void targetPoseCB(const geometry_msgs::PoseStamped::ConstPtr &msg, std::vector<double> *target_ps, bool *flag, NavMap *map)
{
    *target_ps = std::vector<double>{msg->pose.position.x,
                                     msg->pose.position.y,
                                     tf2::getYaw(msg->pose.orientation)};
    *flag = true;
    ROS_INFO_STREAM("target pose set to: "
                    << target_ps->operator[](0) << " "
                    << target_ps->operator[](1) << " "
                    << target_ps->operator[](2));
    map->updateGoal(target_ps->at(0), target_ps->at(1));
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");

    // set up ROS logger level
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::NodeHandle nh;

    // components for pipeline
    NavMap map(nh);
    Visualizer vis(nh);
    auto profiler_ptr = std::make_shared<SpeedProfiler>();
    auto smoother_ptr = std::make_unique<smoother::SmootherWrapper>();
    std::vector<PathPoint> path, traj;

    smoother_ptr->config(nh);
    profiler_ptr->config(nh);
    map.config(nh);

    // // initial and target configuration for interactive motion planning
    // std::vector<double> start_pt = {0,0,0}; // position 1
    // std::vector<double> goal_pt;
    // // flags for set up initial and target poses
    // bool start_ps_inited = false, goal_ps_inited = false;

    // // for fixed start and goal in arena parking lot 
    std::vector<double> start_pt = {0,0,0}; // position 1
    std::vector<double> goal_pt = {-25.0,-1.5,3.14};
    bool start_ps_inited = true, goal_ps_inited = true;

    // publishers and subscribers
    ros::Publisher traj_pub = nh.advertise<mpros_msgs::Bahn>("global/bahn", 1, true);
    ros::Publisher gazebo_model_pub = nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
    ros::Subscriber start_ps_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        "initialpose", 10, boost::bind(startPoseCB, _1, gazebo_model_pub, &start_pt, &start_ps_inited));
    ros::Subscriber target_ps_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "move_base_simple/goal", 10, boost::bind(targetPoseCB, _1, &goal_pt, &goal_ps_inited, &map));

    // loop rate
    ros::Rate rate(1.0);

    while (nh.ok())
    {
        ros::spinOnce();

        // publish all map layers
        vis.publish(map.getStaticMapMsg(), "/static_map");
        vis.publish(map.getVoroMapMsg(), "/voro_map");
        vis.publish(map.getStaticOrigMapMsg(), "/static_orig_map");
        vis.publish(map.getInflationOrigMapMsg(), "/inflation_orig_map");
        vis.publish(map.getGoalMapMsg(), "/goal_map");

        if (map.get_map_ && start_ps_inited && goal_ps_inited)
        {
            // update target position in map
            map.updateGoal(goal_pt[0], goal_pt[1]);
            // init planner
            auto planner = std::make_shared<HybridAstar>(start_pt, goal_pt, map, vis, nh);
            // make plan
            if (planner->Plan())
            {
                // get the plan
                path = planner->getNewPath();
                PlotCurv(path);
                vis.publishPath(planner->getNewPath());
                vis.publishPathTangent(planner->getNewPath());
                vis.publishFootprints(planner->getNewPath());

                // smoother
                std::vector<PathPoint> path_orig(path.begin(), path.end());
                smoother_ptr->smooth(&map, path, 10);
                vis.publishSmoothedPath(path);
                PlotCurv(path);
                if (planner->pathInCollision(path))
                {
                    path = path_orig;
                }

                // speed profiler
                profiler_ptr->makeProfile(path);
                traj = profiler_ptr->getTrajectory();
                vis.publishTrajectory(traj);
                PubTrajectory(traj, traj_pub);
                PlotCurv(traj);

                // reset container
                traj.clear();
                path.clear();

            }
            vis.PublishSearchedTree(planner->getSearchTree());
            // reset the flag
            goal_ps_inited = false;
        }
        rate.sleep();
    }

    return 0;
}
