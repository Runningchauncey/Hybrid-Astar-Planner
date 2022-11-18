#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <vector>

#include <Eigen/Core>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "mpros_navigation_map/nav_map.h"
#include "mpros_utils/pathpoint.h"

class Visualizer
{
public:
    /**
     * @brief empty constructor
     *
     */
    Visualizer() {}
    /**
     * @brief Destroy the Visualizer object
     *
     */
    ~Visualizer() {}
    /**
     * @brief pass a nodehandle to initialise visualizer
     *
     * @param nh
     */
    Visualizer(ros::NodeHandle &nh)
    {
        nh_ = &nh;
        waypoints_pub_ = nh.advertise<visualization_msgs::Marker>("waypoints", 1, true);
        map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("inflated_map", 1, true);
        path_pub_ = nh.advertise<nav_msgs::Path>("path", 1, true);
        footprints_pub_ = nh.advertise<visualization_msgs::MarkerArray>("path/footprint", 10, true);
        costmap_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("cost_map", 1, true);
        new_path_pub_ = nh.advertise<nav_msgs::Path>("path/smoothed", 1, true);
        trajectory_pub_ = nh.advertise<nav_msgs::Path>("path/trajectory", 1, true);
        start_pt_pub_ = nh.advertise<visualization_msgs::Marker>("start_pts", 1, true);
        goal_pt_pub_ = nh.advertise<visualization_msgs::Marker>("goal_pt", 1, true);
        search_tree_pub_ = nh.advertise<visualization_msgs::Marker>("searched_tree", 1, true);
        path_tangent_pub_ = nh.advertise<visualization_msgs::MarkerArray>("path_tangent", 1, true);

        pub_vec_.clear();
    }
    /**
     * @brief initialize all publisher
     *
     */
    void initialise();
    /**
     * @brief publish waypoints
     *
     * @param waypoints
     */
    void publishWaypoints(const std::vector<std::vector<double>> &waypoints)
    {
        ros::Time now = ros::Time::now();
        visualization_msgs::Marker points;
        points.header.frame_id = "map";
        points.header.stamp = now;
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.1;
        points.scale.y = 0.1;
        // waypoints in blue
        points.color.b = 1.0f;
        points.color.a = 1.0;

        for (int i = 0; i < waypoints.size(); ++i)
        {
            geometry_msgs::Point pt;
            pt.x = waypoints[i][0];
            pt.y = waypoints[i][1];
            points.points.push_back(pt);
        }
        waypoints_pub_.publish(points);
    }
    /**
     * @brief publish final path or smoothed path
     *
     * @param path_vec
     */
    void publishPath(const std::vector<std::vector<double>> &path_vec)
    {
        nav_msgs::Path path_msg;
        tf2::Quaternion q;

        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map";

        geometry_msgs::PoseStamped ps;
        for (int i = 0; i < path_vec.size(); ++i)
        {
            ps.header.frame_id = "map";
            ps.pose.position.x = path_vec[i][0];
            ps.pose.position.y = path_vec[i][1];
            q.setRPY(0, 0, path_vec[i][2]);
            ps.pose.orientation = tf2::toMsg(q);
            path_msg.poses.push_back(ps);
        }
        path_pub_.publish(path_msg);
    }
    /**
     * @brief
     *
     * @param path_vec
     */
    void publishPath(std::vector<PathPoint> path_vec)
    {
        nav_msgs::Path path_msg;
        tf2::Quaternion q;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map";

        geometry_msgs::PoseStamped ps;

        for (int i = 0; i < path_vec.size(); ++i)
        {
            ps.header.frame_id = "map";
            ps.pose.position.x = path_vec[i].getX();
            ps.pose.position.y = path_vec[i].getY();
            q.setRPY(0, 0, path_vec[i].getYaw());
            ps.pose.orientation = tf2::toMsg(q);
            path_msg.poses.push_back(ps);
        }
        path_pub_.publish(path_msg);
    }
    /**
     * @brief publish footprints along path
     *
     * @param path_vec
     */
    void publishFootprints(const std::vector<PathPoint> &path_vec)
    {
        int count = 0;
        double marker_height = 0.01;
        uint32_t shape = visualization_msgs::Marker::CUBE;
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.id = 0;
        marker.ns = "global_path";
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color.b = 0.0f;
        marker.color.g = 0.5f;
        marker.color.a = 0.2f;
        // TODO: avoid hardcoded vehicle size
        marker.scale.x = 4.2;
        marker.scale.y = 2.0;
        marker.scale.z = marker_height;

        for (auto path_point : path_vec)
        {
            if (count % 5 == 0) // print footprint for every 5 points
            {
                marker.pose.position.x = path_point.getX();
                marker.pose.position.y = path_point.getY();
                marker.pose.position.z = marker_height / 2;
                tf2::Quaternion q;
                q.setRPY(0, 0, path_point.getYaw());
                q.normalize();
                marker.pose.orientation = tf2::toMsg(q);
                marker_array.markers.push_back(marker);
                footprints_pub_.publish(marker_array);
                marker.id++;
            }
            count++;
        }
    }
    /**
     * @brief publish costmap
     *
     * @param map
     */
    void publishCostmap(const NavMap &map, const std::vector<std::vector<double>> &costmap)
    {
        nav_msgs::OccupancyGrid grid_map;
        grid_map.header.stamp = ros::Time::now();
        grid_map.header.frame_id = "map";
        grid_map.info.origin.position.x = map.map_origin_x_;
        grid_map.info.origin.position.y = map.map_origin_y_;
        grid_map.info.origin.position.z = 0.;
        grid_map.info.origin.orientation.w = 1.0;

        grid_map.info.height = map.map_height_;
        grid_map.info.width = map.map_width_;
        grid_map.info.resolution = map.map_resolution_;
        grid_map.info.map_load_time = ros::Time::now();

        std::vector<double> map_1D;
        for (auto row : costmap)
        {
            map_1D.insert(map_1D.end(), row.begin(), row.end());
        }
        double max_cost = *(std::max_element(map_1D.begin(), map_1D.end()));

        for (int i = 0; i < costmap.size(); ++i)
        {
            for (int j = 0; j < costmap[0].size(); ++j)
            {
                grid_map.data.push_back(costmap[i][j] / max_cost * 98);
            }
        }

        costmap_pub_.publish(grid_map);
    }

    void publishStartPoints(const std::vector<double> &x_vec, const std::vector<double> &y_vec)
    {
        ros::Time now = ros::Time::now();
        visualization_msgs::Marker points;
        points.header.frame_id = "map";
        points.header.stamp = now;
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.1;
        points.scale.y = 0.1;
        // start points in green
        points.color.g = 1.0f;
        points.color.a = 1.0;

        for (int i = 0; i < x_vec.size(); ++i)
        {
            geometry_msgs::Point pt;
            pt.x = x_vec[i];
            pt.y = y_vec[i];
            points.points.push_back(pt);
        }
        start_pt_pub_.publish(points);
    }

    void publishGoalPoint(const double &x, const double &y)
    {
        ros::Time now = ros::Time::now();
        visualization_msgs::Marker points;
        points.header.frame_id = "map";
        points.header.stamp = now;
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.1;
        points.scale.y = 0.1;
        // goal points in red
        points.color.r = 1.0f;
        points.color.a = 1.0;

        geometry_msgs::Point pt;
        pt.x = x;
        pt.y = y;
        points.points.push_back(pt);
        goal_pt_pub_.publish(points);
    }

    void publishSmoothedPath(std::vector<PathPoint> path_vec)
    {
        nav_msgs::Path path_msg;
        tf2::Quaternion q;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map";

        geometry_msgs::PoseStamped ps;

        for (int i = 0; i < path_vec.size(); ++i)
        {
            ps.header.frame_id = "map";
            ps.pose.position.x = path_vec[i].getX();
            ps.pose.position.y = path_vec[i].getY();
            q.setRPY(0, 0, path_vec[i].getYaw());
            ps.pose.orientation = tf2::toMsg(q);
            path_msg.poses.push_back(ps);
        }
        new_path_pub_.publish(path_msg);
    }

    void publishTrajectory(std::vector<PathPoint> path_vec)
    {
        nav_msgs::Path path_msg;
        tf2::Quaternion q;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map";

        geometry_msgs::PoseStamped ps;

        for (int i = 0; i < path_vec.size(); ++i)
        {
            ps.header.frame_id = "map";
            ps.pose.position.x = path_vec[i].getX();
            ps.pose.position.y = path_vec[i].getY();
            q.setRPY(0, 0, path_vec[i].getYaw());
            ps.pose.orientation = tf2::toMsg(q);
            path_msg.poses.push_back(ps);
        }
        trajectory_pub_.publish(path_msg);
    }

    template <typename T>
    void publish(const T &rcv_msg, std::string topic_name)
    {
        T msg = rcv_msg;
        // update timestamp
        msg.header.stamp = ros::Time::now();
        // if the topic owned by an existing publisher, publish there
        if (pub_vec_.size())
        {
            for (auto iter = pub_vec_.begin(); iter != pub_vec_.end(); ++iter)
            {
                if (iter->getTopic().compare(topic_name.c_str()) == 0)
                {
                    iter->publish(msg);
                    return;
                }
            }
        }
        // else create new publisher and push to vector
        ros::Publisher pub = nh_->advertise<T>(topic_name, 10, true);
        pub_vec_.push_back(pub);
        pub.publish(msg);
    }

    void PublishSearchedTree(const std::vector<Eigen::Vector4d> &searched_tree)
    {
        visualization_msgs::Marker tree_list;
        tree_list.header.frame_id = "map";
        tree_list.header.stamp = ros::Time::now();
        tree_list.type = visualization_msgs::Marker::LINE_LIST;
        tree_list.action = visualization_msgs::Marker::ADD;
        tree_list.ns = "searched_tree";
        tree_list.scale.x = 0.02;

        tree_list.color.a = 0.7;
        tree_list.color.r = 0;
        tree_list.color.g = 1;
        tree_list.color.b = 0;

        tree_list.pose.orientation.w = 1.0;
        tree_list.pose.orientation.x = 0.0;
        tree_list.pose.orientation.y = 0.0;
        tree_list.pose.orientation.z = 0.0;

        geometry_msgs::Point point;
        for (const auto &i : searched_tree)
        {
            point.x = i(0);
            point.y = i(1);
            point.z = 0.0;
            tree_list.points.emplace_back(point);

            point.x = i(2);
            point.y = i(3);
            point.z = 0.0;
            tree_list.points.emplace_back(point);
        }

        search_tree_pub_.publish(tree_list);
    }

    void publishPathTangent(const std::vector<PathPoint>& path)
    {
        ros::Time now = ros::Time::now();
        visualization_msgs::MarkerArray arrows;
        visualization_msgs::Marker arr;
        arr.header.frame_id = "map";
        arr.header.stamp = now;
        arr.action = visualization_msgs::Marker::ADD;
        arr.pose.orientation.w = 1.0;
        arr.id = 0;
        arr.type = visualization_msgs::Marker::ARROW;
        arr.scale.x = 0.5;
        arr.scale.y = 0.1;
        // arrows in red
        arr.color.r = 1.0f;
        arr.color.a = 0.5;

        for (int i = 0; i < path.size(); ++i)
        {
            arr.pose.position.x = path[i].getX();
            arr.pose.position.y = path[i].getY();
            tf2::Quaternion q;
            q.setEuler(0, 0, path[i].getYaw());
            arr.pose.orientation = tf2::toMsg(q);
            arrows.markers.push_back(arr);
            arr.id++;
        }
        path_tangent_pub_.publish(arrows);
    }

private:
    ros::NodeHandle *nh_;
    // publisher for path during planning, inflated map, final plan, footprint along path, costmap, smoothed path
    ros::Publisher waypoints_pub_,
        map_pub_,
        path_pub_,
        footprints_pub_,
        costmap_pub_,
        new_path_pub_,
        trajectory_pub_,
        start_pt_pub_,
        goal_pt_pub_,
        search_tree_pub_,
        path_tangent_pub_;

    std::vector<ros::Publisher> pub_vec_;
};
#endif // !VISUALIZER_H