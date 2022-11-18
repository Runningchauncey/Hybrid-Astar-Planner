#include <iostream>
#include <math.h>
#include <algorithm>
#include <list>
#include <time.h>
#include <random>
#include <chrono>

#include "mpros_algorithm/hybrid_a_star.h"

HybridAstar::HybridAstar(const std::vector<double> &start_pt,
                         const std::vector<double> &goal_pt,
                         NavMap &map,
                         Visualizer &vis,
                         ros::NodeHandle &nh) : map_(&map)
{
    // first config, then initialize the planner in compact constructor
    config(nh);
    initialize(start_pt, goal_pt, map, vis);
}

HybridAstar::~HybridAstar()
{
    // clean up all containers with pointers
    node_set_.clear();
    while (!open_map_.empty())
    {
        open_map_.erase(open_map_.begin());
    }
}

void HybridAstar::reset()
{
    // clear all containers
    node_set_.clear();
    while (!open_map_.empty())
    {
        open_map_.erase(open_map_.begin());
    }
    cost_set_.clear();
    path_.clear();
    goal_path_.clear();
    new_path_.clear();

    // reset all timer
    time_rs_shot_ = std::chrono::duration<double>::zero();
    time_colli_ = std::chrono::duration<double>::zero();
    time_spawn_ = std::chrono::duration<double>::zero();
    time_plan_ = std::chrono::duration<double>::zero();
}

void HybridAstar::initialize(const std::vector<double> &start_pt,
                             const std::vector<double> &goal_pt,
                             NavMap &map,
                             Visualizer &vis)
{
    // break if not initialized
    if (!configured_)
    {
        ROS_ERROR("[H_Astar] Planner not properly configured, stop initializing!");
        return;
    }

    // pass the pointers
    // navigation map pointer
    map_ = &map;
    // visualizer
    vis_ = &vis;

    // check collision of start and goal poses if footprint non zero
    if (footPrintInCollision4(start_pt[0], start_pt[1], start_pt[2]))
    {
        ROS_ERROR("[H_Astar] Start pose in collision");
        return;
    }
    if (footPrintInCollision4(goal_pt[0], goal_pt[1], goal_pt[2]))
    {
        ROS_ERROR("[H_Astar] Goal pose in collision");
        return;
    }

    // spawn start node and goal node
    start_node_ = std::make_shared<HybridAstarNode>(start_pt[0], start_pt[1], start_pt[2], 0, 1, nullptr, map_);
    goal_node_ = std::make_shared<HybridAstarNode>(goal_pt[0], goal_pt[1], goal_pt[2], 0, 1, nullptr, map_);
    ROS_INFO("[H_Astar] Start point (%f, %f, %f)", start_node_->x_, start_node_->y_, start_node_->yaw_);
    ROS_INFO("[H_Astar] Goal point (%f, %f, %f)", goal_node_->x_, goal_node_->y_, goal_node_->yaw_);

    // set heuristic to start node
    if (map_->get_goal_)
    {
        setNodeCost(start_node_);
        setNodeHeuristic(start_node_);
        goal_node_->setCost(std::numeric_limits<double>::infinity());
        goal_node_->setHeuristic(0.0);
    }
    else
    {
        ROS_WARN("[H_Astar] Goal map unavailable, planner aborted");
        return;
    }

    // initialise cost set and node set
    cost_set_.resize(map_->map_height_ * map_->map_width_ * num_yaw_ * num_direc, std::numeric_limits<double>::infinity());
    node_set_.resize(map_->map_height_ * map_->map_width_ * num_yaw_ * num_direc, nullptr);

    // insert start node and goal node to both sets
    int start_idx = calIndex(start_node_->idx_i_, start_node_->idx_j_, yawToIdx(start_node_->yaw_), 1);
    int goal_idx = calIndex(goal_node_->idx_i_, goal_node_->idx_j_, yawToIdx(goal_node_->yaw_), 1);
    node_set_[start_idx] = start_node_;
    cost_set_[start_idx] = start_node_->gvalue_;
    node_set_[goal_idx] = goal_node_;
    cost_set_[goal_idx] = 10000; // set the cost of goal node to a high value

    // increment number of nodes
    num_nodes_ += 2;
    // set flag
    initialized_ = true;
}

void HybridAstar::config(ros::NodeHandle &nh)
{
    if (!nh.ok())
    {
        throw std::runtime_error("[Smoother]: Node died while configuring");
    }

    // parameters of the vehicle
    std::string prefix_vehicle = "/mpros/vehicle/";
    // get parameters from ros param server
    nh.param<double>(prefix_vehicle + "max_curvature", max_curvature_, 0.4);
    nh.param<double>(prefix_vehicle + "width", V_WIDTH_, 2.0);
    nh.param<double>(prefix_vehicle + "wheelbase", V_WB_, 3.0);
    nh.param<double>(prefix_vehicle + "length", V_LENGTH_, 4.2);
    nh.param<double>(prefix_vehicle + "center_to_rotate_center_longitudinal", V_STEER_OFFSET_, 0.0);
    nh.param<double>(prefix_vehicle + "max_steering_angle", V_MAX_STEER_, 0.35);

    // set up parameters dependent on above params
    min_r_ = 1.0 / max_curvature_; // min turning radius
    ROS_INFO("[H_Astar] Min radius set to %f", min_r_);

    // initialise rs generator
    rs_gen_.SetRadius(min_r_);
    rs_gen_.setSteeringAngle(V_MAX_STEER_);
    // initialise ompl rs statespace
    rs_ss_.setRadius(min_r_);

    // parameters of the planner
    std::string prefix_planner = "/mpros/planner/";
    // get parameters from ros param server
    nh.param<double>(prefix_planner + "gear_penalty", PEN_GEAR_COST_, 10.0);
    nh.param<double>(prefix_planner + "reverse_penalty", PEN_BACKWARD_COST_, 0);
    nh.param<double>(prefix_planner + "steering_penalty", PEN_STEER_ANGLE_COST_, 0.1);
    nh.param<double>(prefix_planner + "steering_change_penalty", PEN_STEER_CHANGE_, 10.0);
    nh.param<double>(prefix_planner + "analytic_solution_range", RS_shot_radius_, 10.0);
    nh.param<int>(prefix_planner + "yaw_discretization", num_yaw_, 36);
    nh.param<double>(prefix_planner + "step_size", step_size_, 1.1);
    nh.param<double>(prefix_planner + "path_point_distance", path_granularity_, 0.2);
    nh.param<int>(prefix_planner + "steering_discretization", num_steer_, 3);
    nh.param<int>(prefix_planner + "max_iteration", iteration_limit_, 10000);
    nh.param<bool>(prefix_planner + "optimal_path", optimal_, false);
    nh.param<bool>(prefix_planner + "interpolation_enable", inter_enable_, true);

    // set up parameters dependent on above params
    upsample_factor_ = std::ceil(step_size_ / path_granularity_);
    yaw_resol_ = 2 * M_PI / num_yaw_;
    // !not considering one direction yet
    num_direc = 2;
    // set up rs path generator
    rs_gen_.setStepSize(step_size_);
    rs_gen_.setPenalty(PEN_GEAR_COST_, PEN_BACKWARD_COST_, PEN_STEER_ANGLE_COST_, PEN_STEER_CHANGE_);
    // call to setup footprints
    setFootprint();

    // initialise steering list and direction list
    // number of steering on positive side
    int num_pos_steer = num_steer_ / 2;
    for (int i = 1; i <= num_pos_steer; ++i)
    {
        // symmetrical steering set
        steer_set_.emplace_back(V_MAX_STEER_ / num_pos_steer * i);
        steer_set_.emplace_back(-V_MAX_STEER_ / num_pos_steer * i);
    }
    steer_set_.emplace_back(0.0);
    direction_set_ = {1.0, -1.0};

    // set flag
    configured_ = true;
}

void HybridAstar::setFootprint()
{
    // corner points of foorprint clockwise
    footprint_m_.col(0) << V_LENGTH_ / 2 + V_STEER_OFFSET_, V_WIDTH_ / 2;
    footprint_m_.col(1) << V_LENGTH_ / 2 + V_STEER_OFFSET_, -V_WIDTH_ / 2;
    footprint_m_.col(2) << -V_LENGTH_ / 2 + V_STEER_OFFSET_, -V_WIDTH_ / 2;
    footprint_m_.col(3) << -V_LENGTH_ / 2 + V_STEER_OFFSET_, V_WIDTH_ / 2;
    // footprint mapped to longitudinal axis
    footprint_longi_.col(0) << V_LENGTH_ / 2 + V_STEER_OFFSET_ - 1.0 / 2 * V_WIDTH_, 0;
    footprint_longi_.col(1) << -V_LENGTH_ / 2 + V_STEER_OFFSET_ + 1.0 / 2 * V_WIDTH_, 0;

    // circumcircle radius
    footprint_r_ = std::fmax(footprint_m_.col(0).norm(), footprint_m_.col(2).norm());
    // footprint kernel
    footprint_kernel_.configure(V_LENGTH_, V_WIDTH_, V_STEER_OFFSET_, 0, map_->map_resolution_orig_, 2.0 * M_PI / num_yaw_);
    footprint_kernel_.setKernel();
}

void HybridAstar::setNodeCost(NodePtr node)
{
    // if the node is start node
    if (node->parent_ == nullptr)
    {
        node->setCost(0.0);
        return;
    }

    // get the index of node on map
    int node_i = node->idx_i_, node_j = node->idx_j_;

    // cost 1: total cost from parent node
    double pre_cost = node->parent_->gvalue_;
    // cost 2: voronoi cost
    double voro_cost = map_->getVoronoiCost(node_i, node_j);
    // cost 3: path cost
    double path_cost = step_size_;

    // cost 4: penalize on reverse driving and change direction
    double direc_cost = 0;
    if (node->direction_ < 0)
        // driving backwards
        direc_cost += PEN_BACKWARD_COST_ * step_size_;
    if (node->direction_ != node->parent_->direction_)
        // if the node has different direction than parent node -> from forward to backward or otherwise
        direc_cost += PEN_GEAR_COST_;

    // cost 5: penalizing steering and keep changing steering angle
    double steer_cost = 0;
    if (node->steer_ != 0)
        // this node generated by steering
        steer_cost += PEN_STEER_ANGLE_COST_ * std::abs(node->steer_);
    if (node->steer_ != node->parent_->steer_)
        // steering angle different from parent node
        steer_cost += PEN_STEER_CHANGE_ * std::abs(node->steer_ - node->parent_->steer_);
    // set the cost of this node
    node->setCost(pre_cost + voro_cost + path_cost + direc_cost + steer_cost);
}

void HybridAstar::setNodeHeuristic(NodePtr node)
{
    // get the index of node on map
    int node_i = node->idx_i_, node_j = node->idx_j_;

    // Heuristic 1: Manhattan distance to goal node considering obstacle
    double h1 = map_->getGoalCost(node_i, node_j);

    // Heuristic 2: Non-holonomic path length to goal node without obstacle
    // ompl rs state space
    rs_ss_.setStart(node->x_, node->y_, node->yaw_);
    rs_ss_.setEnd(goal_node_->x_, goal_node_->y_, goal_node_->yaw_);
    double rs_cost = rs_ss_.getCost();
    double h2 = rs_cost;
    // set the heuristic as max of both heuristics, ensure admissbility
    node->setHeuristic(std::fmax(h1, h2) * (1 + 1e-3));
}

void HybridAstar::regularYaw(double &yaw)
{
    // not considering yaw > 3pi or yaw < -3pi yet
    if (yaw <= -M_PI)
    {
        yaw += 2 * M_PI;
        return;
    }
    if (yaw > M_PI)
    {
        yaw -= 2 * M_PI;
        return;
    }
}

int HybridAstar::calIndex(const int &idx_i, const int &idx_j, const int &idx_yaw, const double &direc)
{
    // index of direction
    int idx_direc = direc > 0 ? 0 : 1;

    return idx_i * map_->map_width_ * num_yaw_ * num_direc +
           idx_j * num_yaw_ * num_direc +
           idx_yaw * num_direc +
           idx_direc;
}

int HybridAstar::yawToIdx(const double &yaw)
{
    // index of yaw start from 0 to 2*M_PI
    double yaw_pos = yaw;
    // [-pi, 0] mapped to [pi, 2pi]
    if (yaw < 0)
    {
        yaw_pos = yaw + 2 * M_PI;
    }
    return int(yaw_pos / yaw_resol_);
}

void HybridAstar::expandNode(NodePtr node)
{
    ROS_DEBUG("[H_Astar] Expanding node");

    // set as constant value to avoid change
    const double step = step_size_;
    const double curr_x = node->x_;
    const double curr_y = node->y_;
    const double curr_yaw = node->yaw_;

    // accessible points from current point
    std::vector<std::vector<double>> next_points;
    // helper values
    double radius, steer_sign, dx, dy, dyaw, next_x, next_y, next_yaw;

    // apply all possible manuver and check if final point in collision
    for (auto steer : steer_set_)
    {
        for (auto direc : direction_set_)
        {
            // going straight
            if (steer == 0)
            {
                // position change in vehicle frame
                dx = direc * step_size_;
                dy = 0;
                dyaw = 0;
                // transform into map frame
                next_x = dx * cos(curr_yaw) - dy * sin(curr_yaw) + curr_x;
                next_y = dx * sin(curr_yaw) + dy * cos(curr_yaw) + curr_y;
                next_yaw = curr_yaw;
            }
            // take a curve
            else
            {
                radius = (V_WB_ / 2 + V_STEER_OFFSET_) / std::tan(std::abs(steer));
                steer_sign = steer / std::abs(steer);
                // position change in vehicle frame
                dyaw = step_size_ / radius;
                dx = sin(dyaw) * radius * direc;
                dy = (1 - cos(dyaw)) * radius * steer_sign;
                // transform into map frame
                next_x = dx * cos(curr_yaw) - dy * sin(curr_yaw) + curr_x;
                next_y = dx * sin(curr_yaw) + dy * cos(curr_yaw) + curr_y;
                next_yaw = curr_yaw + dyaw * steer_sign * direc;
                regularYaw(next_yaw);
            }
            // store visited point in tree as a leaf
            tree_.emplace_back(curr_x, curr_y, curr_yaw, next_x, next_y, next_yaw, direc, steer);
            // collision checking
            if (!footPrintInCollision4(next_x, next_y, next_yaw))
            {
                next_points.push_back({next_x,
                                       next_y,
                                       next_yaw,
                                       steer,
                                       direc});
            }
        }
    }

    // timer
    auto start_spawn = std::chrono::system_clock::now();

    // for all new and feasible new points, generate path and new node
    for (auto &point : next_points)
    {
        // new node pointer
        NodePtr newnode = std::make_shared<HybridAstarNode>(point[0], point[1], point[2], point[3], point[4], node, map_);
        // set up gvalue and fvalue
        setNodeCost(newnode);
        setNodeHeuristic(newnode);
        // accumulate node number
        num_nodes_++;

        // old cost of this grid and old node in open set
        int index = calIndex(newnode->idx_i_, newnode->idx_j_, yawToIdx(newnode->yaw_), newnode->direction_);
        double old_cost = cost_set_[index];
        NodePtr oldnode = node_set_[index];

        if (old_cost > newnode->gvalue_)
        {
            // the grid is taken by an open node with higher cost
            // the new node is to be pruned
            // replace the old node by new node
            if (oldnode != nullptr)
            {
                oldnode->setClose();
            }
            cost_set_[index] = newnode->gvalue_;
            node_set_[index] = newnode;

            open_map_.insert(std::pair<double, NodePtr>(newnode->fvalue_, newnode));
        }
    }
    // accumulate time count
    time_spawn_ += std::chrono::system_clock::now() - start_spawn;
    ROS_DEBUG("[H_Astar] Expanded a node");
}

bool HybridAstar::pathInCollision(const std::vector<std::vector<double>> &path)
{
    // check each point in path
    for (auto &point : path)
    {
        if (footPrintInCollision4(point[0], point[1], point[2]))
        {
            return true;
        }
    }

    return false;
}

bool HybridAstar::pathInCollision(const std::vector<PathPoint> &path)
{
    // check each point in path
    for (auto &point : path)
    {
        if (footPrintInCollision4(point.getX(), point.getY(), point.getYaw()))
        {
            return true;
        }
    }

    return false;
}

bool HybridAstar::footPrintInCollision(const double &path_x, const double &path_y, const double &yaw)
{
    // timer
    auto start_colli = std::chrono::system_clock::now();

    // copy the footprint
    std::vector<std::vector<double>> curr_footprint;
    curr_footprint.reserve(5);
    for (int i = 0; i < footprint_m_.cols(); ++i)
    {
        curr_footprint.emplace_back(footprint_m_.col(i).begin(), footprint_m_.col(i).end());
    }

    // values for transform
    double c = cos(yaw), s = sin(yaw);

    // calculate footprint on this pose
    for (auto &fp_pt : curr_footprint)
    {
        double x = fp_pt[0];
        double y = fp_pt[1];
        fp_pt[0] = x * c - y * s + path_x;
        fp_pt[1] = x * s + y * c + path_y;
    }

    // push back the first pt of footprint to make a closed form
    curr_footprint.push_back({curr_footprint[0][0], curr_footprint[0][1]});
    for (int i = 0; i < curr_footprint.size() - 1; ++i)
    {
        int idx_i1, idx_j1, idx_i2, idx_j2;
        map_->posToIndexOrig(curr_footprint[i][0], curr_footprint[i][1], idx_i1, idx_j1);
        map_->posToIndexOrig(curr_footprint[i + 1][0], curr_footprint[i + 1][1], idx_i2, idx_j2);

        // check straight line between each pair of corner points
        LineIterator line_iter(idx_i1, idx_j1, idx_i2, idx_j2);
        for (; !line_iter.isEnd(); line_iter.advance())
        {
            // current grid
            int curr_i = line_iter.getCurrX();
            int curr_j = line_iter.getCurrY();
            // collision checking using original occupancy grid
            if (map_->pointInCollisionOrig(curr_i, curr_j))
            {
                time_colli_ += std::chrono::system_clock::now() - start_colli;
                return true;
            }
        }
    }

    time_colli_ += std::chrono::system_clock::now() - start_colli;
    return false;
}

bool HybridAstar::footPrintInCollision2(const double &x, const double &y, const double &yaw)
{
    // timer
    auto start_colli = std::chrono::system_clock::now();

    // grid coordinates
    int pos_i, pos_j;
    map_->posToIndexOrig(x, y, pos_i, pos_j);

    // set up helper values
    std::vector<int> nearby_i, nearby_j;
    int r_in_pixel = std::ceil(footprint_r_ / map_->map_resolution_orig_);
    bool obstacle_nearby = false;

    // calculate the pixel position of nearby square
    int max_i = (int)std::fmin(pos_i + r_in_pixel, map_->map_height_orig_);
    int max_j = (int)std::fmin(pos_j + r_in_pixel, map_->map_width_orig_);
    int min_i = (int)std::fmax(pos_i - r_in_pixel, 0);
    int min_j = (int)std::fmax(pos_j - r_in_pixel, 0);

    // calculate sine and cosine for this yaw
    double s = std::sin(yaw),
           c = std::cos(yaw);

    // get nearby obstacles
    for (int i = min_i; i < max_i + 1; ++i)
    {
        for (int j = min_j; j < max_j + 1; ++j)
        {
            if (map_->pointInCollisionOrig(i, j))
            {
                obstacle_nearby = true;
                nearby_i.push_back(i);
                nearby_j.push_back(j);
            }
        }
    }

    // no obstacle inside the square -> return false
    if (!obstacle_nearby)
    {
        time_colli_ += std::chrono::system_clock::now() - start_colli;
        return false;
    }
    // obstacles inside the square -> examine if the obstacle inside footprint
    else
    {
        // sum of angles between obstacle point and all corner points
        double angle_sum = 0;
        for (int i = 0; i < nearby_i.size(); ++i)
        {
            // get a nearby obstacle position
            double ob_x, ob_y;
            map_->indexToPosOrig(nearby_i[i], nearby_j[i], ob_x, ob_y);

            // rotate obstacle into static vehicle coordinates
            double ob_x_static = c * (ob_x - x) + s * (ob_y - y);
            double ob_y_static = -s * (ob_x - x) + c * (ob_y - y);

            // loop all static footprint corner points
            for (int j = 0; j < footprint_x_.size() - 1; ++j)
            {
                // vector from obstacle to first corner point
                double dx1 = footprint_x_[j] - ob_x_static;
                double dy1 = footprint_y_[j] - ob_y_static;
                // vector from obstacle to first corner point
                double dx2 = footprint_x_[j + 1] - ob_x_static;
                double dy2 = footprint_y_[j + 1] - ob_y_static;
                // angle of first vector
                double theta1 = std::atan2(dy1, dx1);
                // cosine value of angle between 2 vectors
                double cos_angle = (dx1 * dx2 + dy1 * dy2) / (std::hypot(dx1, dy1) * std::hypot(dx2, dy2));
                // ensure cosine value less equal 1
                if (cos_angle >= 1.0)
                {
                    cos_angle = 1.0;
                }
                // if vector2 counterclockwise to vector1
                if (-std::sin(theta1) * dx2 + std::cos(theta1) * dy2 > 0)
                {
                    angle_sum += std::acos(cos_angle);
                }
                else
                {
                    angle_sum -= std::acos(cos_angle);
                }
            }
        }

        time_colli_ += std::chrono::system_clock::now() - start_colli;
        // if sum of all angles greater equal to pi -> the obstacle point inside footprint
        return angle_sum >= M_PI ? true : false;
    }
}

bool HybridAstar::footPrintInCollision3(const double &x, const double &y, const double &yaw)
{
    // timer
    auto start_colli = std::chrono::system_clock::now();

    // get kernel for current heading angle
    std::vector<std::vector<int8_t>> kernel = footprint_kernel_.getKernel(yaw);
    int kernel_size = footprint_kernel_.getKernelSize();
    int idx_origin = footprint_kernel_.getOrigin();

    // grid index
    int idx_i, idx_j;
    map_->posToIndexOrig(x, y, idx_i, idx_j);

    // check all kernels and corresponding map grid
    for (int i = 0; i < kernel_size; ++i)
    {
        int map_i = idx_i - idx_origin + i;
        if (map_i < 0 || map_i >= map_->map_height_orig_)
            return true;
        for (int j = 0; j < kernel_size; ++j)
        {
            int map_j = idx_j - idx_origin + j;
            if (map_j < 0 || map_j >= map_->map_width_orig_)
                return true;
            if (map_->pointInCollisionOrig(map_i, map_j) && kernel[i][j])
            {
                time_colli_ += std::chrono::system_clock::now() - start_colli;
                return true;
            }
        }
    }

    time_colli_ += std::chrono::system_clock::now() - start_colli;
    return false;
}

bool HybridAstar::footPrintInCollision4(const double &path_x, const double &path_y, const double &yaw)
{
    // timer
    auto start_colli = std::chrono::system_clock::now();

    // transform static footprint
    Eigen::Matrix2d curr_footprint;
    double c = cos(yaw), s = sin(yaw);
    Eigen::Rotation2Dd rot(yaw);
    // calculate footprint on this pose
    curr_footprint = rot * footprint_longi_;
    curr_footprint.row(0).array() += path_x;
    curr_footprint.row(1).array() += path_y;

    // check the line between points
    double footprint_len = V_LENGTH_ - V_WIDTH_;
    int num_checking_step = std::ceil(footprint_len / map_->map_resolution_orig_);
    double step = footprint_len / num_checking_step;
    Eigen::Vector2d orient = (curr_footprint.col(1) - curr_footprint.col(0)) / num_checking_step;

    for (int i = 0; i <= num_checking_step; ++i)
    {
        Eigen::Vector2d curr_pt = orient * i + curr_footprint.col(0);
        int curr_i, curr_j;
        map_->posToIndexOrig(curr_pt(0), curr_pt(1), curr_i, curr_j);

        // collision checking using original occupancy grid
        if (map_->pointInCollisionINFLOrig(curr_i, curr_j))
        {
            time_colli_ += std::chrono::system_clock::now() - start_colli;
            return true;
        }
    }

    // check corner points
    // copy the footprint
    std::vector<std::vector<double>> curr_corners;
    curr_corners.reserve(5);
    for (int i = 0; i < footprint_m_.cols(); ++i)
    {
        curr_corners.emplace_back(footprint_m_.col(i).begin(), footprint_m_.col(i).end());
    }
    // calculate footprint on this pose
    for (auto &fp_pt : curr_corners)
    {
        double x = fp_pt[0];
        double y = fp_pt[1];
        fp_pt[0] = x * c - y * s + path_x;
        fp_pt[1] = x * s + y * c + path_y;
    }
    for (int i = 0; i < curr_corners.size(); ++i)
    {
        int curr_i, curr_j;
        map_->posToIndexOrig(curr_corners[i][0], curr_corners[i][1], curr_i, curr_j);
        if (map_->pointInCollisionOrig(curr_i, curr_j))
        {
            time_colli_ += std::chrono::system_clock::now() - start_colli;
            return true;
        }
    }
    // end timer
    time_colli_ += std::chrono::system_clock::now() - start_colli;
    return false;
}

bool HybridAstar::Plan()
{
    // break if not initialized
    if (!initialized_)
    {
        ROS_ERROR("[H_Astar] Planner not properly initialized!");
        return false;
    }

    // set flags
    bool goal_found = false, limit_reached = false;
    // counter
    int iteration = 0;
    // timer
    auto start_plan = std::chrono::system_clock::now();

    // start node as first node in open set
    open_map_.insert(std::pair<double, NodePtr>(start_node_->fvalue_, start_node_));

    // start iteration
    while (!open_map_.empty())
    {
        // check current iteration
        if (iteration > iteration_limit_)
        {
            limit_reached = true;
            break;
        }

        // get the node with lowest fvalue
        NodePtr curr_node = open_map_.begin()->second;
        open_map_.erase(open_map_.begin());

        if (curr_node->status_ == HybridAstarNodeStatus::CLOSE)
        {
            continue;
        }

        // check analytical expansion
        if (genRSPath(curr_node))
        {
            // set flag
            goal_found = true;
            if (!optimal_)
            {
                break;
            }
        }
        else
        {
            // expand this node for new nodes
            expandNode(curr_node);
        }

        if (optimal_)
        {
            if (std::hypot(goal_node_->x_ - curr_node->x_, goal_node_->y_ - curr_node->y_) <= stop_radius)
            {
                ROS_INFO("Reach stop radius");
                break;
            }
        }
        // counter
        ++iteration;
    }

    // summary for pure planning
    time_plan_ += std::chrono::system_clock::now() - start_plan;
    // time cost during planning
    ROS_INFO("Planning costs %f s", time_plan_.count());
    ROS_INFO("RS shots cost %f s, takes %f percent", time_rs_shot_.count(), time_rs_shot_.count() / time_plan_.count() * 100);
    ROS_INFO("Collision checking costs %f s, takes %f percent", time_colli_.count(), time_colli_.count() / time_plan_.count() * 100);
    ROS_INFO("Spawning nodes costs %f s, takes %f percent", time_spawn_.count(), time_spawn_.count() / time_plan_.count() * 100);
    ROS_INFO("Interpolating path costs %f s, takes %f percent", time_interpolate_.count(), time_interpolate_.count() / time_plan_.count() * 100);

    ROS_INFO("Spawned %d new nodes", num_nodes_);

    // determin the return
    if (goal_found)
    {
        ROS_INFO("[H_Astar] Goal found");
        // goal is found by analytical expansion, set up the final path
        setPath(goal_node_->parent_);
        return true;
    }
    else if (limit_reached)
    {
        ROS_ERROR("[H_Astar] Iteration limit reached! Plan failed!");
        return false;
    }
    else
    {
        ROS_ERROR("[H_Astar] Open set empty! Plan failed!");
        return false;
    }
}

void HybridAstar::setPath(NodePtr curr_node)
{
    ROS_DEBUG("[H_Astar] Setting path");

    // trace back from give node till start node, start node excluded
    // curr node is parent node of goal node
    while (curr_node->parent_ != nullptr)
    {
        path_.push_back({curr_node->x_, curr_node->y_, curr_node->yaw_, curr_node->steer_, curr_node->direction_});
        curr_node = curr_node->parent_;
    }

    // set the steering angle of start node
    start_node_->steer_ = 0;
    // set the direction of start node the same as last in path for path is now reversed
    // if the path now only consists of goal node
    if (path_.size() > 1)
    {
        // direction as last point in path so far
        start_node_->direction_ = path_.back().back();
    }
    else
    {
        // direction as first point in goal path
        start_node_->direction_ = goal_path_.front().back();
    }

    // push the start node to path
    path_.push_back({start_node_->x_, start_node_->y_, start_node_->yaw_, start_node_->steer_, start_node_->direction_});
    // set right order
    std::reverse(path_.begin(), path_.end());
    // concatenate rs path
    path_.insert(path_.end(), goal_path_.begin(), goal_path_.end());

    // set up the vector of PathPoint
    for (int i = 0; i < path_.size(); ++i)
    {
        // a path point with x, y, yaw
        PathPoint pathpoint(path_[i][0], path_[i][1], path_[i][2]);
        // pass the steering angle and direction
        pathpoint.steering_ = path_[i][3];
        pathpoint.direc_ = path_[i][4];
        // push point to vec
        new_path_.push_back(pathpoint);
    }
    // set path length and cusp info
    mpros_utils::updatePathLength(new_path_);
    mpros_utils::setCusp(new_path_);
    mpros_utils::updateCurvatureNew(new_path_);

    if (inter_enable_)
    {
        // timer
        auto start = std::chrono::system_clock::now();
        // upsample the path and set infos
        upsampleAndPopulatePath(new_path_);
        mpros_utils::updateCurvatureNew(new_path_);
        mpros_utils::updatePathLength(new_path_);
        time_interpolate_ = std::chrono::system_clock::now() - start;
    }

    ROS_DEBUG("[H_Astar] Set path");
}

std::vector<std::vector<double>> HybridAstar::getPath()
{
    return path_;
}

std::vector<PathPoint> HybridAstar::getNewPath()
{
    return new_path_;
}

bool HybridAstar::genRSPath(NodePtr node)
{
    // timer
    auto start_rs_shot = std::chrono::system_clock::now();
    // random number generator in range [0, 1)
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distri(0, 1);

    // helper value to set rs shot probability
    // squared goal distance
    double goal_dist_sq = std::pow(node->x_ - goal_node_->x_, 2) + std::pow(node->y_ - goal_node_->y_, 2);
    // probability of making RS shot is in range [0, 1] and increases when the node get closer to goal
    double prob_rs_shot = std::fmax(RS_shot_radius_ * RS_shot_radius_ - goal_dist_sq, 0.0) / RS_shot_radius_ / RS_shot_radius_;

    // if (distri(gen) < prob_rs_shot)
    if (goal_dist_sq < RS_shot_radius_ * RS_shot_radius_)
    {
        // set up generator
        rs_gen_.SetStart(node->x_, node->y_, node->yaw_, node->direction_, node->steer_);
        rs_gen_.SetEnd(goal_node_->x_, goal_node_->y_, goal_node_->yaw_);
        rs_gen_.FindOptimalPath();
        // get the rs path
        std::vector<std::vector<double>> rs_path = rs_gen_.GetTraj();
        double rs_cost = rs_gen_.GetLength();

        // collision check
        if (!pathInCollision(rs_path))
        {
            // set up goal node and goal path
            if (goal_node_->parent_ == nullptr || goal_node_->gvalue_ > node->gvalue_ + rs_cost)
            {
                goal_node_->parent_ = node;
                goal_node_->setCost(node->gvalue_ + rs_cost);
                goal_path_ = rs_path;
            }

            // end timer
            time_rs_shot_ += std::chrono::system_clock::now() - start_rs_shot;

            return true;
        }
    }

    // end timer
    time_rs_shot_ += std::chrono::system_clock::now() - start_rs_shot;
    return false;
}

void HybridAstar::upsampleAndPopulatePath(std::vector<PathPoint> &path)
{
    ROS_DEBUG("[H_Astar] Interpolating path");

    // copy of original path
    const std::vector<PathPoint> path_orig = path;

    // for debug, print all path point
    if (debug_)
    {
        for (int i = 0; i < path.size(); ++i)
        {
            auto &pt = path[i];
            ROS_DEBUG("point %d, (%f %f %f), curv %f, steering %f",
                      i, pt.getX(), pt.getY(), pt.getYaw(), pt.curv_, pt.steering_);
        }
    }

    // clear the path container
    path.clear();

    // find all cusp points
    std::vector<int> cusp_idx;
    for (int i = 0; i < path_orig.size() - 1; ++i)
    {
        if (path_orig[i].is_cusp_)
        {
            cusp_idx.emplace_back(i);
        }
    }

    double control_pt_len = 0.0;
    // interpolate path
    for (int i = 0; i < path_orig.size() - 1;)
    {
        // start endpoint of Bezier curve
        int start_idx = i;
        int end_idx;

        // push the first endpoint as original path point
        path.emplace_back(path_orig[start_idx]);

        // get the closest following cusp point as first end point
        int j;
        for (j = 0; j < cusp_idx.size(); ++j)
        {
            if (cusp_idx[j] > i)
            {
                end_idx = cusp_idx[j];
                break;
            }
        }
        // no cusp point following, set the end point of whole path as endpoint
        if (j == cusp_idx.size())
        {
            end_idx = path_orig.size() - 1;
        }

        ROS_DEBUG("[H_Astar] Interpolating: Try with start idx %d end idx %d", start_idx, end_idx);

        // loop end points to generate collision free curve
        int max_iter = end_idx - start_idx;
        int iter = 0;
        while (start_idx <= end_idx)
        {
            // iter all later points are iterated
            if (iter >= max_iter)
            {
                ROS_DEBUG("can't find spline from pt (%f, %f, %f), skipped",
                          path_orig[start_idx].getX(),
                          path_orig[start_idx].getY(),
                          path_orig[start_idx].getYaw());
                // start from next point
                end_idx = start_idx + 1;
                control_pt_len = 0.0;
                break;
            }
            ROS_DEBUG("[H_Astar] Interpolating: Generate curve between pt %d %d", start_idx, end_idx);

            // index difference of both endpoints
            int idx_diff = end_idx - start_idx;

            // copy endpoints
            PathPoint start_pt = path_orig[start_idx];
            PathPoint end_pt = path_orig[end_idx];
            // path length inbetween
            double dist = end_pt.dist_ - start_pt.dist_;
            if (control_pt_len == 0.0)
            {
                control_pt_len = 0.4 * dist;
            }

            // create extra control points
            // orientation vector of both ends
            Eigen::Vector2d prev_orient(cos(start_pt.yaw_), sin(start_pt.yaw_));
            Eigen::Vector2d next_orient(cos(end_pt.yaw_), sin(end_pt.yaw_));

            // cusp point has orientation tangential to last arc, mirrored here
            if (start_pt.is_cusp_)
            {
                prev_orient = prev_orient * -1;
            }

            // set the sample number of this segment
            int num_sample = std::ceil(std::fmin((end_pt.dist_ - start_pt.dist_) / path_granularity_,
                                                 upsample_factor_ * idx_diff));

            // generate new control point between end points
            // cusp point's direction along its previous segment, need to be mirrored
            Eigen::Vector2d pt1 = start_pt.point_ + control_pt_len * prev_orient * start_pt.direc_;
            Eigen::Vector2d pt2 = end_pt.point_ - 0.4 * dist * next_orient * end_pt.direc_;

            // update control point length
            double prev_control_pt_len = control_pt_len;
            control_pt_len = 0.4 * dist;

            // this segment too short
            if (num_sample * path_granularity_ <= 2 * step_size_)
            {
                pt1 = start_pt.point_ + control_pt_len * prev_orient * start_pt.direc_;
                pt2 = end_pt.point_ - 0.1 * dist * next_orient * end_pt.direc_;
                // update control point length
                control_pt_len = 0.1 * dist;
            }

            ROS_DEBUG("curv start %d (%f %f %f), curv %f, steering %f",
                      start_idx, start_pt.getX(), start_pt.getY(), start_pt.getYaw(), start_pt.curv_, start_pt.steering_);
            ROS_DEBUG("curv end %d (%f %f %f), curv %f, steering %f",
                      end_idx, end_pt.getX(), end_pt.getY(), end_pt.getYaw(), end_pt.curv_, end_pt.steering_);

            // interpolate Bezier curve point from prev point to next point
            Eigen::Vector2d new_pt;
            std::vector<PathPoint> temp_path;

            // push back the start point of this arc
            temp_path.emplace_back(start_pt);
            // collision flag of current point
            bool in_collision = false;
            bool curvature_exceeds = false;

            // start to interpolate
            for (int j = 1; j <= num_sample; ++j)
            {
                // distance factor: distance to start pt / path length
                double mu = (double)j / num_sample;
                // create new path point
                new_pt = mpros_utils::cubicBezier(start_pt.point_, pt1, pt2, end_pt.point_, mu);
                // push into temp path
                temp_path.emplace_back(new_pt(0), new_pt(1), 0.0);
                // set the direction and steering as end pt
                // for later curvature update
                temp_path.back().direc_ = end_pt.direc_;
                temp_path.back().steering_ = end_pt.steering_;
                // set yaw of previous point
                if (j > 1)
                {
                    // get the yaw angle for previous point
                    Eigen::Vector2d orient = mpros_utils::tangentDir(temp_path[j - 2].point_,
                                                                     temp_path[j - 1].point_,
                                                                     new_pt,
                                                                     false);
                    // if the whole segment is moving backward
                    // mirror the orientation vector
                    if (end_pt.direc_ < 0)
                    {
                        orient = -orient;
                    }
                    // calculate the heading value
                    temp_path[j - 1].yaw_ = std::atan2(orient(1), orient(0));
                }

                // info of end pt stays the same
                if (j == num_sample)
                {
                    temp_path.back().yaw_ = end_pt.yaw_;
                    temp_path.back().is_cusp_ = end_pt.is_cusp_;
                }
                // check the previous point for collision
                // for collision checking needs orientation
                if (footPrintInCollision4(temp_path[j - 1].getX(), temp_path[j - 1].getY(), temp_path[j - 1].getYaw()))
                {
                    // this arc should be gave up
                    in_collision = true;
                    break;
                }
            }
            // check the max curvature in intepolated segment
            double max_curv_seg = mpros_utils::updateCurvatureNew(temp_path);
            curvature_exceeds = max_curv_seg > 1.5 * max_curvature_;
            if (!in_collision && !curvature_exceeds)
            {
                // interpolated segment is free, intert to new path vector
                path.insert(path.end(), temp_path.begin() + 1, temp_path.end() - 1);
                break;
            }
            else
            {
                // check a closer point
                end_idx--;
            }
            // update iteration number
            iter++;
        }
        // start from end point of last segment
        i = end_idx;
    }
    // push the end point
    path.emplace_back(path_orig.back());

    ROS_DEBUG("[H_Astar] Interpolated path");
}

std::vector<Eigen::Vector4d> HybridAstar::getSearchTree()
{
    std::vector<Eigen::Vector4d> search_tree;
    // generate search tree with node granuality
    for (auto &branch : tree_)
    {
        double curr_x = branch(0);
        double curr_y = branch(1);
        double curr_yaw = branch(2);
        double direc = branch(6);
        double steer = branch(7);
        double dx, dy, dyaw;
        double next_x, next_y, next_yaw;
        double radius, steer_sign;

        // number of leaf on one branch
        int leaf_num = std::ceil(step_size_ / path_granularity_);
        // generate branch as expanding node
        for (int i = 0; i < leaf_num; ++i)
        {
            if (steer == 0)
            {
                dx = direc * path_granularity_;
                dy = 0;
                dyaw = 0;
                next_x = dx * cos(curr_yaw) - dy * sin(curr_yaw) + curr_x;
                next_y = dx * sin(curr_yaw) + dy * cos(curr_yaw) + curr_y;
                next_yaw = curr_yaw;
            }
            // take a curve
            else
            {
                radius = (V_WB_ / 2 + V_STEER_OFFSET_) / std::tan(std::abs(steer));
                steer_sign = steer / std::abs(steer);
                dyaw = path_granularity_ / radius;
                dx = sin(dyaw) * radius * direc;
                dy = (1 - cos(dyaw)) * radius * steer_sign;
                next_x = dx * cos(curr_yaw) - dy * sin(curr_yaw) + curr_x;
                next_y = dx * sin(curr_yaw) + dy * cos(curr_yaw) + curr_y;
                next_yaw = curr_yaw + dyaw * steer_sign * direc;
                regularYaw(next_yaw);
            }
            search_tree.emplace_back(next_x, next_y, curr_x, curr_y);
            curr_x = next_x;
            curr_y = next_y;
            curr_yaw = next_yaw;
        }
    }

    return search_tree;
}

nav_msgs::Path HybridAstar::getPathMsg()
{
    nav_msgs::Path path_msg;
    tf2::Quaternion q;
    path_msg.header.frame_id = "map";

    geometry_msgs::PoseStamped ps;

    for (int i = 0; i < new_path_.size(); ++i)
    {
        ps.header.frame_id = "map";
        ps.pose.position.x = new_path_[i].getX();
        ps.pose.position.y = new_path_[i].getY();

        q.setRPY(0, 0, new_path_[i].getYaw());
        ps.pose.orientation = tf2::toMsg(q);
        path_msg.poses.push_back(ps);
    }
    return path_msg;
}
