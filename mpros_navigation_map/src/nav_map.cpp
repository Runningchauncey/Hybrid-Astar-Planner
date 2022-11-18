#include <vector>
#include <iostream>
#include <chrono>
#include <math.h>

#include "mpros_navigation_map/nav_map.h"

#define NOW std::chrono::system_clock::now()

NavMap::NavMap()
{
}

NavMap::NavMap(ros::NodeHandle &nh)
{
    nh_ = nh;
    goal_cost_max_ = 0;
    map_sub = nh.subscribe("map", 1, &NavMap::getOccupancyMap, this);
}

NavMap::~NavMap()
{
}

void NavMap::config(ros::NodeHandle &nh)
{
    std::string prefix = "/mpros/map/";

    nh.param<double>(prefix + "occupied_thresh", occ_thres_, 0.68);
    nh.param<double>(prefix + "free_thresh", free_thres_, 0.2);
    nh.param<float>(prefix + "voro_fallout_rate", voro_alpha_, 10.0);
    nh.param<float>(prefix + "voro_max_region", voro_d_o_max_, 50.0);
    nh.param<int>(prefix + "downsampling_factor", downsample_factor_, 1);
    nh.param<double>(prefix + "obstacle_inflation_radius", obstacle_inflation_radius_, 0.0);
}

void NavMap::getOccupancyMap(const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
{
    setOccupancyMap(*map_msg);
}

void NavMap::setOccupancyMap(const nav_msgs::OccupancyGrid &og)
{
    ROS_INFO("[Nav_Map] ---Loading Map---");
    bool update_map = false;

    // check if it is the initial map
    if (og.header.stamp.toNSec() == 0) // is initial map
    {

        return;
    }

    if (!get_map_) // map uninitialized
    {
        ROS_DEBUG("[Nav_Map] Loading first map-------------");
        // pass the map specification
        map_height_orig_ = og.info.height;
        map_width_orig_ = og.info.width;
        map_resolution_orig_ = og.info.resolution;

        map_origin_x_ = og.info.origin.position.x;
        map_origin_y_ = og.info.origin.position.y;
        tf2::fromMsg(og.info.origin.orientation, map_origin_q_);
        map_origin_yaw_ = tf2::getYaw(map_origin_q_);
        map_origin_sin_ = std::sin(map_origin_yaw_);
        map_origin_cos_ = std::cos(map_origin_yaw_);
        map_origin_ << map_origin_x_, map_origin_y_;

        ROS_INFO("[Nav_Map] Map of height %d, width %d, resolution %f", map_height_orig_, map_width_orig_, map_resolution_orig_);
        ROS_INFO("[Nav_Map] Map of origin %f, %f, %f", map_origin_x_, map_origin_y_, map_origin_yaw_);

        // calculate range of map, TODO: if needed
        x_max_ = map_origin_x_ + map_resolution_orig_ * map_width_orig_;
        x_min_ = map_origin_x_;
        y_max_ = map_origin_y_ + map_resolution_orig_ * map_height_orig_;
        y_min_ = map_origin_y_;

        // initialize map member
        static_map_orig_.resize(map_height_orig_ * map_width_orig_, 0);
        inflate_map_orig_.resize(map_height_orig_ * map_width_orig_, 0);
    }

    if (get_map_ &&
        (map_height_orig_ != og.info.height ||
         map_width_orig_ != og.info.width ||
         map_resolution_orig_ != og.info.resolution ||
         map_origin_x_ != og.info.origin.position.x ||
         map_origin_y_ != og.info.origin.position.y ||
         map_origin_yaw_ != tf2::getYaw(og.info.origin.orientation))) // map info changes
    {
        map_height_orig_ = og.info.height;
        map_width_orig_ = og.info.width;
        map_resolution_orig_ = og.info.resolution;

        map_origin_x_ = og.info.origin.position.x;
        map_origin_y_ = og.info.origin.position.y;
        tf2::fromMsg(og.info.origin.orientation, map_origin_q_);
        map_origin_yaw_ = tf2::getYaw(map_origin_q_);
        map_origin_sin_ = std::sin(map_origin_yaw_);
        map_origin_cos_ = std::cos(map_origin_yaw_);
        map_origin_ << map_origin_x_, map_origin_y_;
        ROS_INFO("[Nav_Map] New map of height %d, width %d, resolution %f", map_height_orig_, map_width_orig_, map_resolution_orig_);
        ROS_INFO("[Nav_Map] Map of origin %f, %f, %f", map_origin_x_, map_origin_y_, map_origin_yaw_);

        // reinitialize the map container
        static_map_orig_.resize(map_height_orig_ * map_width_orig_, 0);
        inflate_map_orig_.resize(map_height_orig_ * map_width_orig_, 0);
        ROS_DEBUG("[Nav_Map] Map resized, replanning is needed.");
        update_map = true;
    }

    // store the map and compare the diff
    for (int i = 0; i < static_map_orig_.size(); ++i)
    {
        // if the grid i is not free, set as occupied in navigation map
        // if the static map not yet as occupied, add to obstacle list
        if ((og.data[i] > free_thres_ || og.data[i] < 0) && static_map_orig_[i] == 0)
        {
            static_map_orig_[i] = 1;
            new_obstacle_list_.emplace_back(i);
        }
        // if the grid i is free yet static map set as occupied
        // add to vacancy list
        else if (og.data[i] < free_thres_ && static_map_orig_[i] == 1)
        {
            static_map_orig_[i] = 0;
            new_vacancy_list_.emplace_back(i);
        }
        // else static map already set to right value, continue
    }

    if (!new_obstacle_list_.empty() || !new_vacancy_list_.empty())
    {
        ROS_DEBUG("[Nav_Map] Map changes, replanning is needed.");
        ROS_DEBUG("[Nav_Map] Obstacle size %lu vacancy size %lu", new_obstacle_list_.size(), new_vacancy_list_.size());
        update_map = true;

        new_obstacle_list_.clear();
        new_vacancy_list_.clear();
    }

    if (!get_map_ || update_map) // get the first map or map to be updated
    {
        downsampleMap();
        auto clock = NOW;
        inflateObstacle();
        ROS_DEBUG_STREAM("[Nav_Map] Time cost on inflation map: " << std::chrono::duration<double>(NOW - clock).count() << "s.");

        if (get_goal_)
        {
            generateGoalMap();
        }

        clock = NOW;
        generateVoronoiMap();
        ROS_DEBUG_STREAM("[Nav_Map] Time cost on voronoi map: " << std::chrono::duration<double>(NOW - clock).count() << "s.");
    }

    ROS_INFO("[Nav_Map] Map ready");
    get_map_ = true;
}

void NavMap::inflateObstacle()
{
    ROS_DEBUG("[Nav_Map] Inflating with radius %f", obstacle_inflation_radius_);
    // parameter to inflate map
    // change obstacle_inflation_radius to pixel number
    int obstacle_inflation_pixel_num = std::ceil(obstacle_inflation_radius_ / map_resolution_orig_);
    double inflat_pix_sq = obstacle_inflation_pixel_num * obstacle_inflation_pixel_num;
    // temp map for inflation
    std::vector<std::vector<uint8_t>> map_inflation;
    std::vector<uint8_t> map_inflt_empty_row(map_width_orig_ + 2 * obstacle_inflation_pixel_num, 0);
    map_inflation.resize(map_height_orig_ + 2 * obstacle_inflation_pixel_num, map_inflt_empty_row);

    for (int i = 0; i < map_height_orig_; ++i)
    {
        for (int j = 0; j < map_width_orig_; ++j)
        {
            // inflate occupied grid
            if (static_map_orig_[getIndexOriginal(i, j)] == 1)
            {
                // index range in inflation map
                int inflation_i_min = i - obstacle_inflation_pixel_num + obstacle_inflation_pixel_num;
                int inflation_i_max = i + obstacle_inflation_pixel_num + obstacle_inflation_pixel_num;
                int inflation_j_min = j - obstacle_inflation_pixel_num + obstacle_inflation_pixel_num;
                int inflation_j_max = j + obstacle_inflation_pixel_num + obstacle_inflation_pixel_num;
                for (int inflation_i = inflation_i_min;
                     inflation_i < inflation_i_max + 1;
                     ++inflation_i)
                {
                    for (int inflation_j = inflation_j_min;
                         inflation_j < inflation_j_max + 1;
                         ++inflation_j)
                    {
                        double i_dist = std::abs(inflation_i - i - obstacle_inflation_pixel_num) - 0.5;
                        double j_dist = std::abs(inflation_j - j - obstacle_inflation_pixel_num) - 0.5;
                        if (i_dist * i_dist + j_dist * j_dist <= inflat_pix_sq)
                            map_inflation[inflation_i][inflation_j] = 1;
                    }
                }
            }
        }
    }
    // flatten the inflation map
    // assign inflated map to map member
    for (int i = 0; i < map_height_orig_; ++i)
    {
        for (int j = 0; j < map_width_orig_; ++j)
        {
            inflate_map_orig_[getIndexOriginal(i, j)] = map_inflation[i + obstacle_inflation_pixel_num][j + obstacle_inflation_pixel_num];
        }
    }
}

bool NavMap::pointInMap(const double &x, const double &y)
{
    // transform location to index
    int i, j;
    posToIndex(x, y, i, j);
    if (i < 0 || i >= map_height_ ||
        j < 0 || j >= map_width_)
    {
        return false;
    }
    return true;
}

bool NavMap::pointInMap(const int &i, const int &j)
{
    if (i < 0 || i >= map_height_ ||
        j < 0 || j >= map_width_)
    {
        return false;
    }
    return true;
}

bool NavMap::pointInMapOrig(const int &i, const int &j)
{
    if (i < 0 || i >= map_height_orig_ ||
        j < 0 || j >= map_width_orig_)
    {
        return false;
    }
    return true;
}

bool NavMap::pointInCollision(const int &i, const int &j)
{
    // return grid value if grid within the map
    if (pointInMap(i, j))
    {
        return static_map_[getIndex(i, j)];
    }
    return true;
}

bool NavMap::pointInCollisionOrig(const int &i, const int &j)
{
    // return grid value if grid within the map
    if (pointInMapOrig(i, j))
    {
        return static_map_orig_[getIndexOriginal(i, j)];
    }
    return true;
}

bool NavMap::pointInCollisionINFLOrig(const int &i, const int &j)
{
    // return grid value if grid within the map
    if (pointInMapOrig(i, j))
    {
        return inflate_map_orig_[getIndexOriginal(i, j)];
    }
    return true;
}

void NavMap::generateGoalMap()
{
    ROS_DEBUG("[Nav_Map] Start to generate goal map");
    // resize goal map as map
    goal_map_.resize(map_height_ * map_width_);
    // assign coordinates to all nodes
    for (int i = 0; i < map_height_; ++i)
    {
        for (int j = 0; j < map_width_; ++j)
        {
            goal_map_[getIndex(i, j)] = GoalDistNode(i, j);
        }
    }
    // open set for goal map
    std::vector<GoalDistNode *> openset_;
    // transform to index coordinates
    int goal_i, goal_j;
    posToIndex(goal_x_, goal_y_, goal_i, goal_j);
    if (!pointInMap(goal_i, goal_j))
    {
        get_goal_ = false;
        ROS_WARN("Goal point out of map, goal cost map not generated");
        return;
    }
    // set goal node cost to 0
    goal_map_[getIndex(goal_i, goal_j)].setCost(0);
    // push start node into open set
    openset_.push_back(getGoalNodePtr(goal_i, goal_j));
    // make heap
    make_heap(openset_.begin(), openset_.end(), NodeGreater{});

    while (!openset_.empty())
    {
        // order the openset as min heap on cost
        pop_heap(openset_.begin(), openset_.end(), NodeGreater{});
        // get the node ptr with lowest cost
        GoalDistNode *curr_node_ptr = openset_.back();
        // remove the node ptr from openset
        openset_.pop_back();

        int curr_cost = curr_node_ptr->cost_;
        // check the neighbour of current node, replace with lower cost and push to open set
        for (int direc = 0; direc < num_direc_; ++direc)
        {
            int i_neigh, j_neigh;
            getNeighbourIndex(curr_node_ptr->i_, curr_node_ptr->j_, direc, i_neigh, j_neigh);

            if (!pointInCollision(i_neigh, j_neigh))
            {
                GoalDistNode *neigh_node_ptr = &(goal_map_[getIndex(i_neigh, j_neigh)]);
                if (neigh_node_ptr->cost_ > (curr_cost + 1))
                {
                    neigh_node_ptr->setCost(curr_cost + 1);
                    openset_.push_back(neigh_node_ptr);
                    goal_cost_max_ = (int)std::fmax(goal_cost_max_, neigh_node_ptr->cost_);
                }
            }
        }

        // push heap for openset
        push_heap(openset_.begin(), openset_.end());
    }

    // set the goal cost map
    auto getvalue = [](const GoalDistNode &node)
    { return node.cost_; };
    goal_cost_map_.resize(goal_map_.size());
    std::transform(goal_map_.begin(), goal_map_.end(), goal_cost_map_.begin(), getvalue);

    ROS_DEBUG("[Nav_Map] Generation of goal map done");
}

void NavMap::getNeighbourIndex(int i, int j, int direction, int &n_i, int &n_j)
{
    switch (direction)
    {
    case 0:
        // index up
        n_i = i - 1;
        n_j = j;
        break;
    case 1:
        // index right
        n_i = i;
        n_j = j + 1;
        break;
    case 2:
        // index down
        n_i = i + 1;
        n_j = j;
        break;
    case 3:
        // index left
        n_i = i;
        n_j = j - 1;
        break;

    default:
        // default output invalid index
        n_i = -1;
        n_j = -1;
        break;
    }
}

void NavMap::markObs()
{
    // init region index
    int region = 0;
    for (auto &grid : voronoi_map_)
    {
        VoroNode *curr_node_ptr = &grid;
        // is this node is an obstacle and not marked yet
        if (pointInCollision(curr_node_ptr->x_, curr_node_ptr->y_) && curr_node_ptr->region_ == -1)
        {
            curr_node_ptr->region_ = region;
            curr_node_ptr->obs_dist_ = 0;
            obs_queue_.push(curr_node_ptr);
            obs_dist_open_set_.push_back(curr_node_ptr);
            floodFillObs();
            ++region;
        }
    }
    ROS_DEBUG("[Nav_Map] mark obs done");
}

void NavMap::floodFillObs()
{
    while (!obs_queue_.empty())
    {
        // get the top node in obstacle queue
        VoroNode *curr_node_ptr = obs_queue_.front();
        obs_queue_.pop();
        // check all neightbors, TODO: make compatible with 8 connected map
        for (int i = 0; i < 4; ++i)
        {
            int x_n, y_n;
            getNeighbourIndex(curr_node_ptr->x_, curr_node_ptr->y_, i, x_n, y_n);
            if (!pointInCollision(x_n, y_n) || !pointInMap(x_n, y_n))
            {
                continue;
            }
            // assign neighbor node as obstacle node and push to queue
            VoroNode *neigh_node_ptr = &(voronoi_map_[getIndex(x_n, y_n)]);
            if (pointInCollision(x_n, y_n) && neigh_node_ptr->region_ == -1)
            {
                neigh_node_ptr->region_ = curr_node_ptr->region_;
                neigh_node_ptr->obs_dist_ = 0;
                obs_queue_.push(neigh_node_ptr);
                obs_dist_open_set_.push_back(neigh_node_ptr);
            }
        }
    }
}

void NavMap::calObsDist()
{
    // order the obstacle distance set
    make_heap(obs_dist_open_set_.begin(), obs_dist_open_set_.end(), NodeFurtherFromObs());
    while (!obs_dist_open_set_.empty())
    {
        // order the vector by put the one with lowest distance to obstacles moved to end
        pop_heap(obs_dist_open_set_.begin(), obs_dist_open_set_.end(), NodeFurtherFromObs());
        // get the ptr of the node nearest to obstacles
        VoroNode *curr_node_ptr = obs_dist_open_set_.back();
        // get the current obstacle distance
        int curr_obs_dist = curr_node_ptr->obs_dist_;
        // delete the node from vector
        obs_dist_open_set_.pop_back();
        // check all neighbours, TODO: make compatiable to 8 connect map
        for (int i = 0; i < 4; ++i)
        {
            int x_n, y_n;
            getNeighbourIndex(curr_node_ptr->x_, curr_node_ptr->y_, i, x_n, y_n);
            if (pointInCollision(x_n, y_n) || !pointInMap(x_n, y_n))
            {
                continue;
            }
            VoroNode *neigh_node_ptr = &(voronoi_map_[getIndex(x_n, y_n)]);
            // if the neighbor node further from obstacle or not set yet
            if (neigh_node_ptr->obs_dist_ > (curr_obs_dist + 1))
            {
                // update the obstacle distance
                neigh_node_ptr->obs_dist_ = curr_obs_dist + 1;
                // assign region number acoording to the lower obstacle dist
                neigh_node_ptr->region_ = curr_node_ptr->region_;
                obs_dist_open_set_.push_back(neigh_node_ptr);
                free_set_.push_back(neigh_node_ptr);
            }
        }
        // add end elements into the heap
        push_heap(obs_dist_open_set_.begin(), obs_dist_open_set_.end());
    }
    ROS_DEBUG("[Nav_Map] Obstacle distance calculated");
}

void NavMap::findEdge()
{
    // loop through all elements in free set
    for (auto &grid : free_set_)
    {
        // skip occupied cells in any case
        if (pointInCollision(grid->x_, grid->y_))
        {
            continue;
        }
        for (int i = 0; i < 4; ++i)
        {
            int x_n, y_n;
            getNeighbourIndex(grid->x_, grid->y_, i, x_n, y_n);
            if (!pointInMap(x_n, y_n))
            {
                continue;
            }
            VoroNode *neigh_node = &voronoi_map_[getIndex(x_n, y_n)];
            if (grid->region_ != neigh_node->region_ && !grid->isedge_)
            {
                grid->edge_dist_ = 0;
                neigh_node->edge_dist_ = 0;
                // set the grid and the neighbour as edge
                grid->isedge_ = true;
                neigh_node->isedge_ = true;
                edge_set_.push_back(grid);
                edge_set_.push_back(neigh_node);
            }
        }
    }
    ROS_DEBUG("[Nav_Map] Edge found");
}

void NavMap::calEdgeDist()
{
    make_heap(edge_set_.begin(), edge_set_.end(), NodeFurtherFromEdge());
    while (!edge_set_.empty())
    {
        // get the node closest to edge
        pop_heap(edge_set_.begin(), edge_set_.end(), NodeFurtherFromEdge());
        VoroNode *curr_node_ptr = edge_set_.back();
        edge_set_.pop_back();

        int curr_edge_dist = curr_node_ptr->edge_dist_;
        // check all neighbours, TODO: make compatible
        for (int i = 0; i < 4; ++i)
        {
            int x_n, y_n;
            getNeighbourIndex(curr_node_ptr->x_, curr_node_ptr->y_, i, x_n, y_n);
            if (!pointInMap(x_n, y_n))
            {
                continue;
            }
            VoroNode *neigh_node = &(voronoi_map_[getIndex(x_n, y_n)]);
            if (neigh_node->edge_dist_ > (curr_edge_dist + 1))
            {
                neigh_node->edge_dist_ = curr_edge_dist + 1;
                edge_set_.push_back(neigh_node);
            }
        }
        push_heap(edge_set_.begin(), edge_set_.end());
    }
    ROS_DEBUG("[Nav_Map] Edge cost calculated");
}

void NavMap::generateVoronoiMap()
{
    // init voronoi map
    voronoi_map_.resize(map_height_ * map_width_);
    // assign coordinates to all nodes
    for (int i = 0; i < map_height_; ++i)
    {
        for (int j = 0; j < map_width_; ++j)
        {
            voronoi_map_[getIndex(i, j)] = VoroNode(i, j);
        }
    }
    // 1. mark obstacle area and enumerate them
    markObs();
    // 2. flood fill all obstacle grid with obs dist = 0
    // floodFillObs();
    // 3. flow field algo on all grids for lowest obs dist
    calObsDist();
    // 4. mark all the edge grids, where one neighbour belongs to the other region
    findEdge();
    // 5. flow field algo on all free grid for lowest edge dist
    calEdgeDist();

    // calculate traversal cost for all grids
    for (auto &grid : voronoi_map_)
    {
        if (pointInCollision(grid.x_, grid.y_))
        {
            grid.cost_ = 1;
        }
        else if (grid.isedge_)
        {
            grid.cost_ = 0;
        }
        else
        {
            grid.cost_ = (voro_alpha_ / (voro_alpha_ + (float)grid.obs_dist_)) *
                         ((float)grid.edge_dist_ / (float)(grid.edge_dist_ + grid.obs_dist_)) *
                         ((float)grid.obs_dist_ - voro_d_o_max_) * ((float)grid.obs_dist_ - voro_d_o_max_) /
                         voro_d_o_max_ / voro_d_o_max_;
        }
    }
    // set voronoi value map
    auto getvalue = [](const VoroNode &node)
    { return node.cost_; };
    voronoi_value_map_.resize(voronoi_map_.size());
    std::transform(voronoi_map_.begin(), voronoi_map_.end(), voronoi_value_map_.begin(), getvalue);

    ROS_DEBUG("[Nav_Map] Voronoi generated");
}

void NavMap::downsampleMap()
{
    // copy the full size map
    static_map_ = static_map_orig_;
    // modify the info
    map_height_ = ceil(static_cast<double>(map_height_orig_) / downsample_factor_);
    map_width_ = ceil(static_cast<double>(map_width_orig_) / downsample_factor_);
    map_resolution_ = map_resolution_orig_ * downsample_factor_;

    if (downsample_factor_ != 1)
    {
        // resize the downsample map
        static_map_.resize(map_height_ * map_width_);
        // change cell values
        for (int i = 0; i < map_height_; ++i)
        {
            for (int j = 0; j < map_width_; ++j)
            {
                if (getIndex(i, j) >= static_map_.size())
                {
                    continue;
                }
                static_map_[getIndex(i, j)] = downsampleCellValue(i, j);
            }
        }
    }
    else
    {
        static_map_ = static_map_orig_;
    }
}

int8_t NavMap::downsampleCellValue(int new_i, int new_j)
{
    int8_t value = 0;
    int old_i, old_j;
    int i_offset = new_i * downsample_factor_,
        j_offset = new_j * downsample_factor_;
    for (int i = 0; i < downsample_factor_; ++i)
    {
        old_i = i + i_offset;
        if (old_i > map_height_orig_)
        {
            break;
        }
        for (int j = 0; j < downsample_factor_; ++j)
        {
            old_j = j + j_offset;
            if (old_j > map_width_orig_)
            {
                break;
            }
            if (getIndexOriginal(old_i, old_j) > static_map_orig_.size())
            {
                continue;
            }
            value = (int8_t)std::fmax(value, static_map_orig_[getIndexOriginal(old_i, old_j)]);
        }
    }
    return value;
}

template <typename T>
nav_msgs::OccupancyGrid NavMap::vecToMsg(const std::vector<T> &layer, bool is_original)
{
    nav_msgs::OccupancyGrid msg;
    msg.header.frame_id = "map";
    if (!get_map_ || layer.size() == 0)
    {
        return msg;
    }
    if (is_original)
    {
        msg.info.height = map_height_orig_;
        msg.info.width = map_width_orig_;
        msg.info.resolution = map_resolution_orig_;
    }
    else
    {
        msg.info.height = map_height_;
        msg.info.width = map_width_;
        msg.info.resolution = map_resolution_;
    }
    msg.info.origin.position.x = map_origin_x_;
    msg.info.origin.position.y = map_origin_y_;
    msg.info.origin.orientation = tf2::toMsg(map_origin_q_);

    T maxval = *(std::max_element(layer.begin(), layer.end()));
    maxval = std::min((T)100, maxval);

    std::vector<int8_t> modified_layer(layer.size(), 0);

    for (int i = 0; i < layer.size(); ++i)
    {
        modified_layer[i] = std::floor(static_cast<float>(layer[i]) * 100 / maxval);
    }

    msg.data = modified_layer;
    return msg;
}

nav_msgs::OccupancyGrid NavMap::getStaticMapMsg()
{
    return vecToMsg(static_map_, false);
}

nav_msgs::OccupancyGrid NavMap::getGoalMapMsg()
{
    return vecToMsg(goal_cost_map_, false);
}

nav_msgs::OccupancyGrid NavMap::getVoroMapMsg()
{
    return vecToMsg(voronoi_value_map_, false);
}

nav_msgs::OccupancyGrid NavMap::getStaticOrigMapMsg()
{
    return vecToMsg(static_map_orig_, true);
}

nav_msgs::OccupancyGrid NavMap::getInflationOrigMapMsg()
{
    return vecToMsg(inflate_map_orig_, true);
}
