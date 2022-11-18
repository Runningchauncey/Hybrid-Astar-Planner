#ifndef MPROS_NAVIGATION_MAP__NAV_MAP_H
#define MPROS_NAVIGATION_MAP__NAV_MAP_H

#include <vector>
#include <queue>
#include <math.h>
#include <algorithm>
#include <Eigen/Core>

#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"
#include "nav_msgs/OccupancyGrid.h"

#include "mpros_navigation_map/node.h"

class NavMap
{
public:
    /**
     * @brief Construct a empty object
     *
     */
    NavMap();
    /**
     * @brief Construct a new NavMap object
     *
     * @param nh ros nodehandle
     */
    NavMap(ros::NodeHandle &nh);
    /**
     * @brief Destroy the Nav Map object
     *
     */
    ~NavMap();
    /**
     * @brief configure the map parameters
     *
     */
    void config(ros::NodeHandle &nh);
    /**
     * @brief callback to store map
     *
     * @param map_msg
     */
    void getOccupancyMap(const nav_msgs::OccupancyGrid::ConstPtr &map_msg);
    /**
     * @brief set map
     *
     * @param og occupancy grid acquired from message
     */
    void setOccupancyMap(const nav_msgs::OccupancyGrid &og);
    /**
     * @brief If a point in collision
     *
     * @param i location in index
     * @param j
     * @return true
     * @return false
     */
    bool pointInCollision(const int &i, const int &j);
    /**
     * @brief check collision in original map
     *
     * @param i
     * @param j
     * @return true
     * @return false
     */
    bool pointInCollisionOrig(const int &i, const int &j);
    /**
     * @brief check collision in inflated map
     *
     * @param i
     * @param j
     * @return true
     * @return false
     */
    bool pointInCollisionINFLOrig(const int &i, const int &j);
    /**
     * @brief transform location into index
     *
     * @param x
     * @return int
     */
    inline int xToCol(const double &x) { return int((x - map_origin_x_) / map_resolution_); }
    /**
     * @brief transform location into index
     *
     * @param y
     * @return int
     */
    inline int yToRow(const double &y) { return int((y - map_origin_y_) / map_resolution_); }
    /**
     * @brief transform location into index
     *
     * @param x
     * @return int
     */
    inline int xToColOrig(const double &x) { return int((x - map_origin_x_) / map_resolution_orig_); }
    /**
     * @brief transform location into index
     *
     * @param y
     * @return int
     */
    inline int yToRowOrig(const double &y) { return int((y - map_origin_y_) / map_resolution_orig_); }
    /**
     * @brief transform index into location
     *
     * @param i
     * @return double
     */
    inline double colToX(const int &i) { return ((double)i + 0.5) * map_resolution_ + map_origin_x_; }
    /**
     * @brief transform index into location
     *
     * @param j
     * @return double
     */
    inline double rowToY(const int &j) { return ((double)j + 0.5) * map_resolution_ + map_origin_y_; }
    /**
     * @brief transform index into location
     *
     * @param i
     * @return double
     */
    inline double colToXOrig(const int &i) { return ((double)i + 0.5) * map_resolution_orig_ + map_origin_x_; }
    /**
     * @brief transform index into location
     *
     * @param j
     * @return double
     */
    inline double rowToYOrig(const int &j) { return ((double)j + 0.5) * map_resolution_orig_ + map_origin_y_; }
    /**
     * @brief Transform point position in map frame to map grid index in downsampled map
     *
     * @param x Input: position x
     * @param y Input: position y
     * @param i Output: grid row i
     * @param j Output: grid col j
     */
    inline void posToIndex(const double &x, const double &y, int &i, int &j)
    {
        double dx = x - map_origin_x_;
        double dy = y - map_origin_y_;
        // rotation against map yaw
        j = static_cast<int>((dx * map_origin_cos_ + dy * map_origin_sin_) / map_resolution_);
        i = static_cast<int>((dx * -map_origin_sin_ + dy * map_origin_cos_) / map_resolution_);
    }
    /**
     * @brief Transform point position in map frame to map grid index in original map
     *
     * @param x Input: position x
     * @param y Input: position y
     * @param i Output: grid row i
     * @param j Output: grid col j
     */
    inline void posToIndexOrig(const double &x, const double &y, int &i, int &j)
    {
        double dx = x - map_origin_x_;
        double dy = y - map_origin_y_;
        // rotation against map yaw
        j = static_cast<int>((dx * map_origin_cos_ + dy * map_origin_sin_) / map_resolution_orig_);
        i = static_cast<int>((dx * -map_origin_sin_ + dy * map_origin_cos_) / map_resolution_orig_);
    }
    /**
     * @brief Transfrom map grid index in downsampled map to point position in map frame
     *
     * @param i Input: grid row i
     * @param j Input: grid col j
     * @param x Output: position x
     * @param y Output: position y
     */
    inline void indexToPos(const int &i, const int &j, double &x, double &y)
    {
        double dx = ((double)j + 0.5) * map_resolution_;
        double dy = ((double)i + 0.5) * map_resolution_;
        // rotation by map yaw
        x = (dx * map_origin_cos_ - dy * map_origin_sin_) + map_origin_x_;
        y = (dx * map_origin_sin_ + dy * map_origin_cos_) + map_origin_y_;
    }
    /**
     * @brief Transfrom map grid index in original map to point position in map frame
     *
     * @param i Input: grid row i
     * @param j Input: grid col j
     * @param x Output: position x
     * @param y Output: position y
     */
    inline void indexToPosOrig(const int &i, const int &j, double &x, double &y)
    {
        double dx = ((double)j + 0.5) * map_resolution_orig_;
        double dy = ((double)i + 0.5) * map_resolution_orig_;
        // rotation by map yaw
        x = (dx * map_origin_cos_ - dy * map_origin_sin_) + map_origin_x_;
        y = (dx * map_origin_sin_ + dy * map_origin_cos_) + map_origin_y_;
    }
    /**
     * @brief Update goal point and call to generate goal cost map
     *
     */
    void updateGoal(const double &x, const double &y)
    {
        goal_x_ = x;
        goal_y_ = y;
        get_goal_ = true;
        generateGoalMap();
    }
    /**
     * @brief Get the Goal Cost object
     *
     * @param x
     * @param y
     * @return double
     */
    double getGoalCost(const double &x, const double &y)
    {
        int i, j;
        posToIndex(x, y, i, j);
        return getGoalCost(i, j);
    }
    /**
     * @brief Get the Goal Cost object
     *
     * @param i
     * @param j
     * @return double
     */
    double getGoalCost(const int &i, const int &j)
    {
        return goal_map_[getIndex(i, j)].cost_ * map_resolution_;
    }
    /**
     * @brief Get the voronoi cost
     *
     * @param x location
     * @param y
     * @return double
     */
    inline double getVoronoiCost(const double &x, const double &y)
    {
        int i, j;
        posToIndex(x, y, i, j);
        return getVoronoiCost(i, j);
    }
    /**
     * @brief Get the Voronoi Cost
     *
     * @param i col index
     * @param j
     * @return double
     */
    inline double getVoronoiCost(const int &i, const int &j) { return voronoi_map_[getIndex(i, j)].cost_; }
    /**
     * @brief Get goal cost map
     *
     * @return std::vector<int>
     */
    std::vector<int> getGoalMap()
    {
        return goal_cost_map_;
    }
    /**
     * @brief Get Voronoi Field
     *
     * @return std::vector<float>
     */
    std::vector<float> getVoronoiMap() const
    {
        return voronoi_value_map_;
    }
    /**
     * @brief Get the msg of static map
     *
     * @return nav_msgs::OccupancyGrid
     */
    nav_msgs::OccupancyGrid getStaticMapMsg();
    /**
     * @brief Get the msg of goal cost map
     *
     * @return nav_msgs::OccupancyGrid
     */
    nav_msgs::OccupancyGrid getGoalMapMsg();
    /**
     * @brief Get the msg of Voronoi Field map
     *
     * @return nav_msgs::OccupancyGrid
     */
    nav_msgs::OccupancyGrid getVoroMapMsg();
    /**
     * @brief Get the msg of static map of original resolution
     *
     * @return nav_msgs::OccupancyGrid
     */
    nav_msgs::OccupancyGrid getStaticOrigMapMsg();
    /**
     * @brief Get the msg of inflated map of original resolution
     *
     * @return nav_msgs::OccupancyGrid
     */
    nav_msgs::OccupancyGrid getInflationOrigMapMsg();

private:
    /**
     * @brief inflate map
     *
     */
    void inflateObstacle();
    /**
     * @brief if a point within map
     *
     * @param i location in index
     * @param j
     * @return true
     * @return false
     */
    bool pointInMap(const int &i, const int &j);
    /**
     * @brief If a point within map
     *
     * @param x location in map frame
     * @param y location in map frame
     * @return true
     * @return false
     */
    bool pointInMap(const double &x, const double &y);
    /**
     * @brief if the grid within original map
     *
     * @param i
     * @param j
     * @return true
     * @return false
     */
    bool pointInMapOrig(const int &i, const int &j);
    /**
     * @brief Generate a goal cost map
     *
     */
    void generateGoalMap();
    /**
     * @brief Generate a Voronoi Field
     *
     */
    void generateVoronoiMap();
    /**
     * @brief Get the index in 1D vector with 2D index(i, j)
     *
     * @param i
     * @param j
     * @return int
     */
    inline int getIndex(int i, int j) { return i * map_width_ + j; }
    /**
     * @brief Get the index in 1D vector with 2D index(i, j) in original map
     *
     * @param i
     * @param j
     * @return int
     */
    inline int getIndexOriginal(int i, int j) { return i * map_width_orig_ + j; }
    /**
     * @brief Get index in 2D vector with 1D index
     *
     * @param index
     * @param i
     * @param j
     */
    inline void getLocation(int index, int &i, int &j)
    {
        i = index / map_width_;
        j = index % map_width_;
    }
    /**
     * @brief Get index in 2D vector with 1D index in original map
     *
     * @param index
     * @param i
     * @param j
     */
    inline void getLocationOriginal(int index, int &i, int &j)
    {
        i = index / map_width_orig_;
        j = index % map_width_orig_;
    }
    /**
     * @brief Get the neighbour cell index in a 4-connection-map
     *
     * @param i current index i
     * @param j current index j
     * @param direction to current cell
     * @param n_i neighbour index i
     * @param n_j neighbour index j
     */
    void getNeighbourIndex(int i, int j, int direction, int &n_i, int &n_j);
    /**
     * @brief Get the pointer to map cell
     *
     * @param x
     * @param y
     * @return GoalDistNode*
     */
    inline GoalDistNode *getGoalNodePtr(int x, int y)
    {
        return &(goal_map_[getIndex(x, y)]);
    }
    /**
     * @brief mark obstacle cell with increasing region number
     *
     */
    void markObs();
    /**
     * @brief fill all obstacle cell with obstacle dist = 0
     *
     */
    void floodFillObs();
    /**
     * @brief calculate Manhattan distance to obstacle
     *
     */
    void calObsDist();
    /**
     * @brief find edge cells
     *
     */
    void findEdge();
    /**
     * @brief calculate Manhattan distance to edge
     *
     */
    void calEdgeDist();
    /**
     * @brief if the node is edge
     *
     * @param i
     * @param j
     * @return true
     * @return false
     */
    inline bool isEdge(const int &i, const int &j) { return voronoi_map_[getIndex(i, j)].isedge_; }
    /**
     * @brief Downsample map
     *
     */
    void downsampleMap();
    /**
     * @brief Calculate cell value in downsampled map
     *
     * @param new_i
     * @param new_j
     * @return int8_t
     */
    int8_t downsampleCellValue(int new_i, int new_j);

    /**
     * @brief Convert vector form map into message OccupancyGrid
     *
     * @tparam T Type of cell value
     * @param layer Input: map layer to be converted
     * @param is_original Input: if the layer of origianl resolution
     * @return nav_msgs::OccupancyGrid Output: message
     */
    template <typename T>
    nav_msgs::OccupancyGrid vecToMsg(const std::vector<T> &layer, bool is_original);
    

public:
    // flag, if map is received and ready to use
    bool get_map_ = false;
    // flag is goal point is set
    bool get_goal_ = false;
    // info of the map
    // map size
    int map_height_, map_width_;
    int map_height_orig_, map_width_orig_;
    // map resolution m/pixel
    double map_resolution_;
    double map_resolution_orig_;
    // origin of map in [m] in world frame
    double map_origin_x_, map_origin_y_, map_origin_yaw_;
    double map_origin_sin_, map_origin_cos_;
    tf2::Quaternion map_origin_q_;
    Eigen::Vector2d map_origin_;
    // boundary of map in world frame
    double x_max_, x_min_, y_max_, y_min_;
    
private:
    // ros nodehandle
    ros::NodeHandle nh_;
    // subscriber to map published by map server
    ros::Subscriber map_sub;
    // goal map
    std::vector<GoalDistNode> goal_map_;
    // goal cost map
    std::vector<int> goal_cost_map_;
    // voronoi map
    std::vector<VoroNode> voronoi_map_;
    // voronoi map values
    std::vector<float> voronoi_value_map_;
    // static map layer
    std::vector<int8_t> static_map_orig_;
    // downsampled map layer
    std::vector<int8_t> static_map_;
    // inflation map layer
    std::vector<int8_t> inflate_map_orig_;

    /* container for dynamic map*/
    // list of index of new occupied grid
    std::vector<int> new_obstacle_list_;
    // list of index of new vacant grid
    std::vector<int> new_vacancy_list_;

    /* container for voronoi map */
    // set to calculate obstacle distance
    std::vector<VoroNode *> obs_dist_open_set_;
    // set of all free nodes
    std::vector<VoroNode *> free_set_;
    // set of all edge nodes
    std::vector<VoroNode *> edge_set_;
    // queue for obstacle cells, used for floodfill all obstacle cells
    std::queue<VoroNode *> obs_queue_;

    // goal point in world frame
    double goal_x_, goal_y_, goal_yaw_;
    // max goal cost for visualization
    int goal_cost_max_;

    //----------- Parameters -------------
    // // occupancy threshold, "occupied_thresh"
    double occ_thres_ = 68;
    // free threshold, "free_thresh"
    double free_thres_ = 20;
    // map connection type, default = 4, "map/connection"
    int num_direc_ = 4;
    // voronoi fallout rate, "map/voro_fallout"
    float voro_alpha_ = 10.0;
    // voronoi max region size, "map/max_region"
    float voro_d_o_max_ = 50;
    // inflation radius in [m], "map/inflation"
    double obstacle_inflation_radius_ = 0;
    // discretization of yaw
    int num_yaw_ = 36;
    // downsample factor
    int downsample_factor_ = 5;
};

#endif // !MPROS_NAVIGATION_MAP__NAV_MAP_H