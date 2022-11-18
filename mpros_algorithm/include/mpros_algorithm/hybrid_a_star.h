#ifndef HYBRID_ASTAR_H
#define HYBRID_ASTAR_H

#include <string>
#include <iterator>
#include <algorithm>
#include <fstream>
#include <vector>
#include <time.h>
#include <queue>
#include <memory>

#include <ros/ros.h>
#include <Eigen/Core>

#include "mpros_algorithm/hybrid_a_star_node.h"
#include "mpros_navigation_map/nav_map.h"
#include "mpros_rs_path/rs_curve.h"
#include "mpros_utils/rs_statespace.h"
#include "mpros_utils/pathpoint.h"
#include "mpros_utils/line_iterator.h"
#include "mpros_utils/visualizer.h"
#include "mpros_utils/planner_utils.h"
#include "mpros_utils/footprint.h"

class HybridAstar
{
public:
    // due to member matrix with fixed size
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**
     * @brief Construct a empty object, config and then initialize before use
     *
     */
    HybridAstar() = default;
    /**
     * @brief Compact constructor,
     * construct a new Hybrid Astar object with all inputs for config and initialize
     *
     * @param start_pt Input: point in x,y,theta
     * @param goal_pt Input: point in x,y,theta
     * @param map Input: refer to navigation map object
     * @param vis Input: refer to visualizer object
     * @param nh Input: refer to nodehandle
     */
    HybridAstar(const std::vector<double> &start_pt,
                const std::vector<double> &goal_pt,
                NavMap &map,
                Visualizer &vis,
                ros::NodeHandle &nh);
    /**
     * @brief Destroy the Hybrid Astar object
     *
     */
    ~HybridAstar();
    /**
     * @brief Reset this planner by clearing all containers
     *
     */
    void reset();
    /**
     * @brief Configure the parameters
     *
     * @param nh  Input: refer to nodehandle
     */
    void config(ros::NodeHandle &nh);
    /**
     * @brief Initial all members for planning, must be after config called
     *
     * @param start_pt Input: point in x,y,theta
     * @param goal_pt Input: point in x,y,theta
     * @param map Input: refer to navigation map object
     * @param vis Input: refer to visualizer object
     */
    void initialize(const std::vector<double> &start_pt,
                    const std::vector<double> &goal_pt,
                    NavMap &map,
                    Visualizer &vis);
    /**
     * @brief Main workflow, manage the open set and close set
     *
     * @return true: plan successfully,
     * @return false: plan failed
     */
    bool Plan();
    /**
     * @brief convert heading angle to index according to heading resolution
     *
     * @param yaw Input: heading angle in [rad]
     * @return int Output: index of heading angle
     */
    int yawToIdx(const double &yaw);
    /**
     * @brief apply all possible steer and direction to this node
     *
     * @param node Input: node ptr to be expanded
     */
    void expandNode(NodePtr node);
    /**
     * @brief Check if any point on this path is in collision by calling footprint in collision function
     * for each path point
     *
     * @param path Input: path in form of {x, y, yaw}
     * @return true: in collision;
     * @return false: collision-free
     */
    bool pathInCollision(const std::vector<std::vector<double>> &path);
    /**
     * @brief Check if any point on this path is in collision
     *
     * @param path Input: vector of PathPoint
     * @return true: in collision;
     * @return false: collision-free
     */
    bool pathInCollision(const std::vector<PathPoint> &path);
    /**
     * @brief if the footprint at this location with given yaw is in collision with obstacles
     * This function implements a method only considers 4 edges
     *
     * @param x Input: x position of steering center
     * @param y Input: y position of steering center
     * @param yaw Input: heading angle the vehicle
     * @return true: in collision;
     * @return false: collision-free
     */
    bool footPrintInCollision(const double &x, const double &y, const double &yaw);
    /**
     * @brief if the footprint at this location with given yaw is in collision with obstacles
     * This function implements a method also considering inner cells by calculate the inner products of vector
     * connecting to each corner point
     *
     * @param x Input: x position of steering center
     * @param y Input: y position of steering center
     * @param yaw Input: heading angle the vehicle
     * @return true: in collision;
     * @return false: collision-free
     */
    bool footPrintInCollision2(const double &x, const double &y, const double &yaw);
    /**
     * @brief if the footprint at this location with given yaw is in collision with obstacles
     * This function implements a method also considering inner cells by checking all inner points
     *
     * @param x Input: x position of steering center
     * @param y Input: y position of steering center
     * @param yaw Input: heading angle the vehicle
     * @return true: in collision;
     * @return false: collision-free
     */
    bool footPrintInCollision3(const double &x, const double &y, const double &yaw);
    /**
     * @brief if the footprint at this location with given yaw is in collision with obstacles
     * This function uses inflated map and eroded footprint
     *
     * @param x Input: x position of steering center
     * @param y Input: y position of steering center
     * @param yaw Input: heading angle the vehicle
     * @return true: in collision;
     * @return false: collision-free
     */
    bool footPrintInCollision4(const double &x, const double &y, const double &yaw);
    /**
     * @brief Set the whole path by tracking back from given node
     * Called by plan(), set up path_ and new_path_
     * After extracted path from nodes, the path will be interpolated by calling
     * upsampleAndInterpolate() and update curvature, cusp, length and heading info
     *
     * @param curr_node Input: normally goal node, the node to be traced back
     */
    void setPath(NodePtr curr_node);
    /**
     * @brief Get the path_ {{x, y, heading, steering, direction}, ...}
     *
     * @return std::vector<std::vector<double>> Path in form of 2D vector
     */
    std::vector<std::vector<double>> getPath();
    /**
     * @brief generate a Reeds-Shepp path from this node to goal node,
     * the generated collision free path saved in member goal_path_
     *
     * @param node this node
     * @return true: path collision-free;
     * @return false: path in collision or not generating one
     */
    bool genRSPath(NodePtr node);
    /**
     * @brief Set the gvalue to given node, i.e. path cost.
     * It consists of following term:
     * 1. accumulated cost from parent node
     * 2. voronoi cost of this node
     * 3. step size
     * 4. direction cost by adding up all penalty related with driving direction
     * 5. steering cost by adding up all penalty related with steering maneuver
     * Final gvalue will be the sum of all above terms
     *
     * @param node Input: the node whose gvalue is to be set
     */
    void setNodeCost(NodePtr node);
    /**
     * @brief Set the heuristic value to given node
     * It will be the maximum of following term to ensure admissible:
     * 1. Manhattan distance to goal node considering obstacles
     * 2. Reeds-Shepp path distance without considering obstacles
     * To avoid same hvalue, a tie-breaker is multiplied
     *
     * @param node Input: the node whose gvalue is to be set
     */
    void setNodeHeuristic(NodePtr node);
    /**
     * @brief Get the path as a vector of pathpoint,
     * containing elaborating path information
     *
     * @return std::vector<PathPoint> Output: path
     */
    std::vector<PathPoint> getNewPath();
    /**
     * @brief Get the path in nav_msgs::Path form, can be used for visualization
     * The path will be in "map" frame
     *
     * @return nav_msgs::Path
     */
    nav_msgs::Path getPathMsg();
    /**
     * @brief Calculate the node index with grid coordinate, yaw index and direction value
     *
     * @param idx_i Grid index col
     * @param idx_j Grid index row
     * @param idx_yaw Heading angle index
     * @param direc Direction value {-1, 1}
     * @return int: Index of this node in node set
     */
    int calIndex(const int &idx_i, const int &idx_j, const int &idx_yaw, const double &direc);
    /**
     * @brief Regularize yaw in range (-pi, pi]
     *
     * @param yaw Input: heading angle should not be over 2pi far from the range
     */
    void regularYaw(double &yaw);
    /**
     * @brief Incrementally interpolate the primitive path by Bezier curve
     * The whole path will first divided into segments by cusp points,
     * starting from first node, try to interpolate a collision free Bezier curve
     * with next cusp point/goal point as the other endpoint.
     * When collision, move the endpoint forward to generate a shorter curve.
     *
     * @param path To be updated with new path.
     * Before input, the path should have set the cusp point and
     * path length at each point.
     */
    void upsampleAndPopulatePath(std::vector<PathPoint> &path);
    /**
     * @brief set the static coordinates of footprint of the vehicle
     * the footprint is assumed to be rectangle with following shape
     * off-L/2 0   offset   L/2 + offset     \n
     *    r    -   c        f                \n
     *    ------------------- w / 2          \n
     *    -    -   -        -                \n
     *    -    s----        -                \n
     *    -    -   -        -                \n
     *    ------------------- -w / 2         \n
     * c: lateral center line of the vehicle \n
     * f: front border                       \n
     * r: rear border                        \n
     * s: rotate center; normal car on the center of rear axis, flexcar on the shape center
     *
     * The footprints are stored in members
     */
    void setFootprint();
    /**
     * @brief Generate and get the search tree
     *
     * @return std::vector<Eigen::Vector4d> Output: a pair of points for leaf {{x1, y1, x2, y2}, ...}
     */
    std::vector<Eigen::Vector4d> getSearchTree();

private:
    // debug flag
    bool debug_ = false;
    // if configuration succeeds
    bool configured_ = false;
    // if initialization succeeds
    bool initialized_ = false;
    // if output an optimal path
    bool optimal_ = false;
    // if the path is to be interpolated
    bool inter_enable_ = true;
    // number of generated nodes
    int num_nodes_ = 0;

    // For heuristics
    // rs state space using ompl reedsshepp statespace to calculate heuristic
    RSStateSpace rs_ss_;

    // rs path generator, now only used to generate goal path
    RS::RSCurve rs_gen_;

    // pointer to navigation map
    NavMap *map_;
    // pointer to wrapper for visualization functions
    Visualizer *vis_;

    // Planning components
    // start node
    NodePtr start_node_;
    // end node
    NodePtr goal_node_;
    // open set, use multimap to manage it
    std::multimap<double, NodePtr> open_map_;
    // all costs for open nodes
    std::vector<double> cost_set_;
    // all nodes in open set
    std::vector<NodePtr> node_set_;
    // search tree branch, {x1, y1, yaw1, x2, y2, yaw2, direc1->2, steer1->2}
    std::vector<Eigen::Vector<double, 8>> tree_;
    // all possible steering angle
    std::vector<double> steer_set_;
    // all possible directions
    std::vector<double> direction_set_;
    // collision free path generated by rs generator to the goal point
    std::vector<std::vector<double>> goal_path_;

    // result of planning
    // simple array of path
    // {{x,y,yaw,steer,direction}, ...}
    std::vector<std::vector<double>> path_;
    // path array including information about curvature
    std::vector<PathPoint> new_path_;

    // PARAMETERS
    // Planning parameters
    // iteration limit to avoid endless loop
    int iteration_limit_ = 10000;
    // number of directions
    int num_direc = 2;
    // discretisization of steering angle
    int num_steer_ = 3;
    // stepsize for each manuever [m]
    double step_size_ = 1.1;
    // desired distance between two path point [m]
    double path_granularity_ = 0.2;
    // upsample to reach granularity in path
    int upsample_factor_ = 1;
    // discretization of yaw for each grid
    int num_yaw_ = 36;
    // resolution for discretization of yaw
    double yaw_resol_ = 360 / num_yaw_ * M_PI / 180;
    // Cost penalty
    // penalizing changing direction
    double PEN_GEAR_COST_ = 10;
    // penalizing reverse driving
    double PEN_BACKWARD_COST_ = 0;
    // penalizing changing steering angle
    double PEN_STEER_CHANGE_ = 10;
    // penalizing not driving straightly
    double PEN_STEER_ANGLE_COST_ = 0;
    // range in which RS shot will start
    double RS_shot_radius_ = 10.0;
    // stop planning when a node is within this radius
    double stop_radius = 1.0;

    // Vehicle paramters
    // vehicle width [m]
    double V_WIDTH_ = 2.;
    // wheelbase [m]
    double V_WB_ = 3.0;
    // max steer angle [rad]
    double V_MAX_STEER_ = 0.35;
    // vehicle length [m]
    double V_LENGTH_ = 4.2;
    // distance between vehicle center and steering center
    double V_STEER_OFFSET_ = 0;
    // curvature upper-bound
    double max_curvature_ = 0.125;
    // minimal radius according to curvature bound
    double min_r_ = 1 / max_curvature_;

    // static footprint of the vehicle, vehicle along the x axis heading 0
    // footprint as a rectangle in method1, to be ordered clockwise
    // {{x1, y1}, ...}
    Eigen::Matrix<double, 2, 4> footprint_m_;
    // longitudinal footprint as one line, used for with inflated map in method 4
    Eigen::Matrix<double, 2, 2> footprint_longi_;
    // footprint as a grid with original map resolution, used as a kernel method 3
    Footprint footprint_kernel_;
    // footprint used in collision checking method 2
    // circumcircle radius of the vehicle footprint
    double footprint_r_;
    // vector of x coordinates of corner points, ordered clockwise
    std::vector<double> footprint_x_ = {V_LENGTH_ / 2 + V_STEER_OFFSET_,
                                        V_LENGTH_ / 2 + V_STEER_OFFSET_,
                                        -V_LENGTH_ / 2 + V_STEER_OFFSET_,
                                        -V_LENGTH_ / 2 + V_STEER_OFFSET_};
    // vector of y coordinatesof corner points, ordered clockwise
    std::vector<double> footprint_y_ = {V_WIDTH_ / 2,
                                        -V_WIDTH_ / 2,
                                        -V_WIDTH_ / 2,
                                        V_WIDTH_ / 2};

    // all timers to benchmark the time cost
    std::chrono::duration<double> time_rs_shot_ = std::chrono::duration<double>::zero(),
                                  time_colli_ = std::chrono::duration<double>::zero(),
                                  time_spawn_ = std::chrono::duration<double>::zero(),
                                  time_interpolate_ = std::chrono::duration<double>::zero(),
                                  time_plan_ = std::chrono::duration<double>::zero();
};

#endif // HYBRID_ASTAR_H
