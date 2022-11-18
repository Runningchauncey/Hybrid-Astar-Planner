#ifndef HYBRID_A_STAR_NODE_H
#define HYBRID_A_STAR_NODE_H

#include <vector>
#include <memory>
#include "mpros_navigation_map/nav_map.h"

enum HybridAstarNodeStatus
{
    OPEN = 0,
    CLOSE = 1
};

class HybridAstarNode
{
public:
    /**
     * @brief empty constructor
     *
     */
    HybridAstarNode() = default;
    /**
     * @brief Construct a new Hybrid Astar Node object
     *
     * @param x
     * @param y
     * @param yaw
     * @param steer steer from parent to this node
     * @param direction direction from parent to this node
     * @param parent pointer to parent node
     * @param map pointer to current map
     */
    HybridAstarNode(const double &x,
                    const double &y,
                    const double &yaw,
                    const double &steer,
                    const double &direction,
                    std::shared_ptr<HybridAstarNode> parent,
                    NavMap *map)
        : x_(x), y_(y), yaw_(yaw),
          steer_(steer), direction_(direction), parent_(parent), 
          gvalue_(0), hvalue_(0), fvalue_(0), 
          status_(HybridAstarNodeStatus::OPEN)
    {
        map->posToIndex(x_, y_, idx_i_, idx_j_);
    }
    /**
     * @brief Destroy the Hybrid Astar Node object
     *
     */
    ~HybridAstarNode() {}
    /**
     * @brief Set the node cost
     *
     * @param cost
     */
    void setCost(const double &cost)
    {
        gvalue_ = cost;
        fvalue_ = gvalue_ + hvalue_;
    }
    /**
     * @brief Set node heuristic
     *
     * @param heuristic
     */
    void setHeuristic(const double &heuristic)
    {
        hvalue_ = heuristic;
        fvalue_ = gvalue_ + hvalue_;
    }
    void setClose()
    {
        status_ = HybridAstarNodeStatus::CLOSE;
    }
    /**
     * @brief copy constructor
     *
     * @param other_node
     * @return HybridAstarNode&
     */
    HybridAstarNode &operator=(const HybridAstarNode &other_node)
    {
        this->x_ = other_node.x_;
        this->y_ = other_node.y_;
        this->yaw_ = other_node.yaw_;
        this->steer_ = other_node.steer_;
        this->direction_ = other_node.direction_;
        this->gvalue_ = other_node.gvalue_;
        this->hvalue_ = other_node.hvalue_;
        this->fvalue_ = other_node.fvalue_;
        this->parent_ = other_node.parent_;
        this->status_ = other_node.status_;
        return *this;
    }

    double x_;
    double y_;
    double yaw_;
    double steer_;     // steer to reach this node
    double direction_; // direction to reach this node
    /* index of grid */
    int idx_i_;
    int idx_j_;
    /* value of this node */
    double gvalue_; // cost of this node (path cost, voronoi cost, cost of parent, penalty)
    double hvalue_; // heutistic of this node
    double fvalue_; // path cost + heuristic
    // status
    HybridAstarNodeStatus status_;
    // ptr to parent node
    std::shared_ptr<HybridAstarNode> parent_;
};

// define the type
typedef std::shared_ptr<HybridAstarNode> NodePtr;

/**
 * @brief comparator for hybrid astar nodes, for descending order of open set
 *
 */
struct HybridAstarNodeCmpGreater
{
    bool operator()(const NodePtr lhs, const NodePtr rhs) const
    {
        return lhs->fvalue_ > rhs->fvalue_;
    }
};

#endif // HYBRID_A_STAR_NODE_H