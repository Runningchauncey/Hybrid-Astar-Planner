#ifndef MPROS_NAV_MAP__NODE_H
#define MPROS_NAV_MAP__NODE_H

#define HUGE_INT 1000

// map cell node
struct GoalDistNode
{
    GoalDistNode() : i_(-1), j_(-1), cost_(HUGE_INT)
    {
    }
    GoalDistNode(int x, int y, int cost = HUGE_INT) : i_(x), j_(y), cost_(cost)
    {
    }
    void setCost(int cost)
    {
        cost_ = cost;
    }
    void reset()
    {
        cost_ = HUGE_INT;
    }
    // index of the cell, i: row, j: col
    int i_, j_;
    // goal cost (Manhattan distance to goal point considering obstacle)
    int cost_;
};

struct VoroNode
{
    VoroNode() : VoroNode(-1, -1)
    {
    }
    VoroNode(int x, int y) : x_(x), y_(y), region_(-1), obs_dist_(HUGE_INT), edge_dist_(HUGE_INT), isedge_(false), cost_(-1)
    {
    }
    int x_;         // row index
    int y_;         // col index
    int obs_dist_;  // distance to nearest obstacle
    int edge_dist_; // distance to nearest edge
    int region_;    // region index
    bool isedge_;   // if the node belongs to edge
    float cost_;    // voronoi cost
};

/**
 * @brief comparator for ordering GoalDistNode
 *
 */
class NodeGreater
{
public:
    bool operator()(GoalDistNode *const node1, GoalDistNode *const node2)
    {
        return node1->cost_ > node2->cost_;
    }
};

/**
 * @brief comparator for ordering VoroNode
 *
 */
class NodeFurtherFromObs
{
public:
    bool operator()(VoroNode *const node1, VoroNode *const node2)
    {
        return node1->obs_dist_ > node2->obs_dist_;
    }
};

/**
 * @brief comparator for ordering VoroNode
 *
 */
class NodeFurtherFromEdge
{
public:
    bool operator()(VoroNode *const node1, VoroNode *const node2)
    {
        return node1->edge_dist_ > node2->edge_dist_;
    }
};

#endif // MPROS_NAV_MAP__NODE_H
