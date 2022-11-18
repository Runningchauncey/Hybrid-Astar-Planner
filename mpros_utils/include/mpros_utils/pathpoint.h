#ifndef PATHPOINT_H
#define PATHPOINT_H

#include <vector>
#include <Eigen/Core>

class PathPoint
{
public:
    PathPoint() {}
    PathPoint(const double &x, const double &y, const double &yaw)
    {
        point_(0) = x;
        point_(1) = y;
        yaw_ = yaw;
    }

    PathPoint(const Eigen::Vector2d &point, const double &yaw)
    : point_(point), yaw_(yaw)
    {
    }
    inline double getX() const { return point_(0); }
    inline double getY() const { return point_(1); }
    inline double getYaw() const { return yaw_; }

    Eigen::Vector2d point_;
    // Vector2D point_;
    double yaw_, curv_, vel_, accel_, jerk_;
    // manuver direction to this pt : 1 - forward, -1 - backward
    double direc_, steering_;
    bool fixed_ = false;
    bool is_cusp_ = false;
    double dist_; // distance from start to this point
    double time_;
};

#endif //! PATHPOINT_H