#ifndef RS_CURVE_BASIC_H
#define RS_CURVE_BASIC_H

#include <math.h>
#include <vector>
#include <string>

namespace RS
{
    /**
     * @brief Steering direction
     *
     */
    enum Steer
    {
        LEFT = 1,
        STRAIGHT = 0,
        RIGHT = -1
    };
    /**
     * @brief Directions
     *
     */
    enum Direc
    {
        FORWARD = 1,
        BACKWARD = -1
    };

    struct Point
    {
        Point() : x(.0), y(.0), phi(.0) {}
        Point(double val1, double val2) : x(val1), y(val2), phi(.0) {}
        Point(double val1, double val2, double val3) : x(val1), y(val2), phi(val3) {}
        Point(const std::vector<double> &vec) : x(vec[0]), y(vec[1]), phi(vec[2]) {}
        double x;
        double y;
        double phi;
        /**
         * @brief return point mirrored by y-axis (-x, y, -phi)
         *
         * @return Point
         */
        Point timeflip() { return Point(-x, y, -phi); }
        /**
         * @brief return point mirrored by x-axis (x, -y, -phi)
         *
         * @return Point
         */
        Point reflect() { return Point(x, -y, -phi); }
    };
    /**
     * @brief Transform input point into polar coordinates
     * 
     */
    struct PolarCoord
    {
        double dist;
        double theta;
        PolarCoord(Point pt) : dist(std::hypot(pt.x, pt.y)), theta(std::atan2(pt.y, pt.x)) {}
        PolarCoord(double x, double y) : dist(std::hypot(x, y)), theta(std::atan2(y, x)) {}
    };

    /**
     * @brief Segment in RS path marked by maneuver
     *
     */
    class PathSeg
    {
    private:
        // length for straight segment or angle for circular segment
        double value_;
        Steer steer_;
        Direc direc_;

    public:
        /**
         * @brief Construct a new Path Seg object
         *
         * @param value Input: length for straight segment or angle for circular segment
         * @param steer Input: steering direction
         * @param direc Input: driving direction
         */
        PathSeg(double value, Steer steer, Direc direc) : value_(std::abs(value)),
                                                          steer_(steer),
                                                          direc_((value > 0 ? direc : Direc(-direc)))
        {
        }
        /**
         * @brief Get the manuever string: "Direction: Steer: Value:"
         *
         * @return std::string
         */
        std::string GetManuever() const { return "Steer: " + std::to_string(GetSteer()) +
                                                 "; Direction: " + std::to_string(GetDirec()) +
                                                 "; Value: " + std::to_string(GetLength()); }
        /**
         * @brief Get the path length
         *
         * @return double
         */
        double GetLength() const { return value_; }
        Steer GetSteer() const { return steer_; }
        Direc GetDirec() const { return direc_; }
    };

    /**
     * @brief Set angle in [-pi, pi]
     *
     * @param theta Input: angle in rad
     * @return double Regularized angle
     */
    inline double regularRad(const double &theta)
    {
        if (theta > M_PI)
        {
            return theta - 2 * M_PI;
        }
        else if (theta < -M_PI)
        {
            return theta + 2 * M_PI;
        }
        return theta;
    }
    /**
     * @brief Reverse the direction of given segment
     *
     * @param seg Input: path segment
     * @return PathSeg
     */
    inline PathSeg reverseDirec(const PathSeg &seg)
    {
        return PathSeg(seg.GetLength(), seg.GetSteer(), Direc(-seg.GetDirec()));
    }
    /**
     * @brief Reverse the steering direction of given segment
     *
     * @param seg Input: path segment
     * @return PathSeg
     */
    inline PathSeg reverseSteer(const PathSeg &seg)
    {
        return PathSeg(seg.GetLength(), Steer(-seg.GetSteer()), seg.GetDirec());
    }

}

#endif // !RS_CURVE_BASIC_H