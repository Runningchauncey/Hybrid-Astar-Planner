#ifndef PLANNER_UTILS_H
#define PLANNER_UTILS_H

#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include <ceres/ceres.h>
#include <ros/ros.h>

#include "mpros_utils/pathpoint.h"

namespace mpros_utils
{

    /**
     * @brief return vector of index range [start, end)
     * cf. https://www.geeksforgeeks.org/slicing-a-vector-in-c/
     *
     * @tparam T
     * @param v
     * @param start
     * @param end position of end iterator
     * @return std::vector<T>
     */
    template <typename T>
    static std::vector<T> slicing(const std::vector<T> &v, int start, int end)
    {
        return std::vector<T>(v.begin() + start, v.begin() + end);
    }

    /**
     * @brief set the cusp flag along the path
     *
     * @param path
     */
    void setCusp(std::vector<PathPoint> &path)
    {
        for (int i = 0; i < path.size() - 1; ++i)
        {
            if (path[i].direc_ != path[i + 1].direc_)
            {
                path[i].is_cusp_ = true;
            }
        }
    }

    /**
     * @brief set distance of each point
     *
     * @param path
     */
    void updatePathLength(std::vector<PathPoint> &path)
    {
        double len = 0;
        path[0].dist_ = 0;
        for (int i = 1; i < path.size(); ++i)
        {
            len += (path[i].point_ - path[i - 1].point_).norm();
            path[i].dist_ = len;
        }
    }

    /**
     * @brief find the center of given 3 points
     *
     * @param prev_pt
     * @param pt
     * @param next_pt
     * @param iscusp if the point is cusp point
     * @return Eigen::Vector2d
     */
    template <typename T>
    Eigen::Matrix<T, 2, 1> findArcCenter(Eigen::Matrix<T, 2, 1> prev_pt,
                                         Eigen::Matrix<T, 2, 1> pt,
                                         Eigen::Matrix<T, 2, 1> next_pt,
                                         bool iscusp)
    {
        Eigen::Matrix<T, 2, 1> seg1 = pt - prev_pt,
                               seg2 = next_pt - pt;
        // pt is cusp point, mirror the next pt and seg2
        if (iscusp)
        {
            seg2 = -seg2;
            next_pt = pt + seg2;
        }
        // if two segments are colinear
        T det = seg1(0) * seg2(1) - seg1(1) * seg2(0);
        if (ceres::abs(det) < (T)1e-7)
        {
            return Eigen::Matrix<T, 2, 1>(std::numeric_limits<double>::infinity(),
                                          std::numeric_limits<double>::infinity());
        }

        // mid points of two segments
        Eigen::Matrix<T, 2, 1> mid1 = (pt + prev_pt) / (T)2,
                               mid2 = (pt + next_pt) / (T)2;
        // direction of perpendicular
        Eigen::Matrix<T, 2, 1> direc1(-seg1(1), seg1(0));
        Eigen::Matrix<T, 2, 1> direc2(-seg2(1), seg2(0));
        // find the intersection
        T det1 = (mid1(0) + direc1(0)) * mid1(1) - (mid1(1) + direc1(1)) * mid1(0);
        T det2 = (mid2(0) + direc2(0)) * mid2(1) - (mid2(1) + direc2(1)) * mid2(0);
        Eigen::Matrix<T, 2, 1> center((det1 * direc2(0) - det2 * direc1(0)) / det,
                                      (det1 * direc2(1) - det2 * direc1(1)) / det);
        return center;
    }

    /**
     * @brief update the curvature of all path points
     *
     * @param path
     */
    double updateCurvature(std::vector<PathPoint> &path)
    {
        double max_curv = 0.0;
        Eigen::Vector2d prev_pt, pt, next_pt;
        // loop over all points
        for (int i = 0; i < path.size(); ++i)
        {
            PathPoint &this_pt = path[i];
            // set neighbour points
            if (i == 0)
            {
                // mirror the second point against first point
                pt = this_pt.point_;
                next_pt = path[i + 1].point_;
                // orientation vector of pt
                Eigen::Vector2d orient(cos(this_pt.yaw_), sin(this_pt.yaw_));
                // vector from pt to next pt
                Eigen::Vector2d d12 = next_pt - pt;
                // vector from prevpt to nextpt
                Eigen::Vector2d d02 = orient.normalized() * orient.dot(d12) * 2;
                prev_pt = pt - d12;
            }
            else if (i == path.size() - 1)
            {
                // mirror the second last point against last point
                pt = this_pt.point_;
                prev_pt = path[i - 1].point_;
                // orientation vector of pt
                Eigen::Vector2d orient(cos(this_pt.yaw_), sin(this_pt.yaw_));
                // vector from prev pt to pt
                Eigen::Vector2d d01 = pt - prev_pt;
                // vector from prevpt to nextpt
                Eigen::Vector2d d02 = orient.normalized() * orient.dot(d01) * 2;
                next_pt = pt + d01;
            }
            else
            {
                prev_pt = path[i - 1].point_;
                pt = this_pt.point_;
                next_pt = path[i + 1].point_;
            }
            // find the center
            Eigen::Vector2d center = findArcCenter(prev_pt, pt, next_pt, this_pt.is_cusp_);
            // calculate the curvature
            if (std::isinf(center(0)))
            {
                this_pt.curv_ = 0;
            }
            else
            {
                // calculate the curvature with direction
                this_pt.curv_ = (this_pt.steering_ >= 0 ? 1 : -1) / (center - pt).norm();
                max_curv = std::fmax(std::abs(this_pt.curv_), max_curv);
            }
        }

        return max_curv;
    }

    template <typename T>
    T calCurvature(Eigen::Matrix<T, 2, 1> prev_pt,
                   Eigen::Matrix<T, 2, 1> pt,
                   Eigen::Matrix<T, 2, 1> next_pt)
    {
        // 1. derivative by central difference
        Eigen::Matrix<T, 2, 1> deriv1 = (next_pt - prev_pt) / (next_pt - prev_pt).norm();
        // 2. derivative by central difference
        Eigen::Matrix<T, 2, 1> deriv2 = (next_pt - pt) / (next_pt - pt).norm() - (pt - prev_pt) / (pt - prev_pt).norm();
        T l2_norm = deriv1.norm();
        return (deriv1(0) * deriv2(1) - deriv1(1) * deriv2(0)) / l2_norm / l2_norm / l2_norm;
    }

    double updateCurvatureNew(std::vector<PathPoint> &path)
    {
        double max_curv = 0.0;
        Eigen::Vector2d prev_pt, pt, next_pt;
        // loop over all points
        for (int i = 0; i < path.size(); ++i)
        {
            PathPoint &this_pt = path[i];
            // set neighbour points
            if (i == 0)
            {
                // mirror the second point against first point
                pt = this_pt.point_;
                next_pt = path[i + 1].point_;
                // orientation vector of pt
                Eigen::Vector2d orient(cos(this_pt.yaw_), sin(this_pt.yaw_));
                // vector from pt to next pt
                Eigen::Vector2d d12 = next_pt - pt;
                // vector from prevpt to nextpt
                Eigen::Vector2d d02 = orient.normalized() * orient.dot(d12) * 2;
                prev_pt = next_pt - d02;
            }
            else if (i == path.size() - 1)
            {
                // mirror the second last point against last point
                pt = this_pt.point_;
                prev_pt = path[i - 1].point_;
                // orientation vector of pt
                Eigen::Vector2d orient(cos(this_pt.yaw_), sin(this_pt.yaw_));
                // vector from prev pt to pt
                Eigen::Vector2d d01 = pt - prev_pt;
                // vector from prevpt to nextpt
                Eigen::Vector2d d02 = orient.normalized() * orient.dot(d01) * 2;
                next_pt = prev_pt + d02;
            }
            else
            {
                prev_pt = path[i - 1].point_;
                pt = this_pt.point_;
                if (this_pt.is_cusp_)
                {
                    next_pt = pt + pt - path[i + 1].point_;
                }
                else
                {
                    next_pt = path[i + 1].point_;
                }
            }
            // 1. derivative by central difference
            Eigen::Vector2d deriv1 = (next_pt - prev_pt).normalized();
            Eigen::Vector2d deriv2 = (next_pt - pt).normalized() - (pt - prev_pt).normalized();
            this_pt.curv_ = (deriv1(0) * deriv2(1) - deriv1(1) * deriv2(0)) / pow(deriv1.norm(), 3);
            max_curv = std::fmax(std::abs(this_pt.curv_), max_curv);
        }
        return max_curv;
    }
    /**
     * @brief Interpolate by cubic Bezier curve
     *
     * @param pt0 Control pt
     * @param pt1 Control pt
     * @param pt2 Control pt
     * @param pt3 Control pt
     * @param mu Distance ratio of length to start to length between start and end
     * @return Eigen::Vector2d Interpolated point
     */
    Eigen::Vector2d cubicBezier(
        const Eigen::Vector2d &pt0,
        const Eigen::Vector2d &pt1,
        const Eigen::Vector2d &pt2,
        const Eigen::Vector2d &pt3,
        double mu)
    {
        Eigen::Vector2d a, b, c, pt;

        c[0] = 3 * (pt1[0] - pt0[0]);
        c[1] = 3 * (pt1[1] - pt0[1]);
        b[0] = 3 * (pt2[0] - pt1[0]) - c[0];
        b[1] = 3 * (pt2[1] - pt1[1]) - c[1];
        a[0] = pt3[0] - pt0[0] - c[0] - b[0];
        a[1] = pt3[1] - pt0[1] - c[1] - b[1];

        pt[0] = a[0] * mu * mu * mu + b[0] * mu * mu + c[0] * mu + pt0[0];
        pt[1] = a[1] * mu * mu * mu + b[1] * mu * mu + c[1] * mu + pt0[1];

        return pt;
    }

    /**
     * @brief Interpolate by quadratic Bezier curve
     *
     * @param pt0 Control pt
     * @param pt1 Control pt
     * @param pt2 Control pt
     * @param mu Distance ratio of length to start to length between start and end
     * @return Eigen::Vector2d Interpolated point
     */
    Eigen::Vector2d quadBezier(
        const Eigen::Vector2d &pt0,
        const Eigen::Vector2d &pt1,
        const Eigen::Vector2d &pt2,
        double mu)
    {
        Eigen::Vector2d a, b, c, pt;

        c[0] = 2 * (pt1[0] - pt0[0]);
        c[1] = 2 * (pt1[1] - pt0[1]);
        b[0] = (pt2[0] - pt0[0]) - c[0];
        b[1] = (pt2[1] - pt0[1]) - c[1];

        pt[0] = b[0] * mu * mu + c[0] * mu + pt0[0];
        pt[1] = b[1] * mu * mu + c[1] * mu + pt0[1];

        return (1 - mu) * (1 - mu) * pt0 + 2 * mu * (1 - mu) * pt1 + mu * mu * pt2;
    }

    /**
     * @brief Calculate orientation vector of tangential line at pt to arc formed by given three points
     * Direction of vector is towards next point
     *
     * @param prev_pt Previous point
     * @param pt This point
     * @param next_pt Next point
     * @param is_cusp If this point is cusp point
     * @return Eigen::Vector2d Orientation vector
     */
    Eigen::Vector2d tangentDir(
        Eigen::Vector2d prev_pt,
        Eigen::Vector2d pt,
        Eigen::Vector2d next_pt,
        bool is_cusp)
    {
        Eigen::Vector2d center = findArcCenter(prev_pt, pt, next_pt, is_cusp);
        Eigen::Vector2d d1 = pt - prev_pt,
                        d2 = next_pt - pt;
        if (is_cusp)
        { // mirror the point after cusp point
            d2 = -d2;
            next_pt = pt + d2;
        }
        if (ceres::isinf(center(0)) || ceres::isinf(center(1)))
        {
            // along the straight line
            Eigen::Vector2d dir{next_pt(0) - prev_pt(0), next_pt(1) - prev_pt(1)};
            return dir;
        }
        // perpendicular to line between pt and arc center
        Eigen::Vector2d dir{center(1) - pt(1), pt(0) - center(0)};
        // adjust the direction as pointing to next pt
        if (dir.dot(d2) < 0)
        {
            // direction pointing to prev pt
            dir = -dir;
        }
        return dir;
    }

    /**
     * @brief cf. https://www.tutorialspoint.com/program-for-point-of-intersection-of-two-lines-in-cplusplus#:~:text=Given%20points%20A%20and%20B,on%20X%20and%20Y%20coordinates.
     *
     * @param pt1
     * @param pt2
     * @param yaw1
     * @param yaw2
     * @return Eigen::Vector2d
     */
    Eigen::Vector2d findIntersection(
        Eigen::Vector2d pt1,
        Eigen::Vector2d pt2,
        double yaw1,
        double yaw2)
    {
        if (yaw1 == yaw2)
        {
            return Eigen::Vector2d(std::numeric_limits<double>::infinity(),
                                   std::numeric_limits<double>::infinity());
        }

        Eigen::Vector2d orient1(cos(yaw1), sin(yaw1));
        Eigen::Vector2d orient2(cos(yaw2), sin(yaw2));

        double a = orient1(1);
        double b = -orient1(0);
        double c = a * pt1(0) + b * pt1(1);

        double a1 = -orient2(1);
        double b1 = orient2(0);
        double c1 = a1 * pt2(0) + b1 * pt2(1);

        double det = a * b1 - a1 * b;
        if (det == 0)
        {
            return Eigen::Vector2d(std::numeric_limits<double>::infinity(),
                                   std::numeric_limits<double>::infinity());
        }
        else
        {
            return Eigen::Vector2d((b1 * c - b * c1) / det,
                                   (a * c1 - a1 * c) / det);
        }
    }
} // namespace
#endif // PLANNER_UTILS_H
