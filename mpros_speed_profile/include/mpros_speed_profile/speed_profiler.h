#ifndef SPEED_PROFILER_H
#define SPEED_PROFILER_H

#include <iostream>
#include <vector>
#include <string>

#include "mpros_utils/pathpoint.h"
#include "mpros_utils/planner_utils.h"

/**
 * @brief a segment of trajectory consists of points of low curvature
 *
 */
struct TrajSegment
{
    TrajSegment(const std::vector<PathPoint> &pathseg)
        : path(pathseg)
    {
    }
    std::vector<PathPoint> path; // path points of this segment
    std::string name;            // "circular", "non-circular"
    double curvature;            // max curvature of this path
    double vs, as;               // profile at start point
    double vg, ag;               // profile at goal point
    double s0;                   // distance of start point of this seg to start point of whole traj
    double length;               // length of this segment
    // to be determined var
    double ts, tg; // start time, end time
    double vmax;   // maximal reachable velocity of this segment
    int direction; // direction of this segment
};

class SpeedProfiler
{
public:
    /**
     * @brief Construct a new Speed Profiler object
     *
     */
    SpeedProfiler() = default;
    /**
     * @brief configure the profiler with inputs
     *
     */
    void config(ros::NodeHandle &nh)
    {
        std::string prefix = "/mpros/profile/";

        nh.param<bool>(prefix + "debug", debug_, false);
        nh.param<double>(prefix + "jerk_limit", j_lim_, 0.5);
        nh.param<double>(prefix + "longitudinal_acc_limit", a_x_lim_, 1.0);
        nh.param<double>(prefix + "lateral_acc_limit", a_y_lim_, 0.5);
        nh.param<double>(prefix + "velocity_limit", v_lim_, 5.0);
        nh.param<double>(prefix + "time_step", dt_, 0.2);
        nh.param<double>(prefix + "start_time", t_start_, 0.0);
        nh.param<double>(prefix + "start_speed", v_start_, 0.0);
        nh.param<double>(prefix + "end_speed", v_end_, 0.0);

        prefix = "/mpros/vehicle/";
        nh.param<double>(prefix + "max_curvature", curv_lim_, 0.0);
    }

    /**
     * @brief call this to generate profile
     *
     * @param path
     */
    void makeProfile(std::vector<PathPoint> &path);

    /**
     * @brief Get the Trajectory object
     *
     * @return std::vector<PathPoint>
     */
    inline std::vector<PathPoint> getTrajectory()
    {
        return trajectory_;
    }

private:

    /**
     * @brief segment trajectory section according to curvature
     *
     * @param path
     */
    void sectionInSegment(const std::vector<PathPoint> &path);

    /**
     * @brief calculate acceleration after dt_
     *
     * @param a0
     * @param j
     * @param dt_
     * @return double
     */
    inline double calAccel(double a0, double j, double dt_)
    {
        return a0 + j * dt_;
    }

    /**
     * @brief calculate velocity after dt_
     *
     * @param v0
     * @param a0
     * @param j
     * @param dt_
     * @return double
     */
    inline double calVelocity(double v0, double a0, double j, double dt_)
    {
        return v0 + a0 * dt_ + 1.0 / 2.0 * j * dt_ * dt_;
    }

    /**
     * @brief calculate distance after dt_
     *
     * @param s0
     * @param v0
     * @param a0
     * @param j
     * @param dt_
     * @return double
     */
    inline double calDist(double s0, double v0, double a0, double j, double dt_)
    {
        return s0 + v0 * dt_ + 1.0 / 2.0 * a0 * dt_ * dt_ + 1.0 / 6.0 * j * dt_ * dt_ * dt_;
    }

    /**
     * @brief find the path point after the point to be interpolated
     *
     * @param dist
     * @param path
     * @return int
     */
    inline int findPathPoint(double dist, const std::vector<PathPoint> &path)
    {
        for (int i = 1; i < path.size(); ++i)
        {
            if (path[i].dist_ > dist)
            {
                return i;
            }
        }
        return path.size() - 1;
    }

    /**
     * @brief set the S-Curve for given segment
     *
     * @param idx
     */
    void setSCurve(int idx);

    /**
     * @brief interpolate trajectory within given path
     *
     * @param path
     * @param tstart
     * @param tend
     * @param j
     * @param a0
     * @param v0
     * @param dist0
     */
    void generateTrajSeg(const std::vector<PathPoint> &path, double tstart, double tend, double j, double a0, double v0, double dist0);

    /**
     * @brief interpolate path point between given points
     *
     * @param prev_pt
     * @param next_pt
     * @param dist location of to be interpolated point
     * @return PathPoint
     */
    bool pathPointInterpolate(const PathPoint &prev_pt, const PathPoint &next_pt, double dist, PathPoint &pt);

    /**
     * @brief reset
     *
     */
    void reset();

    /**
     * @brief calculate minimal brake dist with initial velocity of curvature velocity limit
     *
     */
    inline double calMinBrakeDist(double vs, double vg)
    {
        double dt = a_x_lim_ / j_lim_ * 2 + (vs - vg - a_x_lim_ * a_x_lim_ / j_lim_) / a_x_lim_;
        return dt * (vs + vg) / 2 + vg * dt;
    }

private:
    /* configuration */
    bool debug_ = false;
    double j_lim_ = 0.5;    // maximal jerk
    double a_x_lim_ = 1;    // maximal longitudinal acceleration
    double a_y_lim_ = 0.5;  // maximal lateral acceleration
    double v_lim_ = 5;      // maximal velocity
    double curv_lim_ = 0.1; // maximal curvature
    double dt_ = 0.2;       // time granuality
    double t_start_ = 0;    // timestamp of first path point
    double t_end_ = 0;      // timestamp of last path point
    double v_start_ = 0;    // start speed of whole trajectory
    double v_end_ = 0;      // end speed of whole trajectory

    // helper value
    double v_lim_cir_ = std::sqrt(a_y_lim_ / curv_lim_); // velocity limit for circular path with large curvature
    double dt_reach_ax_lim_ = a_x_lim_ / j_lim_;         // time required to reach ax limit from a=0
    double min_brake_dist_;                              // lower bound of solely braking segment
    double direction_;                                   // direction of a section

    // describing the trajectory
    double path_len_;

    // containers
    // container for all segments
    std::vector<TrajSegment> segment_vec_;
    // index of divided sections according to cusp
    std::vector<int> section_begin_idx_, section_end_idx_;

    // interpolated trajectory
    std::vector<PathPoint> trajectory_;
};

#endif // SPEED_PROFILER_H