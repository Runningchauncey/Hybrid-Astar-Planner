#include <map>

#include "mpros_rs_path/rs_curve.h"
#include "ros/ros.h"

namespace RS
{
    RSCurve::RSCurve()
    {
        start_pt_ = {.0, .0, .0};
        end_pt_ = {.0, .0, .0};
        min_radius_ = 1.0;
    }

    RSCurve::RSCurve(const Point &start, const Point &end, const double &step_size)
    {
        start_pt_ = start;
        end_pt_ = end;
        min_radius_ = 1.0;
        step_size_ = step_size;
    }

    RSCurve::RSCurve(const Point &start, const Point &end, const double &r, const double &step_size)
    {
        start_pt_ = start;
        end_pt_ = end;
        min_radius_ = r;
        step_size_ = step_size;
    }

    Path RSCurve::Timeflip(const Path &path)
    {
        Path tf_path;
        for (int i = 0; i < path.size(); ++i)
        {
            tf_path.push_back(reverseDirec(path[i]));
        }
        return tf_path;
    }

    Path RSCurve::Reflect(const Path &path)
    {
        Path r_path;
        for (int i = 0; i < path.size(); ++i)
        {
            r_path.push_back(reverseSteer(path[i]));
        }
        return r_path;
    }

    Point RSCurve::NewGoalPoint(Point pt1, Point pt2)
    {
        double dx = pt2.x - pt1.x;
        double dy = pt2.y - pt1.y;
        double new_x = (dx * cos(pt1.phi) + dy * sin(pt1.phi)) / min_radius_;
        double new_y = (-dx * sin(pt1.phi) + dy * cos(pt1.phi)) / min_radius_;
        return Point(new_x, new_y, pt2.phi - pt1.phi);
    }

    double RSCurve::PathLength(const Path &path)
    {
        double length = 0, penalty = 0;
        int last_direc = start_direc_;
        int last_steer = start_steer_;
        
        if (path.size() == 0)
        {
            return 10000;
        }
        for (int i = 0; i < path.size(); ++i)
        {
            auto &seg = path[i];
            // penalty on reversely ddriving
            if (seg.GetDirec() != start_direc_)
            {
                penalty += PEN_GEAR_COST_;
            }
            if (seg.GetDirec() < 0)
            {
                penalty += PEN_BACKWARD_COST_ * seg.GetLength() * min_radius_;
            }
            if (seg.GetSteer() != last_steer)
            {
                penalty += PEN_STEER_CHANGE_ * std::abs(seg.GetSteer() - last_steer) * steer_angle_;
            }
            if (seg.GetSteer() != 0)
            {
                penalty += PEN_STEER_ANGLE_COST_ * std::abs(seg.GetSteer()) * steer_angle_;
            }

            length += seg.GetLength() * min_radius_;

            last_direc = seg.GetDirec();
            last_steer = seg.GetSteer();
        }
        return (length + penalty) * (1.0 + 1e-3); // tie breaker
    }

    bool RSCurve::noNanSeg(const Path &path)
    {
        for (auto &seg : path)
        {
            if (std::isnan(seg.GetLength()))
            {
                return false;
            }
        }
        return true;
    }

    void RSCurve::FindOptimalPath()
    {
        using namespace PathFunction;
        PathFuncVec path_funcs{path1, path2, path3,
                               path4, path5, path6,
                               path7, path8, path9,
                               path10, path11, path12};

        Point goal_pt = NewGoalPoint(start_pt_, end_pt_);
        // use map to organize all paths
        std::multimap<double, Path> path_map;
        Path path;
        double path_len;
        for (int i = 0; i < path_funcs.size(); ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                switch (j)
                {
                case 0:
                    path = path_funcs[i](goal_pt);

                    break;
                case 1:
                    path = Timeflip(path_funcs[i](goal_pt.timeflip()));

                    break;
                case 2:
                    path = Reflect(path_funcs[i](goal_pt.reflect()));

                    break;
                case 3:
                    path = Reflect(Timeflip(path_funcs[i](goal_pt.timeflip().reflect())));

                    break;
                default:
                    break;
                }
                path_len = PathLength(path);
                if (path_len && noNanSeg(path))
                {
                    path_map.insert(std::pair<double, RS::Path>(path_len, path));
                    // for debugging all path functions
                    // std::vector<std::vector<double>> temp_traj = GenTraj(path);
                    // std::vector<double> endpoint = temp_traj[temp_traj.size()-1];
                    // ROS_INFO("path func No. %d terminal point %f %f %f", i,  endpoint[0], endpoint[1], endpoint[2]);
                }
            }
        }

        optimal_len_ = path_map.begin()->first;
        optimal_path_ = path_map.begin()->second;
    }

    void RSCurve::GenTraj(const Path &path)
    {
        traj_with_info_.clear();
        optimal_traj_.clear();
        double curr_x = start_pt_.x, curr_y = start_pt_.y, curr_yaw = start_pt_.phi;
        int steer;
        double dyaw, step, direc;
        double dx, dy;
        int num_pt;
        for (auto &seg : path)
        {
            steer = seg.GetSteer();
            if (steer != 0)
            {
                // sample number for the trajectory
                num_pt = (int)std::fmax(std::ceil(seg.GetLength() * min_radius_ / step_size_), curve_min_num_);
                // yaw difference between two points
                dyaw = seg.GetLength() / num_pt;
                // get the direction of the segment
                direc = seg.GetDirec();
                for (int i = 0; i < num_pt; ++i)
                {
                    // position difference in vehicle frame with vehicle along x axis
                    dx = sin(dyaw) * min_radius_ * direc;
                    dy = (1 - cos(dyaw)) * min_radius_ * steer;
                    // transform to map frame
                    curr_x += dx * cos(curr_yaw) - dy * sin(curr_yaw);
                    curr_y += dx * sin(curr_yaw) + dy * cos(curr_yaw);
                    curr_yaw += dyaw * direc * (double)steer;
                    curr_yaw = regularRad(curr_yaw);
                    // push to path vector
                    optimal_traj_.push_back({curr_x, curr_y, curr_yaw, (double)steer * steer_angle_, direc});
                    // push in form of PathPoint
                    PathPoint point(curr_x, curr_y, curr_yaw);
                    point.curv_ = 1 / min_radius_ * steer;
                    point.direc_ = direc;
                    traj_with_info_.push_back(point);
                }
            }
            else
            {
                direc = seg.GetDirec();
                num_pt =  (int)std::fmax(std::ceil(seg.GetLength() * min_radius_ / step_size_), curve_min_num_);
                step = seg.GetLength() * min_radius_ / num_pt;
                for (int i = 0; i < num_pt; ++i)
                {
                    curr_x += step * cos(curr_yaw) * direc;
                    curr_y += step * sin(curr_yaw) * direc;
                    optimal_traj_.push_back({curr_x, curr_y, curr_yaw, (double)steer * steer_angle_, direc});
                    // push in form of Pathpoint
                    PathPoint point(curr_x, curr_y, curr_yaw);
                    point.curv_ = 0;
                    point.direc_ = direc;
                    traj_with_info_.push_back(point);
                }
            }
        }
    }

    std::vector<std::vector<double>> RSCurve::GenSegTraj(const PathSeg &seg)
    {
        traj_with_info_.clear();
        std::vector<std::vector<double>> traj;
        double curr_x = .0, curr_y = .0, curr_yaw = .0;
        traj.push_back({curr_x, curr_y, curr_yaw});
        double dyaw, step, direc, steer;
        int num_pt;
        steer = seg.GetSteer();
        if (steer != 0)
        {
            num_pt = (int)std::fmax(std::ceil(seg.GetLength() * min_radius_ / step_size_), curve_min_num_);
            dyaw = steer * std::abs(seg.GetLength()) / num_pt;
            step = std::abs(dyaw) * min_radius_;
            direc = seg.GetDirec();
            for (int i = 0; i < num_pt; ++i)
            {
                // update waypoint point
                curr_x += step * cos(curr_yaw) * direc;
                curr_y += step * sin(curr_yaw) * direc;
                curr_yaw += dyaw * direc;
                curr_yaw = regularRad(curr_yaw);
                // push to waypoint vector
                traj.push_back({curr_x, curr_y, curr_yaw, direc});
                // push in form of PathPoint
                PathPoint point(curr_x, curr_y, curr_yaw);
                point.curv_ = 1 / min_radius_ * steer * direc;
                point.direc_ = direc;
                traj_with_info_.push_back(point);
            }
        }
        else
        {
            direc = seg.GetDirec();
            step = step_size_;
            num_pt = std::ceil(seg.GetLength() / step_size_);
            for (int i = 0; i < num_pt; ++i)
            {
                curr_x += step * cos(curr_yaw) * direc;
                curr_y += step * sin(curr_yaw) * direc;
                traj.push_back({curr_x, curr_y, curr_yaw, direc});
                // push in form of Pathpoint
                PathPoint point(curr_x, curr_y, curr_yaw);
                point.curv_ = 0;
                point.direc_ = direc;
                traj_with_info_.push_back(point);
            }
        }
        return traj;
    }

    double RSCurve::GetLength()
    {
        return optimal_len_;
    }

    std::vector<std::vector<double>> RSCurve::GetTraj()
    {
        GenTraj(optimal_path_);
        return optimal_traj_;
    }

    std::string RSCurve::GetManuever(const Path &path)
    {
        std::string manuever;
        for (auto &seg : path)
        {
            manuever += seg.GetManuever() + "\n";
        }
        return manuever;
    }

    void RSCurve::SetRadius(const double &r)
    {
        min_radius_ = r;
    }

    void RSCurve::SetPoints(const Point &start, const Point &end)
    {
        start_pt_ = start;
        end_pt_ = end;
    }

    void RSCurve::RSClear()
    {
        optimal_traj_.clear();
        optimal_path_.clear();
        all_paths_.clear();
    }

    void RSCurve::setPenalty(double PEN_GEAR_COST,
                             double PEN_BACKWARD_COST,
                             double PEN_STEER_CHANGE,
                             double PEN_STEER_ANGLE_COST)
    {
        PEN_GEAR_COST_ = PEN_GEAR_COST;
        PEN_BACKWARD_COST_ = PEN_BACKWARD_COST;
        PEN_STEER_CHANGE_ = PEN_STEER_CHANGE;
        PEN_STEER_ANGLE_COST_ = PEN_STEER_ANGLE_COST;
    }

    void RSCurve::SetStart(const Point &start)
    {
        RSClear();
        start_pt_ = start;
    }

    void RSCurve::SetStart(const double &x, const double &y, const double &yaw, int direction, double steer)
    {
        RSClear();
        start_pt_ = Point(x, y, yaw);
        start_direc_ = direction;
        start_steer_ = steer;
    }

    void RSCurve::SetEnd(const Point &end)
    {
        RSClear();
        end_pt_ = end;
    }

    void RSCurve::SetEnd(const double &x, const double &y, const double &yaw)
    {
        RSClear();
        end_pt_ = Point(x, y, yaw);
    }

    namespace PathFunction
    {
        Path path1(const Point &pt)
        {
            Path path;
            PolarCoord polar(pt.x - sin(pt.phi), pt.y - 1 + cos(pt.phi));
            double u = polar.dist;
            double t = polar.theta;
            double v = regularRad(pt.phi - t);

            path.push_back(PathSeg(t, LEFT, FORWARD));
            path.push_back(PathSeg(u, STRAIGHT, FORWARD));
            path.push_back(PathSeg(v, LEFT, FORWARD));

            return path;
        }

        Path path2(const Point &pt)
        {
            Path path;

            PolarCoord polar(pt.x + sin(pt.phi), pt.y - 1 - cos(pt.phi));

            if (polar.dist * polar.dist >= 4)
            {
                double u = sqrt(polar.dist * polar.dist - 4);
                double t = regularRad(polar.theta + std::atan2(2, u));
                double v = regularRad(t - pt.phi);

                path.push_back(PathSeg(t, LEFT, FORWARD));
                path.push_back(PathSeg(u, STRAIGHT, FORWARD));
                path.push_back(PathSeg(v, RIGHT, FORWARD));
            }

            return path;
        }

        Path path3(const Point &pt)
        {
            Path path;
            double epsi = pt.x - sin(pt.phi), enta = pt.y - 1 + cos(pt.phi);
            PolarCoord polar(epsi, enta);

            if (polar.dist <= 4)
            {
                double A = acos(polar.dist / 4);
                double u = regularRad(M_PI - 2 * A);
                double t = regularRad(M_PI_2 + A + polar.theta);
                double v = regularRad(pt.phi - t - u);

                path.push_back(PathSeg(t, LEFT, FORWARD));
                path.push_back(PathSeg(u, RIGHT, BACKWARD));
                path.push_back(PathSeg(v, LEFT, FORWARD));
            }

            return path;
        }

        Path path4(const Point &pt)
        {
            Path path;
            PolarCoord polar(pt.x - sin(pt.phi), pt.y - 1 + cos(pt.phi));

            if (polar.dist <= 4)
            {
                double A = acos(polar.dist / 4);
                double u = regularRad(M_PI - 2 * A);
                double t = asin(polar.theta + M_PI_2 + A);
                double v = regularRad(t + u - pt.phi);

                path.push_back(PathSeg(t, LEFT, FORWARD));
                path.push_back(PathSeg(u, RIGHT, BACKWARD));
                path.push_back(PathSeg(v, LEFT, BACKWARD));
            }

            return path;
        }

        Path path5(const Point &pt)
        {
            Path path;
            PolarCoord polar(pt.x - sin(pt.phi), pt.y - 1 + cos(pt.phi));

            if (polar.dist <= 4)
            {
                double u = acos(1 - polar.dist * polar.dist / 8);
                double A = asin(2 * sin(u) / polar.dist);
                double t = regularRad(polar.theta + M_PI_2 - A);
                double v = regularRad(t - u - pt.phi);

                path.push_back(PathSeg(t, LEFT, FORWARD));
                path.push_back(PathSeg(u, RIGHT, FORWARD));
                path.push_back(PathSeg(v, LEFT, BACKWARD));
            }

            return path;
        }

        Path path6(const Point &pt)
        {
            Path path;
            PolarCoord polar(pt.x + sin(pt.phi), pt.y - 1 - cos(pt.phi));

            if (polar.dist <= 4)
            {
                double A, t, u, v;
                if (polar.dist <= 2)
                {
                    A = acos((polar.dist + 2) / 4);
                    t = regularRad(polar.theta + M_PI_2 + A);
                    u = regularRad(A);
                    v = regularRad(pt.phi - t + 2 * u);
                }
                else
                {
                    A = acos((polar.dist - 2) / 4);
                    t = regularRad(polar.theta + M_PI_2 - A);
                    u = regularRad(M_PI - A);
                    v = regularRad(pt.phi - t + 2 * u);
                }
                path.push_back(PathSeg(t, LEFT, FORWARD));
                path.push_back(PathSeg(u, RIGHT, FORWARD));
                path.push_back(PathSeg(u, LEFT, BACKWARD));
                path.push_back(PathSeg(v, RIGHT, BACKWARD));
            }

            return path;
        }

        Path path7(const Point &pt)
        {
            Path path;
            PolarCoord polar(pt.x + sin(pt.phi), pt.y - 1 - cos(pt.phi));

            double u1 = (20 - polar.dist * polar.dist) / 16;

            if (polar.dist <= 6 && u1 >= 0 && u1 <= 1)
            {
                double u = acos(u1);
                double A = asin(2 * sin(u) / polar.dist);
                double t = regularRad(polar.theta + M_PI_2 + A);
                double v = regularRad(t - pt.phi);

                path.push_back(PathSeg(t, LEFT, FORWARD));
                path.push_back(PathSeg(u, RIGHT, BACKWARD));
                path.push_back(PathSeg(u, LEFT, BACKWARD));
                path.push_back(PathSeg(v, RIGHT, FORWARD));
            }
            return path;
        }

        Path path8(const Point &pt)
        {
            Path path;
            PolarCoord polar(pt.x - sin(pt.phi), pt.y - 1 + cos(pt.phi));

            if (polar.dist >= 2)
            {
                double u = sqrt(polar.dist * polar.dist - 4) - 2;
                double A = atan2(2, u + 2);
                double t = regularRad(polar.theta + M_PI_2 + A);
                double v = regularRad(t - pt.phi + M_PI_2);

                path.push_back(PathSeg(t, LEFT, FORWARD));
                path.push_back(PathSeg(M_PI_2, RIGHT, BACKWARD));
                path.push_back(PathSeg(u, STRAIGHT, BACKWARD));
                path.push_back(PathSeg(v, LEFT, BACKWARD));
            }
            return path;
        }

        Path path9(const Point &pt)
        {
            Path path;
            PolarCoord polar(pt.x + sin(pt.phi), pt.y - 1 - cos(pt.phi));

            if (polar.dist >= 2)
            {
                double t = regularRad(polar.theta + M_PI_2);
                double u = polar.dist - 2;
                double v = regularRad(pt.phi - t - M_PI_2);

                path.push_back(PathSeg(t, LEFT, FORWARD));
                path.push_back(PathSeg(M_PI_2, RIGHT, BACKWARD));
                path.push_back(PathSeg(u, STRAIGHT, BACKWARD));
                path.push_back(PathSeg(v, RIGHT, BACKWARD));
            }
            return path;
        }

        Path path10(const Point &pt)
        {
            Path path;
            PolarCoord polar(pt.x - sin(pt.phi), pt.y - 1 + cos(pt.phi));

            if (polar.dist >= 2)
            {
                double u = sqrt(polar.dist * polar.dist - 4) - 2;
                double A = atan2(u + 2, 2);
                double t = regularRad(polar.theta + M_PI_2 - A);
                double v = regularRad(t - pt.phi - M_PI_2);

                path.push_back(PathSeg(t, LEFT, FORWARD));
                path.push_back(PathSeg(u, STRAIGHT, FORWARD));
                path.push_back(PathSeg(M_PI_2, RIGHT, FORWARD));
                path.push_back(PathSeg(v, LEFT, BACKWARD));
            }
            return path;
        }

        Path path11(const Point &pt)
        {
            Path path;
            PolarCoord polar(pt.x + sin(pt.phi), pt.y - 1 - cos(pt.phi));

            if (polar.dist >= 2)
            {
                double u = polar.dist - 2;
                double t = regularRad(polar.theta);
                double v = regularRad(pt.phi - t - M_PI_2);

                path.push_back(PathSeg(t, LEFT, FORWARD));
                path.push_back(PathSeg(u, STRAIGHT, FORWARD));
                path.push_back(PathSeg(M_PI_2, LEFT, FORWARD));
                path.push_back(PathSeg(v, RIGHT, BACKWARD));
            }
            return path;
        }

        Path path12(const Point &pt)
        {
            Path path;
            PolarCoord polar(pt.x + sin(pt.phi), pt.y - 1 - cos(pt.phi));

            if (polar.dist >= 4)
            {
                double u = sqrt(polar.dist * polar.dist - 4) - 4;
                double A = atan2(2, u + 4);
                double t = regularRad(polar.theta + M_PI_2 + A);
                double v = regularRad(t - pt.phi);

                path.push_back(PathSeg(t, LEFT, FORWARD));
                path.push_back(PathSeg(M_PI_2, RIGHT, BACKWARD));
                path.push_back(PathSeg(u, STRAIGHT, BACKWARD));
                path.push_back(PathSeg(M_PI_2, LEFT, BACKWARD));
                path.push_back(PathSeg(v, RIGHT, FORWARD));
            }
            return path;
        }
    } // namespace PathFunction
} // namespace RS