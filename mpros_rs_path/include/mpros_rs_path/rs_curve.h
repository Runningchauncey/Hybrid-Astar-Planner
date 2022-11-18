#ifndef RS_CURVE_H
#define RS_CURVE_H

#include <ros/ros.h>
#include "mpros_rs_path/rs_curve_basic.h"
#include "mpros_utils/pathpoint.h"

namespace RS
{
    using Path = std::vector<PathSeg>;
    using PathFuncVec = std::vector<Path (*)(const Point &pt)>;

    /**
     * @brief RSCurve class to generate feasible path from start point to goal point with desired heading
     *
     */
    class RSCurve
    {
    public:
        /**
         * @brief Construct a new RSCurve object
         *
         */
        RSCurve();
        /**
         * @brief Construct a new RSCurve object
         *
         * @param start (x,y,phi)
         * @param end (x,y,phi)
         */
        RSCurve(const Point &start, const Point &end, const double &step_size);
        /**
         * @brief Construct a new RSCurve object
         *
         * @param start (x,y,phi)
         * @param end (x,y,phi)
         * @param r
         */
        RSCurve(const Point &start, const Point &end, const double &r, const double &step_size);
        /**
         * @brief Get the optimal(min. length) path
         *
         * @return Path
         */
        void FindOptimalPath();
        /**
         * @brief Get the length of optimal trajectory
         *
         * @return double
         */
        double GetLength();
        /**
         * @brief Set the minimum radius [m]
         *
         * @param r
         */
        void SetRadius(const double &r);
        /**
         * @brief Set the maximum steering angle
         * 
         * @param angle Input: in rad
         */
        void setSteeringAngle(const double &angle)
        {
            steer_angle_ = angle;
        }
        /**
         * @brief Set the Step Size
         *
         * @param s
         */
        void setStepSize(const double &s)
        {
            step_size_ = s;
        }
        /**
         * @brief Set the Points object
         *
         * @param start
         * @param end
         */
        void SetPoints(const Point &start, const Point &end);
        /**
         * @brief Get the Traj object
         *
         * @return std::vector<std::vector<double>> {x, y, yaw, direction}
         */
        std::vector<std::vector<double>> GetTraj();
        /**
         * @brief Set the penalty weights
         * 
         * @param PEN_GEAR_COST Input: weight for driving direction change
         * @param PEN_BACKWARD_COST Input: weight for driving reversely
         * @param PEN_STEER_CHANGE Input: weight for steering angle change
         * @param PEN_STEER_ANGLE_COST Input: weight for non-zero steering angle
         */
        void setPenalty(double PEN_GEAR_COST,
                        double PEN_BACKWARD_COST,
                        double PEN_STEER_CHANGE,
                        double PEN_STEER_ANGLE_COST);
        /**
         * @brief Set the Points object
         *
         * @param start
         */
        void SetStart(const Point &start);
        /**
         * @brief Set the start point
         *
         * @param x
         * @param y
         * @param yaw
         */
        void SetStart(const double &x, const double &y, const double &yaw, int direction, double steer);
        /**
         * @brief Set the Points object
         *
         * @param end
         */
        void SetEnd(const Point &end);
        /**
         * @brief Set the end point
         *
         * @param x
         * @param y
         * @param yaw
         */
        void SetEnd(const double &x, const double &y, const double &yaw);
        /**
         * @brief Get the maneuver of the path
         *
         * @param path
         * @return std::string
         */
        std::string GetManuever(const Path &path);
        /**
         * @brief Get the optimal path
         *
         * @return Path
         */
        Path getOptimalPath() { return optimal_path_; }
        /**
         * @brief Get the trajectory with curvature, direction info
         *
         */
        std::vector<PathPoint> getTrajectory()
        {
            GenTraj(optimal_path_);
            return traj_with_info_;
        }
        /**
         * @brief Print optimal path maneuver for debug
         *
         */
        void printManeuver()
        {
            ROS_INFO_STREAM("Maneuver of optimal path: \n"
                            << GetManuever(optimal_path_));
        }

    private:
        double step_size_ = 0.2; // distance between two trajectory points
        const double curve_min_num_ = 3;
        std::vector<std::vector<double>> optimal_traj_; // storing path in points(x,y,phi)
        double optimal_len_;
        Point start_pt_, end_pt_;
        double min_radius_;
        double steer_angle_{0};
        Path optimal_path_;
        std::vector<Path> all_paths_;
        std::vector<PathPoint> traj_with_info_;

        int start_direc_{0};
        double start_steer_{0.0};

        // penalties
        // penalizing changing direction
        double PEN_GEAR_COST_ = 0;
        // penalizing reverse driving
        double PEN_BACKWARD_COST_ = 0;
        // penalizing changing steering angle
        double PEN_STEER_CHANGE_ = 0;
        // penalizing not driving straightly
        double PEN_STEER_ANGLE_COST_ = 0;

    protected:
        /**
         * @brief Reverse direction of all segments to reach pt(-x, y, -phi)
         * @param path
         * @return Path
         */
        Path Timeflip(const Path &path);
        /**
         * @brief Reverse steer (LEFT<->RIGHT) of all segments to reach pt(x, -y, -phi)
         *
         * @param path
         * @return Path
         */
        Path Reflect(const Path &path);
        /**
         * @brief Generate new point in coordinates of origin (x1, y1, phi1)
         *
         * @param pt1 origin of new coordinates
         * @param pt2 to be transformed
         * @return Point
         */
        Point NewGoalPoint(Point pt1, Point pt2);
        /**
         * @brief Get length of one path
         *
         * @param path vector of PathSeg
         * @param radius minimal turning radius
         * @return double
         */
        double PathLength(const Path &path);
        /**
         * @brief Get the trajectory of given path
         *
         * @param path vector of PathSeg
         * @param radius minimal turning radius
         * @return
         */
        void GenTraj(const Path &path);
        /**
         * @brief Generate trajectory of a path segment
         *
         * @param seg
         * @return std::vector<std::vector<double>>
         */
        std::vector<std::vector<double>> GenSegTraj(const PathSeg &seg);
        /**
         * @brief reset the object
         *
         */
        void RSClear();
        /**
         * @brief check if exists segment of nan length
         *
         * @param path
         * @return true
         * @return false
         */
        bool noNanSeg(const Path &path);
    };
    /*
    ------------------------------end of class------------------------
    */
    namespace PathFunction
    {
        /**
         * @brief Cal path with word Formula 8.1: CSC (same turns)
         *
         * @param pt (x,y,phi)
         * @return Path
         */
        Path path1(const Point &pt);
        /**
         * @brief Cal path with word Formula 8.2: CSC (opposite turns)
         *
         * @param pt (x,y,phi)
         * @return Path
         */
        Path path2(const Point &pt);
        /**
         * @brief Formula 8.3: C|C|C
         *
         * @param pt (x,y,phi)
         * @return Path
         */
        Path path3(const Point &pt);
        /**
         * @brief Formula 8.4 (1): C|CC
         *
         * @param pt (x,y,phi)
         * @return Path
         */
        Path path4(const Point &pt);
        /**
         * @brief Formula 8.4 (2): CC|C
         *
         * @param pt (x,y,phi)
         * @return Path
         */
        Path path5(const Point &pt);
        /**
         * @brief Formula 8.7: CCu|CuC
         *
         * @param pt (x,y,phi)
         * @return Path
         */
        Path path6(const Point &pt);
        /**
         * @brief Formula 8.8: C|CuCu|C
         *
         * @param pt (x,y,phi)
         * @return Path
         */
        Path path7(const Point &pt);
        /**
         * @brief Formula 8.9 (1): C|C[pi/2]SC
         *
         * @param pt (x,y,phi)
         * @return Path
         */
        Path path8(const Point &pt);
        /**
         * @brief Formula 8.10 (1): C|C[pi/2]SC
         *
         * @param pt (x,y,phi)
         * @return Path
         */
        Path path9(const Point &pt);
        /**
         * @brief Formula 8.9 (2): CSC[pi/2]|C
         *
         * @param pt (x,y,phi)
         * @return Path
         */
        Path path10(const Point &pt);
        /**
         * @brief Formula 8.10 (2): CSC[pi/2]|C
         *
         * @param pt (x,y,phi)
         * @return Path
         */
        Path path11(const Point &pt);
        /**
         * @brief Formula 8.11: C|C[pi/2]SC[pi/2]|C
         *
         * @param pt (x,y,phi)
         * @return Path
         */
        Path path12(const Point &pt);
    }
}

#endif // !RS_CURVE_H