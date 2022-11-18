#ifndef SMOOTHER_PARAMS_H
#define SMOOTHER_PARAMS_H

#include <vector>
#include <string>
#include <map>

#include <ros/ros.h>
#include <ceres/ceres.h>

namespace smoother
{
    struct SmootherParams
    {
        SmootherParams() = default;

        void config(ros::NodeHandle nh)
        {
            std::string prefix = "/mpros/smoother/";

            nh.param<double>(prefix + "smooth_weight", w_smooth, 10.0);
            nh.param<double>(prefix + "curv_weight", w_curvature, 10.0);
            nh.param<double>(prefix + "curv_first_deriv_weight", w_curv_deriv_1, 0.0);
            nh.param<double>(prefix + "curv_second_deriv_weight", w_curv_deriv_2, 0.0);
            nh.param<double>(prefix + "dist_weight", w_dist, 0.1);
            nh.param<double>(prefix + "cost_weight", w_cost, 0.015);
            nh.param<double>(prefix + "cusp_weight", cusp_costmap_weight, 3.0);
            nh.param<double>(prefix + "cusp_zone_length", cusp_zone_length, 2.5);
            nh.param<double>(prefix + "max_solver_time_in_seconds", max_time, 10.0);
            nh.param<bool>(prefix + "enable_smoother", enable, true);

            prefix = "/mpros/vehicle/";
            nh.param<double>(prefix + "max_curvature", max_curvature, 0.4);
            nh.param<double>(prefix + "max_curv_deriv", max_curv_deriv, 0.4);
            ROS_INFO("configured smoother param");
        }

        bool enable;
        double w_smooth{0.0};    // smoothness weight
        double w_curvature{0.0}; // curvature weight
        double w_curv_deriv_1{0.0}; // curvature 1st derivative weight
        double w_curv_deriv_2{0.0}; // curvature 2nd derivative weight
        double w_dist{0.0};      // distance to original weight
        double w_cost{0.0};      // voronoi cost weight
        double cusp_costmap_weight{0.0};
        double cusp_zone_length{0.0};
        double max_curvature{0.0};
        double max_curv_deriv{0.0};
        double max_time{1.0};
    };

    struct OptimizerParams
    {
        OptimizerParams() = default;

        void config(ros::NodeHandle nh)
        {
            std::string prefix = "/mpros/optimizer/";

            nh.param<bool>(prefix + "debug", debug, true);
            nh.param<std::string>(prefix + "minimizer", minimizer_type, "LINE_SEARCH");
            nh.param<std::string>(prefix + "linear_solver", linear_solver_type, "SPARSE_NORMAL_CHOLESKY");
            nh.param<int>(prefix + "max_iterations", max_iterations, 100);
            nh.param<double>(prefix + "parameter_tolerance", param_tol, 1.0e-15);
            nh.param<double>(prefix + "function_tolerance", fn_tol, 1.0e-7);
            nh.param<double>(prefix + "gradient_tolerance", gradient_tol, 1.0e-10);
        }

        ceres::LinearSolverType solver() const
        {
            return solver_types.at(linear_solver_type);
        }

        const std::map<std::string, ceres::LinearSolverType> solver_types = {
            {"DENSE_QR", ceres::DENSE_QR},
            {"SPARSE_NORMAL_CHOLESKY", ceres::SPARSE_NORMAL_CHOLESKY}};
        
        const std::map<std::string, ceres::MinimizerType> minimizer_types = {
            {"LINE_SEARCH", ceres::LINE_SEARCH},
            {"TRUST_REGION", ceres::TRUST_REGION}};

        bool debug;
        std::string linear_solver_type;
        std::string minimizer_type;
        int max_iterations; // Ceres default: 50

        double param_tol;    // Ceres default: 1e-8
        double fn_tol;       // Ceres default: 1e-6
        double gradient_tol; // Ceres default: 1e-10
    };
}

#endif // SMOOTHER_PARAMS_H
