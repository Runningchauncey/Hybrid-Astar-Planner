#ifndef MPROS_SMOOTHER_H
#define MPROS_SMOOTHER_H

#include <iostream>
#include <vector>
#include <deque>
#include <memory>
#include <limits>

#include "ceres/ceres.h"
#include "ceres/cubic_interpolation.h"
#include "Eigen/Core"

#include "mpros_navigation_map/nav_map.h"
#include "mpros_smoother/smoother_params.h"
#include "mpros_smoother/smoother_cost_function.h"
#include "mpros_utils/planner_utils.h"
#include "mpros_utils/pathpoint.h"

namespace smoother
{
    class Smoother
    {
    public:
        Smoother() = default;

        ~Smoother() {}

        /**
         * @brief Set parameters for optimizer
         *
         * @param params
         */
        void config(const OptimizerParams &params)
        {
            debug_ = params.debug;

            options_.linear_solver_type = params.solver_types.at(params.linear_solver_type);
            options_.max_num_iterations = params.max_iterations;
            options_.function_tolerance = params.fn_tol;
            options_.gradient_tolerance = params.gradient_tol;
            options_.parameter_tolerance = params.param_tol;
            options_.minimizer_type = params.minimizer_types.at(params.minimizer_type);

            // set logs
            if (debug_)
            {
                options_.minimizer_progress_to_stdout = true;
            }
        }
        /**
         * @brief Smooth the path
         *
         * @param path Input: path to be optimized, will be updated
         * @param map Input: map pointer
         * @param params Input: smoother params
         * @return true
         * @return false
         */
        bool smooth(
            std::vector<Eigen::Vector3d> &path,
            const NavMap *map,
            const SmootherParams &params)
        {
            if (path.size() < 3)
            {
                throw std::runtime_error("[Smoother]: Path must have at least 3 points.");
            }

            options_.max_solver_time_in_seconds = params.max_time;
            options_.logging_type = ceres::LoggingType::SILENT;

            ceres::Problem problem;
            std::vector<Eigen::Vector3d> path_optim;
            std::vector<bool> optimized;

            if (setProblem(path, map, params, problem, path_optim, optimized))
            {
                // solve the problem
                ceres::Solver::Summary summary;
                ceres::Solve(options_, &problem, &summary);

                if (debug_)
                {
                    ROS_INFO("[Smoother]: %s", summary.FullReport().c_str());
                }
                if (!summary.IsSolutionUsable() || summary.initial_cost < summary.final_cost)
                {
                    // failed
                    return false;
                }
            }
            else
            {
                ROS_INFO("[Smoother]: Set problem failed.");
                return false;
            }

            path = path_optim;
            return true;
        }

    private:
        /**
         * @brief Set the optimization problem by generating cost function for each path point (excluding both ends)
         *
         * @param path Input: path to be optimized, will be updated
         * @param map Input: map pointer
         * @param params Input: smoother params
         * @param problem Output: optimization problem
         * @param path_optim Output: optimized path
         * @param optimized Output: flag for each path point if it is optimized
         * @return true successful
         * @return false failed
         */
        bool setProblem(
            const std::vector<Eigen::Vector3d> &path,
            const NavMap *map,
            const SmootherParams &params,
            ceres::Problem &problem,
            std::vector<Eigen::Vector3d> &path_optim,
            std::vector<bool> &optimized)
        {
            if (path.size() < 3)
            {
                ROS_ERROR("Path should have length larger than 3 to set the problem.");
                throw std::runtime_error("Path too short to set opt problem");
            }

            // init outputs
            path_optim = path;

            // helper index
            int prev_i = 0;
            int next_i = 2;

            // direction differs from previous makes this point a cusp point
            bool prev_is_cusp = path_optim[prev_i][2] != path_optim[prev_i + 1][2];
            bool next_is_cusp; // to be determined in the loop

            for (int i = 1; i < path_optim.size() - 1; ++i)
            {
                // update next points index
                next_i = i + 1;
                // refer to points
                auto &pt = path_optim[i];
                auto &next_pt = path_optim[next_i];

                // check if point i is cusp point
                bool is_cusp = pt[2] != next_pt[2];
                // set cost function for point i
                SmootherCostFunction *cost_function =
                    new SmootherCostFunction(
                        path[i].head(2),
                        is_cusp,
                        params);
                // add to problem
                problem.AddResidualBlock(
                    cost_function->AutoDiff(),
                    NULL,
                    path_optim[prev_i].data(),
                    path_optim[i].data(),
                    path_optim[next_i].data());

                // update var to slide window forwards
                // update helper index
                prev_i = i;
                // update previous is cusp
                prev_is_cusp = is_cusp;
            }

            // set constant parameters
            // initial pose
            problem.SetParameterBlockConstant(path_optim.front().data());
            // second pose to keep orientation for initial pose
            problem.SetParameterBlockConstant(path_optim[1].data());
            // last second pose to keep orientation for target pose
            problem.SetParameterBlockConstant(path_optim[path_optim.size() - 2].data());
            // target pose
            problem.SetParameterBlockConstant(path_optim.back().data());

            return true;
        }
        // flag for debug logs
        bool debug_;
        // solver settings
        ceres::Solver::Options options_;
    };
}

#endif // MPROS_SMOOTHER_H