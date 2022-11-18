#ifndef SMOOTHER_COST_FUNCTION_H
#define SMOOTHER_COST_FUNCTION_H

#include <vector>
#include <iostream>
#include <memory>

#include "ceres/ceres.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "mpros_smoother/smoother_params.h"
#include "mpros_utils/planner_utils.h"

#define EPSILON (double)1e-5

namespace smoother
{
    class SmootherCostFunction
    {
    public:
        /**
         * @brief Construct a new cost function
         *
         * @param original_pos Input: Original position of this point
         * @param is_cusp Input: if this point is a cusp point
         * @param params Input: smoother params
         */
        SmootherCostFunction(
            const Eigen::Vector2d &original_pos,
            bool is_cusp,
            const SmootherParams &params)
            : original_pos_(original_pos),
              is_cusp_(is_cusp),
              params_(params)
        {
        }
        /**
         * @brief Automatically differentiate cost function
         *
         * @return ceres::CostFunction*
         */
        ceres::CostFunction *AutoDiff()
        {
            return new ceres::AutoDiffCostFunction<SmootherCostFunction, 3, 2, 2, 2>(this);
        }
        /**
         * @brief Evaluate cost function value
         *
         * @tparam T Datatype of point
         * @param prev_pt Input: previous point
         * @param pt Input: point
         * @param next_pt Input: next point
         * @param pt_residual Output: value
         * @return true Evaluated successfully
         * @return false Failed
         */
        template <typename T>
        bool operator()(
            const T *const prev_pt,
            const T *const pt,
            const T *const next_pt,
            T *pt_residual) const
        {
            // map input points mapped to vector with size of 2: only location
            Eigen::Map<const Eigen::Matrix<T, 2, 1>> xi(pt);
            Eigen::Map<const Eigen::Matrix<T, 2, 1>> xi_prev(prev_pt);
            Eigen::Map<const Eigen::Matrix<T, 2, 1>> xi_next(next_pt);
            // map residual
            Eigen::Map<Eigen::Matrix<T, 3, 1>> residual(pt_residual);
            residual.setZero();

            // add residuals
            // smoothness term
            addSmoothnessResidual<T>(params_.w_smooth, xi_prev, xi, xi_next, residual(0));
            // // curvature term
            addCurvatureResidual<T>(params_.w_curvature, xi, xi_prev, xi_next, residual(1));
            // // distance term
            addDistanceResidual<T>(params_.w_dist, xi, original_pos_.template cast<T>(), residual(2));

            return true;
        }

    protected:
        /**
         * @brief Evaluate cost of smoothness term
         *
         * @tparam T Datatype of point
         * @param weight Input: cost weight
         * @param prev_pt Input: previous point
         * @param pt Input: this point
         * @param next_pt Input: next point
         * @param r Output: accumulated cost value
         */
        template <typename T>
        inline void addSmoothnessResidual(
            const double &weight,
            const Eigen::Matrix<T, 2, 1> &prev_pt,
            const Eigen::Matrix<T, 2, 1> &pt,
            const Eigen::Matrix<T, 2, 1> &next_pt,
            T &r) const
        {
            Eigen::Matrix<T, 2, 1> d_prev = pt - prev_pt;
            Eigen::Matrix<T, 2, 1> d_next = next_pt - pt;
            // mirror the previous segment if this point is cusp point
            r += (T)weight * (d_next - d_prev * (is_cusp_ ? (T)-1 : (T)1)).squaredNorm();
        }

        /**
         * @brief Evaluate cost of curvature term
         * 
         * @tparam T Datatype of point
         * @param weight Input: cost weight
         * @param prev_pt Input: previous point
         * @param pt Input: this point
         * @param next_pt Input: next point
         * @param r Output: accumulated cost value
         */
        template <typename T>
        inline void addCurvatureResidual(
            const double &weight,
            const Eigen::Matrix<T, 2, 1> &pt,
            const Eigen::Matrix<T, 2, 1> &prev_pt,
            const Eigen::Matrix<T, 2, 1> &next_pt,
            T &r) const
        {
            T curv;
            if (is_cusp_)
            {
                Eigen::Matrix<T, 2, 1> new_next_pt = pt + pt - next_pt;
                curv = mpros_utils::calCurvature(prev_pt, pt, new_next_pt);
            }
            else
            {
                curv = mpros_utils::calCurvature(prev_pt, pt, next_pt);
            }
            T diff_ki_kmax = ceres::abs(curv) - (T)params_.max_curvature;

            // Only valid for curvature over the upper bound
            if (diff_ki_kmax < (T)EPSILON)
            {
                return;
            }

            r += (T)weight * diff_ki_kmax * diff_ki_kmax;
        }

        /**
         * @brief Evaluate cost of distance term
         * 
         * @tparam T Datatype of point
         * @param weight Input: cost weight
         * @param pt Input: this point
         * @param pt_originInput:  original position of this point
         * @param r Output: accumulated value
         */
        template <typename T>
        inline void addDistanceResidual(
            const double &weight,
            const Eigen::Matrix<T, 2, 1> &pt,
            const Eigen::Matrix<T, 2, 1> &pt_origin,
            T &r) const
        {
            r += (T)weight * (pt_origin - pt).squaredNorm();
        }

    private:
        // original position of point to be optimized
        const Eigen::Vector2d original_pos_;
        // flag for cusp point
        bool is_cusp_;
        // parameters
        SmootherParams params_;
    };
}

#endif // SMOOTHER_COST_FUNCTION_H