#ifndef SMOOTHER_WRAPPER_H
#define SMOOTHER_WRAPPER_H

#include <vector>
#include <memory>

#include "mpros_smoother/smoother.h"
#include "mpros_smoother/smoother_params.h"

#include "ros/ros.h"
#include "tf2/utils.h"
#include "mpros_navigation_map/nav_map.h"
#include "mpros_utils/pathpoint.h"

namespace smoother
{
    class SmootherWrapper
    {
    public:
        SmootherWrapper() = default;

        ~SmootherWrapper()
        {
        }
        /**
         * @brief Configure the smoother by getting parameters from ROS Server
         *
         * @param nh
         */
        void config(
            ros::NodeHandle &nh);

        /**
         * @brief Smooth the given path         *
         * @param map Input: map pointer
         * @param path Input: path to be optimized, will be updated
         * @param max_time Input: maximal solving time
         * @return true optimization successful
         * @return false optimization failed
         */
        bool smooth(
            const NavMap *map,
            std::vector<PathPoint> &path,
            const int max_time);
        // params for smoother
        SmootherParams smoother_params_;

    protected:
        // ptr to smoother
        std::unique_ptr<Smoother> smoother_;
        // params for setting ceres optimizer
        OptimizerParams optimizer_params_;
    };
}

#endif // SMOOTHER_WRAPPER_H
