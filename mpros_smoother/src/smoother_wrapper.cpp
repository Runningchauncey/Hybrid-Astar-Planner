#include "mpros_smoother/smoother_wrapper.h"

#include <Eigen/Core>

namespace smoother
{
    void SmootherWrapper::config(ros::NodeHandle &nh)
    {
        if (!nh.ok())
        {
            throw std::runtime_error("[Smoother]: Node died while configuring");
        }

        smoother_ = std::make_unique<Smoother>();

        optimizer_params_.config(nh);
        smoother_params_.config(nh);
        smoother_->config(optimizer_params_);
    }

    bool SmootherWrapper::smooth(
        const NavMap *map,
        std::vector<PathPoint> &path,
        const int max_time)
    {
        // directly return when disabled
        if(!smoother_params_.enable)
        {
            return true;
        }
        // populate smoother input with (x, y, forward/reverse direction)
        std::vector<Eigen::Vector3d> path_optim;
        path_optim.reserve(path.size());

        for (int i = 0; i < path.size(); ++i)
        {
            PathPoint &pt = path[i];
            path_optim.emplace_back(pt.getX(), pt.getY(), pt.direc_);
        }

        smoother_params_.max_time = max_time;
        if (!smoother_->smooth(path_optim, map, smoother_params_))
        {
            ROS_WARN("[Smoother]: failed to smooth the path.");
            return false;
        }

        // populate final path
        Eigen::Vector2d orient;
        double yaw;
        for (int i = 0; i < path_optim.size(); ++i)
        {
            yaw = path[i].getYaw();
            if (i != 0 && i != path_optim.size() - 1)
            {
                // orientation vector
                orient = mpros_utils::tangentDir(path_optim[i - 1].head(2),
                                    path_optim[i].head(2),
                                    path_optim[i + 1].head(2),
                                    path[i].is_cusp_);
                if (path[i].direc_ < 0)
                {
                    orient = -orient;
                }
                yaw = std::atan2(orient(1), orient(0));
            }
            path[i].point_ = path_optim[i].head(2);
            path[i].yaw_ = yaw;
        }
        mpros_utils::updateCurvatureNew(path);
        mpros_utils::updatePathLength(path);

        return true;
    }
}