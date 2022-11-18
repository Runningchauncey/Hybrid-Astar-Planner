#ifndef RS_STATESPACE_H
#define RS_STATESPACE_H

#include <memory>

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

class RSStateSpace
{
public:
    RSStateSpace()
    {
        RSStateSpace(1.0);
    }
    /**
     * @brief Construct a new RSStateSpace object
     *
     * @param r min_radius
     */
    RSStateSpace(double r)
    {
        rs_ss_ = std::make_shared<ompl::base::ReedsSheppStateSpace>(r);
        min_r_ = r;
    }

    ~RSStateSpace()
    {
    }

    void setRadius(const double &r)
    {
        min_r_ = r;
        rs_ss_ = std::make_shared<ompl::base::ReedsSheppStateSpace>(r);
    }

    void setStart(const double &x, const double &y, const double &yaw)
    {
        rsstart_ = (ompl::base::SE2StateSpace::StateType *)rs_ss_->allocState();
        rsstart_->setXY(x, y);
        rsstart_->setYaw(yaw);
    }

    void setEnd(const double &x, const double &y, const double &yaw)
    {
        rsend_ = (ompl::base::SE2StateSpace::StateType *)rs_ss_->allocState();
        rsend_->setXY(x, y);
        rsend_->setYaw(yaw);
    }
    double getCost()
    {
        cost_ = rs_ss_->distance(rsstart_, rsend_);
        return cost_;
    }

private:
    std::shared_ptr<ompl::base::ReedsSheppStateSpace> rs_ss_;
    ompl::base::SE2StateSpace::StateType *rsstart_;
    ompl::base::SE2StateSpace::StateType *rsend_;
    double min_r_;
    double cost_;
};

#endif // !RS_STATESPACE_H