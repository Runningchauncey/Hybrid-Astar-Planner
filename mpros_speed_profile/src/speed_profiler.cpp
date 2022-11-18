#include "mpros_speed_profile/speed_profiler.h"

void SpeedProfiler::sectionInSegment(const std::vector<PathPoint> &path)
{
    // circular path flag
    bool in_circle = std::abs(path[0].curv_) > curv_lim_;
    // start index of next segment
    int start_idx = 0;
    // maximal curvature within this segment
    double max_curv = 0;
    // start profile for next segment
    double vs = 0, as = 0;
    // divide section into segments
    for (int i = 0; i < path.size(); ++i)
    {
        // path ends, make segment of passed points
        if (i == path.size() - 1)
        {
            TrajSegment seg(mpros_utils::slicing(path, start_idx, i + 1));
            seg.name = in_circle ? "circular" : "non-circular";
            seg.curvature = max_curv;
            seg.vs = vs;
            seg.as = as;
            seg.vg = 0;
            seg.ag = 0;
            seg.vmax = in_circle ? v_lim_cir_ : v_lim_;
            seg.s0 = path[start_idx].dist_;
            seg.length = path[i].dist_ - path[start_idx].dist_;
            seg.direction = path[i].direc_;
            segment_vec_.push_back(seg);

            start_idx = i;
            vs = segment_vec_.back().vg;
            as = segment_vec_.back().ag;
        }
        // i == circular path start index, make segment of passed points
        else if (std::abs(path[i].curv_) > curv_lim_ && !in_circle)
        {
            TrajSegment seg(mpros_utils::slicing(path, start_idx, i + 1));
            seg.name = "non-circular";
            seg.curvature = max_curv;
            seg.vs = vs;
            seg.as = as;
            seg.vg = v_lim_cir_;
            seg.ag = 0;
            seg.vmax = v_lim_;
            seg.s0 = path[start_idx].dist_;
            seg.length = path[i].dist_ - path[start_idx].dist_;
            seg.direction = path[i].direc_;
            segment_vec_.push_back(seg);

            start_idx = i;
            vs = segment_vec_.back().vg;
            as = segment_vec_.back().ag;
            // update circular path flag
            in_circle = true;
            // reset
            max_curv = 0;
        }
        // i == circular path end index, make segment of passed points
        else if (std::abs(path[i].curv_) < curv_lim_ && in_circle)
        {
            TrajSegment seg(mpros_utils::slicing(path, start_idx, i + 1));
            seg.name = "circular";
            seg.curvature = max_curv;
            seg.vs = vs;
            seg.as = as;
            seg.vg = v_lim_cir_;
            seg.ag = 0;
            seg.vmax = v_lim_cir_;
            seg.s0 = path[start_idx].dist_;
            seg.length = path[i].dist_ - path[start_idx].dist_;
            seg.direction = path[i].direc_;
            segment_vec_.push_back(seg);

            start_idx = i;
            vs = segment_vec_.back().vg;
            as = segment_vec_.back().ag;
            // update circular path flag
            in_circle = false;
            // reset
            max_curv = 0;
        }
        // update loop params
        max_curv = std::fmax(max_curv, std::abs(path[i].curv_));
    }

    // go through the velocity constaints backwards
    double vs_next_seg = 0;
    for (int i = segment_vec_.size() - 1; i > -1; --i)
    {
        auto &segment = segment_vec_[i];
        // check the distance of stopping segments
        segment.vg = vs_next_seg;
        if (segment.vg < segment.vs)
        {
            double min_brake_dist = calMinBrakeDist(segment.vs, segment.vg);
            ROS_DEBUG("[Profiler] segment %d min brake dist %f, segment len %f, vs %f vg %f",
                      i, min_brake_dist, segment.length, segment.vs, segment.vg);
            if (segment.length < min_brake_dist)
            {
                double brake_dt = 2 * a_x_lim_ / j_lim_;
                if (segment.length < segment.vg * brake_dt)
                {
                    ROS_DEBUG("[Profiler] modifying end vel at segment %d, old vs %f", i, segment.vg);
                    // the end velocity too large
                    // set this segment of constant velocity
                    segment.vg = segment.length / brake_dt;
                    // set the vs of next segment
                    segment_vec_[i + 1].vs = segment.vg;
                    ROS_DEBUG("[Profiler] new end vel at segment %d, old vs %f", i, segment.vg);
                }
                ROS_DEBUG("[Profiler] modifying start vel at segment %d, old vs %f", i, segment.vs);
                segment.vs = segment.length / a_x_lim_ * j_lim_ - segment.vg;
                ROS_DEBUG("[Profiler] new vs %f", segment.vs);
            }
        }
        vs_next_seg = segment.vs;
    }

    // go through the velocity constaints forwards again
    double vg_last_seg = 0;
    for (int i = 0; i < segment_vec_.size(); ++i)
    {
        auto &segment = segment_vec_[i];
        // check the distance of stopping segments
        segment.vs = vg_last_seg;
        if (segment.vs < segment.vg)
        {
            // minimal acceleration distance calculation same as brake distance
            // just exchange the start and end velocity
            double min_acc_dist = calMinBrakeDist(segment.vg, segment.vs);
            ROS_DEBUG("[Profiler] segment %d min acc dist %f, segment len %f, vs %f vg %f", i, min_acc_dist, segment.length, segment.vs, segment.vg);
            if (segment.length < min_acc_dist)
            {
                double acc_dt = 2 * a_x_lim_ / j_lim_;
                if (segment.length < segment.vs * acc_dt)
                {
                    ROS_DEBUG("[Profiler] modifying start vel at segment %d, old vs %f", i, segment.vs);
                    // the start velocity too large
                    // set this segment of constant velocity
                    segment.vs = segment.length / acc_dt;
                    // set the vg of next segment
                    segment_vec_[i - 1].vg = segment.vs;
                    ROS_DEBUG("[Profiler] new start vel at segment %d, old vs %f", i, segment.vs);
                }
                ROS_DEBUG("[Profiler] modifying end vel at segment %d, old vs %f", i, segment.vg);
                segment.vg = segment.length / a_x_lim_ * j_lim_ - segment.vs;
                ROS_DEBUG("[Profiler] new vg %f", segment.vg);
            }
        }
        vg_last_seg = segment.vg;
    }

    if (debug_)
    {
        // for debug
        std::cout << "\nsegmentation done----------\n";
        for (int i = 0; i < segment_vec_.size(); ++i)
        {
            std::cout << "index " << i
                      << " name: " << segment_vec_[i].name
                      << " startlen " << segment_vec_[i].s0
                      << " endlen " << segment_vec_[i].length + segment_vec_[i].s0
                      << " vs " << segment_vec_[i].vs
                      << " vg " << segment_vec_[i].vg
                      << " start pt " << segment_vec_[i].path.front().getX() << "," << segment_vec_[i].path.front().getY()
                      << " curvature " << segment_vec_[i].curvature << std::endl;
        }
        std::cout << "\n";
    }
}
void SpeedProfiler::makeProfile(std::vector<PathPoint> &path)
{
    reset();
    path_len_ = path.back().dist_;
    // divide path into sections according to cusp points
    section_begin_idx_.push_back(0);
    for (int i = 1; i < path.size(); ++i)
    {
        if (path[i].direc_ != path[i - 1].direc_)
        {
            section_begin_idx_.push_back(i - 1);
            section_end_idx_.push_back(i);
        }
    }
    section_end_idx_.push_back(path.size());

    // divide sections into segments
    for (int i = 0; i < section_begin_idx_.size(); ++i)
    {
        auto section = mpros_utils::slicing(path, section_begin_idx_[i], section_end_idx_[i]);
        // cusp point follows the direction of last section, correct here
        section[0].direc_ = section[1].direc_;
        if (debug_)
        {
            std::cout << "for section " << i << " the direction is " << section[0].direc_ << std::endl;
        }
        sectionInSegment(section);
    }
    // specially set the end speed of whole trajectory
    segment_vec_.back().vg = v_end_;

    // make profile for each segment
    // start timestamp
    double tg_prev_sec = t_start_;
    // start speed of whole trajectory
    double vg_prev_sec = v_start_;
    for (int i = 0; i < segment_vec_.size(); ++i)
    {
        direction_ = segment_vec_[i].direction;
        segment_vec_[i].ts = tg_prev_sec;
        segment_vec_[i].vs = vg_prev_sec;
        setSCurve(i);
        tg_prev_sec = segment_vec_[i].tg;
        vg_prev_sec = segment_vec_[i].vg;
    }
    t_end_ = segment_vec_.back().tg; // end time of this section

    // add end point to trajectory
    PathPoint end_pt = path.back();
    end_pt.vel_ = 0;
    end_pt.accel_ = 0;
    end_pt.jerk_ = 0;
    end_pt.time_ = t_end_;
    trajectory_.push_back(end_pt);

    // for debug, check the continuity of segments
    if (debug_)
    {
        std::cout << "\nchecking continuity through segments\n";
        for (int i = 0; i < segment_vec_.size(); ++i)
        {
            auto &seg = segment_vec_[i];
            std::cout << "segment ts: " << seg.ts
                      << " tg: " << seg.tg
                      << " ds: " << seg.path.front().dist_
                      << " dg: " << seg.path.back().dist_
                      << " vs: " << seg.vs
                      << " vg: " << seg.vg
                      << std::endl;
        }
    }
}

void SpeedProfiler::setSCurve(int idx)
{
    // for debug
    double temp_len = 0;
    // get the segment
    TrajSegment &seg = segment_vec_[idx];
    seg.vmax = seg.name == "circular" ? v_lim_cir_ : v_lim_;

    double dist_this_pt = seg.s0;
    double tstart = seg.ts, tend = seg.ts;

    std::vector<double> seg_dt(7, 0);
    std::vector<double> seg_jerk = {j_lim_, 0, -j_lim_, 0, -j_lim_, 0, j_lim_};
    double seg_v0 = seg.vs;
    double seg_a0 = seg.as;

    // calculate the acceleration needed, if acc less than axlim, second part is not needed
    double acc_needed = std::fmin(std::sqrt((seg.vmax - seg.vs) * j_lim_), a_x_lim_);
    double decc_needed = std::fmin(std::sqrt((seg.vmax - seg.vg) * j_lim_), a_x_lim_);
    // estimate distance for non-constant-speed sections
    double d03 = acc_needed == a_x_lim_ ? (seg.vmax + seg.vs) / 2 * ((seg.vmax - seg.vs) / a_x_lim_ + dt_reach_ax_lim_)
                                        : (seg.vmax + seg.vs) * std::sqrt((seg.vmax - seg.vs) / j_lim_);
    double d47 = decc_needed == a_x_lim_ ? (seg.vmax + seg.vg) / 2 * ((seg.vmax - seg.vg) / a_x_lim_ + dt_reach_ax_lim_)
                                         : (seg.vmax + seg.vg) * std::sqrt((seg.vmax - seg.vg) / j_lim_);
    // adjust the velocity limit within this segment
    while (seg.length - d03 - d47 < 0)
    {
        seg.vmax = seg.vs + (seg.vmax - seg.vs) / 2;
        // if the velocity difference is a small number, set to constant velocity
        if (seg.vmax - seg.vs < 0.01)
        {
            seg.vmax = seg.vs;
            seg.vg = seg.vg > seg.vmax ? seg.vmax : seg.vg;
            acc_needed = 0;
            decc_needed = std::fmin(std::sqrt((seg.vmax - seg.vg) * j_lim_), a_x_lim_);
            break;
        }
        seg.vg = seg.vg > seg.vmax ? seg.vmax : seg.vg;
        acc_needed = std::fmin(std::sqrt((seg.vmax - seg.vs) * j_lim_), a_x_lim_);
        decc_needed = std::fmin(std::sqrt((seg.vmax - seg.vg) * j_lim_), a_x_lim_);
        d03 = acc_needed == a_x_lim_ ? seg.vmax / 2 * ((seg.vmax - seg.vs) / a_x_lim_ + dt_reach_ax_lim_)
                                     : (seg.vmax + seg.vs) * std::sqrt((seg.vmax - seg.vs) / j_lim_);
        d47 = decc_needed == a_x_lim_ ? seg.vmax / 2 * ((seg.vmax - seg.vg) / a_x_lim_ + dt_reach_ax_lim_)
                                      : (seg.vmax + seg.vg) * std::sqrt((seg.vmax - seg.vg) / j_lim_);
    }

    // calculate dt for each section
    seg_dt[0] = acc_needed / j_lim_;
    seg_dt[1] = acc_needed == 0 || acc_needed != a_x_lim_ ? 0 : (seg.vmax - seg.vs) / acc_needed - seg_dt[0];
    seg_dt[2] = seg_dt[0];
    seg_dt[3] = (seg.length - d03 - d47) / seg.vmax;
    seg_dt[4] = decc_needed / j_lim_;
    seg_dt[5] = decc_needed == 0 || decc_needed != a_x_lim_ ? 0 : (seg.vmax - seg.vg) / decc_needed - seg_dt[4];
    seg_dt[6] = seg_dt[4];

    // interpolate path with speed profile
    for (int i = 0; i < 7; ++i)
    {
        double dt = seg_dt[i];
        if (dt < 1e-5)
        {
            continue;
        }
        tend += dt;
        if (debug_)
        {
            temp_len += calDist(dist_this_pt, seg_v0, seg_a0, seg_jerk[i], dt) - dist_this_pt;
            std::cout << "part idx: " << i
                      << " dt " << dt
                      << " tstart " << tstart
                      << " tend " << tend
                      << " jerk " << seg_jerk[i]
                      << " a0 " << seg_a0
                      << " v0 " << seg_v0
                      << " vmax " << seg.vmax
                      << " s0 " << seg.s0
                      << " d0 " << dist_this_pt
                      << " length " << calDist(dist_this_pt, seg_v0, seg_a0, seg_jerk[i], dt) - dist_this_pt
                      << std::endl;
        }
        generateTrajSeg(seg.path, tstart, tend, seg_jerk[i], seg_a0, seg_v0, dist_this_pt);
        // update the distance, velocity, acceleration, order important
        dist_this_pt = calDist(dist_this_pt, seg_v0, seg_a0, seg_jerk[i], dt);
        seg_v0 = calVelocity(seg_v0, seg_a0, seg_jerk[i], dt);
        seg_a0 = calAccel(seg_a0, seg_jerk[i], dt);
        tstart = tend;
    }
    seg.tg = tend;
}

void SpeedProfiler::generateTrajSeg(const std::vector<PathPoint> &path, double tstart, double tend, double j, double a0, double v0, double dist0)
{
    for (double t = tstart; t < tend; t += dt_)
    {
        // calculate point info
        double delta_t = t - tstart;
        double pt_j = j * direction_;
        double pt_a = calAccel(a0, j, delta_t) * direction_;
        double pt_v = calVelocity(v0, a0, j, delta_t) * direction_;
        double pt_dist = calDist(dist0, v0, a0, j, delta_t);
        ROS_DEBUG_COND(pt_dist < dist0, "[Profiler] negative distance");
        // generate new point
        int idx_next = findPathPoint(pt_dist, path);
        PathPoint new_pt;
        if (pathPointInterpolate(path[idx_next - 1], path[idx_next], pt_dist, new_pt))
        {
            // set point info
            new_pt.time_ = t;
            new_pt.jerk_ = pt_j;
            new_pt.accel_ = pt_a;
            new_pt.vel_ = pt_v;
            new_pt.dist_ = pt_dist;

            ROS_DEBUG_COND(!trajectory_.empty() && pt_dist < trajectory_.back().dist_,
                                      "[Profiler] negative distance");
            trajectory_.push_back(new_pt);
        }
    }
}

bool SpeedProfiler::pathPointInterpolate(const PathPoint &prev_pt, const PathPoint &next_pt, double dist, PathPoint &pt)
{
    // distance to next pt / interval length
    double a = (next_pt.dist_ - dist) / (next_pt.dist_ - prev_pt.dist_);
    ROS_DEBUG_COND(dist > next_pt.dist_, "[Profiler] error next pt next dist %f this dist %f",
                              next_pt.dist_, dist);

    // linear interpolation when on straight segment
    // create extra control points
    Eigen::Vector2d pt1 = mpros_utils::findIntersection(prev_pt.point_, next_pt.point_, prev_pt.is_cusp_ ? -prev_pt.yaw_ : prev_pt.yaw_, next_pt.yaw_);
    if (pt1(0) > std::fmax(prev_pt.point_(0), next_pt.point_(0)) ||
        pt1(0) < std::fmin(prev_pt.point_(0), next_pt.point_(0)) ||
        pt1(1) > std::fmax(prev_pt.point_(1), next_pt.point_(1)) ||
        pt1(1) < std::fmin(prev_pt.point_(1), next_pt.point_(1)))
    {
        pt.point_ = a * prev_pt.point_ + (1 - a) * next_pt.point_;
        pt.yaw_ = a * prev_pt.yaw_ + (1 - a) * next_pt.yaw_;
        ROS_DEBUG("[Profiler] odd control pt among %f, %f   %f, %f, control point %f %f",
                  prev_pt.point_(0), prev_pt.point_(1),
                  next_pt.point_(0), next_pt.point_(1),
                  pt1(0), pt1(1));
    }
    else
    {

        // interpolation by quadratic Bezier
        // interpolate Bezier curve point between prev point and next point
        double mu = (dist - prev_pt.dist_) / (next_pt.dist_ - prev_pt.dist_);
        // create new path point
        pt.point_ = mpros_utils::quadBezier(prev_pt.point_, pt1, next_pt.point_, mu);
    }
    // copy the next point's direction and steering for this interval's maneuver is stored in next pt
    pt.direc_ = next_pt.direc_;
    pt.steering_ = next_pt.steering_;
    // to be updated
    Eigen::Vector2d orient = mpros_utils::tangentDir(prev_pt.point_, pt.point_, next_pt.point_, false);
    if (next_pt.direc_ < 0)
    {
        orient = -orient;
    }
    pt.yaw_ = std::atan2(orient(1), orient(0));

    pt.curv_ = mpros_utils::calCurvature(prev_pt.point_, pt.point_, next_pt.point_);

    return true;
}

void SpeedProfiler::reset()
{
    trajectory_.clear();
    segment_vec_.clear();
    section_begin_idx_.clear();
    section_end_idx_.clear();
}
