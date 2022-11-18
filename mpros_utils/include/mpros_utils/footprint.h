#ifndef MPROS_UTILS__FOOTPRINT_H
#define MPROS_UTILS__FOOTPRINT_H

#include <iostream>
#include <math.h>
#include <vector>

#include <Eigen/Eigen>

#include "mpros_utils/line_iterator.h"

typedef std::vector<int8_t> FlatKerType;
typedef std::vector<std::vector<int8_t>> KerType;

class Footprint
{
public:
    Footprint() = default;

    ~Footprint()
    {
    }
    /**
     * @brief configure the footprint
     *
     * @param len Input: vehicle length 
     * @param wid Input: vehicle width
     * @param xoff Input: vehicle rotate center x offset from shape center
     * @param yoff Input: vehicle rotate center y offset from shape center
     * @param reso Input: map resolution
     */
    void configure(double len, double wid, double xoff, double yoff, double reso, double yaw_reso)
    {
        vehicle_length_ = len;
        vehicle_width_ = wid;
        center_x_offset_ = xoff;
        center_y_offset_ = yoff;
        resolution_ = reso;
        yaw_resolution_ = yaw_reso;
        // number of yaw discretization
        num_yaw_ = std::ceil(2 * M_PI / yaw_resolution_);
        // set corner point location
        x_front_ = len / 2 + xoff;
        x_back_ = len / 2 - xoff;
        y_left_ = wid / 2 + yoff;
        y_right_ = wid / 2 - yoff;
        // set the kernel size
        kernel_size_ = 2 * std::ceil(std::hypot(std::fmax(x_front_, x_back_), std::fmax(y_left_, y_right_)) / resolution_) + 1;
        origin_ = kernel_size_ / 2;
        // init vector to store the kernel
        kernel_.resize(kernel_size_, FlatKerType(kernel_size_, 0));
        all_kernel_.resize(num_yaw_, kernel_);
        // use reserve for using insert method later
        kernel_flatten_.reserve(kernel_size_ * kernel_size_);
        all_kernel_flatten_.resize(num_yaw_, kernel_flatten_);
    }
    /**
     * @brief set yaw angle in rad
     *
     * @param yaw
     */
    void setYaw(const double &yaw)
    {
        yaw_ = yaw;
        sin_yaw_ = std::sin(yaw);
        cos_yaw_ = std::cos(yaw);
    }
    /**
     * @brief get the flattened kernel
     *
     * @return FlatKerType
     */
    FlatKerType getFlatKernel(double yaw)
    {
        regularYaw(yaw);
        int idx_yaw = std::floor(yaw / yaw_resolution_);
        return all_kernel_flatten_[idx_yaw];
    }
    /**
     * @brief Get the kernel with given yaw angle
     * 
     * @param yaw Input: heading angle
     * @return KerType 
     */
    KerType getKernel(double yaw)
    {
        regularYaw(yaw);
        int idx_yaw = std::floor(yaw / yaw_resolution_);
        return all_kernel_[idx_yaw];
    }
    /**
     * @brief Set the kernel
     * 
     */
    void setKernel()
    {
        for (int i = 0; i < num_yaw_; ++i)
        {
            setYaw(i * yaw_resolution_);
            KerType &ki = all_kernel_[i];
            FlatKerType &kfi = all_kernel_flatten_[i];
            setOneKernel(ki, kfi);
        }
    }
    /**
     * @brief Set the kernel for one yaw
     *
     */
    void setOneKernel(KerType &k, FlatKerType &kf)
    {
        // rotation matrix according to yaw
        Eigen::Matrix2d rotate{{cos_yaw_, sin_yaw_}, {-sin_yaw_, cos_yaw_}};
        // corner points in vehicle coordinate
        Eigen::Matrix<double, 2, 4> corners{{x_front_, -x_back_, -x_back_, x_front_},
                                            {y_left_, y_left_, -y_right_, -y_right_}};
        // transform the corner to kernel coordinate
        corners = (rotate * corners).array() / resolution_ + origin_;
        std::vector<int> x_vec(corners.row(0).begin(), corners.row(0).end()),
            y_vec(corners.row(1).begin(), corners.row(1).end());
        // set the kernel within corner points
        // transpose due to y-i, x-j in
        fillRectangular(k, y_vec, x_vec);
        // set the flatten kernel
        for (int i = 0; i < kernel_size_; ++i)
        {
            kf.insert(kf.end(), k[i].begin(), k[i].end());
        }
    }
    /**
     * @brief Get the kernel side length
     * 
     * @return int 
     */
    int getKernelSize()
    {
        return kernel_size_;
    }
    /**
     * @brief Get the origin coordinate (i, j)
     * 
     * @return int 
     */
    int getOrigin()
    {
        return origin_;
    }
    /**
     * @brief print kernel for debug
     * 
     * @param kernel 
     */
    static void printKernel(KerType kernel)
    {
        for (int i = 0; i < kernel.size(); ++i)
        {
            for (int j = 0; j < kernel[i].size(); ++j)
            {
                std::cout << (int)kernel[i][j] << " ";
            }
            std::cout << "\n";
        }
    }

private:
    /**
     * @brief set yaw within [-pi, pi]
     * 
     * @param yaw 
     */
    void regularYaw(double &yaw)
    {
        while (yaw < 0)
        {
            yaw = std::fmod(yaw, -2.0 * M_PI) + 2.0 * M_PI;
        }
        while (yaw > 2 * M_PI)
        {
            yaw = std::fmod(yaw, 2.0 * M_PI);
        }
        return;
    }
    /**
     * @brief fill a rectangular surrounded by given points, point vector size of 4
     *
     * @param k
     * @param x_vec
     * @param y_vec
     */
    void fillRectangular(KerType &k, std::vector<int> &x_vec, const std::vector<int> &y_vec)
    {
        if (x_vec.size() != 4 || y_vec.size() != 4)
        {
            std::cerr << "corner location size wrong\n";
            return;
        }
        // two orthogonal border lines of footprint
        LineIterator line1(x_vec.at(0), y_vec.at(0), x_vec.at(1), y_vec.at(1));
        LineIterator line2(x_vec.at(1), y_vec.at(1), x_vec.at(2), y_vec.at(2));
        // corner point 1, intersection of line1, line2
        int x1 = x_vec.at(1), y1 = y_vec.at(1);
        // fill the rectangle by slide one line along the other
        for (; !line1.isEnd(); line1.advance())
        {
            int x_line1 = line1.getCurrX(),
                y_line1 = line1.getCurrY();

            for (; !line2.isEnd(); line2.advance())
            {
                int dx = line2.getCurrX() - x1,
                    dy = line2.getCurrY() - y1;
                k[x_line1 + dx][y_line1 + dy] = 1;
            }
            line2.reset();
        }
        // fill the missed grids
        for (int i = 1; i < kernel_size_ - 1; ++i)
        {
            for (int j = 1; j < kernel_size_ - 1; ++j)
            {
                // 4 connected neighbours are filled
                if (k[i - 1][j] &&
                    k[i + 1][j] &&
                    k[i][j - 1] &&
                    k[i][j + 1])
                {
                    k[i][j] = 1;
                }
            }
        }
    }
    /**
     * @brief reset kernel
     *
     */
    void reset()
    {
        kernel_ = std::vector<FlatKerType>(kernel_size_, FlatKerType(kernel_size_, 0));
        kernel_flatten_ = FlatKerType(kernel_size_ * kernel_size_, 0);
    }

    // member variable
    // heading angle and its trigonometry
    double yaw_ = 0, sin_yaw_ = 0, cos_yaw_ = 1;
    // vehicle shape
    double vehicle_length_, vehicle_width_;
    double center_x_offset_, center_y_offset_;
    // boundary of vehicle footprint
    double x_front_, x_back_, y_left_, y_right_;
    // one coordinate of kernel center
    double origin_;
    // kernel resolution
    double resolution_;
    // yaw resolution due to discretization
    double yaw_resolution_;
    // number of yaw discretization
    int num_yaw_;
    // side length of kernel
    int kernel_size_;

    // kernel of one heading angle
    KerType kernel_;
    // kernels of all discrete heading angles
    std::vector<KerType> all_kernel_;
    // flattened kernel as 1D vector
    FlatKerType kernel_flatten_;
    // all flattened kernels
    std::vector<FlatKerType> all_kernel_flatten_;
};

#endif // MPROS_UTILS__FOOTPRINT_H
