#ifndef LINE_ITERATOR_H
#define LINE_ITERATOR_H

#include <vector>
#include <math.h>
#include <algorithm>

class LineIterator
{
public:
    /**
     * @brief Construct a new Line Iterator to implement Bresenham's line algorithm(https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm)
     * for discretized grid map, all positions are integer
     *
     * @param x1 position of first point
     * @param y1
     * @param x2 position of second point
     * @param y2
     */
    LineIterator(const int &x1, const int &y1, const int &x2, const int &y2) : x1_(x1), x2_(x2), y1_(y1), y2_(y2)
    {
        index_ = 0;
        steep_ = std::abs(y2_ - y1_) > std::abs(x2_ - x1_);
        drawLine();
    }
    /**
     * @brief draw the line
     *
     */
    void drawLine()
    {
        if (steep_)
        {
            std::swap(x1_, y1_);
            std::swap(x2_, y2_);
        }
        if (x1_ > x2_)
        {
            std::swap(x1_, x2_);
            std::swap(y1_, y2_);
        }
        int deltax = x2_ - x1_;
        int deltay = std::abs(y2_ - y1_);
        int error = deltax / 2;
        int ystep = y1_ < y2_ ? 1 : -1;
        int y = y1_;
        for (int x = x1_; x <= x2_; ++x)
        {
            if (steep_)
            {
                x_pos_.push_back(y);
                y_pos_.push_back(x);
            }
            else
            {
                x_pos_.push_back(x);
                y_pos_.push_back(y);
            }
            error -= deltay;
            if (error < 0)
            {
                y += ystep;
                error += deltax;
            }
        }
    }
    /**
     * @brief Get the position the index is pointing to
     *
     * @param x
     * @param y
     */
    void getCurr(int &x, int &y)
    {
        x = x_pos_[index_];
        y = y_pos_[index_];
    }
    int getCurrX() { return x_pos_[index_]; }
    int getCurrY() { return y_pos_[index_]; }
    /**
     * @brief return to the beginning of the line
     *
     */
    void reset()
    {
        index_ = 0;
    }
    /**
     * @brief move the index by num
     *
     * @param num default=1
     */
    void advance(int num = 1)
    {
        index_ += num;
    }
    /**
     * @brief if the iterator points to the end
     *
     * @return true
     * @return false
     */
    bool isEnd()
    {
        return index_ >= x_pos_.size();
    }
    /**
     * @brief size of the line
     *
     * @return int
     */
    int size()
    {
        return x_pos_.size();
    }

private:
    bool steep_;
    int index_;
    int x1_, y1_;
    int x2_, y2_;
    std::vector<int> x_pos_, y_pos_;
};

#endif // LINE_ITERATOR_H