#ifndef OAL_TPOINT_HPP
#define OAL_TPOINT_HPP

#include <eigen3/Eigen/Eigen>


struct TPoint {
    Eigen::Vector2d pos;
    double time = 0;

    TPoint() = default;

    TPoint(Eigen::Vector2d pos, double time) : pos(std::move(pos)), time(time) {}
};


#endif //OAL_TPOINT_HPP
