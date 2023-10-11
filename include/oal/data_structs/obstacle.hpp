#ifndef OAL_OBSTACLE_HPP
#define OAL_OBSTACLE_HPP

#include <eigen3/Eigen/Eigen>
#include <utility>
#include <memory>
#include <iostream> //debug

#include "data_structs/vertex.hpp"

class Obstacle {
//private:
public:
    std::string id;
    Eigen::Vector2d position;  // the original position at time 0
    double heading;
    double speed;
    double dim_x;
    double dim_y;
    double bb_x_bow_ratio;
    double bb_x_stern_ratio;
    double bb_y_ratio;
    // double max_bb_ratio; // max_bb_dim / bb_dim
    double safety_bb_ratio; // safety_bb_dim / bb_dim
    std::vector<Vertex> vxs; // local position (wrt obs)

    bool higher_priority;

    static void
    SetSize(double dist_x, double dist_y, double dim_x, double dim_y, double safety_ratio, double bb_ratio_x,
            double bb_ratio_y, double &bb_dim_x, double &bb_dim_y);


    void ComputeLocalVertexes(const Eigen::Vector2d &vhPos);

//public:
    Obstacle(std::string name, Eigen::Vector2d position, double heading, double speed, double dim_x, double dim_y,
             double bb_y_ratio, double bb_x_bow_ratio, double bb_x_stern_ratio, double safety_bb_ratio,
             bool high_priority = false)
            : id(std::move(name)), position(std::move(position)), heading(heading), speed(speed), dim_x(dim_x),
              dim_y(dim_y), bb_y_ratio(bb_y_ratio), bb_x_bow_ratio(bb_x_bow_ratio), bb_x_stern_ratio(bb_x_stern_ratio),
              safety_bb_ratio(safety_bb_ratio), higher_priority(high_priority) {

      if (dim_x <= 0 || dim_y <= 0) {
        throw std::invalid_argument("Obstacle dimension must be strictly positive.");
      }

      if (safety_bb_ratio < 1 || bb_y_ratio < 1 || bb_x_bow_ratio < 1 || bb_x_stern_ratio < 1) {
        throw std::invalid_argument("Bounding box ratios cannot be less than one.");
      }

      if (safety_bb_ratio > bb_y_ratio || safety_bb_ratio > bb_x_bow_ratio || safety_bb_ratio > bb_x_stern_ratio) {
        throw std::invalid_argument("Generic bounding box ratios cannot be smaller than safety one.");
      }

      if (abs(heading) > M_PI) {
        throw std::invalid_argument("Obstacle heading must be expressed in radians [-pi, +pi]");
      }
    }
};


#endif //OAL_OBSTACLE_HPP
