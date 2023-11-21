#ifndef OAL_OBSTACLE_HPP
#define OAL_OBSTACLE_HPP

#include <eigen3/Eigen/Eigen>
#include <utility>
#include <memory>
#include <iostream> //debug

#include "data_structs/vertex.hpp"
#include "tpoint.hpp"

class Obstacle {
//private:
public:
    std::string id;
    Eigen::Vector2d position;  //absolute position at time 0
    double head;
    double vel_dir;
    double speed;
    double dim_x;
    double dim_y;
    double bb_x_bow_ratio;
    double bb_x_stern_ratio;
    double bb_y_ratio;
    double safety_bb_ratio; // safety_bb_dim / bb_dim
    std::vector<Vertex> vxs; // local position (wrt obs)

    bool higher_priority;

    // Set bb size according to own ship distance
    void SetSize(double dist_x, double dist_y, bool isAhead, double &bb_dim_x_stern, double &bb_dim_x_bow,
                 double &bb_dim_y) const;

    // Compute local position of bb vxs
    void FindLocalVxs(const Eigen::Vector2d &vhPos);

    // Project vxs position in world frame
    void FindAbsVxs(double time, std::vector<Vertex> &vxs_abs);

    // Project absolute position in obstacle frame (depends on time-instant)
    Eigen::Vector2d GetProjectionInLocalFrame(TPoint &time_point);

    // Check if point is in obs bb (depends on time-instant)
    bool IsInBB(TPoint &time_point);

    std::string plotStuff(double time);
    void print() const{
//      std::cout<<id<<std::endl<< position.x()<<" "<<position.y()<<std::endl<<heading<<std::endl<<speed<<std::endl;
//std::cout<<"Obstacle(\""<<id<<"\", {"<<position.x()<<", "<<position.y()<<"}, "<<heading<<", "<<speed<<"2, 0.5, 2, 2, 2, 1);"<<std::endl;
    }

//public:
    Obstacle(std::string name, Eigen::Vector2d position, double heading, double speed, double vel_dir, double dim_x, double dim_y,
             double bb_y_ratio, double bb_x_bow_ratio, double bb_x_stern_ratio, double safety_bb_ratio,
             bool high_priority = false)
            : id(std::move(name)), position(std::move(position)), head(heading), speed(speed), vel_dir(vel_dir), dim_x(dim_x),
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
