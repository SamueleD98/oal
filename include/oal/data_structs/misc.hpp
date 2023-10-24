#ifndef DATA_STRUCTS_HPP
#define DATA_STRUCTS_HPP

#include <eigen3/Eigen/Eigen>
#include <utility>
#include <memory>
#include <iostream> //debug
#include <stack>
#include "obstacle.hpp"
#include "node.hpp"

typedef std::shared_ptr<Obstacle> obs_ptr;

struct VehicleInfo {
    Eigen::Vector2d position;
    std::vector<double> velocities;
    //double speed;

    /*double GetMaxSpeed(){
      auto max_ptr = std::min_element(velocities.begin(), velocities.end());
      auto max_v = std::distance(velocities.begin(), max_ptr);
      return *max_ptr;
      //vxs_abs[min_v].isVisible = true;
    }*/
};

struct ObstaclesInfo {
    std::vector<std::shared_ptr<Obstacle>> obstacles;
};

struct Path {
    std::stack<Node> waypoints;
    std::vector<std::string> overtakingObsList;
};


#endif
