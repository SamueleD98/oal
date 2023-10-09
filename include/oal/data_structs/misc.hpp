#ifndef DATA_STRUCTS_HPP
#define DATA_STRUCTS_HPP

#include <eigen3/Eigen/Eigen>
#include <utility>
#include <memory>
#include <iostream> //debug
#include "obstacle.hpp"
#include "node.hpp"
#include "vertex.hpp"

struct VehicleInfo {
    Eigen::Vector2d position;
    double speed;
};

struct ObstaclesInfo {
  std::vector<Obstacle> obstacles;
};


#endif
